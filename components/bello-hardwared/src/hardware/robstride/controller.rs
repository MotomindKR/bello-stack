use std::collections::{BTreeMap, HashSet};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU64};
use std::time::{Duration, Instant};

use anyhow::Result;
use parking_lot::{Mutex, RwLock, RwLockWriteGuard};
use robstride_protocol::{
    ControlCommand, active_report_frame, control_frame, enable_frame, param_write_u8_frame,
    stop_frame,
};

use super::health::{MotorFaultCause, ThermalHealth, classify_feedback};
use super::velocity::MotorVelocityFilter;
use super::{Feedback, MotorSpec};
use crate::config::MotorVelocityFilterConfig;
use crate::domain::{
    BusName, DeviceStatus, GroupName, ImpedanceCommand, MotorName, MotorTelemetry, TelemetryPayload,
};
use crate::hardware::can::{BusError, BusHandle, CanFrame, PendingBatch, PeriodicControl};
use crate::hardware::lifecycle::DeviceLifecycle;
use crate::telemetry::Telemetry;

const RUN_MODE_INDEX: u16 = 0x7005;
const IMPEDANCE_RUN_MODE: u8 = 0;

#[derive(Default)]
struct PendingControlBatch {
    started: Option<Instant>,
    motors: HashSet<MotorName>,
    buses: BTreeMap<BusName, (BusHandle, Vec<CanFrame>)>,
}

pub(super) struct ControlBatcher {
    group: GroupName,
    expected_motors: usize,
    fresh_for: Duration,
    pending: Mutex<PendingControlBatch>,
}

impl ControlBatcher {
    pub(super) fn new(group: GroupName, expected_motors: usize, fresh_for: Duration) -> Self {
        Self {
            group,
            expected_motors,
            fresh_for,
            pending: Mutex::new(PendingControlBatch::default()),
        }
    }

    fn submit(&self, motor: &MotorName, bus: &BusHandle, frame: CanFrame) -> Result<(), BusError> {
        let mut pending = self.pending.lock();
        if !pending.motors.insert(motor.clone()) {
            *pending = PendingControlBatch::default();
            return Err(BusError::ControlBatchInterrupted {
                group: self.group.clone(),
            });
        }
        let started = *pending.started.get_or_insert_with(Instant::now);
        pending
            .buses
            .entry(bus.spec.name.clone())
            .or_insert_with(|| (bus.clone(), Vec::new()))
            .1
            .push(frame);
        if pending.motors.len() != self.expected_motors {
            return Ok(());
        }

        let batches = std::mem::take(&mut pending.buses);
        *pending = PendingControlBatch::default();
        drop(pending);
        let expires_at = started + self.fresh_for;
        // A failed bus must not prevent the other buses from receiving this tick.
        let mut first_error = None;
        for (bus, frames) in batches.into_values() {
            if let Err(error) = bus.enqueue_control_batch(frames, expires_at) {
                if first_error.is_none() {
                    first_error = Some(error);
                } else {
                    tracing::error!(group=%self.group, %error, "additional CAN bus rejected control batch");
                }
            }
        }
        first_error.map_or(Ok(()), Err)
    }
}

#[derive(Clone, Debug)]
pub(crate) struct MotorFault {
    pub(crate) cause: MotorFaultCause,
    pub(crate) detail: String,
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct TimedFeedback {
    pub(crate) value: Feedback,
    pub(crate) published_velocity: f64,
    pub(crate) received: Instant,
}

struct OperatingFeedback {
    since: Instant,
    confirmed: bool,
}

#[derive(Default)]
struct FeedbackState {
    latest: Option<TimedFeedback>,
    operating: Option<OperatingFeedback>,
    current_fault: Option<MotorFault>,
    latched_fault: Option<MotorFault>,
}

pub(crate) struct MotorRecovery<'a> {
    state: RwLockWriteGuard<'a, FeedbackState>,
    pub(crate) feedback: Feedback,
}

impl MotorRecovery<'_> {
    pub(crate) fn clear_latched_fault(&mut self) {
        self.state.latched_fault = None;
    }
}

pub(super) struct FeedbackProcessingConfig {
    pub(super) timeout: Duration,
    pub(super) velocity_filter: MotorVelocityFilterConfig,
}

#[derive(Clone)]
pub(crate) struct MotorController {
    pub(crate) spec: MotorSpec,
    pub(crate) group: GroupName,
    bus: BusHandle,
    control_batcher: Arc<ControlBatcher>,
    feedback: Arc<RwLock<FeedbackState>>,
    velocity_filter: Arc<Mutex<MotorVelocityFilter>>,
    monitoring_request: Arc<Mutex<Option<(u64, Instant)>>>,
    periodic_reporting: Arc<Mutex<bool>>,
    thermal_health: Arc<Mutex<ThermalHealth>>,
    feedback_timeout: Duration,
    lifecycle: DeviceLifecycle,
    telemetry: Telemetry,
}

impl MotorController {
    pub(crate) fn periodic_control(
        &self,
        period: Duration,
        lease: Arc<AtomicU64>,
        shutdown: Arc<AtomicBool>,
    ) -> PeriodicControl {
        self.bus
            .periodic_control(&self.group, period, lease, shutdown)
    }

    pub(super) fn new(
        spec: MotorSpec,
        group: GroupName,
        bus: BusHandle,
        control_batcher: Arc<ControlBatcher>,
        feedback_processing: FeedbackProcessingConfig,
        lifecycle: DeviceLifecycle,
        telemetry: Telemetry,
    ) -> Self {
        Self {
            spec,
            group,
            bus,
            control_batcher,
            feedback: Arc::new(RwLock::new(FeedbackState::default())),
            velocity_filter: Arc::new(Mutex::new(MotorVelocityFilter::new(
                feedback_processing.velocity_filter.cutoff_hz,
                feedback_processing.timeout,
            ))),
            monitoring_request: Arc::new(Mutex::new(None)),
            periodic_reporting: Arc::new(Mutex::new(true)),
            thermal_health: Arc::new(Mutex::new(ThermalHealth::default())),
            feedback_timeout: feedback_processing.timeout,
            lifecycle,
            telemetry,
        }
    }

    pub(super) fn lifecycle(&self) -> &DeviceLifecycle {
        &self.lifecycle
    }
    pub(crate) fn status(&self) -> DeviceStatus {
        self.lifecycle.status()
    }

    pub(crate) fn feedback(&self) -> Option<TimedFeedback> {
        self.feedback.read().latest
    }

    pub(crate) fn control_fault(&self) -> Option<MotorFault> {
        let state = self.feedback.read();
        if let Some(fault) = state
            .latched_fault
            .as_ref()
            .or(state.current_fault.as_ref())
        {
            return Some(fault.clone());
        }
        let status = self.lifecycle.status();
        (!status.state.is_online()).then(|| MotorFault {
            cause: MotorFaultCause::Communication,
            detail: format!("{} ({})", status.state, status.detail),
        })
    }

    pub(crate) fn feedback_ready(&self) -> bool {
        let state = self.feedback.read();
        let Some(operating) = &state.operating else {
            return false;
        };
        operating.confirmed
            && state.latched_fault.is_none()
            && self.lifecycle.state().is_online()
            && state.latest.is_some_and(|feedback| {
                feedback.received >= operating.since
                    && feedback.received.elapsed() <= self.feedback_timeout
            })
    }

    pub(crate) fn recovery_feedback(&self) -> Option<MotorRecovery<'_>> {
        let state = self.feedback.write();
        let latest = state.latest?;
        if !self.bus.status().state.is_online()
            || !self.lifecycle.state().is_online()
            || state.current_fault.is_some()
            || latest.received.elapsed() > self.feedback_timeout
        {
            return None;
        }
        Some(MotorRecovery {
            state,
            feedback: latest.value,
        })
    }

    pub(crate) fn configure_enable(&self) -> Result<(), BusError> {
        *self.periodic_reporting.lock() = true;
        *self.monitoring_request.lock() = None;
        {
            let mut state = self.feedback.write();
            state.latest = None;
            state.operating = None;
        }
        self.velocity_filter.lock().reset();
        self.bus.enqueue_configuration_batch(vec![
            param_write_u8_frame(
                self.bus.spec.host_id,
                self.spec.address.id,
                RUN_MODE_INDEX,
                IMPEDANCE_RUN_MODE,
            )
            .into(),
            active_report_frame(self.bus.spec.host_id, self.spec.address.id, true).into(),
            enable_frame(self.bus.spec.host_id, self.spec.address.id).into(),
        ])?;
        let mut state = self.feedback.write();
        self.lifecycle
            .await_feedback("motor enabled; waiting for fresh operating feedback");
        state.operating = Some(OperatingFeedback {
            since: Instant::now(),
            confirmed: false,
        });
        Ok(())
    }

    pub(crate) fn set_periodic_reporting(&self, enabled: bool) -> Result<(), BusError> {
        let mut desired = self.periodic_reporting.lock();
        if *desired == enabled {
            return Ok(());
        }
        *desired = enabled;
        *self.monitoring_request.lock() = None;
        self.bus.enqueue_configuration_batch(vec![
            active_report_frame(self.bus.spec.host_id, self.spec.address.id, enabled).into(),
        ])?;
        *self.monitoring_request.lock() =
            Some((self.bus.status().transition_count, Instant::now()));
        Ok(())
    }

    pub(crate) fn send_control(&self, command: ImpedanceCommand) -> Result<(), BusError> {
        self.control_batcher.submit(
            &self.spec.name,
            &self.bus,
            control_frame(
                self.spec.params,
                protocol_command(self.spec.address.id, command),
            )
            .into(),
        )
    }

    pub(crate) fn control_frame(&self, command: ImpedanceCommand) -> CanFrame {
        control_frame(
            self.spec.params,
            protocol_command(self.spec.address.id, command),
        )
        .into()
    }

    pub(crate) fn safety_stop_frame(&self, clear_fault: bool) -> CanFrame {
        stop_frame(self.bus.spec.host_id, self.spec.address.id, clear_fault).into()
    }

    pub(crate) fn submit_safety_batch(
        &self,
        frames: Vec<CanFrame>,
    ) -> Result<PendingBatch, BusError> {
        self.bus.submit_safety_batch(frames)
    }

    pub(crate) fn stop_expecting_operating_feedback(&self) {
        self.feedback.write().operating = None;
    }

    pub(super) fn accepts_feedback_target(&self, target: u8) -> bool {
        target == self.bus.spec.host_id
    }

    fn mark_fault(&self, state: &mut FeedbackState, fault: MotorFault) {
        self.lifecycle.enter_fault(fault.detail.clone());
        state.current_fault = Some(fault);
    }

    pub(super) fn sync_bus(&self) {
        let bus = self.bus.status();
        if bus.state.is_online() {
            // Serialize with reporting changes; monitoring must not re-enable
            // periodic reports while a group relies on command replies.
            let periodic_reporting = self.periodic_reporting.lock();
            if !*periodic_reporting {
                return;
            }
            let feedback_is_fresh = self
                .feedback()
                .is_some_and(|feedback| feedback.received.elapsed() <= self.feedback_timeout);
            let mut monitoring_request = self.monitoring_request.lock();
            let request_generation_is_current = monitoring_request
                .is_some_and(|(generation, _)| generation == bus.transition_count);
            let request_is_current = monitoring_request.is_some_and(|(generation, requested)| {
                generation == bus.transition_count && requested.elapsed() < self.feedback_timeout
            });
            if feedback_is_fresh && request_generation_is_current {
                return;
            }
            if request_is_current {
                return;
            }
            let operating_feedback_expected = self.feedback.read().operating.is_some();
            if !operating_feedback_expected {
                self.lifecycle.await_feedback(format!(
                    "requesting active feedback on CAN bus {}",
                    self.bus.spec.name
                ));
            }
            match self.bus.enqueue_configuration_batch(vec![
                active_report_frame(self.bus.spec.host_id, self.spec.address.id, true).into(),
            ]) {
                Ok(()) => *monitoring_request = Some((bus.transition_count, Instant::now())),
                Err(error) => self.lifecycle.degrade_or_update(format!(
                    "could not request active feedback on CAN bus {}: {error}",
                    self.bus.spec.name
                )),
            }
            return;
        }
        *self.monitoring_request.lock() = None;
        let detail = format!(
            "CAN bus {} unavailable: {} ({})",
            self.bus.spec.name, bus.state, bus.detail
        );
        let mut state = self.feedback.write();
        state.current_fault = Some(MotorFault {
            cause: MotorFaultCause::Communication,
            detail: detail.clone(),
        });
        self.lifecycle.begin_reconnect(detail);
    }

    pub(super) fn check_feedback_watchdog(&self) {
        if !self.bus.status().state.is_online() {
            return;
        }
        let mut state = self.feedback.write();
        let Some(operating) = &state.operating else {
            return;
        };
        let stale_for = match state.latest {
            Some(feedback) if feedback.received >= operating.since => feedback.received.elapsed(),
            _ => operating.since.elapsed(),
        };
        if stale_for >= self.feedback_timeout {
            self.mark_fault(
                &mut state,
                MotorFault {
                    cause: MotorFaultCause::StaleFeedback,
                    detail: format!(
                        "no fresh motor feedback within {} ms",
                        self.feedback_timeout.as_millis()
                    ),
                },
            );
        }
    }

    pub(super) fn record_received_feedback(&self, value: Feedback, received: Instant) {
        let published_velocity = self.velocity_filter.lock().apply(value.velocity, received);
        {
            let mut state = self.feedback.write();
            self.update_health(&mut state, value, received);
            state.latest = Some(TimedFeedback {
                value,
                published_velocity,
                received,
            });
        }
        self.telemetry.send(TelemetryPayload::Motor(MotorTelemetry {
            group: self.group.clone(),
            motor: self.spec.name.clone(),
            position: value.position,
            velocity: published_velocity,
            torque: value.torque,
            temperature: value.temperature,
            fault_bits: value.fault_bits,
            mode_status: value.mode_status,
            feedback_age_ms: received.elapsed().as_millis().min(u64::MAX as u128) as u64,
        }));
    }

    fn update_health(&self, state: &mut FeedbackState, value: Feedback, received: Instant) {
        let thermal_fault = self.thermal_health.lock().fault_detail(
            value.temperature,
            self.spec.health.maximum_temperature_c,
            self.spec.health.temperature_recovery_c,
            Duration::from_millis(self.spec.health.temperature_recovery_duration_ms),
            received,
        );
        let operating = state
            .operating
            .as_mut()
            .filter(|operating| received >= operating.since);
        let fault = classify_feedback(value, &self.spec, operating.is_some(), thermal_fault);

        if let Some((cause, detail)) = fault {
            let fault = MotorFault { cause, detail };
            // A non-running mode can precede the first operating sample after enable.
            if operating.is_some_and(|operating| {
                cause != MotorFaultCause::ModeStatus || operating.confirmed
            }) {
                state.latched_fault.get_or_insert_with(|| fault.clone());
            }
            self.mark_fault(state, fault);
        } else {
            if let Some(operating) = operating {
                operating.confirmed = true;
            }
            state.current_fault = None;
            if value.temperature >= self.spec.health.temperature_warning_c {
                self.lifecycle.mark_online(format!(
                    "temperature warning: {:.1} C is at or above {:.1} C",
                    value.temperature, self.spec.health.temperature_warning_c
                ));
            } else {
                self.lifecycle.mark_online("healthy motor feedback");
            }
        }
    }
}

fn protocol_command(motor_id: u8, command: ImpedanceCommand) -> ControlCommand {
    ControlCommand {
        motor_id,
        position: command.position,
        velocity: command.velocity,
        torque: command.torque,
        kp: command.kp,
        kd: command.kd,
    }
}
