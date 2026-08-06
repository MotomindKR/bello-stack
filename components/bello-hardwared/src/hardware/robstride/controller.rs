use std::collections::{BTreeMap, HashSet};
use std::sync::Arc;
use std::time::{Duration, Instant};

use anyhow::Result;
use parking_lot::{Mutex, RwLock};
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
use crate::hardware::can::{BusError, BusHandle, CanFrame, PendingBatch};
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
        // Group control invokes each motor handle once per tick. Hold those frames
        // until the tick is complete, then enqueue one atomic queue item per bus.
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
        // Attempt every bus before reporting failure. This cannot make separate
        // CAN interfaces physically atomic, but it keeps a failed interface
        // from suppressing the same logical tick on every later interface.
        let mut first_error = None;
        for (bus, frames) in batches.into_values() {
            if let Err(error) = bus.try_enqueue_control_batch(frames, expires_at) {
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

/// Timestamped protocol feedback owned by the motor device layer.
#[derive(Clone, Copy, Debug)]
pub(crate) struct TimedFeedback {
    pub(crate) value: Feedback,
    pub(crate) published_velocity: f64,
    pub(crate) received: Instant,
}

pub(super) struct FeedbackProcessingConfig {
    pub(super) timeout: Duration,
    pub(super) velocity_filter: MotorVelocityFilterConfig,
}

/// One Robstride device bound to one raw CAN transport.
///
/// This is the protocol and device-lifecycle boundary. Group control uses this
/// type rather than constructing frames or inspecting CAN transport internals.
#[derive(Clone)]
pub(crate) struct MotorController {
    pub(crate) spec: MotorSpec,
    pub(crate) group: GroupName,
    bus: BusHandle,
    control_batcher: Arc<ControlBatcher>,
    feedback: Arc<RwLock<Option<TimedFeedback>>>,
    velocity_filter: Arc<Mutex<MotorVelocityFilter>>,
    feedback_expected_since: Arc<RwLock<Option<Instant>>>,
    monitoring_request: Arc<Mutex<Option<(u64, Instant)>>>,
    thermal_health: Arc<Mutex<ThermalHealth>>,
    fault: Arc<RwLock<Option<MotorFault>>>,
    feedback_timeout: Duration,
    lifecycle: DeviceLifecycle,
    telemetry: Telemetry,
}

impl MotorController {
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
            feedback: Arc::new(RwLock::new(None)),
            velocity_filter: Arc::new(Mutex::new(MotorVelocityFilter::new(
                feedback_processing.velocity_filter.cutoff_hz,
                feedback_processing.timeout,
            ))),
            feedback_expected_since: Arc::new(RwLock::new(None)),
            monitoring_request: Arc::new(Mutex::new(None)),
            thermal_health: Arc::new(Mutex::new(ThermalHealth::default())),
            fault: Arc::new(RwLock::new(None)),
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
        *self.feedback.read()
    }

    pub(crate) fn fault(&self) -> Option<MotorFault> {
        self.fault.read().clone()
    }

    pub(crate) fn feedback_ready(&self) -> bool {
        let Some(expected_since) = *self.feedback_expected_since.read() else {
            return false;
        };
        self.lifecycle.state().is_online()
            && self.feedback.read().is_some_and(|feedback| {
                feedback.received >= expected_since
                    && feedback.received.elapsed() <= self.feedback_timeout
            })
    }

    pub(crate) fn recovery_ready(&self) -> bool {
        self.bus.status().state.is_online()
            && self.lifecycle.state().is_online()
            && self.fault.read().is_none()
            && self
                .feedback
                .read()
                .is_some_and(|feedback| feedback.received.elapsed() <= self.feedback_timeout)
    }

    pub(crate) fn configure_enable(&self) -> Result<(), BusError> {
        *self.feedback.write() = None;
        self.velocity_filter.lock().reset();
        *self.feedback_expected_since.write() = None;
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
        self.prepare_feedback_wait();
        *self.feedback_expected_since.write() = Some(Instant::now());
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

    pub(crate) fn safety_control_frame(&self, command: ImpedanceCommand) -> CanFrame {
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

    pub(crate) fn mark_stopped(&self) {
        *self.feedback_expected_since.write() = None;
    }

    pub(super) fn accepts_feedback_target(&self, target: u8) -> bool {
        target == self.bus.spec.host_id
    }

    fn mark_fault(&self, cause: MotorFaultCause, detail: impl Into<String>) {
        let detail = detail.into();
        *self.fault.write() = Some(MotorFault {
            cause,
            detail: detail.clone(),
        });
        self.lifecycle.enter_fault(detail);
    }

    fn prepare_feedback_wait(&self) {
        self.lifecycle
            .await_feedback("motor enabled; waiting for fresh operating feedback");
    }

    pub(super) fn sync_bus(&self) {
        let bus = self.bus.status();
        if bus.state.is_online() {
            let feedback_is_fresh = self
                .feedback
                .read()
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
            let operating_feedback_expected = self.feedback_expected_since.read().is_some();
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
        *self.fault.write() = Some(MotorFault {
            cause: MotorFaultCause::Communication,
            detail: detail.clone(),
        });
        self.lifecycle.begin_reconnect(detail);
    }

    pub(super) fn check_feedback_watchdog(&self) {
        if !self.bus.status().state.is_online() {
            return;
        }
        let Some(expected_since) = *self.feedback_expected_since.read() else {
            return;
        };
        let feedback = *self.feedback.read();
        let stale_for = match feedback {
            Some(feedback) if feedback.received >= expected_since => feedback.received.elapsed(),
            _ => expected_since.elapsed(),
        };
        if stale_for >= self.feedback_timeout {
            self.mark_fault(
                MotorFaultCause::StaleFeedback,
                format!(
                    "no fresh motor feedback within {} ms",
                    self.feedback_timeout.as_millis()
                ),
            );
        }
    }

    pub(super) fn record_received_feedback(&self, value: Feedback, received: Instant) {
        let published_velocity = self.velocity_filter.lock().apply(value.velocity, received);
        *self.feedback.write() = Some(TimedFeedback {
            value,
            published_velocity,
            received,
        });
        self.update_health(value, received);
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

    fn update_health(&self, value: Feedback, received: Instant) {
        let thermal_fault = self.thermal_health.lock().fault_detail(
            value.temperature,
            self.spec.health.maximum_temperature_c,
            self.spec.health.temperature_recovery_c,
            Duration::from_millis(self.spec.health.temperature_recovery_duration_ms),
            received,
        );
        let expecting_operating_feedback = self
            .feedback_expected_since
            .read()
            .is_some_and(|expected| received >= expected);
        let fault = classify_feedback(
            value,
            &self.spec,
            expecting_operating_feedback,
            thermal_fault,
        );

        if let Some((cause, detail)) = fault {
            self.mark_fault(cause, detail);
        } else {
            let warning =
                (value.temperature >= self.spec.health.temperature_warning_c).then(|| {
                    format!(
                        "temperature warning: {:.1} C is at or above {:.1} C",
                        value.temperature, self.spec.health.temperature_warning_c
                    )
                });
            self.mark_online(warning.as_deref());
        }
    }

    fn mark_online(&self, warning: Option<&str>) {
        *self.fault.write() = None;
        let detail = warning.unwrap_or("healthy motor feedback");
        self.lifecycle.mark_online(detail);
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
