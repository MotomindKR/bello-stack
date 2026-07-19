use std::sync::Arc;
use std::time::{Duration, Instant};

use anyhow::Result;
use parking_lot::{Mutex, RwLock};
use robstride_protocol::{
    ControlCommand, active_report_frame, control_frame, enable_frame, param_write_u8_frame,
    stop_frame,
};

use super::{Feedback, MotorSpec};
use crate::domain::{
    DeviceState, DeviceStatus, GroupName, ImpedanceCommand, MotorTelemetry, TelemetryEvent,
    TelemetryPayload,
};
use crate::hardware::can::{BusError, BusHandle};
use crate::hardware::lifecycle::DeviceLifecycle;
use crate::telemetry::Telemetry;

const RUN_MODE_INDEX: u16 = 0x7005;
const IMPEDANCE_RUN_MODE: u8 = 0;
const RUNNING_MODE_STATUS: u8 = 2;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum MotorFaultCause {
    Communication,
    StaleFeedback,
    SafePosition,
    Temperature,
    ReportedFault,
    ModeStatus,
}

#[derive(Clone, Debug)]
pub(crate) struct MotorFault {
    pub(crate) cause: MotorFaultCause,
    pub(crate) detail: String,
}

#[derive(Default)]
struct ThermalHealth {
    over_temperature: bool,
    recovery_started: Option<Instant>,
}

impl ThermalHealth {
    fn fault_detail(
        &mut self,
        temperature: f64,
        maximum: f64,
        recovery: f64,
        recovery_duration: Duration,
        received: Instant,
    ) -> Option<String> {
        if !self.over_temperature && temperature >= maximum {
            self.over_temperature = true;
            self.recovery_started = None;
        }
        if !self.over_temperature {
            return None;
        }

        if temperature <= recovery {
            let recovery_started = *self.recovery_started.get_or_insert(received);
            if received.saturating_duration_since(recovery_started) >= recovery_duration {
                self.over_temperature = false;
                self.recovery_started = None;
                return None;
            }
        } else {
            self.recovery_started = None;
        }

        Some(format!(
            "motor over-temperature latched; cool to <= {recovery:.1} C for {} ms",
            recovery_duration.as_millis()
        ))
    }
}

/// Timestamped protocol feedback owned by the motor device layer.
#[derive(Clone, Copy, Debug)]
pub(crate) struct TimedFeedback {
    pub(crate) value: Feedback,
    pub(crate) received: Instant,
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
    feedback: Arc<RwLock<Option<TimedFeedback>>>,
    feedback_expected_since: Arc<RwLock<Option<Instant>>>,
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
        feedback_timeout: Duration,
        lifecycle: DeviceLifecycle,
        telemetry: Telemetry,
    ) -> Self {
        Self {
            spec,
            group,
            bus,
            feedback: Arc::new(RwLock::new(None)),
            feedback_expected_since: Arc::new(RwLock::new(None)),
            thermal_health: Arc::new(Mutex::new(ThermalHealth::default())),
            fault: Arc::new(RwLock::new(None)),
            feedback_timeout,
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
        *self.feedback_expected_since.write() = None;
        self.bus.enqueue_safety(
            param_write_u8_frame(
                self.bus.spec.host_id,
                self.spec.address.id,
                RUN_MODE_INDEX,
                IMPEDANCE_RUN_MODE,
            )
            .into(),
        )?;
        self.bus.enqueue_safety(
            active_report_frame(self.bus.spec.host_id, self.spec.address.id, true).into(),
        )?;
        self.bus
            .enqueue_safety(enable_frame(self.bus.spec.host_id, self.spec.address.id).into())?;
        self.prepare_feedback_wait();
        *self.feedback_expected_since.write() = Some(Instant::now());
        Ok(())
    }

    pub(crate) fn send_control(&self, command: ImpedanceCommand) -> Result<(), BusError> {
        self.bus.try_enqueue(
            control_frame(
                self.spec.params,
                protocol_command(self.spec.address.id, command),
            )
            .into(),
        )
    }

    pub(crate) fn send_safety_command(&self, command: ImpedanceCommand) -> Result<(), BusError> {
        self.bus.enqueue_safety(
            control_frame(
                self.spec.params,
                protocol_command(self.spec.address.id, command),
            )
            .into(),
        )
    }

    pub(crate) fn stop(&self, clear_fault: bool) -> Result<(), BusError> {
        self.bus.enqueue_safety(
            stop_frame(self.bus.spec.host_id, self.spec.address.id, clear_fault).into(),
        )?;
        *self.feedback_expected_since.write() = None;
        Ok(())
    }

    fn mark_fault(&self, cause: MotorFaultCause, detail: impl Into<String>) {
        let detail = detail.into();
        *self.fault.write() = Some(MotorFault {
            cause,
            detail: detail.clone(),
        });
        match self.lifecycle.state() {
            DeviceState::Online => {
                let _ = self
                    .lifecycle
                    .transition(DeviceState::Degraded, detail.clone());
                let _ = self.lifecycle.transition(DeviceState::Fault, detail);
            }
            DeviceState::Connecting | DeviceState::Degraded | DeviceState::Reconnecting => {
                let _ = self.lifecycle.transition(DeviceState::Fault, detail);
            }
            DeviceState::Fault if self.lifecycle.status().detail != detail => {
                let _ = self.lifecycle.transition(DeviceState::Fault, detail);
            }
            _ => {}
        }
    }

    fn prepare_feedback_wait(&self) {
        match self.lifecycle.state() {
            DeviceState::Online => {
                let _ = self.lifecycle.transition(
                    DeviceState::Degraded,
                    "motor enabled; waiting for fresh operating feedback",
                );
            }
            DeviceState::Reconnecting => {
                let _ = self.lifecycle.transition(
                    DeviceState::Connecting,
                    "motor enabled; waiting for fresh operating feedback",
                );
            }
            _ => {}
        }
    }

    pub(super) fn sync_bus(&self) {
        let bus = self.bus.status();
        if bus.state.is_online() {
            if self.lifecycle.state() == DeviceState::Reconnecting {
                let _ = self.lifecycle.transition(
                    DeviceState::Connecting,
                    format!(
                        "CAN bus {} restored; waiting for feedback",
                        self.bus.spec.name
                    ),
                );
            }
            return;
        }
        let detail = format!(
            "CAN bus {} unavailable: {} ({})",
            self.bus.spec.name, bus.state, bus.detail
        );
        *self.fault.write() = Some(MotorFault {
            cause: MotorFaultCause::Communication,
            detail: detail.clone(),
        });
        match self.lifecycle.state() {
            DeviceState::Starting => {
                let _ = self.lifecycle.transition(DeviceState::Connecting, detail);
            }
            DeviceState::Online => {
                let _ = self
                    .lifecycle
                    .transition(DeviceState::Degraded, detail.clone());
                let _ = self
                    .lifecycle
                    .transition(DeviceState::Fault, detail.clone());
                let _ = self.lifecycle.transition(DeviceState::Reconnecting, detail);
            }
            DeviceState::Degraded | DeviceState::Fault => {
                if self.lifecycle.state() == DeviceState::Degraded {
                    let _ = self
                        .lifecycle
                        .transition(DeviceState::Fault, detail.clone());
                }
                let _ = self.lifecycle.transition(DeviceState::Reconnecting, detail);
            }
            DeviceState::Connecting | DeviceState::Reconnecting => {}
            _ => {}
        }
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
        *self.feedback.write() = Some(TimedFeedback { value, received });
        self.update_health(value, received);
        self.telemetry.send(TelemetryEvent {
            monotonic_ns: 0,
            payload: TelemetryPayload::Motor(MotorTelemetry {
                group: self.group.clone(),
                motor: self.spec.name.clone(),
                address: self.spec.address.clone(),
                status: self.lifecycle.status(),
                position: value.position,
                velocity: value.velocity,
                torque: value.torque,
                temperature: value.temperature,
                fault_bits: value.fault_bits,
                mode_status: value.mode_status,
                feedback_age_ms: received.elapsed().as_millis().min(u64::MAX as u128) as u64,
            }),
        });
    }

    fn update_health(&self, value: Feedback, received: Instant) {
        let thermal_fault = self.thermal_health.lock().fault_detail(
            value.temperature,
            self.spec.health.maximum_temperature_c,
            self.spec.health.temperature_recovery_c,
            Duration::from_millis(self.spec.health.temperature_recovery_duration_ms),
            received,
        );
        let fault = if value.fault_bits != 0 {
            Some((
                MotorFaultCause::ReportedFault,
                format!("motor reported fault bits 0x{:04x}", value.fault_bits),
            ))
        } else if let Some(detail) = thermal_fault {
            Some((MotorFaultCause::Temperature, detail))
        } else if !(self.spec.health.safe_position_min..=self.spec.health.safe_position_max)
            .contains(&value.position)
        {
            Some((
                MotorFaultCause::SafePosition,
                format!(
                    "feedback position {:.4} outside configured safe range [{:.4}, {:.4}]",
                    value.position,
                    self.spec.health.safe_position_min,
                    self.spec.health.safe_position_max
                ),
            ))
        } else if self
            .feedback_expected_since
            .read()
            .is_some_and(|expected| received >= expected)
            && value.mode_status != RUNNING_MODE_STATUS
        {
            Some((
                MotorFaultCause::ModeStatus,
                format!(
                    "motor reported mode status {} while operating status {} was expected",
                    value.mode_status, RUNNING_MODE_STATUS
                ),
            ))
        } else {
            None
        };

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
        match self.lifecycle.state() {
            DeviceState::Starting => {
                let _ = self
                    .lifecycle
                    .transition(DeviceState::Connecting, "motor feedback detected");
                let _ = self.lifecycle.transition(DeviceState::Online, detail);
            }
            DeviceState::Connecting | DeviceState::Degraded => {
                let _ = self.lifecycle.transition(DeviceState::Online, detail);
            }
            DeviceState::Fault => {
                let _ = self
                    .lifecycle
                    .transition(DeviceState::Reconnecting, "motor feedback resumed");
                let _ = self
                    .lifecycle
                    .transition(DeviceState::Connecting, "validating motor feedback");
                let _ = self.lifecycle.transition(DeviceState::Online, detail);
            }
            DeviceState::Reconnecting => {
                let _ = self
                    .lifecycle
                    .transition(DeviceState::Connecting, "motor feedback resumed");
                let _ = self.lifecycle.transition(DeviceState::Online, detail);
            }
            DeviceState::Online if self.lifecycle.status().detail != detail => {
                let _ = self.lifecycle.transition(DeviceState::Online, detail);
            }
            _ => {}
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn thermal_fault_requires_hysteresis_and_continuous_cooldown() {
        let mut health = ThermalHealth::default();
        let started = Instant::now();
        let cooldown = Duration::from_secs(2);

        assert!(
            health
                .fault_detail(80.0, 80.0, 75.0, cooldown, started)
                .is_some()
        );
        assert!(
            health
                .fault_detail(
                    74.0,
                    80.0,
                    75.0,
                    cooldown,
                    started + Duration::from_millis(100),
                )
                .is_some()
        );
        assert!(
            health
                .fault_detail(
                    76.0,
                    80.0,
                    75.0,
                    cooldown,
                    started + Duration::from_millis(1_500),
                )
                .is_some()
        );
        assert!(
            health
                .fault_detail(
                    74.0,
                    80.0,
                    75.0,
                    cooldown,
                    started + Duration::from_millis(1_600),
                )
                .is_some()
        );
        assert!(
            health
                .fault_detail(
                    74.0,
                    80.0,
                    75.0,
                    cooldown,
                    started + Duration::from_millis(3_600),
                )
                .is_none()
        );
    }
}
