use std::collections::{BTreeMap, VecDeque};
use std::time::Duration;

use anyhow::{Result, anyhow, ensure};
use robstride_protocol::{
    ControlCommand, Feedback, Frame, MotorParams, Request, feedback_frame, frame_target,
    parse_control, parse_request,
};

use crate::config::{Config, DynamicsConfig, FaultAction, ScheduledEvent};
use crate::maintenance::Maintenance;

const RUN_MODE_INDEX: u16 = 0x7005;
const IMPEDANCE_RUN_MODE: u8 = 0;
const RUNNING_MODE_STATUS: u8 = 2;
const ENABLED_NOT_RUNNING_MODE_STATUS: u8 = 1;
const MAX_FEEDBACK_FRAMES_PER_ADVANCE: usize = 8_192;

#[derive(Clone, Copy, Debug)]
pub struct ScheduledFrame {
    pub delay: Duration,
    pub frame: Frame,
}

#[derive(Clone, Copy, Debug)]
pub struct MotorState {
    /// Physical position in the same coordinate system as the initial position.
    pub position: f64,
    pub velocity: f64,
    pub torque: f64,
}

#[derive(Clone, Debug)]
struct Overrides {
    drop_feedback: bool,
    feedback_delay: Duration,
    feedback_jitter: Duration,
    fault_bits: u16,
    temperature: Option<f64>,
    mode_status: Option<u8>,
}

impl Default for Overrides {
    fn default() -> Self {
        Self {
            drop_feedback: false,
            feedback_delay: Duration::ZERO,
            feedback_jitter: Duration::ZERO,
            fault_bits: 0,
            temperature: None,
            mode_status: None,
        }
    }
}

#[derive(Clone, Debug)]
struct Actuator {
    id: u8,
    params: MotorParams,
    enabled: bool,
    impedance_mode: bool,
    active_reporting: bool,
    physical_position: f64,
    zero_position: f64,
    velocity: f64,
    torque: f64,
    temperature: f64,
    command: Option<ControlCommand>,
    next_feedback: Duration,
    overrides: Overrides,
    maintenance: Maintenance,
}

impl Actuator {
    fn step(&mut self, elapsed: Duration, dynamics: &DynamicsConfig) {
        let seconds = elapsed.as_secs_f64().min(0.1);
        if self.enabled {
            if let Some(command) = self.command {
                let error = command.position - self.physical_position;
                let desired_velocity = (error * dynamics.position_response_hz + command.velocity)
                    .clamp(self.params.velocity_min, self.params.velocity_max);
                self.velocity = desired_velocity;
                self.physical_position = (self.physical_position + self.velocity * seconds)
                    .clamp(self.params.position_min, self.params.position_max);
                self.torque = (command.torque + command.kp * error
                    - command.kd * (self.velocity - command.velocity))
                    .clamp(self.params.torque_min, self.params.torque_max);
            }
        } else {
            let decay = (-dynamics.idle_velocity_decay_hz * seconds).exp();
            self.velocity *= decay;
            self.torque = 0.0;
        }
    }

    fn feedback(&self) -> Feedback {
        Feedback {
            motor_id: self.id,
            mode_status: self.overrides.mode_status.unwrap_or(
                if self.enabled && self.impedance_mode {
                    RUNNING_MODE_STATUS
                } else if self.enabled {
                    ENABLED_NOT_RUNNING_MODE_STATUS
                } else {
                    0
                },
            ),
            fault_bits: self.overrides.fault_bits,
            position: self.physical_position - self.zero_position,
            velocity: self.velocity,
            torque: self.torque,
            temperature: self.overrides.temperature.unwrap_or(self.temperature),
        }
    }

    fn apply(&mut self, action: FaultAction) {
        match action {
            FaultAction::DropFeedback { enabled } => self.overrides.drop_feedback = enabled,
            FaultAction::SetFeedbackDelay { delay_ms } => {
                self.overrides.feedback_delay = Duration::from_millis(delay_ms);
            }
            FaultAction::SetFeedbackJitter { jitter_ms } => {
                self.overrides.feedback_jitter = Duration::from_millis(jitter_ms);
            }
            FaultAction::SetFaultBits { value } => self.overrides.fault_bits = value,
            FaultAction::SetTemperature { celsius } => {
                self.overrides.temperature = Some(celsius);
            }
            FaultAction::ClearTemperature => self.overrides.temperature = None,
            FaultAction::SetModeStatus { value } => self.overrides.mode_status = Some(value),
            FaultAction::ClearModeStatus => self.overrides.mode_status = None,
        }
    }
}

pub struct Emulator {
    host_id: u8,
    feedback_period: Duration,
    dynamics: DynamicsConfig,
    elapsed: Duration,
    actuators: BTreeMap<u8, Actuator>,
    events: VecDeque<ScheduledEvent>,
    random_state: u64,
    status_reports: Vec<u8>,
    maintenance_replies: Vec<ScheduledFrame>,
}

impl Emulator {
    /// Clear active control without rewinding transport time or fault schedules.
    pub fn disarm(&mut self) {
        self.status_reports.clear();
        for actuator in self.actuators.values_mut() {
            actuator.enabled = false;
            actuator.command = None;
            actuator.torque = 0.0;
        }
    }

    pub fn new(config: Config) -> Result<Self> {
        config.validate()?;
        let interface = config.interface.to_string();
        let mut actuators = BTreeMap::new();
        for config in config.actuators {
            let params = MotorParams::for_model(&config.model, config.direction)
                .ok_or_else(|| anyhow!("unsupported Robstride model {}", config.model))?;
            let maintenance = Maintenance::new(&config.model, &interface, config.id)
                .ok_or_else(|| anyhow!("missing firmware family for {}", config.model))?;
            actuators.insert(
                config.id,
                Actuator {
                    id: config.id,
                    params,
                    enabled: false,
                    impedance_mode: false,
                    active_reporting: false,
                    physical_position: config.initial_position,
                    zero_position: 0.0,
                    velocity: 0.0,
                    torque: 0.0,
                    temperature: config.initial_temperature_c,
                    command: None,
                    next_feedback: Duration::ZERO,
                    overrides: Overrides::default(),
                    maintenance,
                },
            );
        }
        let mut events = config.events;
        events.sort_by_key(|event| event.at_ms);
        Ok(Self {
            host_id: config.host_id,
            feedback_period: Duration::from_secs_f64(1.0 / config.feedback_rate_hz),
            dynamics: config.dynamics,
            elapsed: Duration::ZERO,
            actuators,
            events: events.into(),
            random_state: 0x4d595df4d0f33173,
            status_reports: Vec::new(),
            maintenance_replies: Vec::new(),
        })
    }

    pub fn handle(&mut self, frame: Frame) {
        let motor_id = frame_target(&frame);
        let field = (frame.id >> 8) as u16;
        let Some(actuator) = self.actuators.get_mut(&motor_id) else {
            return;
        };
        if let Some(mut command) = parse_control(&frame, actuator.params) {
            if actuator.enabled && actuator.impedance_mode {
                command.position += actuator.zero_position;
                actuator.command = Some(command);
                self.status_reports.push(motor_id);
            }
            return;
        }
        if let Some(reply) =
            actuator
                .maintenance
                .reply(self.host_id, motor_id, !actuator.enabled, frame)
        {
            self.maintenance_replies.push(ScheduledFrame {
                delay: Duration::ZERO,
                frame: reply,
            });
            return;
        }
        if frame.id >> 24 == 6
            && field == u16::from(self.host_id)
            && frame.data == [1, 0, 0, 0, 0, 0, 0, 0]
            && !actuator.enabled
            && !actuator.maintenance.updating()
        {
            actuator.zero_position = actuator.physical_position;
            self.status_reports.push(motor_id);
            return;
        }
        match parse_request(&frame) {
            Some(Request::Enable { host_id, .. })
                if host_id == self.host_id && !actuator.maintenance.updating() =>
            {
                actuator.enabled = true;
            }
            Some(Request::Stop { clear_fault, .. }) if field == u16::from(self.host_id) => {
                actuator.enabled = false;
                actuator.command = None;
                if clear_fault {
                    actuator.overrides.fault_bits = 0;
                }
                self.status_reports.push(motor_id);
            }
            Some(Request::ActiveReport {
                host_id, enabled, ..
            }) if host_id == self.host_id => {
                actuator.active_reporting = enabled;
                actuator.next_feedback = self.elapsed;
            }
            Some(Request::ParamWriteU8 {
                host_id,
                index: RUN_MODE_INDEX,
                value,
                ..
            }) if host_id == self.host_id => {
                actuator.impedance_mode = value == IMPEDANCE_RUN_MODE;
                if !actuator.impedance_mode {
                    actuator.command = None;
                }
            }
            _ => {}
        }
    }

    pub fn advance(&mut self, elapsed: Duration) -> Vec<ScheduledFrame> {
        for actuator in self.actuators.values_mut() {
            actuator.step(elapsed, &self.dynamics);
        }
        self.advance_feedback(elapsed)
    }

    /// Return the active command in the physical coordinate system of `MotorState`.
    pub fn command(&self, motor_id: u8) -> Result<Option<ControlCommand>> {
        let actuator = self
            .actuators
            .get(&motor_id)
            .ok_or_else(|| anyhow!("unknown motor {motor_id}"))?;
        Ok(if actuator.enabled && actuator.impedance_mode {
            actuator.command
        } else {
            None
        })
    }

    pub fn set_motor_state(&mut self, motor_id: u8, state: MotorState) -> Result<()> {
        ensure!(
            [state.position, state.velocity, state.torque]
                .iter()
                .all(|v| v.is_finite()),
            "nonfinite motor feedback"
        );
        let actuator = self
            .actuators
            .get_mut(&motor_id)
            .ok_or_else(|| anyhow!("unknown motor {motor_id}"))?;
        actuator.physical_position = state.position;
        actuator.velocity = state.velocity;
        actuator.torque = state.torque;
        Ok(())
    }

    /// Schedule protocol feedback after an external physics engine updates motor state.
    /// Does not advance the simplified first-order actuator dynamics.
    pub fn advance_feedback(&mut self, elapsed: Duration) -> Vec<ScheduledFrame> {
        self.elapsed += elapsed;
        while self
            .events
            .front()
            .is_some_and(|event| Duration::from_millis(event.at_ms) <= self.elapsed)
        {
            let event = self.events.pop_front().expect("front checked");
            tracing::info!(
                elapsed_ms = self.elapsed.as_millis(),
                actuator_id = event.actuator_id,
                action = ?event.action,
                "applying emulator event"
            );
            if let Some(actuator) = self.actuators.get_mut(&event.actuator_id) {
                actuator.apply(event.action);
            }
        }
        let mut due = std::mem::take(&mut self.maintenance_replies);
        let mut reports = Vec::new();
        for motor_id in std::mem::take(&mut self.status_reports) {
            if let Some(actuator) = self.actuators.get(&motor_id)
                && !actuator.overrides.drop_feedback
            {
                reports.push((
                    actuator.feedback(),
                    actuator.params,
                    actuator.overrides.feedback_delay,
                    actuator.overrides.feedback_jitter,
                ));
            }
        }
        let mut feedback_steps = 0_usize;
        for actuator in self.actuators.values_mut() {
            if !actuator.active_reporting {
                continue;
            }
            while actuator.next_feedback <= self.elapsed
                && feedback_steps < MAX_FEEDBACK_FRAMES_PER_ADVANCE
            {
                feedback_steps += 1;
                if !actuator.overrides.drop_feedback {
                    reports.push((
                        actuator.feedback(),
                        actuator.params,
                        actuator.overrides.feedback_delay,
                        actuator.overrides.feedback_jitter,
                    ));
                }
                actuator.next_feedback += self.feedback_period;
            }
            if actuator.next_feedback <= self.elapsed {
                tracing::warn!(
                    actuator_id = actuator.id,
                    limit = MAX_FEEDBACK_FRAMES_PER_ADVANCE,
                    "feedback catch-up limit reached; skipping stale reports"
                );
                actuator.next_feedback = self.elapsed + self.feedback_period;
            }
        }
        for (feedback, params, delay, jitter) in reports {
            let jitter = self.random_delay(jitter);
            due.push(ScheduledFrame {
                delay: delay + jitter,
                frame: feedback_frame(self.host_id, feedback, params),
            });
        }
        due
    }

    fn random_delay(&mut self, maximum: Duration) -> Duration {
        if maximum.is_zero() {
            return Duration::ZERO;
        }
        self.random_state ^= self.random_state << 13;
        self.random_state ^= self.random_state >> 7;
        self.random_state ^= self.random_state << 17;
        let maximum_ns = maximum.as_nanos().min(u64::MAX as u128) as u64;
        Duration::from_nanos(self.random_state % maximum_ns.saturating_add(1))
    }
}

#[cfg(test)]
mod tests {
    use robstride_protocol::{
        ControlCommand, active_report_frame, control_frame, enable_frame, param_write_u8_frame,
        parse_feedback,
    };

    use super::*;
    use crate::config::{ActuatorConfig, DynamicsConfig};

    fn config(events: Vec<ScheduledEvent>) -> Config {
        Config {
            interface: crate::config::CanInterface::Socketcan("vcan0".into()),
            host_id: 0xfd,
            feedback_rate_hz: 100.0,
            dynamics: DynamicsConfig::default(),
            actuators: vec![ActuatorConfig {
                id: 1,
                model: "RS_03".into(),
                direction: 1,
                initial_position: 0.0,
                initial_temperature_c: 25.0,
            }],
            events,
        }
    }

    #[test]
    fn enabled_actuator_tracks_commands_and_reports_feedback() {
        let mut emulator = Emulator::new(config(Vec::new())).unwrap();
        let params = MotorParams::for_model("RS_03", 1).unwrap();
        emulator.handle(active_report_frame(0xfd, 1, true));
        emulator.handle(param_write_u8_frame(
            0xfd,
            1,
            RUN_MODE_INDEX,
            IMPEDANCE_RUN_MODE,
        ));
        emulator.handle(enable_frame(0xfd, 1));
        emulator.handle(control_frame(
            params,
            ControlCommand {
                motor_id: 1,
                position: 1.0,
                kp: 20.0,
                kd: 1.0,
                ..Default::default()
            },
        ));
        let frames = emulator.advance(Duration::from_millis(20));
        let feedback = parse_feedback(&frames.last().unwrap().frame, params).unwrap();
        assert_eq!(feedback.mode_status, 2);
        assert!(feedback.position > 0.0);
    }

    #[test]
    fn scripted_feedback_loss_and_fault_bits_change_wire_behavior() {
        let events = vec![
            ScheduledEvent {
                at_ms: 10,
                actuator_id: 1,
                action: FaultAction::DropFeedback { enabled: true },
            },
            ScheduledEvent {
                at_ms: 20,
                actuator_id: 1,
                action: FaultAction::DropFeedback { enabled: false },
            },
            ScheduledEvent {
                at_ms: 20,
                actuator_id: 1,
                action: FaultAction::SetFaultBits { value: 3 },
            },
        ];
        let mut emulator = Emulator::new(config(events)).unwrap();
        let params = MotorParams::for_model("RS_03", 1).unwrap();
        emulator.handle(active_report_frame(0xfd, 1, true));
        emulator.handle(param_write_u8_frame(
            0xfd,
            1,
            RUN_MODE_INDEX,
            IMPEDANCE_RUN_MODE,
        ));
        emulator.handle(enable_frame(0xfd, 1));
        assert!(emulator.advance(Duration::from_millis(10)).is_empty());
        let frames = emulator.advance(Duration::from_millis(10));
        assert!(!frames.is_empty());
        assert_eq!(
            parse_feedback(&frames[0].frame, params).unwrap().fault_bits,
            3
        );
    }

    #[test]
    fn requests_for_another_host_are_ignored() {
        let mut emulator = Emulator::new(config(Vec::new())).unwrap();
        emulator.handle(enable_frame(0xfc, 1));
        emulator.handle(active_report_frame(0xfc, 1, true));
        assert!(emulator.advance(Duration::from_millis(20)).is_empty());

        emulator.handle(active_report_frame(0xfd, 1, true));
        emulator.handle(param_write_u8_frame(
            0xfd,
            1,
            RUN_MODE_INDEX,
            IMPEDANCE_RUN_MODE,
        ));
        emulator.handle(enable_frame(0xfd, 1));
        assert!(!emulator.advance(Duration::from_millis(20)).is_empty());
    }

    #[test]
    fn enabled_actuator_only_reports_running_after_impedance_mode_is_selected() {
        let mut emulator = Emulator::new(config(Vec::new())).unwrap();
        let params = MotorParams::for_model("RS_03", 1).unwrap();
        emulator.handle(active_report_frame(0xfd, 1, true));
        emulator.handle(param_write_u8_frame(0xfd, 1, RUN_MODE_INDEX, 1));
        emulator.handle(enable_frame(0xfd, 1));
        let feedback = parse_feedback(
            &emulator.advance(Duration::from_millis(10))[0].frame,
            params,
        )
        .unwrap();
        assert_eq!(feedback.mode_status, ENABLED_NOT_RUNNING_MODE_STATUS);

        emulator.handle(param_write_u8_frame(
            0xfd,
            1,
            RUN_MODE_INDEX,
            IMPEDANCE_RUN_MODE,
        ));
        let feedback = parse_feedback(
            &emulator.advance(Duration::from_millis(10))[0].frame,
            params,
        )
        .unwrap();
        assert_eq!(feedback.mode_status, RUNNING_MODE_STATUS);
    }

    #[test]
    fn command_replies_continue_with_periodic_reporting_disabled() {
        let mut emulator = Emulator::new(config(Vec::new())).unwrap();
        let params = MotorParams::for_model("RS_03", 1).unwrap();
        emulator.handle(param_write_u8_frame(
            0xfd,
            1,
            RUN_MODE_INDEX,
            IMPEDANCE_RUN_MODE,
        ));
        emulator.handle(enable_frame(0xfd, 1));
        emulator.handle(active_report_frame(0xfd, 1, false));
        assert!(emulator.advance(Duration::from_millis(20)).is_empty());
        emulator.handle(control_frame(
            params,
            ControlCommand {
                motor_id: 1,
                position: 0.0,
                kp: 20.0,
                kd: 1.0,
                ..Default::default()
            },
        ));
        let frames = emulator.advance(Duration::from_millis(1));
        assert_eq!(frames.len(), 1);
        assert_eq!(
            parse_feedback(&frames[0].frame, params)
                .unwrap()
                .mode_status,
            2
        );
        assert!(emulator.advance(Duration::from_millis(20)).is_empty());
        emulator.handle(active_report_frame(0xfd, 1, true));
        assert!(!emulator.advance(Duration::from_millis(10)).is_empty());
    }

    #[test]
    fn a_large_time_jump_has_bounded_feedback_work() {
        let mut emulator = Emulator::new(config(Vec::new())).unwrap();
        emulator.handle(active_report_frame(0xfd, 1, true));
        let frames = emulator.advance(Duration::from_secs(1_000));
        assert_eq!(frames.len(), MAX_FEEDBACK_FRAMES_PER_ADVANCE);
    }
}
