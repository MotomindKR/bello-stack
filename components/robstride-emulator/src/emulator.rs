use std::collections::{HashMap, VecDeque};
use std::time::Duration;

use anyhow::{Result, anyhow};
use robstride_protocol::{
    CommType, ControlCommand, Feedback, Frame, MotorParams, Request, feedback_frame, parse_control,
    parse_request, target_id,
};

use crate::config::{Config, DynamicsConfig, FaultAction, ScheduledEvent};

#[derive(Clone, Copy, Debug)]
pub struct ScheduledFrame {
    pub delay: Duration,
    pub frame: Frame,
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
    active_reporting: bool,
    position: f64,
    velocity: f64,
    torque: f64,
    temperature: f64,
    command: Option<ControlCommand>,
    next_feedback: Duration,
    overrides: Overrides,
}

impl Actuator {
    fn step(&mut self, elapsed: Duration, dynamics: &DynamicsConfig) {
        let seconds = elapsed.as_secs_f64().min(0.1);
        if self.enabled {
            if let Some(command) = self.command {
                let error = command.position - self.position;
                let desired_velocity = (error * dynamics.position_response_hz + command.velocity)
                    .clamp(self.params.velocity_min, self.params.velocity_max);
                self.velocity = desired_velocity;
                self.position = (self.position + self.velocity * seconds)
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
            mode_status: self
                .overrides
                .mode_status
                .unwrap_or(if self.enabled { 2 } else { 0 }),
            fault_bits: self.overrides.fault_bits,
            position: self.position,
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
    actuators: HashMap<u8, Actuator>,
    events: VecDeque<ScheduledEvent>,
    random_state: u64,
}

impl Emulator {
    pub fn new(config: Config) -> Result<Self> {
        config.validate()?;
        let mut actuators = HashMap::new();
        for config in config.actuators {
            let params = MotorParams::for_model(&config.model, config.direction)
                .ok_or_else(|| anyhow!("unsupported Robstride model {}", config.model))?;
            actuators.insert(
                config.id,
                Actuator {
                    id: config.id,
                    params,
                    enabled: false,
                    active_reporting: false,
                    position: config.initial_position,
                    velocity: 0.0,
                    torque: 0.0,
                    temperature: config.initial_temperature_c,
                    command: None,
                    next_feedback: Duration::ZERO,
                    overrides: Overrides::default(),
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
        })
    }

    pub fn handle(&mut self, frame: Frame) {
        let motor_id = target_id(frame.id);
        let Some(actuator) = self.actuators.get_mut(&motor_id) else {
            return;
        };
        if CommType::from_id(frame.id) == Some(CommType::MotorControl) {
            if actuator.enabled
                && let Some(command) = parse_control(&frame, actuator.params)
            {
                actuator.command = Some(command);
            }
            return;
        }
        match parse_request(&frame) {
            Some(Request::Enable { host_id, .. }) => {
                self.host_id = host_id;
                actuator.enabled = true;
            }
            Some(Request::Stop {
                host_id,
                clear_fault,
                ..
            }) => {
                self.host_id = host_id;
                actuator.enabled = false;
                actuator.command = None;
                if clear_fault {
                    actuator.overrides.fault_bits = 0;
                }
            }
            Some(Request::ActiveReport {
                host_id, enabled, ..
            }) => {
                self.host_id = host_id;
                actuator.active_reporting = enabled;
                actuator.next_feedback = self.elapsed;
            }
            Some(Request::ParamWriteU8 { host_id, .. }) => {
                self.host_id = host_id;
            }
            None => {}
        }
    }

    pub fn advance(&mut self, elapsed: Duration) -> Vec<ScheduledFrame> {
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
        for actuator in self.actuators.values_mut() {
            actuator.step(elapsed, &self.dynamics);
        }

        let mut due = Vec::new();
        let mut reports = Vec::new();
        for actuator in self.actuators.values_mut() {
            if !actuator.active_reporting {
                continue;
            }
            while actuator.next_feedback <= self.elapsed {
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
        ControlCommand, active_report_frame, control_frame, enable_frame, parse_feedback,
    };

    use super::*;
    use crate::config::{ActuatorConfig, DynamicsConfig};

    fn config(events: Vec<ScheduledEvent>) -> Config {
        Config {
            interface: "vcan0".into(),
            host_id: 0xfd,
            feedback_rate_hz: 100.0,
            dynamics: DynamicsConfig::default(),
            actuators: vec![ActuatorConfig {
                id: 1,
                model: "ROBSTRIDE_03".into(),
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
        let params = MotorParams::for_model("ROBSTRIDE_03", 1).unwrap();
        emulator.handle(active_report_frame(0xfd, 1, true));
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
        let params = MotorParams::for_model("ROBSTRIDE_03", 1).unwrap();
        emulator.handle(active_report_frame(0xfd, 1, true));
        emulator.handle(enable_frame(0xfd, 1));
        assert!(emulator.advance(Duration::from_millis(10)).is_empty());
        let frames = emulator.advance(Duration::from_millis(10));
        assert!(!frames.is_empty());
        assert_eq!(
            parse_feedback(&frames[0].frame, params).unwrap().fault_bits,
            3
        );
    }
}
