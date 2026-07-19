use std::collections::HashSet;
use std::fs;
use std::path::Path;

use anyhow::{Context, Result, bail};
use robstride_protocol::MotorParams;
use serde::Deserialize;

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Config {
    pub interface: String,
    #[serde(default = "default_host_id")]
    pub host_id: u8,
    #[serde(default = "default_feedback_rate")]
    pub feedback_rate_hz: f64,
    #[serde(default)]
    pub dynamics: DynamicsConfig,
    pub actuators: Vec<ActuatorConfig>,
    #[serde(default)]
    pub events: Vec<ScheduledEvent>,
}

fn default_host_id() -> u8 {
    0xfd
}

fn default_feedback_rate() -> f64 {
    500.0
}

#[derive(Clone, Debug, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct DynamicsConfig {
    pub position_response_hz: f64,
    pub idle_velocity_decay_hz: f64,
}

impl Default for DynamicsConfig {
    fn default() -> Self {
        Self {
            position_response_hz: 20.0,
            idle_velocity_decay_hz: 20.0,
        }
    }
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ActuatorConfig {
    pub id: u8,
    pub model: String,
    #[serde(default = "default_direction")]
    pub direction: i8,
    #[serde(default)]
    pub initial_position: f64,
    #[serde(default = "default_temperature")]
    pub initial_temperature_c: f64,
}

fn default_direction() -> i8 {
    1
}

fn default_temperature() -> f64 {
    25.0
}

#[derive(Clone, Debug, Deserialize)]
pub struct ScheduledEvent {
    pub at_ms: u64,
    pub actuator_id: u8,
    #[serde(flatten)]
    pub action: FaultAction,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(tag = "action", rename_all = "snake_case")]
pub enum FaultAction {
    DropFeedback { enabled: bool },
    SetFeedbackDelay { delay_ms: u64 },
    SetFeedbackJitter { jitter_ms: u64 },
    SetFaultBits { value: u16 },
    SetTemperature { celsius: f64 },
    ClearTemperature,
    SetModeStatus { value: u8 },
    ClearModeStatus,
}

impl Config {
    pub fn load(path: impl AsRef<Path>) -> Result<Self> {
        let path = path.as_ref();
        let text = fs::read_to_string(path)
            .with_context(|| format!("failed to read {}", path.display()))?;
        let config: Self = serde_yaml::from_str(&text)
            .with_context(|| format!("invalid YAML in {}", path.display()))?;
        config.validate()?;
        Ok(config)
    }

    pub fn validate(&self) -> Result<()> {
        if self.interface.trim().is_empty() {
            bail!("interface must be non-empty");
        }
        if !self.feedback_rate_hz.is_finite() || !(1.0..=2_000.0).contains(&self.feedback_rate_hz) {
            bail!("feedback_rate_hz must be in 1..=2000");
        }
        if !self.dynamics.position_response_hz.is_finite()
            || self.dynamics.position_response_hz <= 0.0
            || !self.dynamics.idle_velocity_decay_hz.is_finite()
            || self.dynamics.idle_velocity_decay_hz <= 0.0
        {
            bail!("dynamics rates must be finite and positive");
        }
        if self.actuators.is_empty() {
            bail!("at least one actuator is required");
        }
        let mut ids = HashSet::new();
        for actuator in &self.actuators {
            if !ids.insert(actuator.id) {
                bail!("duplicate actuator id {}", actuator.id);
            }
            if !matches!(actuator.direction, -1 | 1) {
                bail!("actuator {} direction must be -1 or 1", actuator.id);
            }
            if actuator.id == self.host_id {
                bail!("actuator {} conflicts with host_id", actuator.id);
            }
            let Some(params) = MotorParams::for_model(&actuator.model, actuator.direction) else {
                bail!(
                    "actuator {} has unsupported model {}",
                    actuator.id,
                    actuator.model
                );
            };
            if !actuator.initial_position.is_finite()
                || !(params.position_min..=params.position_max).contains(&actuator.initial_position)
            {
                bail!("actuator {} initial_position is invalid", actuator.id);
            }
            if !actuator.initial_temperature_c.is_finite() || actuator.initial_temperature_c < 0.0 {
                bail!("actuator {} initial_temperature_c is invalid", actuator.id);
            }
        }
        for event in &self.events {
            if !ids.contains(&event.actuator_id) {
                bail!(
                    "event at {} ms references unknown actuator {}",
                    event.at_ms,
                    event.actuator_id
                );
            }
            match &event.action {
                FaultAction::SetFeedbackDelay { delay_ms }
                | FaultAction::SetFeedbackJitter {
                    jitter_ms: delay_ms,
                } if *delay_ms > 60_000 => {
                    bail!("feedback delay and jitter must not exceed 60000 ms");
                }
                FaultAction::SetFaultBits { value } if *value > 0x3f => {
                    bail!("fault bits must fit the Robstride 6-bit feedback field");
                }
                FaultAction::SetTemperature { celsius }
                    if !celsius.is_finite() || *celsius < 0.0 =>
                {
                    bail!("event temperature must be finite and non-negative");
                }
                FaultAction::SetModeStatus { value } if *value > 3 => {
                    bail!("mode status must fit the Robstride 2-bit feedback field");
                }
                _ => {}
            }
        }
        Ok(())
    }
}
