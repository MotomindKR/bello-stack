use std::collections::HashSet;
use std::fs;
use std::path::Path;

use anyhow::{Context, Result, bail};
use bello_sim_can::SimBus;
use robstride_protocol::{MAX_MOTOR_ID, MotorParams};
use serde::Deserialize;

#[derive(Clone, Debug, Deserialize)]
#[serde(untagged)]
pub enum CanInterface {
    Socketcan(String),
    Sim(SimBus),
}

impl CanInterface {
    pub fn validate(&self) -> Result<()> {
        match self {
            Self::Socketcan(name) if name.trim().is_empty() => bail!("interface must be non-empty"),
            Self::Socketcan(_) => Ok(()),
            Self::Sim(bus) => Ok(bus.validate()?),
        }
    }
}

impl std::fmt::Display for CanInterface {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Socketcan(name) => f.write_str(name),
            Self::Sim(bus) => write!(f, "sim/{}/{}", bus.daemon_port, bus.simulator_port),
        }
    }
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Config {
    pub interface: CanInterface,
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
#[serde(deny_unknown_fields)]
pub struct ScheduledEvent {
    pub at_ms: u64,
    pub actuator_id: u8,
    pub action: FaultAction,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case", deny_unknown_fields)]
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
        Self::parse(&text).with_context(|| format!("invalid configuration in {}", path.display()))
    }

    pub fn parse(yaml: &str) -> Result<Self> {
        let config: Self = serde_yaml::from_str(yaml)?;
        config.validate()?;
        Ok(config)
    }

    pub fn validate(&self) -> Result<()> {
        self.interface.validate()?;
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
            if actuator.id > MAX_MOTOR_ID {
                bail!(
                    "actuator id {} must be in the Robstride range 0..={MAX_MOTOR_ID}",
                    actuator.id
                );
            }
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn milly_emulators_match_the_hardware_inventory_on_each_bus() {
        #[derive(Deserialize)]
        struct Inventory {
            logical_buses: Vec<Bus>,
            motors: Vec<Motor>,
        }
        #[derive(Deserialize)]
        struct Bus {
            name: String,
            host_id: u8,
        }
        #[derive(Deserialize)]
        struct Motor {
            id: u8,
            model: String,
            direction: i8,
            bus: String,
        }

        let root = Path::new(env!("CARGO_MANIFEST_DIR"));
        for (profile, inventory) in [
            ("milly-7dof", "milly-7dof/hardware.yaml"),
            ("milly-7dof-dual", "milly-7dof/hardware-dual.yaml"),
            ("milly-7dof-gimbal", "milly-7dof-gimbal/hardware.yaml"),
            ("stereo-gimbal", "stereo-gimbal/hardware.yaml"),
            ("milly-7dof-dual-o6", "milly-7dof-o6/hardware.yaml"),
            ("milly-7dof-o6-gimbal", "milly-7dof-o6-gimbal/hardware.yaml"),
        ] {
            let path = root.join("../../descriptions").join(inventory);
            let inventory: Inventory =
                serde_yaml::from_reader(fs::File::open(path).unwrap()).unwrap();
            for bus in &inventory.logical_buses {
                let stem = if inventory.logical_buses.len() == 1 {
                    profile.to_owned()
                } else {
                    format!("{profile}-{}", bus.name)
                };
                let expected: std::collections::BTreeMap<_, _> = inventory
                    .motors
                    .iter()
                    .filter(|m| m.bus == bus.name)
                    .map(|m| (m.id, (m.model.as_str(), m.direction)))
                    .collect();
                for suffix in [".yaml", "-vcan.yaml"] {
                    let path = root.join("config").join(format!("{stem}{suffix}"));
                    let emulator = Config::load(&path).unwrap();
                    assert_eq!(emulator.host_id, bus.host_id, "{}", path.display());
                    let actual = emulator
                        .actuators
                        .iter()
                        .map(|m| (m.id, (m.model.as_str(), m.direction)))
                        .collect();
                    assert_eq!(expected, actual, "{}", path.display());
                }
            }
        }
    }

    #[test]
    fn bundled_configs_follow_the_strict_event_schema() {
        for yaml in [
            include_str!("../config/vcan.yaml"),
            include_str!("../config/faults.yaml"),
            include_str!("../config/milly-vcan.yaml"),
        ] {
            let config: Config = serde_yaml::from_str(yaml).unwrap();
            config.validate().unwrap();
        }
    }

    #[test]
    fn event_action_rejects_unknown_fields() {
        let yaml = r#"
interface: vcan0
actuators:
  - { id: 1, model: RS_03 }
events:
  - at_ms: 10
    actuator_id: 1
    action: { type: drop_feedback, enabled: true, typo: true }
"#;
        let error = serde_yaml::from_str::<Config>(yaml).unwrap_err();
        assert!(error.to_string().contains("unknown field `typo`"));
    }
}
