use std::fs;
use std::path::Path;

use anyhow::{Context, Result};

use super::{Config, ResolvedTopology};

impl Config {
    pub(crate) fn load_raw(path: impl AsRef<Path>) -> Result<Self> {
        let path = path.as_ref();
        let text = fs::read_to_string(path)
            .with_context(|| format!("failed to read {}", path.display()))?;
        let config: Self = serde_yaml::from_str(&text)
            .with_context(|| format!("invalid YAML in {}", path.display()))?;
        Ok(config)
    }

    /// Runs the same side-effect-free resolution pass used by daemon startup.
    pub fn validate(&self) -> Result<()> {
        ResolvedTopology::resolve(self.clone()).map(drop)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn parse(yaml: &str) -> Result<Config> {
        let config: Config = serde_yaml::from_str(yaml)?;
        config.validate()?;
        Ok(config)
    }

    const PREFIX: &str = r#"
buses:
  - { name: shared, interface: vcan0, maximum_utilization: 1.0 }
motors:
  - { name: motor_a, joint: joint_a, bus: shared, id: 1, model: ROBSTRIDE_03 }
  - { name: motor_b, joint: joint_b, bus: shared, id: 2, model: ROBSTRIDE_03 }
"#;

    #[test]
    fn groups_can_share_a_bus() {
        let text = format!(
            "{PREFIX}\nmotor_groups:\n  - {{ name: group_a, motors: [motor_a], control_rate_hz: 100.0 }}\n  - {{ name: group_b, motors: [motor_b], control_rate_hz: 100.0 }}\n"
        );
        assert!(parse(&text).is_ok());
    }

    #[test]
    fn motor_cannot_be_in_two_groups() {
        let text = format!(
            "{PREFIX}\nmotor_groups:\n  - {{ name: a, motors: [motor_a] }}\n  - {{ name: b, motors: [motor_a, motor_b] }}\n"
        );
        assert!(
            parse(&text)
                .unwrap_err()
                .to_string()
                .contains("belongs to both")
        );
    }

    #[test]
    fn imu_must_reference_a_known_serial_device() {
        let text = format!(
            "{PREFIX}\nimus:\n  - {{ name: imu_a, serial_device: missing }}\nmotor_groups:\n  - {{ name: group, motors: [motor_a, motor_b] }}\n"
        );
        assert!(
            parse(&text)
                .unwrap_err()
                .to_string()
                .contains("unknown serial device")
        );
    }

    #[test]
    fn enabled_sensors_cannot_share_an_exclusive_serial_device() {
        let text = format!(
            "serial_devices:\n  - {{ name: serial_a, port: /dev/null, baud_rate: 115200 }}\n{PREFIX}\nimus:\n  - {{ name: imu_a, serial_device: serial_a }}\n  - {{ name: imu_b, serial_device: serial_a }}\nmotor_groups:\n  - {{ name: group, motors: [motor_a, motor_b] }}\n"
        );
        assert!(
            parse(&text)
                .unwrap_err()
                .to_string()
                .contains("more than one enabled sensor")
        );
    }
}
