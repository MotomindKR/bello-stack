use std::fs;
use std::path::Path;

use anyhow::{Context, Result};

use super::Config;
#[cfg(test)]
use super::ResolvedTopology;

impl Config {
    pub(crate) fn load_raw(path: impl AsRef<Path>) -> Result<Self> {
        let path = path.as_ref();
        let text = fs::read_to_string(path)
            .with_context(|| format!("failed to read {}", path.display()))?;
        let config: Self = serde_yaml::from_str(&text)
            .with_context(|| format!("invalid YAML in {}", path.display()))?;
        Ok(config)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn parse(yaml: &str) -> Result<Config> {
        let config: Config = serde_yaml::from_str(yaml)?;
        ResolvedTopology::resolve(config.clone())?;
        Ok(config)
    }

    const PREFIX: &str = r#"
buses:
  - { name: shared, interface: vcan0, maximum_utilization: 1.0 }
motors:
  - { name: motor_a, coordinate: coordinate_a, bus: shared, id: 1, model: ROBSTRIDE_03 }
  - { name: motor_b, coordinate: coordinate_b, bus: shared, id: 2, model: ROBSTRIDE_03 }
"#;

    fn one_group(prefix: &str) -> String {
        format!("{prefix}\nmotor_groups:\n  - {{ name: group, motors: [motor_a, motor_b] }}\n")
    }

    #[test]
    fn unix_listener_requires_an_absolute_path_and_octal_mode() {
        let relative = one_group(&format!(
            "{PREFIX}\ngrpc:\n  listen: {{ type: unix, path: relative.sock }}"
        ));
        assert!(parse(&relative).is_err());

        let bad_mode = one_group(&format!(
            "{PREFIX}\ngrpc:\n  listen: {{ type: unix, path: /tmp/hardwared.sock, mode: \"0680\" }}"
        ));
        assert!(parse(&bad_mode).is_err());
    }

    #[test]
    fn unix_listener_rejects_tls() {
        let source = one_group(&format!(
            "{PREFIX}\ngrpc:\n  listen: {{ type: unix, path: /tmp/hardwared.sock }}\n  tls:\n    certificate: missing-cert.pem\n    private_key: missing-key.pem"
        ));
        let error = parse(&source).unwrap_err().to_string();
        assert!(error.contains("cannot be combined"), "{error}");
    }

    #[test]
    fn groups_can_share_a_bus() {
        let text = format!(
            "{PREFIX}\nmotor_groups:\n  - {{ name: group_a, motors: [motor_a], control_rate_hz: 100.0 }}\n  - {{ name: group_b, motors: [motor_b], control_rate_hz: 100.0 }}\n"
        );
        assert!(parse(&text).is_ok());
    }

    #[test]
    fn rejects_control_load_above_the_configured_bus_budget() {
        let text = r#"
buses:
  - { name: slow, interface: vcan0, bitrate: 10000, maximum_utilization: 1.0 }
motors:
  - { name: motor_a, coordinate: coordinate_a, bus: slow, id: 1, model: ROBSTRIDE_03 }
  - { name: motor_b, coordinate: coordinate_b, bus: slow, id: 2, model: ROBSTRIDE_03 }
motor_groups:
  - { name: group_a, motors: [motor_a], control_rate_hz: 20.0 }
  - { name: group_b, motors: [motor_b], control_rate_hz: 20.0 }
"#;
        let error = parse(text).unwrap_err().to_string();
        assert!(error.contains("estimated utilization"), "{error}");
    }

    #[test]
    fn rejects_control_batch_without_freshness_headroom() {
        let text = r#"
buses:
  - { name: saturated, interface: vcan0, bitrate: 10000, maximum_utilization: 1.0 }
motors:
  - { name: motor_a, coordinate: coordinate_a, bus: saturated, id: 1, model: ROBSTRIDE_03 }
motor_groups:
  - { name: group, motors: [motor_a], control_rate_hz: 33.333333333333336 }
"#;
        let error = parse(text).unwrap_err().to_string();
        assert!(error.contains("freshness period"), "{error}");
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
    fn sensors_cannot_share_an_exclusive_serial_device() {
        let text = format!(
            "serial_devices:\n  - {{ name: serial_a, port: /dev/null, baud_rate: 115200 }}\n{PREFIX}\nimus:\n  - {{ name: imu_a, serial_device: serial_a }}\n  - {{ name: imu_b, serial_device: serial_a }}\nmotor_groups:\n  - {{ name: group, motors: [motor_a, motor_b] }}\n"
        );
        assert!(
            parse(&text)
                .unwrap_err()
                .to_string()
                .contains("more than one sensor")
        );
    }

    #[test]
    fn physical_can_interface_cannot_be_declared_twice() {
        let text = r#"
buses:
  - { name: first, interface: can0 }
  - { name: second, interface: can0 }
motors:
  - { name: motor_a, coordinate: coordinate_a, bus: first, id: 1, model: ROBSTRIDE_03 }
  - { name: motor_b, coordinate: coordinate_b, bus: second, id: 1, model: ROBSTRIDE_03 }
motor_groups:
  - { name: group, motors: [motor_a, motor_b] }
"#;
        let error = parse(text).unwrap_err().to_string();
        assert!(
            error.contains("CAN interface can0 is configured more than once"),
            "{error}"
        );
    }

    #[test]
    fn physical_serial_port_cannot_be_declared_twice() {
        let text = format!(
            "serial_devices:\n  - {{ name: first, port: /dev/ttyUSB0, baud_rate: 115200 }}\n  - {{ name: second, port: /dev/ttyUSB0, baud_rate: 115200 }}\n{PREFIX}\nimus:\n  - {{ name: imu_a, serial_device: first }}\n  - {{ name: imu_b, serial_device: second }}\nmotor_groups:\n  - {{ name: group, motors: [motor_a, motor_b] }}\n"
        );
        let error = parse(&text).unwrap_err().to_string();
        assert!(
            error.contains("serial port /dev/ttyUSB0 is configured more than once"),
            "{error}"
        );
    }

    #[test]
    fn every_can_bus_must_own_a_motor() {
        let text = r#"
buses:
  - { name: used, interface: can0 }
  - { name: unused, interface: can1 }
motors:
  - { name: motor_a, coordinate: coordinate_a, bus: used, id: 1, model: ROBSTRIDE_03 }
motor_groups:
  - { name: group, motors: [motor_a] }
"#;
        let error = parse(text).unwrap_err().to_string();
        assert!(error.contains("CAN bus unused has no motors"), "{error}");
    }

    #[test]
    fn every_serial_device_must_own_a_sensor() {
        let text = format!(
            "serial_devices:\n  - {{ name: used, port: /dev/ttyUSB0, baud_rate: 115200 }}\n  - {{ name: unused, port: /dev/ttyUSB1, baud_rate: 115200 }}\n{PREFIX}\nimus:\n  - {{ name: imu, serial_device: used }}\nmotor_groups:\n  - {{ name: group, motors: [motor_a, motor_b] }}\n"
        );
        let error = parse(&text).unwrap_err().to_string();
        assert!(
            error.contains("serial device unused is not assigned to a sensor"),
            "{error}"
        );
    }
}
