use std::collections::{HashMap, HashSet};
use std::fs;
use std::path::Path;

use anyhow::{Context, Result, bail};

#[cfg(test)]
use super::ResolvedTopology;
use super::{Config, DeploymentConfig, HardwareDescription};
use crate::control::GroupSpec;
use crate::hardware::can::BusSpec;
use crate::hardware::robstride::MotorSpec;

impl Config {
    pub(crate) fn load_raw(path: impl AsRef<Path>) -> Result<Self> {
        let path = path.as_ref();
        let deployment: DeploymentConfig = load_yaml(path)?;
        let description_path = if deployment.description.is_absolute() {
            deployment.description.clone()
        } else {
            path.parent()
                .unwrap_or_else(|| Path::new("."))
                .join(&deployment.description)
        };
        let description: HardwareDescription = load_yaml(&description_path)?;
        Self::compose(description, deployment).with_context(|| {
            format!(
                "failed to compose {} with {}",
                path.display(),
                description_path.display()
            )
        })
    }

    fn compose(description: HardwareDescription, deployment: DeploymentConfig) -> Result<Self> {
        if description.logical_buses.is_empty() {
            bail!("hardware description must declare at least one logical bus");
        }
        let mut logical_buses = HashSet::new();
        for bus in &description.logical_buses {
            if bus.name.trim().is_empty() || !logical_buses.insert(bus.name.clone()) {
                bail!(
                    "hardware description contains an empty or duplicate logical bus {:?}",
                    bus.name
                );
            }
            BusSpec::try_from(&super::BusConfig {
                name: bus.name.clone(),
                interface: format!("description-{}", bus.name),
                host_id: bus.host_id,
                bitrate: bus.bitrate,
                maximum_utilization: bus.maximum_utilization,
                reconnect: bus.reconnect.clone(),
            })?;
        }

        let mut motors = HashMap::new();
        let mut coordinates = HashSet::new();
        let mut addresses = HashSet::new();
        for motor in &description.motors {
            MotorSpec::try_from(motor)?;
            if !logical_buses.contains(&motor.bus) {
                bail!(
                    "motor {} refers to undeclared logical bus {}",
                    motor.name,
                    motor.bus
                );
            }
            if !coordinates.insert(motor.coordinate.as_str()) {
                bail!(
                    "coordinate {} is assigned to more than one motor",
                    motor.coordinate
                );
            }
            if !addresses.insert((motor.bus.as_str(), motor.id)) {
                bail!("duplicate motor address {}:{}", motor.bus, motor.id);
            }
            if motors.insert(motor.name.as_str(), motor).is_some() {
                bail!("duplicate motor name {}", motor.name);
            }
        }
        if motors.is_empty() {
            bail!("hardware description must declare at least one motor");
        }

        let mut groups = HashMap::new();
        let mut membership = HashMap::new();
        for group in &description.motor_groups {
            GroupSpec::try_from(group)?;
            if groups.insert(group.name.as_str(), group).is_some() {
                bail!("duplicate motor group name {}", group.name);
            }
            for motor in &group.motors {
                if !motors.contains_key(motor.as_str()) {
                    bail!("group {} refers to unknown motor {motor}", group.name);
                }
                if let Some(owner) = membership.insert(motor.as_str(), group.name.as_str()) {
                    bail!("motor {motor} belongs to both {owner} and {}", group.name);
                }
            }
        }
        if groups.is_empty() {
            bail!("hardware description must declare at least one motor group");
        }
        if let Some(unowned) = motors.keys().find(|name| !membership.contains_key(**name)) {
            bail!("motor {unowned} does not belong to a group");
        }

        if deployment.enabled_groups.is_empty() {
            bail!("enabled_groups must not be empty");
        }
        let mut enabled = HashSet::new();
        for group in &deployment.enabled_groups {
            if !enabled.insert(group.as_str()) {
                bail!("enabled_groups contains duplicate group {group}");
            }
            if !groups.contains_key(group.as_str()) {
                bail!("enabled_groups refers to unknown group {group}");
            }
        }

        let motor_groups: Vec<_> = description
            .motor_groups
            .into_iter()
            .filter(|group| enabled.contains(group.name.as_str()))
            .collect();
        let selected_motors: HashSet<_> = motor_groups
            .iter()
            .flat_map(|group| group.motors.iter().map(String::as_str))
            .collect();
        let motors: Vec<_> = description
            .motors
            .into_iter()
            .filter(|motor| selected_motors.contains(motor.name.as_str()))
            .collect();

        let configured_buses: HashSet<_> = deployment.bus_interfaces.keys().cloned().collect();
        for bus in &configured_buses {
            if !logical_buses.contains(bus) {
                bail!("deployment maps unknown logical bus {bus}");
            }
        }
        let required_buses: HashSet<String> = motors
            .iter()
            .map(|motor: &super::MotorConfig| motor.bus.clone())
            .collect();
        if configured_buses != required_buses {
            let missing: Vec<_> = required_buses.difference(&configured_buses).collect();
            let unused: Vec<_> = configured_buses.difference(&required_buses).collect();
            bail!(
                "deployment logical bus mappings do not match enabled groups (missing: {missing:?}, unused: {unused:?})"
            );
        }
        let buses = description
            .logical_buses
            .into_iter()
            .filter(|bus| required_buses.contains(&bus.name))
            .map(|bus| super::BusConfig {
                interface: deployment.bus_interfaces[&bus.name].clone(),
                name: bus.name,
                host_id: bus.host_id,
                bitrate: bus.bitrate,
                maximum_utilization: bus.maximum_utilization,
                reconnect: bus.reconnect,
            })
            .collect();

        let described_serial: HashSet<String> = description
            .serial_devices
            .iter()
            .map(|serial| serial.name.clone())
            .collect();
        if described_serial.len() != description.serial_devices.len()
            || described_serial.iter().any(|name| name.trim().is_empty())
        {
            bail!("hardware description contains an empty or duplicate serial device name");
        }
        let configured_serial: HashSet<_> = deployment.serial_ports.keys().cloned().collect();
        if configured_serial != described_serial {
            let missing: Vec<_> = described_serial.difference(&configured_serial).collect();
            let unused: Vec<_> = configured_serial.difference(&described_serial).collect();
            bail!(
                "deployment serial port mappings do not match the hardware description (missing: {missing:?}, unused: {unused:?})"
            );
        }
        let serial_devices = description
            .serial_devices
            .into_iter()
            .map(|serial| super::SerialDeviceConfig {
                port: deployment.serial_ports[&serial.name].clone(),
                name: serial.name,
                baud_rate: serial.baud_rate,
                timeout_ms: serial.timeout_ms,
                reconnect: serial.reconnect,
            })
            .collect();

        Ok(Self {
            runtime: deployment.runtime,
            grpc: deployment.grpc,
            serial_devices,
            imus: description.imus,
            buses,
            motors,
            motor_groups,
        })
    }
}

fn load_yaml<T>(path: &Path) -> Result<T>
where
    T: serde::de::DeserializeOwned,
{
    let text =
        fs::read_to_string(path).with_context(|| format!("failed to read {}", path.display()))?;
    serde_yaml::from_str(&text).with_context(|| format!("invalid YAML in {}", path.display()))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn parse(yaml: &str) -> Result<Config> {
        let config: Config = serde_yaml::from_str(yaml)?;
        ResolvedTopology::resolve(config.clone())?;
        Ok(config)
    }

    #[test]
    fn bundled_deployments_compose_the_canonical_description() {
        let component = Path::new(env!("CARGO_MANIFEST_DIR"));
        let physical = Config::load_raw(component.join("config/bello.yaml")).unwrap();
        let vcan = Config::load_raw(component.join("config/bello-vcan.yaml")).unwrap();

        assert_eq!(physical.motors.len(), 27);
        assert_eq!(physical.motor_groups.len(), 3);
        assert_eq!(vcan.motors.len(), 25);
        assert_eq!(vcan.motor_groups.len(), 2);
        assert!(
            physical
                .motors
                .iter()
                .find(|motor| motor.name == "waist_yaw")
                .is_some_and(|motor| motor.bus == "can1")
        );
        assert!(
            vcan.motors
                .iter()
                .all(|motor| !matches!(motor.name.as_str(), "left_arm_7" | "right_arm_7"))
        );
        for name in ["can0", "can1"] {
            let physical_bus = physical.buses.iter().find(|bus| bus.name == name).unwrap();
            let vcan_bus = vcan.buses.iter().find(|bus| bus.name == name).unwrap();
            assert_eq!(physical_bus.host_id, vcan_bus.host_id);
            assert_eq!(physical_bus.bitrate, vcan_bus.bitrate);
            assert_eq!(
                physical_bus.maximum_utilization,
                vcan_bus.maximum_utilization
            );
            assert_ne!(physical_bus.interface, vcan_bus.interface);
        }
        assert_eq!(
            physical.serial_devices[0].baud_rate,
            vcan.serial_devices[0].baud_rate
        );
        assert_ne!(physical.serial_devices[0].port, vcan.serial_devices[0].port);
    }

    #[test]
    fn deployment_group_selection_is_strict() {
        let component = Path::new(env!("CARGO_MANIFEST_DIR"));
        let description: HardwareDescription =
            load_yaml(&component.join("../../descriptions/bello/hardware.yaml")).unwrap();
        let mut deployment: DeploymentConfig =
            load_yaml(&component.join("config/bello-vcan.yaml")).unwrap();

        deployment.enabled_groups.push("upper_body".into());
        assert!(
            Config::compose(description.clone(), deployment)
                .unwrap_err()
                .to_string()
                .contains("duplicate group")
        );

        let mut deployment: DeploymentConfig =
            load_yaml(&component.join("config/bello-vcan.yaml")).unwrap();
        deployment.enabled_groups.push("missing".into());
        assert!(
            Config::compose(description, deployment)
                .unwrap_err()
                .to_string()
                .contains("unknown group")
        );
    }

    #[test]
    fn disabled_groups_are_still_validated() {
        let component = Path::new(env!("CARGO_MANIFEST_DIR"));
        let mut description: HardwareDescription =
            load_yaml(&component.join("../../descriptions/bello/hardware.yaml")).unwrap();
        description
            .motors
            .iter_mut()
            .find(|motor| motor.name == "left_arm_7")
            .unwrap()
            .model = "UNKNOWN".into();
        let deployment: DeploymentConfig =
            load_yaml(&component.join("config/bello-vcan.yaml")).unwrap();

        assert!(
            Config::compose(description, deployment)
                .unwrap_err()
                .to_string()
                .contains("unsupported Robstride model")
        );
    }

    #[test]
    fn deployment_transport_bindings_must_be_exact() {
        let component = Path::new(env!("CARGO_MANIFEST_DIR"));
        let description: HardwareDescription =
            load_yaml(&component.join("../../descriptions/bello/hardware.yaml")).unwrap();
        let mut missing_bus: DeploymentConfig =
            load_yaml(&component.join("config/bello-vcan.yaml")).unwrap();
        missing_bus.bus_interfaces.remove("can1");
        assert!(
            Config::compose(description.clone(), missing_bus)
                .unwrap_err()
                .to_string()
                .contains("missing")
        );

        let mut extra_serial: DeploymentConfig =
            load_yaml(&component.join("config/bello-vcan.yaml")).unwrap();
        extra_serial
            .serial_ports
            .insert("unknown".into(), "/dev/null".into());
        assert!(
            Config::compose(description, extra_serial)
                .unwrap_err()
                .to_string()
                .contains("unused")
        );
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
