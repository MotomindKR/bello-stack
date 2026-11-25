use std::collections::HashSet;
use std::fs;
use std::path::Path;

use anyhow::{Context, Result, bail};

#[cfg(test)]
use super::ResolvedTopology;
use super::{Config, DeploymentConfig, HardwareDescription};
use crate::hardware::can::BusSpec;

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
        Self::compose(description, deployment, &description_path).with_context(|| {
            format!(
                "failed to compose {} with {}",
                path.display(),
                description_path.display()
            )
        })
    }

    fn compose(
        mut description: HardwareDescription,
        deployment: DeploymentConfig,
        description_path: &Path,
    ) -> Result<Self> {
        let robot_description = if description.robot_description.is_absolute() {
            description.robot_description.clone()
        } else {
            description_path
                .parent()
                .unwrap_or_else(|| Path::new("."))
                .join(&description.robot_description)
        };
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
                transport: super::BusTransportConfig::Socketcan {
                    interface: format!("description-{}", bus.name),
                },
                host_id: bus.host_id,
                bitrate: bus.bitrate,
                maximum_utilization: bus.maximum_utilization,
                reconnect: bus.reconnect.clone(),
            })?;
        }

        for motor in &description.motors {
            if !logical_buses.contains(&motor.bus) {
                bail!(
                    "motor {} refers to undeclared logical bus {}",
                    motor.name,
                    motor.bus
                );
            }
        }

        let mut disabled_groups: HashSet<_> = description
            .motor_groups
            .iter()
            .map(|group| group.name.clone())
            .collect();
        if deployment.enabled_groups.is_empty() {
            bail!("enabled_groups must not be empty");
        }
        let mut enabled = HashSet::new();
        for group in &deployment.enabled_groups {
            if !enabled.insert(group.as_str()) {
                bail!("enabled_groups contains duplicate group {group}");
            }
            if !disabled_groups.remove(group) {
                bail!("enabled_groups refers to unknown group {group}");
            }
        }

        for bus in deployment.bus_transports.keys() {
            if !logical_buses.contains(bus) {
                bail!("deployment maps unknown logical bus {bus}");
            }
        }
        let buses = description
            .logical_buses
            .into_iter()
            .filter(|bus| deployment.bus_transports.contains_key(&bus.name))
            .map(|bus| super::BusConfig {
                transport: deployment.bus_transports[&bus.name].clone(),
                name: bus.name,
                host_id: bus.host_id,
                bitrate: bus.bitrate,
                maximum_utilization: bus.maximum_utilization,
                reconnect: bus.reconnect,
            })
            .collect();

        let mut board_names = HashSet::new();
        let mut claimed_serial: HashSet<_> = description
            .imus
            .iter()
            .map(|imu| imu.serial_device.clone())
            .collect();
        for board in &description.power_boards {
            crate::hardware::power_board::PowerBoardSpec::try_from(board)?;
            if !board_names.insert(board.name.clone())
                || !claimed_serial.insert(board.serial_device.clone())
            {
                bail!("duplicate power board name or serial device ownership");
            }
            if !description
                .serial_devices
                .iter()
                .any(|serial| serial.name == board.serial_device)
            {
                bail!(
                    "power board {} references unknown serial device",
                    board.name
                );
            }
        }
        let enabled_boards: HashSet<_> = deployment.enabled_power_boards.iter().cloned().collect();
        if enabled_boards.len() != deployment.enabled_power_boards.len()
            || !enabled_boards.is_subset(&board_names)
        {
            bail!("enabled_power_boards contains duplicate or unknown board names");
        }
        let disabled_serial: HashSet<_> = description
            .power_boards
            .iter()
            .filter(|b| !enabled_boards.contains(&b.name))
            .map(|b| b.serial_device.clone())
            .collect();
        description
            .power_boards
            .retain(|b| enabled_boards.contains(&b.name));
        description
            .serial_devices
            .retain(|s| !disabled_serial.contains(&s.name));
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
            robot_description,
            disabled_groups,
            simulation: deployment.simulation,
            runtime: deployment.runtime,
            grpc: deployment.grpc,
            serial_devices,
            imus: description.imus,
            power_boards: description.power_boards,
            buses,
            motors: description.motors,
            motor_groups: description.motor_groups,
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
        let mut config: Config = serde_yaml::from_str(yaml)?;
        config.robot_description = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../bello-hardware-e2e/tests/fixtures/vcan.urdf");
        ResolvedTopology::resolve(config.clone())?;
        Ok(config)
    }

    #[test]
    fn gsusb_transport_resolves_and_one_adapter_cannot_own_two_buses() {
        let component = Path::new(env!("CARGO_MANIFEST_DIR"));
        let mut config = Config::load_raw(component.join("config/bello-vcan.yaml")).unwrap();
        config.buses[0].transport = crate::config::BusTransportConfig::Gsusb {
            serial: "ADAPTER-A".into(),
            channel: 0,
        };
        assert!(ResolvedTopology::resolve(config.clone()).is_ok());
        config.buses[1].transport = crate::config::BusTransportConfig::Gsusb {
            serial: "ADAPTER-A".into(),
            channel: 1,
        };
        let error = ResolvedTopology::resolve(config).err().unwrap().to_string();
        assert!(
            error.contains("assigned to more than one CAN bus"),
            "{error}"
        );
    }

    #[test]
    fn sim_buses_cannot_share_daemon_or_simulator_ports() {
        use super::super::BusTransportConfig;
        use bello_sim_can::SimBus;

        let component = Path::new(env!("CARGO_MANIFEST_DIR"));
        let mut config = Config::load_raw(component.join("config/bello-vcan.yaml")).unwrap();
        config.buses[0].transport = BusTransportConfig::Sim(SimBus {
            daemon_port: 24000,
            simulator_port: 24001,
        });
        for (daemon_port, simulator_port) in [(24000, 24002), (24002, 24001), (24001, 24000)] {
            config.buses[1].transport = BusTransportConfig::Sim(SimBus {
                daemon_port,
                simulator_port,
            });
            let error = ResolvedTopology::resolve(config.clone()).err().unwrap();
            assert!(
                error
                    .to_string()
                    .contains("assigned to more than one CAN bus"),
                "{error}"
            );
        }
        config.buses[1].transport = BusTransportConfig::Sim(SimBus {
            daemon_port: 24002,
            simulator_port: 24003,
        });
        ResolvedTopology::resolve(config)
            .unwrap()
            .validate_platform()
            .unwrap();
    }

    #[test]
    fn gsusb_deployment_mapping_deserializes() {
        let deployment: DeploymentConfig = serde_yaml::from_str(
            r#"
description: hardware.yaml
enabled_groups: [group]
bus_transports:
  can0: { type: gsusb, serial: ADAPTER-A, channel: 1 }
"#,
        )
        .unwrap();
        assert_eq!(
            deployment.bus_transports["can0"],
            crate::config::BusTransportConfig::Gsusb {
                serial: "ADAPTER-A".into(),
                channel: 1,
            }
        );
    }

    #[test]
    fn deployment_group_selection_is_strict() {
        let component = Path::new(env!("CARGO_MANIFEST_DIR"));
        let description_path = component.join("../../descriptions/bello/hardware.yaml");
        let description: HardwareDescription = load_yaml(&description_path).unwrap();
        let mut deployment: DeploymentConfig =
            load_yaml(&component.join("config/bello-vcan.yaml")).unwrap();

        let existing_group = deployment.enabled_groups[0].clone();
        deployment.enabled_groups.push(existing_group);
        assert!(
            Config::compose(description.clone(), deployment, &description_path)
                .unwrap_err()
                .to_string()
                .contains("duplicate group")
        );

        let mut deployment: DeploymentConfig =
            load_yaml(&component.join("config/bello-vcan.yaml")).unwrap();
        deployment.enabled_groups.push("missing".into());
        assert!(
            Config::compose(description, deployment, &description_path)
                .unwrap_err()
                .to_string()
                .contains("unknown group")
        );
    }

    #[test]
    fn disabled_groups_are_still_validated() {
        let component = Path::new(env!("CARGO_MANIFEST_DIR"));
        let description_path = component.join("../../descriptions/bello/hardware.yaml");
        let mut description: HardwareDescription = load_yaml(&description_path).unwrap();
        let mut deployment: DeploymentConfig =
            load_yaml(&component.join("config/bello-vcan.yaml")).unwrap();
        let disabled = deployment.enabled_groups.pop().unwrap();
        let motor = &description
            .motor_groups
            .iter()
            .find(|group| group.name == disabled)
            .unwrap()
            .motors[0];
        description
            .motors
            .iter_mut()
            .find(|spec| spec.name == *motor)
            .unwrap()
            .model = "UNKNOWN".into();

        let config = Config::compose(description, deployment, &description_path).unwrap();
        let error = ResolvedTopology::resolve(config).err().unwrap();
        assert!(
            error.to_string().contains("unsupported Robstride model"),
            "{error}"
        );
    }

    #[test]
    fn deployment_transport_bindings_must_be_exact() {
        let component = Path::new(env!("CARGO_MANIFEST_DIR"));
        let description_path = component.join("../../descriptions/bello/hardware.yaml");
        let description: HardwareDescription = load_yaml(&description_path).unwrap();
        let mut missing_bus: DeploymentConfig =
            load_yaml(&component.join("config/bello-vcan.yaml")).unwrap();
        let bus = missing_bus.bus_transports.keys().next().unwrap().clone();
        missing_bus.bus_transports.remove(&bus);
        let config = Config::compose(description.clone(), missing_bus, &description_path).unwrap();
        let error = ResolvedTopology::resolve(config).err().unwrap();
        assert!(
            error
                .to_string()
                .contains(&format!("missing transport for CAN bus {bus}")),
            "{error}"
        );

        let mut extra_serial: DeploymentConfig =
            load_yaml(&component.join("config/bello-vcan.yaml")).unwrap();
        extra_serial
            .serial_ports
            .insert("unknown".into(), "/dev/null".into());
        assert!(
            Config::compose(description, extra_serial, &description_path)
                .unwrap_err()
                .to_string()
                .contains("unused")
        );
    }

    const PREFIX: &str = r#"
buses:
  - { name: shared, transport: { type: socketcan, interface: vcan0 }, maximum_utilization: 1.0 }
motors:
  - { name: motor_a, coordinate: e2e_joint_1, bus: shared, id: 1, model: RS_03 }
  - { name: motor_b, coordinate: e2e_joint_2, bus: shared, id: 2, model: RS_03 }
"#;

    fn one_group(prefix: &str) -> String {
        format!("{prefix}\nmotor_groups:\n  - {{ name: group, motors: [motor_a, motor_b] }}\n")
    }

    #[test]
    fn unix_listener_requires_an_absolute_path_and_octal_mode() {
        let relative = one_group(&format!(
            "{PREFIX}\ngrpc:\n  listen: [{{ type: unix, path: relative.sock }}]"
        ));
        assert!(parse(&relative).is_err());

        let bad_mode = one_group(&format!(
            "{PREFIX}\ngrpc:\n  listen: [{{ type: unix, path: /tmp/hardwared.sock, mode: \"0680\" }}]"
        ));
        assert!(parse(&bad_mode).is_err());
    }

    #[test]
    #[cfg(unix)]
    fn unix_listener_rejects_tls() {
        let source = one_group(&format!(
            "{PREFIX}\ngrpc:\n  listen: [{{ type: unix, path: /tmp/hardwared.sock }}]\n  tls:\n    certificate: missing-cert.pem\n    private_key: missing-key.pem"
        ));
        let error = parse(&source).unwrap_err().to_string();
        assert!(error.contains("requires a TCP listener"), "{error}");
    }

    #[test]
    fn listeners_reject_empty_and_duplicate_endpoints() {
        for listen in [
            "[]",
            "[{type: tcp, address: '127.0.0.1:50051'}, {type: tcp, address: '127.0.0.1:50051'}]",
            "[{type: unix, path: /tmp/test.sock, mode: '0600'}, {type: unix, path: /tmp/test.sock, mode: '0660'}]",
        ] {
            let source = one_group(&format!("{PREFIX}\ngrpc:\n  listen: {listen}"));
            assert!(parse(&source).is_err(), "accepted {listen}");
        }
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
  - { name: slow, transport: { type: socketcan, interface: vcan0 }, bitrate: 10000, maximum_utilization: 1.0 }
motors:
  - { name: motor_a, coordinate: e2e_joint_1, bus: slow, id: 1, model: RS_03 }
  - { name: motor_b, coordinate: e2e_joint_2, bus: slow, id: 2, model: RS_03 }
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
  - { name: saturated, transport: { type: socketcan, interface: vcan0 }, bitrate: 10000, maximum_utilization: 1.0 }
motors:
  - { name: motor_a, coordinate: e2e_joint_1, bus: saturated, id: 1, model: RS_03 }
motor_groups:
  - { name: group, motors: [motor_a], control_rate_hz: 33.333333333333336 }
"#;
        let error = parse(text).unwrap_err().to_string();
        assert!(error.contains("transmission period"), "{error}");
    }

    #[test]
    fn periodic_load_uses_tx_refresh_rate_instead_of_the_safety_tick_rate() {
        let text = r#"
buses:
  - { name: slow, transport: { type: socketcan, interface: vcan0 }, bitrate: 10000, maximum_utilization: 1.0 }
motors:
  - { name: motor_a, coordinate: e2e_joint_1, bus: slow, id: 1, model: RS_03 }
motor_groups:
  - { name: group, motors: [motor_a], control_rate_hz: 1000.0, command_refresh_rate_hz: 20.0, command_transmission: periodic }
"#;
        assert!(parse(text).is_ok());
        let too_fast = text.replace(
            "command_refresh_rate_hz: 20.0",
            "command_refresh_rate_hz: 40.0",
        );
        let error = parse(&too_fast).unwrap_err().to_string();
        assert!(error.contains("transmission period"), "{error}");
    }

    #[test]
    fn periodic_groups_still_enforce_the_shared_bus_load_limit() {
        let text = r#"
buses:
  - { name: slow, transport: { type: socketcan, interface: vcan0 }, bitrate: 10000, maximum_utilization: 1.0 }
motors:
  - { name: motor_a, coordinate: e2e_joint_1, bus: slow, id: 1, model: RS_03 }
  - { name: motor_b, coordinate: e2e_joint_2, bus: slow, id: 2, model: RS_03 }
motor_groups:
  - { name: group_a, motors: [motor_a], control_rate_hz: 1000.0, command_refresh_rate_hz: 20.0, command_transmission: periodic }
  - { name: group_b, motors: [motor_b], control_rate_hz: 1000.0, command_refresh_rate_hz: 20.0, command_transmission: periodic }
"#;
        let error = parse(text).unwrap_err().to_string();
        assert!(error.contains("estimated utilization"), "{error}");
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
            "{PREFIX}\nimus:\n  - {{ name: imu_a, serial_device: missing, source_frame: base, output_frame: base }}\nmotor_groups:\n  - {{ name: group, motors: [motor_a, motor_b] }}\n"
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
            "serial_devices:\n  - {{ name: serial_a, port: /dev/null, baud_rate: 115200 }}\n{PREFIX}\nimus:\n  - {{ name: imu_a, serial_device: serial_a, source_frame: base, output_frame: base }}\n  - {{ name: imu_b, serial_device: serial_a, source_frame: base, output_frame: base }}\nmotor_groups:\n  - {{ name: group, motors: [motor_a, motor_b] }}\n"
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
  - { name: first, transport: { type: socketcan, interface: can0 } }
  - { name: second, transport: { type: socketcan, interface: can0 } }
motors:
  - { name: motor_a, coordinate: coordinate_a, bus: first, id: 1, model: RS_03 }
  - { name: motor_b, coordinate: coordinate_b, bus: second, id: 1, model: RS_03 }
motor_groups:
  - { name: group, motors: [motor_a, motor_b] }
"#;
        let error = parse(text).unwrap_err().to_string();
        assert!(
            error.contains("CAN transport socketcan:can0 is configured more than once"),
            "{error}"
        );
    }

    #[test]
    fn physical_serial_port_cannot_be_declared_twice() {
        let text = format!(
            "serial_devices:\n  - {{ name: first, port: /dev/ttyUSB0, baud_rate: 115200 }}\n  - {{ name: second, port: /dev/ttyUSB0, baud_rate: 115200 }}\n{PREFIX}\nimus:\n  - {{ name: imu_a, serial_device: first, source_frame: base, output_frame: base }}\n  - {{ name: imu_b, serial_device: second, source_frame: base, output_frame: base }}\nmotor_groups:\n  - {{ name: group, motors: [motor_a, motor_b] }}\n"
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
  - { name: used, transport: { type: socketcan, interface: can0 } }
  - { name: unused, transport: { type: socketcan, interface: can1 } }
motors:
  - { name: motor_a, coordinate: e2e_joint_1, bus: used, id: 1, model: RS_03 }
motor_groups:
  - { name: group, motors: [motor_a] }
"#;
        let error = parse(text).unwrap_err().to_string();
        assert!(error.contains("CAN bus unused has no motors"), "{error}");
    }

    #[test]
    fn every_serial_device_must_own_a_sensor() {
        let text = format!(
            "serial_devices:\n  - {{ name: used, port: /dev/ttyUSB0, baud_rate: 115200 }}\n  - {{ name: unused, port: /dev/ttyUSB1, baud_rate: 115200 }}\n{PREFIX}\nimus:\n  - {{ name: imu, serial_device: used, source_frame: base, output_frame: base }}\nmotor_groups:\n  - {{ name: group, motors: [motor_a, motor_b] }}\n"
        );
        let error = parse(&text).unwrap_err().to_string();
        assert!(
            error.contains("serial device unused is not assigned to a sensor"),
            "{error}"
        );
    }
}
