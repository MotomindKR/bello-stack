use std::collections::{HashMap, HashSet};
use std::path::Path;

use anyhow::{Context, Result, anyhow, bail};
use bello_robot_description::RobotDescription;
use garde::Validate;

use super::{CommandTransmission, Config, ImuFilterConfig, MotorVelocityFilterConfig};
use crate::api::grpc::GrpcSpec;
use crate::control::GroupSpec;
use crate::domain::PowerBoardName;
use crate::domain::{BusName, GroupName, ImuName, MotorAddress, MotorName, SerialDeviceName};
use crate::hardware::can::BusSpec;
use crate::hardware::imu::ImuSpec;
use crate::hardware::power_board::PowerBoardSpec;
use crate::hardware::robstride::MotorSpec;
use crate::hardware::serial::SerialSpec;
use crate::mechanics::{MechanicalEnvelope, apply_coordinate_limits};

#[derive(Clone, Debug)]
pub(crate) struct ResolvedMotor {
    pub(crate) spec: MotorSpec,
    pub(crate) group: GroupName,
}

#[derive(Clone)]
pub(crate) struct ResolvedTopology {
    pub(crate) cameras: Option<bello_hardware_api::camera::config::Source>,
    telemetry_capacity: usize,
    motor_velocity_filter: MotorVelocityFilterConfig,
    imu_filter: ImuFilterConfig,
    grpc: GrpcSpec,
    serial_devices: HashMap<SerialDeviceName, SerialSpec>,
    imus: HashMap<ImuName, ImuSpec>,
    power_boards: HashMap<PowerBoardName, PowerBoardSpec>,
    buses: HashMap<BusName, BusSpec>,
    motors: HashMap<MotorName, ResolvedMotor>,
    groups: HashMap<GroupName, GroupSpec>,
}

impl ResolvedTopology {
    pub(crate) fn validate_platform(&self) -> Result<()> {
        for (name, bus) in &self.buses {
            if !cfg!(target_os = "linux")
                && matches!(bus.transport, super::BusTransportConfig::Socketcan { .. })
            {
                bail!(
                    "CAN bus {name}: SocketCAN requires Linux; use gsusb for hardware or sim for simulation"
                );
            }
        }
        Ok(())
    }

    pub(crate) fn load(path: impl AsRef<Path>) -> Result<Self> {
        Self::resolve(Config::load_raw(path)?)
    }

    pub(crate) fn resolve(config: Config) -> Result<Self> {
        config.validate()?;
        if config.robot_description.as_os_str().is_empty() {
            bail!("hardware configuration has no canonical robot description");
        }
        let robot_description =
            RobotDescription::load(&config.robot_description).with_context(|| {
                format!(
                    "invalid robot description {}",
                    config.robot_description.display()
                )
            })?;
        for camera in config.camera_inventory.values() {
            anyhow::ensure!(
                robot_description.link(&camera.link).is_some(),
                "camera {} refers to unknown URDF link {}",
                camera.name,
                camera.link
            );
        }
        let cameras = bello_hardware_api::camera::config::Source::bind(
            config.camera_inventory,
            config.cameras,
        )?;
        let grpc = GrpcSpec::try_from(&config.grpc)?;

        let mut serial_devices = HashMap::new();
        let mut serial_ports = HashSet::new();
        for raw in &config.serial_devices {
            let spec = SerialSpec::resolve(raw, config.simulation)?;
            if !serial_ports.insert(spec.port.clone()) {
                bail!("serial port {} is configured more than once", spec.port);
            }
            if serial_devices.insert(spec.name.clone(), spec).is_some() {
                bail!("duplicate serial device name {}", raw.name);
            }
        }

        let mut imus = HashMap::new();
        let mut claimed_serial_devices = HashSet::new();
        for raw in &config.imus {
            let spec = ImuSpec::resolve(raw, &robot_description)?;
            let nyquist_hz = f64::from(raw.output_rate_hz) / 2.0;
            for (name, cutoff_hz) in [
                (
                    "orientation_cutoff_hz",
                    config.runtime.imu_filter.orientation_cutoff_hz,
                ),
                ("gyro_cutoff_hz", config.runtime.imu_filter.gyro_cutoff_hz),
                (
                    "acceleration_cutoff_hz",
                    config.runtime.imu_filter.acceleration_cutoff_hz,
                ),
            ] {
                if cutoff_hz >= nyquist_hz {
                    bail!(
                        "runtime.imu_filter.{name} must be below the Nyquist frequency {nyquist_hz} Hz for IMU {}",
                        spec.name
                    );
                }
            }
            if !serial_devices.contains_key(&spec.serial_device) {
                bail!(
                    "IMU {} refers to unknown serial device {}",
                    spec.name,
                    spec.serial_device
                );
            }
            if !claimed_serial_devices.insert(spec.serial_device.clone()) {
                bail!(
                    "serial device {} is assigned to more than one sensor",
                    spec.serial_device
                );
            }
            if imus.insert(spec.name.clone(), spec).is_some() {
                bail!("duplicate IMU name {}", raw.name);
            }
        }
        let mut power_boards = HashMap::new();
        for raw in &config.power_boards {
            let spec = PowerBoardSpec::try_from(raw)?;
            if !serial_devices.contains_key(&spec.serial_device) {
                bail!(
                    "power board {} refers to unknown serial device {}",
                    spec.name,
                    spec.serial_device
                );
            }
            if !claimed_serial_devices.insert(spec.serial_device.clone()) {
                bail!(
                    "serial device {} is assigned to more than one sensor",
                    spec.serial_device
                );
            }
            if power_boards.insert(spec.name.clone(), spec).is_some() {
                bail!("duplicate power board name {}", raw.name);
            }
        }
        if power_boards
            .values()
            .filter(|board| board.emergency_stop == super::EmergencyStopMode::Damping)
            .count()
            > 1
        {
            bail!("only one power board may own emergency damping");
        }
        if let Some(unused) = serial_devices
            .keys()
            .find(|name| !claimed_serial_devices.contains(*name))
        {
            bail!("serial device {unused} is not assigned to a sensor");
        }

        let mut buses = HashMap::new();
        let mut can_transports = HashSet::new();
        let mut claimed_usb_serials = HashSet::new();
        let mut sim_ports = HashSet::new();
        for raw in &config.buses {
            let spec = BusSpec::try_from(raw)?;
            if let crate::config::BusTransportConfig::Gsusb { serial, .. } = &spec.transport
                && !claimed_usb_serials.insert(serial.clone())
            {
                bail!("GS-USB device {serial} is assigned to more than one CAN bus");
            }
            if let crate::config::BusTransportConfig::Sim(bus) = &spec.transport {
                for port in [bus.daemon_port, bus.simulator_port] {
                    if !sim_ports.insert(port) {
                        bail!("sim port {port} is assigned to more than one CAN bus");
                    }
                }
            }
            if !can_transports.insert(spec.transport.clone()) {
                bail!(
                    "CAN transport {} is configured more than once",
                    spec.transport.label()
                );
            }
            if buses.insert(spec.name.clone(), spec).is_some() {
                bail!("duplicate CAN bus name {}", raw.name);
            }
        }

        let mut motor_specs = HashMap::new();
        let mut addresses = HashSet::<MotorAddress>::new();
        let mut coordinates = HashSet::new();
        for raw in &config.motors {
            let mut spec = MotorSpec::try_from(raw)?;
            apply_coordinate_limits(&robot_description, &mut spec)?;
            if !coordinates.insert(raw.coordinate.as_str()) {
                bail!(
                    "coordinate {} is assigned to more than one motor",
                    raw.coordinate
                );
            }
            if !addresses.insert(spec.address.clone()) {
                bail!(
                    "duplicate motor address {}:{}",
                    spec.address.bus,
                    spec.address.id
                );
            }
            if motor_specs.insert(spec.name.clone(), spec).is_some() {
                bail!("duplicate motor name {}", raw.name);
            }
        }
        let mut groups = HashMap::new();
        let mut membership = HashMap::<MotorName, GroupName>::new();
        for raw in &config.motor_groups {
            let mut spec = GroupSpec::try_from(raw)?;
            for motor in &spec.motors {
                if !motor_specs.contains_key(motor) {
                    bail!("group {} refers to unknown motor {motor}", spec.name);
                }
                if let Some(owner) = membership.insert(motor.clone(), spec.name.clone()) {
                    bail!("motor {motor} belongs to both {owner} and {}", spec.name);
                }
            }
            spec.mechanical = MechanicalEnvelope::build(
                &robot_description,
                &spec.motors,
                &motor_specs,
                raw.parallel_feedback_position_tolerance,
            )?;
            if groups.insert(spec.name.clone(), spec).is_some() {
                bail!("duplicate group name {}", raw.name);
            }
        }

        groups.retain(|name, _| !config.disabled_groups.contains(name.as_str()));
        let mut motors = HashMap::new();
        let mut used_buses = HashSet::new();
        for (name, spec) in motor_specs {
            let group = membership
                .remove(&name)
                .ok_or_else(|| anyhow!("motor {name} does not belong to a group"))?;
            if !groups.contains_key(&group) {
                continue;
            }
            let bus = &spec.address.bus;
            let binding = buses.get(bus).ok_or_else(|| {
                anyhow!("missing transport for CAN bus {bus} used by motor {name}")
            })?;
            if spec.address.id == binding.host_id {
                bail!("motor {name} id conflicts with bus host_id");
            }
            used_buses.insert(bus.clone());
            motors.insert(name, ResolvedMotor { spec, group });
        }
        if let Some(unused) = buses.keys().find(|name| !used_buses.contains(*name)) {
            bail!("CAN bus {unused} has no motors in the enabled groups");
        }

        validate_bus_utilization(&buses, &motors, &groups)?;
        Ok(Self {
            cameras,
            telemetry_capacity: config.runtime.telemetry_capacity,
            motor_velocity_filter: config.runtime.motor_velocity_filter,
            imu_filter: config.runtime.imu_filter,
            grpc,
            serial_devices,
            imus,
            power_boards,
            buses,
            motors,
            groups,
        })
    }

    pub(crate) fn telemetry_capacity(&self) -> usize {
        self.telemetry_capacity
    }

    pub(crate) fn motor_velocity_filter(&self) -> MotorVelocityFilterConfig {
        self.motor_velocity_filter
    }

    pub(crate) fn imu_filter(&self) -> ImuFilterConfig {
        self.imu_filter
    }

    pub(crate) fn grpc(&self) -> &GrpcSpec {
        &self.grpc
    }

    pub(crate) fn serial_devices(&self) -> impl Iterator<Item = (&SerialDeviceName, &SerialSpec)> {
        self.serial_devices.iter()
    }

    pub(crate) fn imus(&self) -> impl Iterator<Item = (&ImuName, &ImuSpec)> {
        self.imus.iter()
    }

    pub(crate) fn power_boards(&self) -> impl Iterator<Item = (&PowerBoardName, &PowerBoardSpec)> {
        self.power_boards.iter()
    }

    pub(crate) fn motor(&self, name: &MotorName) -> Result<&ResolvedMotor> {
        self.motors
            .get(name)
            .ok_or_else(|| anyhow!("unknown motor {name}"))
    }

    pub(crate) fn buses(&self) -> impl Iterator<Item = (&BusName, &BusSpec)> {
        self.buses.iter()
    }

    pub(crate) fn motors(&self) -> impl Iterator<Item = (&MotorName, &ResolvedMotor)> {
        self.motors.iter()
    }

    pub(crate) fn groups(&self) -> impl Iterator<Item = (&GroupName, &GroupSpec)> {
        self.groups.iter()
    }
}

fn validate_bus_utilization(
    buses: &HashMap<BusName, BusSpec>,
    motors: &HashMap<MotorName, ResolvedMotor>,
    groups: &HashMap<GroupName, GroupSpec>,
) -> Result<()> {
    for bus in buses.values() {
        let mut frames_per_second = 0.0;
        for group in groups.values() {
            let motors_on_bus = group
                .motors
                .iter()
                .filter(|name| {
                    motors
                        .get(*name)
                        .is_some_and(|motor| motor.spec.address.bus == bus.name)
                })
                .count();
            if motors_on_bus == 0 {
                continue;
            }
            let batch_budget = bus.control_batch_budget(motors_on_bus);
            let (transmission_rate, transmission_period) = match group.command_transmission {
                CommandTransmission::OnUpdate => (group.control_rate_hz, group.period()),
                CommandTransmission::Periodic => (
                    group.command_refresh_rate_hz,
                    group.command_refresh_period(),
                ),
            };
            if batch_budget >= transmission_period {
                bail!(
                    "bus {} cannot transmit group {} control batch within its transmission period",
                    bus.name,
                    group.name
                );
            }
            frames_per_second += transmission_rate * motors_on_bus as f64 * 2.0;
        }
        let utilization = bus.estimated_utilization(frames_per_second);
        if utilization > bus.maximum_utilization {
            bail!(
                "bus {} estimated utilization {:.1}% exceeds configured {:.1}%",
                bus.name,
                utilization * 100.0,
                bus.maximum_utilization * 100.0
            );
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::control::GroupSpec;
    use crate::domain::{ImpedanceCommand, SafetyViolation};
    use crate::hardware::robstride::Feedback;

    #[test]
    fn camera_transport_preserves_the_robot_calibration() {
        use bello_hardware_api::camera::{CameraLayout, config::Transport};
        let path = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/config/milly-7dof-gimbal-sim.yaml"
        );
        let mut profile = Config::load_raw(path).unwrap();
        let socket = std::env::temp_dir().join("camera.sock");
        profile.cameras.insert(
            "stereo".into(),
            Transport::Mujoco {
                socket: socket.clone(),
            },
        );
        let sim = ResolvedTopology::resolve(profile.clone())
            .unwrap()
            .cameras
            .unwrap();
        profile.cameras.insert(
            "stereo".into(),
            Transport::V4l2 {
                device: std::env::temp_dir().join("video0"),
            },
        );
        let usb = ResolvedTopology::resolve(profile.clone())
            .unwrap()
            .cameras
            .unwrap();
        let (mut sim, layout) = sim.0[0].stream_info();
        let (usb, usb_layout) = usb.0[0].stream_info();
        assert_eq!(
            (sim.width, sim.height, sim.fps, layout),
            (3200, 1200, 30, CameraLayout::StereoSideBySide)
        );
        assert_eq!(layout, usb_layout);
        assert_eq!(sim.source, "mujoco");
        assert_eq!(usb.source, "v4l2");
        sim.source.clone_from(&usb.source);
        assert_eq!(sim, usb);

        profile
            .cameras
            .insert("typo".into(), Transport::Mujoco { socket });
        assert!(
            ResolvedTopology::resolve(profile.clone())
                .err()
                .unwrap()
                .to_string()
                .contains("unknown camera typo")
        );
        profile.cameras.clear();
        profile.camera_inventory.get_mut("stereo").unwrap().link = "missing_mount".into();
        assert!(
            ResolvedTopology::resolve(profile)
                .err()
                .unwrap()
                .to_string()
                .contains("unknown URDF link missing_mount")
        );
    }

    fn transmission_config() -> Config {
        let mut config: Config = serde_yaml::from_str(
            r#"
buses:
  - { name: first, transport: { type: socketcan, interface: vcan0 } }
  - { name: second, transport: { type: socketcan, interface: vcan1 } }
motors:
  - { name: direct, coordinate: direct_joint, bus: first, id: 1, model: RS_03 }
  - { name: diff_a, coordinate: diff_a, bus: first, id: 2, model: RS_03, health: { feedback_position_tolerance: 0.125 } }
  - { name: diff_b, coordinate: diff_b, bus: second, id: 2, model: RS_03, health: { feedback_position_tolerance: 0.125 } }
  - { name: drive_a, coordinate: parallel_a, bus: first, id: 3, model: RS_03 }
  - { name: drive_b, coordinate: parallel_b, bus: first, id: 4, model: RS_03 }
motor_groups:
  - { name: direct, motors: [direct] }
  - { name: differential, motors: [diff_a, diff_b] }
  - { name: parallel, motors: [drive_a, drive_b], parallel_feedback_position_tolerance: 0.02 }
"#,
        )
        .unwrap();
        config.robot_description =
            Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/transmissions.urdf");
        config
    }

    fn neutral_commands(group: &GroupSpec) -> HashMap<MotorName, ImpedanceCommand> {
        group
            .motors
            .iter()
            .map(|name| (name.clone(), ImpedanceCommand::default()))
            .collect()
    }

    fn neutral_feedback(group: &GroupSpec) -> HashMap<MotorName, Feedback> {
        group
            .motors
            .iter()
            .map(|name| (name.clone(), Feedback::default()))
            .collect()
    }

    #[test]
    fn disabled_groups_need_no_bus_binding_and_publish_no_motors() {
        let mut config = transmission_config();
        config
            .disabled_groups
            .extend(["differential".into(), "parallel".into()]);
        config.buses.retain(|bus| bus.name == "first");
        let topology = ResolvedTopology::resolve(config).unwrap();

        assert_eq!(
            topology.groups.keys().collect::<Vec<_>>(),
            [&GroupName::from("direct")]
        );
        assert_eq!(
            topology.motors.keys().collect::<Vec<_>>(),
            [&MotorName::from("direct")]
        );
    }

    #[test]
    fn parallel_drives_must_agree_on_target_and_measured_joint_positions() {
        let topology = ResolvedTopology::resolve(transmission_config()).unwrap();
        let group = &topology.groups[&GroupName::from("parallel")];
        let first = MotorName::from("drive_a");
        let second = MotorName::from("drive_b");
        let mut commands = neutral_commands(group);

        commands.get_mut(&first).unwrap().position = -0.25;
        commands.get_mut(&second).unwrap().position = 0.5;
        group.mechanical.validate_targets(&commands).unwrap();
        commands.get_mut(&second).unwrap().position = 0.6;
        let error = group.mechanical.validate_targets(&commands).unwrap_err();
        assert!(error.detail.contains("targets disagree"));
        assert_eq!(error.affected_motors, vec![first, second.clone()]);

        let mut feedback = neutral_feedback(group);
        feedback.get_mut(&second).unwrap().position = 0.02;
        group
            .mechanical
            .validate_feedback_positions(&feedback)
            .unwrap();
        feedback.get_mut(&second).unwrap().position = 0.06;
        let error = group
            .mechanical
            .validate_feedback_positions(&feedback)
            .unwrap_err();
        assert!(error.detail.contains("encoder disagreement"));
    }

    #[test]
    fn parallel_drives_must_be_owned_by_one_group() {
        let mut config = transmission_config();
        let parallel = config
            .motor_groups
            .iter_mut()
            .find(|group| group.name == "parallel")
            .unwrap();
        parallel.motors.retain(|motor| motor != "drive_b");
        config
            .motor_groups
            .push(serde_yaml::from_str("name: isolated\nmotors: [drive_b]").unwrap());
        let error = ResolvedTopology::resolve(config).err().unwrap().to_string();
        assert!(error.contains("one motor group"), "{error}");
    }

    #[test]
    fn shared_joint_name_cannot_bypass_parallel_drive_ownership() {
        let mut config = transmission_config();
        config.motors.retain(|motor| motor.name != "drive_b");
        config
            .motors
            .iter_mut()
            .find(|motor| motor.name == "drive_a")
            .unwrap()
            .coordinate = "shared_joint".into();
        config
            .motor_groups
            .iter_mut()
            .find(|group| group.name == "parallel")
            .unwrap()
            .motors
            .retain(|motor| motor != "drive_b");
        let error = ResolvedTopology::resolve(config).err().unwrap().to_string();
        assert!(error.contains("no URDF position limits"), "{error}");
    }

    #[test]
    fn imu_filter_cutoffs_must_be_below_sensor_nyquist_frequency() {
        let component = concat!(env!("CARGO_MANIFEST_DIR"), "/config/bello.yaml");
        let mut config = Config::load_raw(component).unwrap();
        let output_rate_hz = 100;
        for imu in &mut config.imus {
            imu.output_rate_hz = output_rate_hz;
        }
        let nyquist_hz = f64::from(output_rate_hz) / 2.0;
        config.runtime.imu_filter = ImuFilterConfig {
            orientation_cutoff_hz: 12.0,
            gyro_cutoff_hz: 24.0,
            acceleration_cutoff_hz: nyquist_hz - 1.0,
        };
        ResolvedTopology::resolve(config.clone()).unwrap();

        config.runtime.imu_filter.acceleration_cutoff_hz = nyquist_hz;
        let error = ResolvedTopology::resolve(config).err().unwrap().to_string();
        assert!(
            error.contains("acceleration_cutoff_hz must be below the Nyquist frequency"),
            "{error}"
        );
    }

    #[test]
    fn direct_envelope_checks_position_and_velocity() {
        let topology = ResolvedTopology::resolve(transmission_config()).unwrap();
        let group = &topology.groups[&GroupName::from("direct")];
        let motor = MotorName::from("direct");
        let spec = &topology.motors[&motor].spec;
        assert_eq!(
            (spec.limits.torque_min, spec.limits.torque_max),
            (-5.0, 5.0)
        );
        assert_eq!(
            (spec.health.safe_position_min, spec.health.safe_position_max),
            (-1.25, 1.5)
        );
        let mut commands = neutral_commands(group);

        commands.get_mut(&motor).unwrap().position = 1.5;
        group.mechanical.validate_targets(&commands).unwrap();
        commands.get_mut(&motor).unwrap().position = 1.6;
        let position = group.mechanical.validate_targets(&commands).unwrap_err();
        assert_eq!(position.violation, SafetyViolation::PositionLimit);

        commands.get_mut(&motor).unwrap().position = 0.0;
        commands.get_mut(&motor).unwrap().velocity = 4.1;
        let velocity = group.mechanical.validate_targets(&commands).unwrap_err();
        assert_eq!(velocity.violation, SafetyViolation::VelocityLimit);

        commands.remove(&motor);
        let incomplete = group.mechanical.validate_targets(&commands).unwrap_err();
        assert!(incomplete.detail.contains("incomplete mechanical command"));
    }

    #[test]
    fn differential_envelope_checks_resolved_joint_positions_and_velocity() {
        let topology = ResolvedTopology::resolve(transmission_config()).unwrap();
        let group = &topology.groups[&GroupName::from("differential")];
        let mut commands = neutral_commands(group);
        let first = MotorName::from("diff_a");
        let second = MotorName::from("diff_b");
        group.mechanical.validate_targets(&commands).unwrap();

        for (a, b, joint) in [(0.5, 0.5, "roll_joint"), (0.625, -0.625, "pitch_joint")] {
            commands.get_mut(&first).unwrap().position = a;
            commands.get_mut(&second).unwrap().position = b;
            let error = group.mechanical.validate_targets(&commands).unwrap_err();
            assert_eq!(error.violation, SafetyViolation::PositionLimit);
            assert!(error.detail.contains(joint));
        }

        for command in commands.values_mut() {
            command.position = 0.0;
            command.velocity = 3.0;
        }
        let velocity = group.mechanical.validate_targets(&commands).unwrap_err();
        assert_eq!(velocity.violation, SafetyViolation::VelocityLimit);
        assert!(velocity.detail.contains("roll_joint"));
    }

    #[test]
    fn differential_feedback_uses_motor_position_tolerances() {
        let topology = ResolvedTopology::resolve(transmission_config()).unwrap();
        let group = &topology.groups[&GroupName::from("differential")];
        let mut feedback = neutral_feedback(group);

        for sample in feedback.values_mut() {
            sample.position = 0.375;
        }
        group
            .mechanical
            .validate_feedback_positions(&feedback)
            .unwrap();
        for sample in feedback.values_mut() {
            sample.position = 0.4;
        }
        let error = group
            .mechanical
            .validate_feedback_positions(&feedback)
            .unwrap_err();
        assert_eq!(error.violation, SafetyViolation::PositionLimit);
        assert!(error.detail.contains("roll_joint"));
    }

    #[test]
    fn damping_uses_configured_gains_without_scaling() {
        let mut config = transmission_config();
        let gains = [
            ("direct", 1.5),
            ("diff_a", 3.0),
            ("diff_b", 5.0),
            ("drive_b", 2.5),
        ];
        for (name, kd) in gains {
            config
                .motors
                .iter_mut()
                .find(|motor| motor.name == name)
                .unwrap()
                .fault_damping_kd = kd;
        }

        let topology = ResolvedTopology::resolve(config).unwrap();
        let damping: HashMap<_, _> = topology
            .groups
            .values()
            .flat_map(|group| group.mechanical.damping_commands())
            .collect();
        for (name, kd) in gains {
            assert_eq!(
                damping[&MotorName::from(name)],
                ImpedanceCommand {
                    kd,
                    ..Default::default()
                }
            );
        }
    }
}
