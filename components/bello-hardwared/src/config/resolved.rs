use std::collections::{HashMap, HashSet};
use std::path::Path;

use anyhow::{Result, anyhow, bail};
use bello_robot_description::RobotDescription;

use super::Config;
use crate::api::grpc::GrpcSpec;
use crate::control::GroupSpec;
use crate::domain::{BusName, GroupName, ImuName, MotorAddress, MotorName, SerialDeviceName};
use crate::hardware::can::BusSpec;
use crate::hardware::imu::ImuSpec;
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
    telemetry_capacity: usize,
    grpc: GrpcSpec,
    serial_devices: HashMap<SerialDeviceName, SerialSpec>,
    imus: HashMap<ImuName, ImuSpec>,
    buses: HashMap<BusName, BusSpec>,
    motors: HashMap<MotorName, ResolvedMotor>,
    groups: HashMap<GroupName, GroupSpec>,
}

impl ResolvedTopology {
    pub(crate) fn load(path: impl AsRef<Path>) -> Result<Self> {
        Self::resolve(Config::load_raw(path)?)
    }

    pub(crate) fn resolve(config: Config) -> Result<Self> {
        validate_root(&config)?;
        if !(64..=1_000_000).contains(&config.runtime.telemetry_capacity) {
            bail!("runtime.telemetry_capacity must be in 64..=1000000");
        }
        if config.robot_description.as_os_str().is_empty() {
            bail!("hardware configuration has no canonical robot description");
        }
        let robot_description = RobotDescription::load(&config.robot_description)?;
        let grpc = GrpcSpec::try_from(&config.grpc)?;

        let mut serial_devices = HashMap::new();
        let mut serial_ports = HashSet::new();
        for raw in &config.serial_devices {
            let spec = SerialSpec::try_from(raw)?;
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
            let spec = ImuSpec::try_from(raw)?;
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
        if let Some(unused) = serial_devices
            .keys()
            .find(|name| !claimed_serial_devices.contains(*name))
        {
            bail!("serial device {unused} is not assigned to a sensor");
        }

        let mut buses = HashMap::new();
        let mut can_interfaces = HashSet::new();
        for raw in &config.buses {
            let spec = BusSpec::try_from(raw)?;
            if !can_interfaces.insert(spec.interface.clone()) {
                bail!(
                    "CAN interface {} is configured more than once",
                    spec.interface
                );
            }
            if buses.insert(spec.name.clone(), spec).is_some() {
                bail!("duplicate CAN bus name {}", raw.name);
            }
        }

        let mut motor_specs = HashMap::new();
        let mut addresses = HashSet::<MotorAddress>::new();
        for raw in &config.motors {
            let mut spec = MotorSpec::try_from(raw)?;
            apply_coordinate_limits(&robot_description, &mut spec)?;
            let bus = buses.get(&spec.address.bus).ok_or_else(|| {
                anyhow!(
                    "motor {} refers to unknown bus {}",
                    spec.name,
                    spec.address.bus
                )
            })?;
            if spec.address.id == bus.host_id {
                bail!("motor {} id conflicts with bus host_id", spec.name);
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
        let used_buses: HashSet<_> = motor_specs
            .values()
            .map(|motor| &motor.address.bus)
            .collect();
        if let Some(unused) = buses.keys().find(|name| !used_buses.contains(name)) {
            bail!("CAN bus {unused} has no motors");
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
            spec.mechanical =
                MechanicalEnvelope::build(&robot_description, &spec.motors, &motor_specs)?;
            if groups.insert(spec.name.clone(), spec).is_some() {
                bail!("duplicate group name {}", raw.name);
            }
        }

        let motors = motor_specs
            .into_iter()
            .map(|(name, spec)| {
                let group = membership
                    .get(&name)
                    .cloned()
                    .ok_or_else(|| anyhow!("motor {name} does not belong to a group"))?;
                Ok((name, ResolvedMotor { spec, group }))
            })
            .collect::<Result<_>>()?;

        validate_bus_utilization(&buses, &motors, &groups)?;
        Ok(Self {
            telemetry_capacity: config.runtime.telemetry_capacity,
            grpc,
            serial_devices,
            imus,
            buses,
            motors,
            groups,
        })
    }

    pub(crate) fn telemetry_capacity(&self) -> usize {
        self.telemetry_capacity
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

fn validate_root(config: &Config) -> Result<()> {
    if config.buses.is_empty() {
        bail!("at least one CAN bus is required");
    }
    if config.motors.is_empty() {
        bail!("at least one motor is required");
    }
    if config.motor_groups.is_empty() {
        bail!("at least one motor group is required");
    }
    Ok(())
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
            if batch_budget >= group.period() {
                bail!(
                    "bus {} cannot transmit group {} control batch within its freshness period",
                    bus.name,
                    group.name
                );
            }
            frames_per_second += group.control_rate_hz * motors_on_bus as f64 * 2.0;
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

    fn canonical_topology() -> ResolvedTopology {
        ResolvedTopology::load(concat!(env!("CARGO_MANIFEST_DIR"), "/config/bello.yaml")).unwrap()
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
    fn canonical_multibus_group_uses_urdf_limits_and_differentials() {
        let topology = canonical_topology();
        let lower = topology.groups.get(&GroupName::from("lower_body")).unwrap();
        let group_buses = lower
            .motors
            .iter()
            .map(|name| topology.motors[name].spec.address.bus.clone())
            .collect::<HashSet<_>>();
        assert_eq!(group_buses.len(), 2);
        assert_eq!(lower.mechanical.differential_count(), 2);

        let waist = &topology.motors[&MotorName::from("waist_yaw")].spec;
        assert_eq!(waist.limits.position_min, -0.5);
        assert_eq!(waist.limits.position_max, 0.5);
        assert_eq!(waist.health.safe_position_min, -0.5);
        assert_eq!(waist.health.safe_position_max, 0.5);

        let ankle = &topology.motors[&MotorName::from("left_ankle_1")].spec;
        assert!((ankle.limits.position_min + 1.1345).abs() < 1.0e-9);
        assert!((ankle.limits.position_max - 0.6109).abs() < 1.0e-9);
        assert_eq!(ankle.limits.torque_min, -36.0);
        assert_eq!(ankle.limits.torque_max, 36.0);
    }

    #[test]
    fn direct_envelope_checks_position_and_velocity() {
        let topology = canonical_topology();
        let lower = topology.groups.get(&GroupName::from("lower_body")).unwrap();
        let motor = MotorName::from("left_hip_pitch");
        let mut commands = neutral_commands(lower);

        commands.get_mut(&motor).unwrap().position = 2.0;
        let position = lower.mechanical.validate_targets(&commands).unwrap_err();
        assert_eq!(position.violation, SafetyViolation::PositionLimit);
        assert!(position.detail.contains("left_hip_pitch_joint"));

        commands.get_mut(&motor).unwrap().position = 0.0;
        commands.get_mut(&motor).unwrap().velocity = 31.0;
        let velocity = lower.mechanical.validate_targets(&commands).unwrap_err();
        assert_eq!(velocity.violation, SafetyViolation::VelocityLimit);
        assert!(velocity.detail.contains("left_hip_pitch_joint"));

        commands.remove(&motor);
        let incomplete = lower.mechanical.validate_targets(&commands).unwrap_err();
        assert!(incomplete.detail.contains("incomplete mechanical command"));
    }

    #[test]
    fn differential_envelope_checks_position_and_velocity() {
        let topology = canonical_topology();
        let lower = topology.groups.get(&GroupName::from("lower_body")).unwrap();
        let mut feedback = neutral_feedback(lower);
        let mut commands = neutral_commands(lower);
        let first = MotorName::from("left_ankle_1");
        let second = MotorName::from("left_ankle_2");

        feedback.get_mut(&first).unwrap().position = 0.28;
        feedback.get_mut(&second).unwrap().position = 0.28;
        lower
            .mechanical
            .validate_feedback_positions(&feedback)
            .unwrap();
        feedback.get_mut(&first).unwrap().position = 0.29;
        feedback.get_mut(&second).unwrap().position = 0.29;
        let feedback_position = lower
            .mechanical
            .validate_feedback_positions(&feedback)
            .unwrap_err();
        assert_eq!(feedback_position.violation, SafetyViolation::PositionLimit);
        assert!(
            feedback_position
                .detail
                .contains("feedback tolerance 0.200000")
        );

        commands.get_mut(&first).unwrap().position = 0.2;
        commands.get_mut(&second).unwrap().position = 0.2;
        let position = lower.mechanical.validate_targets(&commands).unwrap_err();
        assert_eq!(position.violation, SafetyViolation::PositionLimit);
        assert!(position.detail.contains("left_ankle_roll_joint"));

        commands.get_mut(&first).unwrap().position = 0.0;
        commands.get_mut(&second).unwrap().position = 0.0;
        commands.get_mut(&first).unwrap().velocity = 31.0;
        commands.get_mut(&second).unwrap().velocity = 31.0;
        let velocity = lower.mechanical.validate_targets(&commands).unwrap_err();
        assert_eq!(velocity.violation, SafetyViolation::VelocityLimit);
        assert!(velocity.detail.contains("left_ankle_roll_joint"));
    }

    #[test]
    fn damping_uses_configured_gains_without_scaling() {
        let topology = canonical_topology();

        let wrist = topology.groups.get(&GroupName::from("wrist_yaw")).unwrap();
        let left_wrist = MotorName::from("left_arm_7");
        let wrist_feedback = neutral_feedback(wrist);
        let wrist_damping = wrist.mechanical.damping_commands(&wrist_feedback).unwrap();
        assert!((wrist_damping[&left_wrist].kd - 0.11).abs() < 1.0e-12);

        let lower = topology.groups.get(&GroupName::from("lower_body")).unwrap();
        let first = MotorName::from("left_ankle_1");
        let second = MotorName::from("left_ankle_2");
        let hip = MotorName::from("left_hip_pitch");
        let lower_feedback = neutral_feedback(lower);
        let ankle_damping = lower.mechanical.damping_commands(&lower_feedback).unwrap();
        assert!((ankle_damping[&first].kd - 0.4).abs() < 1.0e-12);
        assert!((ankle_damping[&second].kd - 0.4).abs() < 1.0e-12);
        assert!((ankle_damping[&hip].kd - 8.0).abs() < 1.0e-12);

        let mut missing = lower_feedback.clone();
        missing.remove(&first);
        let error = lower.mechanical.damping_commands(&missing).unwrap_err();
        assert!(error.detail.contains("without fresh motor feedback"));

        let mut nonfinite = lower_feedback;
        nonfinite.get_mut(&first).unwrap().position = f64::NAN;
        let error = lower.mechanical.damping_commands(&nonfinite).unwrap_err();
        assert_eq!(
            error.violation,
            SafetyViolation::NonFinite {
                field: "feedback position"
            }
        );
    }
}
