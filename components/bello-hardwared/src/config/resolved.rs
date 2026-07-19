use std::collections::{HashMap, HashSet};
use std::path::Path;

use anyhow::{Result, anyhow, bail};

use super::Config;
use crate::api::grpc::GrpcSpec;
use crate::app::RuntimeSpec;
use crate::control::GroupSpec;
use crate::domain::{BusName, GroupName, ImuName, MotorAddress, MotorName, SerialDeviceName};
use crate::hardware::can::BusSpec;
use crate::hardware::imu::ImuSpec;
use crate::hardware::robstride::MotorSpec;
use crate::hardware::serial::SerialSpec;
use crate::rerun_sink::RerunSpec;

#[derive(Clone, Debug)]
pub(crate) struct ResolvedMotor {
    pub(crate) spec: MotorSpec,
    pub(crate) group: GroupName,
}

#[derive(Clone)]
pub struct ResolvedTopology {
    runtime: RuntimeSpec,
    grpc: GrpcSpec,
    rerun: RerunSpec,
    serial_devices: HashMap<SerialDeviceName, SerialSpec>,
    imus: HashMap<ImuName, ImuSpec>,
    buses: HashMap<BusName, BusSpec>,
    motors: HashMap<MotorName, ResolvedMotor>,
    groups: HashMap<GroupName, GroupSpec>,
}

impl ResolvedTopology {
    pub fn load(path: impl AsRef<Path>) -> Result<Self> {
        Self::resolve(Config::load_raw(path)?)
    }

    pub fn resolve(config: Config) -> Result<Self> {
        validate_root(&config)?;
        let runtime = RuntimeSpec::try_from(&config.runtime)?;
        let grpc = GrpcSpec::try_from(&config.grpc)?;
        let rerun = RerunSpec::try_from(&config.rerun)?;

        let mut serial_devices = HashMap::new();
        for raw in &config.serial_devices {
            let spec = SerialSpec::try_from(raw)?;
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
            if spec.enabled && !claimed_serial_devices.insert(spec.serial_device.clone()) {
                bail!(
                    "serial device {} is assigned to more than one enabled sensor",
                    spec.serial_device
                );
            }
            if imus.insert(spec.name.clone(), spec).is_some() {
                bail!("duplicate IMU name {}", raw.name);
            }
        }

        let mut buses = HashMap::new();
        for raw in &config.buses {
            let spec = BusSpec::try_from(raw)?;
            if buses.insert(spec.name.clone(), spec).is_some() {
                bail!("duplicate CAN bus name {}", raw.name);
            }
        }

        let mut motor_specs = HashMap::new();
        let mut addresses = HashSet::<MotorAddress>::new();
        for raw in &config.motors {
            let spec = MotorSpec::try_from(raw)?;
            if !buses.contains_key(&spec.address.bus) {
                bail!(
                    "motor {} refers to unknown bus {}",
                    spec.name,
                    spec.address.bus
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
            let spec = GroupSpec::try_from(raw)?;
            for motor in &spec.motors {
                if !motor_specs.contains_key(motor) {
                    bail!("group {} refers to unknown motor {motor}", spec.name);
                }
                if let Some(owner) = membership.insert(motor.clone(), spec.name.clone()) {
                    bail!("motor {motor} belongs to both {owner} and {}", spec.name);
                }
            }
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
            runtime,
            grpc,
            rerun,
            serial_devices,
            imus,
            buses,
            motors,
            groups,
        })
    }

    pub fn runtime(&self) -> &RuntimeSpec {
        &self.runtime
    }

    pub fn grpc(&self) -> &GrpcSpec {
        &self.grpc
    }

    pub fn rerun(&self) -> &RerunSpec {
        &self.rerun
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
    // Conservative budget: 150 wire bits for each extended 8-byte command or reply.
    for bus in buses.values() {
        let frames_per_second = groups
            .values()
            .map(|group| {
                let motors_on_bus = group
                    .motors
                    .iter()
                    .filter(|name| {
                        motors
                            .get(*name)
                            .is_some_and(|motor| motor.spec.address.bus == bus.name)
                    })
                    .count();
                group.control_rate_hz * motors_on_bus as f64 * 2.0
            })
            .sum::<f64>();
        let utilization = frames_per_second * 150.0 / bus.bitrate as f64;
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
