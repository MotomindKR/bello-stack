use std::collections::{BTreeMap, BTreeSet};
use std::sync::Arc;

use bello_hardware_api as proto;

use crate::{DeviceState, Error, GroupState};

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DeviceStatus {
    state: DeviceState,
    detail: String,
    transition_count: u64,
}

impl DeviceStatus {
    pub fn state(&self) -> DeviceState {
        self.state
    }

    pub fn detail(&self) -> &str {
        &self.detail
    }

    pub fn transition_count(&self) -> u64 {
        self.transition_count
    }
}

impl From<proto::DeviceStatus> for DeviceStatus {
    fn from(status: proto::DeviceStatus) -> Self {
        Self {
            state: DeviceState::from_raw(status.state),
            detail: status.detail,
            transition_count: status.transition_count,
        }
    }
}

/// Resolved command bounds for one motor, expressed in coordinate-space SI units.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotorLimits {
    pub(crate) position_min: f64,
    pub(crate) position_max: f64,
    pub(crate) velocity_min: f64,
    pub(crate) velocity_max: f64,
    pub(crate) torque_min: f64,
    pub(crate) torque_max: f64,
    pub(crate) kp_min: f64,
    pub(crate) kp_max: f64,
    pub(crate) kd_min: f64,
    pub(crate) kd_max: f64,
}

impl MotorLimits {
    pub fn position_min(&self) -> f64 {
        self.position_min
    }

    pub fn position_max(&self) -> f64 {
        self.position_max
    }

    pub fn velocity_min(&self) -> f64 {
        self.velocity_min
    }

    pub fn velocity_max(&self) -> f64 {
        self.velocity_max
    }

    pub fn torque_min(&self) -> f64 {
        self.torque_min
    }

    pub fn torque_max(&self) -> f64 {
        self.torque_max
    }

    pub fn kp_min(&self) -> f64 {
        self.kp_min
    }

    pub fn kp_max(&self) -> f64 {
        self.kp_max
    }

    pub fn kd_min(&self) -> f64 {
        self.kd_min
    }

    pub fn kd_max(&self) -> f64 {
        self.kd_max
    }

    fn from_proto(limits: proto::MotorLimits) -> Result<Self, &'static str> {
        let resolved = Self {
            position_min: limits.position_min,
            position_max: limits.position_max,
            velocity_min: limits.velocity_min,
            velocity_max: limits.velocity_max,
            torque_min: limits.torque_min,
            torque_max: limits.torque_max,
            kp_min: limits.kp_min,
            kp_max: limits.kp_max,
            kd_min: limits.kd_min,
            kd_max: limits.kd_max,
        };
        for (name, minimum, maximum) in [
            ("position", resolved.position_min, resolved.position_max),
            ("velocity", resolved.velocity_min, resolved.velocity_max),
            ("torque", resolved.torque_min, resolved.torque_max),
            ("kp", resolved.kp_min, resolved.kp_max),
            ("kd", resolved.kd_min, resolved.kd_max),
        ] {
            if !minimum.is_finite() || !maximum.is_finite() || minimum >= maximum {
                return Err(match name {
                    "position" => "invalid position limits",
                    "velocity" => "invalid velocity limits",
                    "torque" => "invalid torque limits",
                    "kp" => "invalid kp limits",
                    _ => "invalid kd limits",
                });
            }
        }
        if resolved.kp_min < 0.0 || resolved.kd_min < 0.0 {
            return Err("gain limits must be non-negative");
        }
        Ok(resolved)
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Motor {
    pub(crate) name: String,
    pub(crate) coordinate: String,
    pub(crate) group: String,
    pub(crate) bus: String,
    pub(crate) id: u32,
    pub(crate) model: String,
    pub(crate) direction: i32,
    pub(crate) limits: MotorLimits,
    pub(crate) initial_status: Option<DeviceStatus>,
}

impl Motor {
    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn coordinate(&self) -> &str {
        &self.coordinate
    }

    pub fn group(&self) -> &str {
        &self.group
    }

    pub fn bus(&self) -> &str {
        &self.bus
    }

    pub fn id(&self) -> u32 {
        self.id
    }

    pub fn model(&self) -> &str {
        &self.model
    }

    pub fn direction(&self) -> i32 {
        self.direction
    }

    pub fn limits(&self) -> &MotorLimits {
        &self.limits
    }

    pub fn initial_status(&self) -> Option<&DeviceStatus> {
        self.initial_status.as_ref()
    }
}

/// Configured CAN bus metadata and its status at inventory time.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Bus {
    pub(crate) name: String,
    pub(crate) transport: String,
    pub(crate) host_id: u32,
    pub(crate) initial_status: Option<DeviceStatus>,
}

impl Bus {
    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn transport(&self) -> &str {
        &self.transport
    }

    pub fn host_id(&self) -> u32 {
        self.host_id
    }

    pub fn initial_status(&self) -> Option<&DeviceStatus> {
        self.initial_status.as_ref()
    }
}

/// Configured serial transport metadata and its status at inventory time.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SerialDevice {
    pub(crate) name: String,
    pub(crate) port: String,
    pub(crate) baud_rate: u32,
    pub(crate) initial_status: Option<DeviceStatus>,
}

impl SerialDevice {
    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn port(&self) -> &str {
        &self.port
    }

    pub fn baud_rate(&self) -> u32 {
        self.baud_rate
    }

    pub fn initial_status(&self) -> Option<&DeviceStatus> {
        self.initial_status.as_ref()
    }
}

/// Configured IMU metadata and its status at inventory time.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Imu {
    pub(crate) name: String,
    pub(crate) serial_device: String,
    pub(crate) source_frame: String,
    pub(crate) output_frame: String,
    pub(crate) initial_status: Option<DeviceStatus>,
}

impl Imu {
    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn serial_device(&self) -> &str {
        &self.serial_device
    }

    pub fn source_frame(&self) -> &str {
        &self.source_frame
    }

    pub fn output_frame(&self) -> &str {
        &self.output_frame
    }

    pub fn initial_status(&self) -> Option<&DeviceStatus> {
        self.initial_status.as_ref()
    }
}

#[derive(Clone, Debug)]
pub(crate) struct GroupDefinition {
    pub(crate) name: String,
    pub(crate) control_rate_hz: f64,
    pub(crate) initial_state: GroupState,
    pub(crate) motors: Arc<[Motor]>,
    pub(crate) coordinate_indices: BTreeMap<String, usize>,
}

#[derive(Debug)]
pub(crate) struct Inventory {
    pub(crate) buses: BTreeMap<String, Bus>,
    pub(crate) groups: BTreeMap<String, Arc<GroupDefinition>>,
    pub(crate) motors: BTreeMap<String, Motor>,
    pub(crate) coordinates: BTreeMap<String, String>,
    pub(crate) serial_devices: BTreeMap<String, SerialDevice>,
    pub(crate) imus: BTreeMap<String, Imu>,
    pub(crate) power_boards: BTreeMap<String, crate::PowerBoard>,
}

impl Inventory {
    pub(crate) fn from_proto(response: proto::ListHardwareResponse) -> Result<Self, Error> {
        let mut buses = BTreeMap::new();
        for bus in response.buses {
            if bus.name.trim().is_empty() || bus.transport.trim().is_empty() {
                return Err(Error::InvalidInventory(
                    "bus name and transport must be non-empty".into(),
                ));
            }
            let resolved = Bus {
                name: bus.name,
                transport: bus.transport,
                host_id: bus.host_id,
                initial_status: bus.status.map(Into::into),
            };
            let name = resolved.name.clone();
            if buses.insert(name.clone(), resolved).is_some() {
                return Err(Error::InvalidInventory(format!(
                    "bus {name:?} appears more than once"
                )));
            }
        }

        let mut serial_devices = BTreeMap::new();
        for serial in response.serial_devices {
            if serial.name.trim().is_empty() || serial.port.trim().is_empty() {
                return Err(Error::InvalidInventory(
                    "serial device name and port must be non-empty".into(),
                ));
            }
            if serial.baud_rate == 0 {
                return Err(Error::InvalidInventory(format!(
                    "serial device {:?} has zero baud rate",
                    serial.name
                )));
            }
            let resolved = SerialDevice {
                name: serial.name,
                port: serial.port,
                baud_rate: serial.baud_rate,
                initial_status: serial.status.map(Into::into),
            };
            let name = resolved.name.clone();
            if serial_devices.insert(name.clone(), resolved).is_some() {
                return Err(Error::InvalidInventory(format!(
                    "serial device {name:?} appears more than once"
                )));
            }
        }

        let mut group_info = BTreeMap::new();
        for group in response.groups {
            if group.name.trim().is_empty() {
                return Err(Error::InvalidInventory(
                    "motor group has an empty name".into(),
                ));
            }
            let name = group.name.clone();
            if group_info.insert(name.clone(), group).is_some() {
                return Err(Error::InvalidInventory(format!(
                    "motor group {name:?} appears more than once"
                )));
            }
        }

        let mut motors = BTreeMap::new();
        let mut coordinates = BTreeMap::new();
        let mut grouped: BTreeMap<String, Vec<Motor>> = BTreeMap::new();
        for motor in response.motors {
            if motor.name.trim().is_empty()
                || motor.coordinate.trim().is_empty()
                || motor.group.trim().is_empty()
            {
                return Err(Error::InvalidInventory(
                    "motor name, coordinate, and group must be non-empty".into(),
                ));
            }
            if !group_info.contains_key(&motor.group) {
                return Err(Error::InvalidInventory(format!(
                    "motor {:?} references unknown group {:?}",
                    motor.name, motor.group
                )));
            }
            let address = motor.address.ok_or_else(|| {
                Error::InvalidInventory(format!("motor {:?} has no bus address", motor.name))
            })?;
            if address.bus.trim().is_empty() {
                return Err(Error::InvalidInventory(format!(
                    "motor {:?} has an empty bus name",
                    motor.name
                )));
            }
            if !buses.contains_key(&address.bus) {
                return Err(Error::InvalidInventory(format!(
                    "motor {:?} references unknown bus {:?}",
                    motor.name, address.bus
                )));
            }
            let raw_limits = motor.limits.ok_or_else(|| {
                Error::InvalidInventory(format!(
                    "motor {:?} has no resolved command limits",
                    motor.name
                ))
            })?;
            let limits = MotorLimits::from_proto(raw_limits).map_err(|detail| {
                Error::InvalidInventory(format!("motor {:?} has {detail}", motor.name))
            })?;
            if coordinates
                .insert(motor.coordinate.clone(), motor.name.clone())
                .is_some()
            {
                return Err(Error::InvalidInventory(format!(
                    "coordinate {:?} is assigned to multiple motors",
                    motor.coordinate
                )));
            }
            let resolved = Motor {
                name: motor.name.clone(),
                coordinate: motor.coordinate,
                group: motor.group.clone(),
                bus: address.bus,
                id: address.id,
                model: motor.model,
                direction: motor.direction,
                limits,
                initial_status: motor.status.map(Into::into),
            };
            if motors
                .insert(resolved.name.clone(), resolved.clone())
                .is_some()
            {
                return Err(Error::InvalidInventory(format!(
                    "motor {:?} appears more than once",
                    resolved.name
                )));
            }
            grouped.entry(motor.group).or_default().push(resolved);
        }

        let mut groups = BTreeMap::new();
        for (name, info) in group_info {
            if !info.control_rate_hz.is_finite() || info.control_rate_hz <= 0.0 {
                return Err(Error::InvalidInventory(format!(
                    "motor group {name:?} has invalid control rate {}",
                    info.control_rate_hz
                )));
            }
            let mut members = grouped.remove(&name).unwrap_or_default();
            if members.is_empty() {
                return Err(Error::InvalidInventory(format!(
                    "motor group {name:?} has no motors"
                )));
            }
            members.sort_by(|left, right| left.name.cmp(&right.name));

            let reported_count = info.motors.len();
            let reported: BTreeSet<_> = info
                .motors
                .into_iter()
                .map(|address| (address.bus, address.id))
                .collect();
            let resolved: BTreeSet<_> = members
                .iter()
                .map(|motor| (motor.bus.clone(), motor.id))
                .collect();
            if reported_count != members.len() || reported != resolved {
                return Err(Error::InvalidInventory(format!(
                    "motor group {name:?} address membership disagrees with motor inventory"
                )));
            }

            let coordinate_indices = members
                .iter()
                .enumerate()
                .map(|(index, motor)| (motor.coordinate.clone(), index))
                .collect();
            groups.insert(
                name.clone(),
                Arc::new(GroupDefinition {
                    name,
                    control_rate_hz: info.control_rate_hz,
                    initial_state: GroupState::from_raw(info.state),
                    motors: members.into(),
                    coordinate_indices,
                }),
            );
        }

        let mut imus = BTreeMap::new();
        for imu in response.imus {
            if imu.name.trim().is_empty()
                || imu.serial_device.trim().is_empty()
                || imu.source_frame.trim().is_empty()
                || imu.output_frame.trim().is_empty()
            {
                return Err(Error::InvalidInventory(
                    "IMU name, serial device, source frame, and output frame must be non-empty"
                        .into(),
                ));
            }
            if !serial_devices.contains_key(&imu.serial_device) {
                return Err(Error::InvalidInventory(format!(
                    "IMU {:?} references unknown serial device {:?}",
                    imu.name, imu.serial_device
                )));
            }
            let resolved = Imu {
                name: imu.name,
                serial_device: imu.serial_device,
                source_frame: imu.source_frame,
                output_frame: imu.output_frame,
                initial_status: imu.status.map(Into::into),
            };
            let name = resolved.name.clone();
            if imus.insert(name.clone(), resolved).is_some() {
                return Err(Error::InvalidInventory(format!(
                    "IMU {name:?} appears more than once"
                )));
            }
        }
        let mut power_boards = BTreeMap::new();
        let mut claimed: std::collections::BTreeSet<_> =
            imus.values().map(|imu| imu.serial_device.clone()).collect();
        for board in response.power_boards {
            if board.name.trim().is_empty()
                || !serial_devices.contains_key(&board.serial_device)
                || !claimed.insert(board.serial_device.clone())
                || !matches!(board.profile.as_str(), "agx" | "nano_nx")
                || board.stale_timeout_ms == 0
            {
                return Err(Error::InvalidInventory(
                    "invalid power board or serial ownership".into(),
                ));
            }
            let name = board.name.clone();
            let value = crate::PowerBoard {
                name: board.name,
                serial_device: board.serial_device,
                profile: board.profile,
                stale_timeout: std::time::Duration::from_millis(board.stale_timeout_ms),
                sync_period: std::time::Duration::from_millis(board.sync_period_ms),
                initial_status: board.status.map(Into::into),
            };
            if power_boards.insert(name, value).is_some() {
                return Err(Error::InvalidInventory("duplicate power board".into()));
            }
        }
        Ok(Self {
            buses,
            groups,
            motors,
            coordinates,
            serial_devices,
            imus,
            power_boards,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn inventory() -> proto::ListHardwareResponse {
        proto::ListHardwareResponse {
            power_boards: vec![],
            buses: vec![proto::BusInfo {
                name: "can0".into(),
                transport: "socketcan:vcan0".into(),
                host_id: 127,
                status: Some(proto::DeviceStatus {
                    state: proto::DeviceState::Online as i32,
                    detail: "online".into(),
                    transition_count: 3,
                }),
            }],
            groups: vec![proto::GroupInfo {
                name: "arms".into(),
                control_rate_hz: 250.0,
                state: proto::GroupState::Idle as i32,
                motors: vec![proto::MotorAddress {
                    bus: "can0".into(),
                    id: 1,
                }],
            }],
            motors: vec![proto::MotorInfo {
                name: "shoulder_motor".into(),
                coordinate: "shoulder".into(),
                group: "arms".into(),
                address: Some(proto::MotorAddress {
                    bus: "can0".into(),
                    id: 1,
                }),
                model: "RS_03".into(),
                direction: 1,
                status: None,
                limits: Some(proto::MotorLimits {
                    position_min: -1.0,
                    position_max: 1.0,
                    velocity_min: -2.0,
                    velocity_max: 2.0,
                    torque_min: -3.0,
                    torque_max: 3.0,
                    kp_min: 0.0,
                    kp_max: 100.0,
                    kd_min: 0.0,
                    kd_max: 10.0,
                }),
            }],
            serial_devices: vec![proto::SerialDeviceInfo {
                name: "imu_serial".into(),
                port: "/dev/ttyUSB0".into(),
                baud_rate: 921_600,
                status: None,
            }],
            imus: vec![proto::ImuInfo {
                name: "body".into(),
                serial_device: "imu_serial".into(),
                status: None,
                source_frame: "imu_sensor_link".into(),
                output_frame: "imu_link".into(),
            }],
        }
    }

    #[test]
    fn resolves_group_membership_by_coordinate() {
        let inventory = Inventory::from_proto(inventory()).unwrap();
        let group = inventory.groups.get("arms").unwrap();
        assert_eq!(group.motors[0].coordinate(), "shoulder");
        assert_eq!(group.coordinate_indices["shoulder"], 0);
        assert_eq!(group.motors[0].limits().torque_max(), 3.0);
        let bus = &inventory.buses["can0"];
        assert_eq!(bus.transport(), "socketcan:vcan0");
        assert_eq!(bus.host_id(), 127);
        assert_eq!(
            bus.initial_status().map(DeviceStatus::transition_count),
            Some(3)
        );
        assert_eq!(inventory.serial_devices["imu_serial"].baud_rate(), 921_600);
        assert_eq!(inventory.imus["body"].serial_device(), "imu_serial");
        assert_eq!(inventory.imus["body"].source_frame(), "imu_sensor_link");
        assert_eq!(inventory.imus["body"].output_frame(), "imu_link");
    }

    #[test]
    fn rejects_disagreement_between_group_and_motor_inventory() {
        let mut inventory = inventory();
        inventory.groups[0].motors[0].id = 2;
        assert!(matches!(
            Inventory::from_proto(inventory),
            Err(Error::InvalidInventory(_))
        ));
    }

    #[test]
    fn rejects_unknown_bus_and_serial_references() {
        let mut unknown_bus = inventory();
        unknown_bus.motors[0].address.as_mut().unwrap().bus = "missing".into();
        assert!(matches!(
            Inventory::from_proto(unknown_bus),
            Err(Error::InvalidInventory(detail)) if detail.contains("unknown bus")
        ));

        let mut unknown_serial = inventory();
        unknown_serial.imus[0].serial_device = "missing".into();
        assert!(matches!(
            Inventory::from_proto(unknown_serial),
            Err(Error::InvalidInventory(detail)) if detail.contains("unknown serial device")
        ));
    }

    #[test]
    fn rejects_missing_imu_frames() {
        let mut missing_source = inventory();
        missing_source.imus[0].source_frame.clear();
        assert!(matches!(
            Inventory::from_proto(missing_source),
            Err(Error::InvalidInventory(detail)) if detail.contains("source frame")
        ));

        let mut missing_output = inventory();
        missing_output.imus[0].output_frame.clear();
        assert!(matches!(
            Inventory::from_proto(missing_output),
            Err(Error::InvalidInventory(detail)) if detail.contains("output frame")
        ));
    }

    #[test]
    fn rejects_missing_or_invalid_motor_limits() {
        let mut missing = inventory();
        missing.motors[0].limits = None;
        assert!(matches!(
            Inventory::from_proto(missing),
            Err(Error::InvalidInventory(detail)) if detail.contains("no resolved command limits")
        ));

        let mut invalid = inventory();
        invalid.motors[0].limits.as_mut().unwrap().kp_max = 0.0;
        assert!(matches!(
            Inventory::from_proto(invalid),
            Err(Error::InvalidInventory(detail)) if detail.contains("invalid kp limits")
        ));
    }
}
