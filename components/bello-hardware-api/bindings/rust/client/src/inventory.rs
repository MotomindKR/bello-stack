use std::collections::{BTreeMap, btree_map::Entry};
use std::sync::Arc;

use bello_hardware_api as proto;
pub use proto::{BusInfo, ImuInfo, MotorLimits, SerialDeviceInfo};

use crate::{DeviceState, EnumValue, Error, GroupState};

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DeviceStatus {
    state: EnumValue<DeviceState>,
    detail: String,
    transition_count: u64,
}

impl DeviceStatus {
    pub fn state(&self) -> EnumValue<DeviceState> {
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
            state: EnumValue::from_raw(status.state),
            detail: status.detail,
            transition_count: status.transition_count,
        }
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
    pub(crate) firmware_version: Option<String>,
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

    pub fn firmware_version(&self) -> Option<&str> {
        self.firmware_version.as_deref()
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

#[derive(Clone, Debug)]
pub(crate) struct GroupDefinition {
    pub(crate) name: String,
    pub(crate) control_rate_hz: f64,
    pub(crate) initial_state: EnumValue<GroupState>,
    pub(crate) motors: Arc<[Motor]>,
    pub(crate) coordinate_indices: BTreeMap<String, usize>,
}

#[derive(Debug)]
pub(crate) struct Inventory {
    pub(crate) buses: BTreeMap<String, BusInfo>,
    pub(crate) groups: BTreeMap<String, Arc<GroupDefinition>>,
    pub(crate) motors: BTreeMap<String, Motor>,
    pub(crate) coordinates: BTreeMap<String, String>,
    pub(crate) serial_devices: BTreeMap<String, SerialDeviceInfo>,
    pub(crate) imus: BTreeMap<String, ImuInfo>,
    pub(crate) power_boards: BTreeMap<String, crate::PowerBoard>,
}

fn insert_named<T>(
    index: &mut BTreeMap<String, T>,
    kind: &str,
    name: String,
    value: T,
) -> Result<(), Error> {
    if name.trim().is_empty() {
        return Err(Error::InvalidInventory(format!("{kind} has an empty name")));
    }
    match index.entry(name) {
        Entry::Vacant(entry) => {
            entry.insert(value);
            Ok(())
        }
        Entry::Occupied(entry) => Err(Error::InvalidInventory(format!(
            "duplicate {kind} {:?}",
            entry.key()
        ))),
    }
}

impl Inventory {
    pub(crate) fn from_proto(response: proto::ListHardwareResponse) -> Result<Self, Error> {
        let mut buses = BTreeMap::new();
        for bus in response.buses {
            insert_named(&mut buses, "bus", bus.name.clone(), bus)?;
        }

        let mut serial_devices = BTreeMap::new();
        for serial in response.serial_devices {
            insert_named(
                &mut serial_devices,
                "serial device",
                serial.name.clone(),
                serial,
            )?;
        }

        let mut motors = BTreeMap::new();
        let mut coordinates = BTreeMap::new();
        let mut grouped: BTreeMap<String, Vec<Motor>> = BTreeMap::new();
        for motor in response.motors {
            let address = motor.address.ok_or_else(|| {
                Error::InvalidInventory(format!("motor {:?} has no bus address", motor.name))
            })?;
            let limits = motor.limits.ok_or_else(|| {
                Error::InvalidInventory(format!(
                    "motor {:?} has no resolved command limits",
                    motor.name
                ))
            })?;
            insert_named(
                &mut coordinates,
                "coordinate",
                motor.coordinate.clone(),
                motor.name.clone(),
            )?;
            let resolved = Motor {
                name: motor.name,
                coordinate: motor.coordinate,
                group: motor.group.clone(),
                bus: address.bus,
                id: address.id,
                model: motor.model,
                firmware_version: motor.firmware_version,
                direction: motor.direction,
                limits,
                initial_status: motor.status.map(Into::into),
            };
            insert_named(
                &mut motors,
                "motor",
                resolved.name.clone(),
                resolved.clone(),
            )?;
            grouped.entry(motor.group).or_default().push(resolved);
        }

        let mut groups = BTreeMap::new();
        for group in response.groups {
            let mut members = grouped.remove(&group.name).unwrap_or_default();
            members.sort_by(|left, right| left.name.cmp(&right.name));
            let coordinate_indices = members
                .iter()
                .enumerate()
                .map(|(index, motor)| (motor.coordinate.clone(), index))
                .collect();
            insert_named(
                &mut groups,
                "group",
                group.name.clone(),
                Arc::new(GroupDefinition {
                    name: group.name,
                    control_rate_hz: group.control_rate_hz,
                    initial_state: EnumValue::from_raw(group.state),
                    motors: members.into(),
                    coordinate_indices,
                }),
            )?;
        }
        if let Some(name) = grouped.keys().next() {
            return Err(Error::InvalidInventory(format!(
                "motor references unknown group {name:?}"
            )));
        }

        let mut imus = BTreeMap::new();
        for imu in response.imus {
            insert_named(&mut imus, "IMU", imu.name.clone(), imu)?;
        }
        let mut power_boards = BTreeMap::new();
        for board in response.power_boards {
            insert_named(
                &mut power_boards,
                "power board",
                board.name.clone(),
                crate::PowerBoard {
                    name: board.name,
                    serial_device: board.serial_device,
                    profile: board.profile,
                    stale_timeout: std::time::Duration::from_millis(board.stale_timeout_ms),
                    sync_period: std::time::Duration::from_millis(board.sync_period_ms),
                    initial_status: board.status.map(Into::into),
                },
            )?;
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
        let motor = |name: &str, coordinate: &str| proto::MotorInfo {
            name: name.into(),
            coordinate: coordinate.into(),
            group: "arms".into(),
            address: Some(proto::MotorAddress::default()),
            limits: Some(proto::MotorLimits::default()),
            ..Default::default()
        };
        proto::ListHardwareResponse {
            groups: vec![proto::GroupInfo {
                name: "arms".into(),
                ..Default::default()
            }],
            motors: vec![
                motor("shoulder_motor", "shoulder"),
                motor("elbow_motor", "elbow"),
            ],
            ..Default::default()
        }
    }

    #[test]
    fn resolves_coordinates_after_sorting_group_members() {
        let inventory = Inventory::from_proto(inventory()).unwrap();
        let group = &inventory.groups["arms"];
        assert_eq!(group.motors[0].name(), "elbow_motor");
        assert_eq!(
            group.motors[group.coordinate_indices["shoulder"]].name(),
            "shoulder_motor"
        );
    }

    #[test]
    fn rejects_missing_motor_limits() {
        let mut missing = inventory();
        missing.motors[0].limits = None;
        assert!(matches!(
            Inventory::from_proto(missing),
            Err(Error::InvalidInventory(detail)) if detail.contains("no resolved command limits")
        ));
    }
}
