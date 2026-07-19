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

/// Configured actuator metadata resolved from daemon inventory.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Motor {
    pub(crate) name: String,
    pub(crate) joint: String,
    pub(crate) group: String,
    pub(crate) bus: String,
    pub(crate) id: u32,
    pub(crate) model: String,
    pub(crate) direction: i32,
    pub(crate) initial_status: Option<DeviceStatus>,
}

impl Motor {
    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn joint(&self) -> &str {
        &self.joint
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
    pub(crate) joint_indices: BTreeMap<String, usize>,
}

#[derive(Debug)]
pub(crate) struct Inventory {
    pub(crate) groups: BTreeMap<String, Arc<GroupDefinition>>,
    pub(crate) motors: BTreeMap<String, Motor>,
    pub(crate) joints: BTreeMap<String, String>,
    pub(crate) imus: BTreeSet<String>,
}

impl Inventory {
    pub(crate) fn from_proto(response: proto::ListHardwareResponse) -> Result<Self, Error> {
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
        let mut joints = BTreeMap::new();
        let mut grouped: BTreeMap<String, Vec<Motor>> = BTreeMap::new();
        for motor in response.motors {
            if motor.name.trim().is_empty()
                || motor.joint.trim().is_empty()
                || motor.group.trim().is_empty()
            {
                return Err(Error::InvalidInventory(
                    "motor name, joint, and group must be non-empty".into(),
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
            if joints
                .insert(motor.joint.clone(), motor.name.clone())
                .is_some()
            {
                return Err(Error::InvalidInventory(format!(
                    "joint {:?} is assigned to multiple motors",
                    motor.joint
                )));
            }
            let resolved = Motor {
                name: motor.name.clone(),
                joint: motor.joint,
                group: motor.group.clone(),
                bus: address.bus,
                id: address.id,
                model: motor.model,
                direction: motor.direction,
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

            let joint_indices = members
                .iter()
                .enumerate()
                .map(|(index, motor)| (motor.joint.clone(), index))
                .collect();
            groups.insert(
                name.clone(),
                Arc::new(GroupDefinition {
                    name,
                    control_rate_hz: info.control_rate_hz,
                    initial_state: GroupState::from_raw(info.state),
                    motors: members.into(),
                    joint_indices,
                }),
            );
        }

        let mut imus = BTreeSet::new();
        for imu in response.imus {
            if imu.name.trim().is_empty() {
                return Err(Error::InvalidInventory("IMU has an empty name".into()));
            }
            if !imus.insert(imu.name.clone()) {
                return Err(Error::InvalidInventory(format!(
                    "IMU {:?} appears more than once",
                    imu.name
                )));
            }
        }
        Ok(Self {
            groups,
            motors,
            joints,
            imus,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn inventory() -> proto::ListHardwareResponse {
        proto::ListHardwareResponse {
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
                joint: "shoulder".into(),
                group: "arms".into(),
                address: Some(proto::MotorAddress {
                    bus: "can0".into(),
                    id: 1,
                }),
                model: "ROBSTRIDE_03".into(),
                direction: 1,
                status: None,
            }],
            ..Default::default()
        }
    }

    #[test]
    fn resolves_group_membership_by_joint() {
        let inventory = Inventory::from_proto(inventory()).unwrap();
        let group = inventory.groups.get("arms").unwrap();
        assert_eq!(group.motors[0].joint(), "shoulder");
        assert_eq!(group.joint_indices["shoulder"], 0);
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
}
