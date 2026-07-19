use crate::app::HardwareDaemon;
use crate::proto::{
    BusInfo, GroupInfo, ImuInfo, ListHardwareResponse, MotorAddress, MotorInfo, SerialDeviceInfo,
};

use super::conversion;

pub fn list(daemon: &HardwareDaemon) -> ListHardwareResponse {
    let topology = &daemon.topology;
    let mut buses: Vec<_> = topology
        .buses()
        .map(|(_, bus)| BusInfo {
            name: bus.name.to_string(),
            interface: bus.interface.clone(),
            host_id: bus.host_id as u32,
            status: daemon
                .bus_status(&bus.name)
                .as_ref()
                .map(conversion::device_status),
        })
        .collect();
    buses.sort_by(|left, right| left.name.cmp(&right.name));

    let mut motors: Vec<_> = topology
        .motors()
        .map(|(_, motor)| MotorInfo {
            name: motor.spec.name.to_string(),
            joint: motor.spec.joint.clone(),
            group: motor.group.to_string(),
            address: Some(MotorAddress {
                bus: motor.spec.address.bus.to_string(),
                id: motor.spec.address.id as u32,
            }),
            model: motor.spec.model.clone(),
            direction: motor.spec.params.direction as i32,
            status: daemon
                .motor_status(&motor.spec.name)
                .as_ref()
                .map(conversion::device_status),
        })
        .collect();
    motors.sort_by(|left, right| left.name.cmp(&right.name));

    let mut groups: Vec<_> = topology
        .groups()
        .map(|(_, group)| GroupInfo {
            name: group.name.to_string(),
            control_rate_hz: group.control_rate_hz,
            state: daemon
                .groups
                .get(&group.name)
                .map(|handle| conversion::group_state(handle.snapshot.read().state))
                .unwrap_or_default(),
            motors: group
                .motors
                .iter()
                .filter_map(|name| topology.motor(name).ok())
                .map(|motor| MotorAddress {
                    bus: motor.spec.address.bus.to_string(),
                    id: motor.spec.address.id as u32,
                })
                .collect(),
        })
        .collect();
    groups.sort_by(|left, right| left.name.cmp(&right.name));

    let mut serial_devices: Vec<_> = topology
        .serial_devices()
        .map(|(_, serial)| SerialDeviceInfo {
            name: serial.name.to_string(),
            port: serial.port.clone(),
            baud_rate: serial.baud_rate,
            status: daemon
                .serial_status(&serial.name)
                .as_ref()
                .map(conversion::device_status),
        })
        .collect();
    serial_devices.sort_by(|left, right| left.name.cmp(&right.name));

    let mut imus: Vec<_> = topology
        .imus()
        .map(|(_, imu)| ImuInfo {
            name: imu.name.to_string(),
            serial_device: imu.serial_device.to_string(),
            status: daemon
                .imu(&imu.name)
                .map(|handle| conversion::device_status(&handle.lifecycle.status())),
        })
        .collect();
    imus.sort_by(|left, right| left.name.cmp(&right.name));

    ListHardwareResponse {
        buses,
        motors,
        groups,
        serial_devices,
        imus,
    }
}
