use crate::app::HardwareDaemon;
use bello_hardware_api::{
    BusInfo, GroupInfo, ImuInfo, ListHardwareResponse, MotorAddress, MotorInfo, MotorLimits,
    SerialDeviceInfo,
};

use super::conversion;

pub fn list(daemon: &HardwareDaemon) -> ListHardwareResponse {
    let topology = &daemon.topology;
    let mut buses: Vec<_> = topology
        .buses()
        .map(|(_, bus)| BusInfo {
            name: bus.name.to_string(),
            transport: bus.transport.label(),
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
            coordinate: motor.spec.coordinate.clone(),
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
            limits: Some(MotorLimits {
                position_min: motor.spec.limits.position_min,
                position_max: motor.spec.limits.position_max,
                velocity_min: motor.spec.limits.velocity_min,
                velocity_max: motor.spec.limits.velocity_max,
                torque_min: motor.spec.limits.torque_min,
                torque_max: motor.spec.limits.torque_max,
                kp_min: motor.spec.limits.kp_min,
                kp_max: motor.spec.limits.kp_max,
                kd_min: motor.spec.limits.kd_min,
                kd_max: motor.spec.limits.kd_max,
            }),
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
            source_frame: imu.source_frame.clone(),
            output_frame: imu.output_frame.clone(),
        })
        .collect();
    imus.sort_by(|left, right| left.name.cmp(&right.name));

    let mut power_boards: Vec<_> = topology
        .power_boards()
        .map(|(name, spec)| bello_hardware_api::PowerBoardInfo {
            name: name.to_string(),
            serial_device: spec.serial_device.to_string(),
            profile: spec.profile.name().into(),
            stale_timeout_ms: spec.stale_timeout.as_millis() as u64,
            sync_period_ms: spec.sync_period.as_millis() as u64,
            status: daemon
                .power_workers
                .lock()
                .get(name)
                .map(|worker| conversion::device_status(&worker.lifecycle.status())),
        })
        .collect();
    power_boards.sort_by(|a, b| a.name.cmp(&b.name));

    ListHardwareResponse {
        buses,
        motors,
        groups,
        serial_devices,
        imus,
        power_boards,
    }
}
