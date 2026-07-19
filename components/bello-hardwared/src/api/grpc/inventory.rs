use crate::app::HardwareDaemon;
use crate::config::{CommandLimitAction, CommandViolationSeverity, FaultResponse};
use crate::proto::{
    BusInfo, FaultResponseInfo, GroupInfo, ImuInfo, ListHardwareResponse, MotorAddress,
    MotorCommandPolicyInfo, MotorFaultDampingInfo, MotorHealthInfo, MotorInfo, SerialDeviceInfo,
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
            health: Some(MotorHealthInfo {
                safe_position_min: motor.spec.health.safe_position_min,
                safe_position_max: motor.spec.health.safe_position_max,
                temperature_warning_c: motor.spec.health.temperature_warning_c,
                maximum_temperature_c: motor.spec.health.maximum_temperature_c,
                temperature_recovery_c: motor.spec.health.temperature_recovery_c,
                temperature_recovery_duration_ms: motor
                    .spec
                    .health
                    .temperature_recovery_duration_ms,
            }),
            command_policy: Some(MotorCommandPolicyInfo {
                maximum_position_step: motor.spec.command_policy.maximum_position_step,
                limit_action: limit_action_name(motor.spec.command_policy.limit_action).into(),
                limit_severity: severity_name(motor.spec.command_policy.limit_severity).into(),
                position_step_action: limit_action_name(
                    motor.spec.command_policy.position_step_action,
                )
                .into(),
                position_step_severity: severity_name(
                    motor.spec.command_policy.position_step_severity,
                )
                .into(),
            }),
            fault_damping: Some(MotorFaultDampingInfo {
                kd: motor.spec.fault_damping.kd,
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
                .map(|handle| handle.snapshot.read().state.as_str().into())
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
            fault_responses: fault_responses(group),
            damping_send_count: group.damping_send_count,
            damping_send_rate_hz: 1.0 / group.damping_send_period.as_secs_f64(),
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
            model: imu.model.clone(),
            status: daemon
                .imu(&imu.name)
                .map(|state| conversion::device_status(&state.lifecycle.status())),
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

fn limit_action_name(action: CommandLimitAction) -> &'static str {
    match action {
        CommandLimitAction::Clamp => "clamp",
        CommandLimitAction::Reject => "reject",
    }
}

fn severity_name(severity: CommandViolationSeverity) -> &'static str {
    match severity {
        CommandViolationSeverity::Warning => "warning",
        CommandViolationSeverity::Fault => "fault",
    }
}

fn response_name(response: FaultResponse) -> &'static str {
    match response {
        FaultResponse::None => "none",
        FaultResponse::Damping => "damping",
        FaultResponse::Disable => "disable",
    }
}

fn fault_responses(group: &crate::control::GroupSpec) -> Vec<FaultResponseInfo> {
    let responses = &group.fault_responses;
    [
        ("command_limit", responses.command_limit),
        ("command_position_step", responses.command_position_step),
        ("command_timeout", responses.command_timeout),
        ("safe_position", responses.safe_position),
        ("stale_feedback", responses.stale_feedback),
        ("temperature", responses.temperature),
        ("reported_motor_fault", responses.reported_motor_fault),
        ("mode_status", responses.mode_status),
        ("communication", responses.communication),
        ("enable", responses.enable),
        ("start", responses.start),
        ("stop", responses.stop),
        ("transmission", responses.transmission),
        ("lease_loss", responses.lease_loss),
        ("manual", responses.manual),
        ("disable", responses.disable),
        ("exit", responses.exit),
    ]
    .into_iter()
    .map(|(cause, response)| FaultResponseInfo {
        cause: cause.into(),
        response: response_name(response).into(),
    })
    .collect()
}
