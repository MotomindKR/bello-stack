use std::time::Duration;

use tonic::Status;

use crate::config::{CommandBehavior, FaultResponse};
use crate::control::{CommandPolicyCheck, GroupCause, GroupState};
use crate::domain::{
    CommandBatch, CommandError, DeviceState, DeviceStatus, ImpedanceCommand, LeaseToken,
    MotorCommand, MotorName, TelemetryEvent, TelemetryPayload,
};
use bello_hardware_api as proto;

pub fn lease_token(value: &str) -> Result<LeaseToken, Status> {
    value
        .parse::<LeaseToken>()
        .map_err(|error| Status::invalid_argument(error.to_string()))
}

pub fn command_batch(value: proto::StreamCommandsRequest) -> Result<CommandBatch, Status> {
    command_batch_fields(
        value.group,
        value.lease_token,
        value.sequence,
        value.valid_for_ms,
        value.commands,
    )
}

pub fn start_command_batch(value: proto::StartControlRequest) -> Result<CommandBatch, Status> {
    command_batch_fields(
        value.group,
        value.lease_token,
        0,
        value.valid_for_ms,
        value.commands,
    )
}

fn command_batch_fields(
    group: String,
    lease_token_value: String,
    sequence: u64,
    valid_for_ms: u64,
    commands: Vec<proto::MotorCommand>,
) -> Result<CommandBatch, Status> {
    Ok(CommandBatch {
        group: group.into(),
        lease_token: lease_token(&lease_token_value)?,
        sequence,
        valid_for: (valid_for_ms != 0).then(|| Duration::from_millis(valid_for_ms)),
        commands: commands
            .into_iter()
            .map(|command| MotorCommand {
                motor: MotorName::from(command.motor_name),
                impedance: ImpedanceCommand {
                    position: command.position,
                    velocity: command.velocity,
                    torque: command.torque_feedforward,
                    kp: command.kp,
                    kd: command.kd,
                },
            })
            .collect(),
    })
}

pub fn device_status(value: &DeviceStatus) -> proto::DeviceStatus {
    proto::DeviceStatus {
        state: match value.state {
            DeviceState::Starting => proto::DeviceState::Starting,
            DeviceState::Connecting => proto::DeviceState::Connecting,
            DeviceState::Online => proto::DeviceState::Online,
            DeviceState::Degraded => proto::DeviceState::Degraded,
            DeviceState::Reconnecting => proto::DeviceState::Reconnecting,
            DeviceState::Fault => proto::DeviceState::Fault,
            DeviceState::Shutdown => proto::DeviceState::Shutdown,
        } as i32,
        detail: value.detail.clone(),
        transition_count: value.transition_count,
    }
}

pub fn group_state(value: GroupState) -> i32 {
    (match value {
        GroupState::Idle => proto::GroupState::Idle,
        GroupState::Damped => proto::GroupState::Damped,
        GroupState::Running => proto::GroupState::Running,
        GroupState::Fault => proto::GroupState::Fault,
        GroupState::Shutdown => proto::GroupState::Shutdown,
    }) as i32
}

pub fn command_result(value: &CommandError) -> i32 {
    (match value {
        CommandError::UnknownGroup => proto::CommandResult::UnknownGroup,
        CommandError::LeaseLost => proto::CommandResult::LeaseLost,
        CommandError::InvalidState => proto::CommandResult::InvalidState,
        CommandError::IncompleteGroup => proto::CommandResult::IncompleteGroup,
        CommandError::StaleSequence { .. } => proto::CommandResult::StaleSequence,
        CommandError::Unsafe { .. } => proto::CommandResult::Unsafe,
        CommandError::Superseded => proto::CommandResult::Superseded,
        CommandError::Expired => proto::CommandResult::Expired,
        CommandError::ControllerStopped => proto::CommandResult::ControllerStopped,
    }) as i32
}

fn command_check(value: CommandPolicyCheck) -> i32 {
    (match value {
        CommandPolicyCheck::CommandLimit => proto::CommandPolicyCheck::Limit,
        CommandPolicyCheck::PositionStep => proto::CommandPolicyCheck::PositionStep,
    }) as i32
}

fn command_behavior(value: CommandBehavior) -> i32 {
    (match value {
        CommandBehavior::Clamp => proto::CommandBehavior::Clamp,
        CommandBehavior::Reject => proto::CommandBehavior::Reject,
        CommandBehavior::Fault => proto::CommandBehavior::Fault,
    }) as i32
}

fn fault_response(value: Option<FaultResponse>) -> i32 {
    (match value {
        None => proto::FaultResponse::Unspecified,
        Some(FaultResponse::Damping) => proto::FaultResponse::Damping,
        Some(FaultResponse::Disable) => proto::FaultResponse::Disable,
    }) as i32
}

pub fn group_cause(value: GroupCause) -> i32 {
    (match value {
        GroupCause::Unspecified => proto::GroupCause::Unspecified,
        GroupCause::Enable => proto::GroupCause::Enable,
        GroupCause::Start => proto::GroupCause::Start,
        GroupCause::Stop => proto::GroupCause::Stop,
        GroupCause::CommandTimeout => proto::GroupCause::CommandTimeout,
        GroupCause::Disable => proto::GroupCause::Disable,
        GroupCause::Recover => proto::GroupCause::Recover,
        GroupCause::CommandLimit => proto::GroupCause::CommandLimit,
        GroupCause::CommandPositionStep => proto::GroupCause::CommandPositionStep,
        GroupCause::SafePosition => proto::GroupCause::SafePosition,
        GroupCause::StaleFeedback => proto::GroupCause::StaleFeedback,
        GroupCause::Temperature => proto::GroupCause::Temperature,
        GroupCause::ReportedMotorFault => proto::GroupCause::ReportedMotorFault,
        GroupCause::ModeStatus => proto::GroupCause::ModeStatus,
        GroupCause::Communication => proto::GroupCause::Communication,
        GroupCause::Transmission => proto::GroupCause::Transmission,
        GroupCause::LeaseLoss => proto::GroupCause::LeaseLoss,
        GroupCause::Exit => proto::GroupCause::Exit,
        GroupCause::Internal => proto::GroupCause::Internal,
    }) as i32
}

pub fn quaternion(value: [f64; 4]) -> proto::Quaternion {
    proto::Quaternion {
        x: value[0],
        y: value[1],
        z: value[2],
        w: value[3],
    }
}

pub fn vector3(value: [f64; 3]) -> proto::Vector3 {
    proto::Vector3 {
        x: value[0],
        y: value[1],
        z: value[2],
    }
}

pub fn telemetry_event(
    value: TelemetryEvent,
    observed_monotonic_ns: u64,
) -> proto::WatchTelemetryResponse {
    let transport_age_ms = observed_monotonic_ns.saturating_sub(value.monotonic_ns) / 1_000_000;
    let payload = match value.payload {
        TelemetryPayload::Motor(value) => {
            proto::watch_telemetry_response::Payload::Motor(proto::MotorTelemetry {
                group: value.group.to_string(),
                motor_name: value.motor.to_string(),
                position: value.position,
                velocity: value.velocity,
                torque: value.torque,
                temperature: value.temperature,
                fault_bits: value.fault_bits as u32,
                mode_status: value.mode_status as u32,
                feedback_age_ms: value.feedback_age_ms.saturating_add(transport_age_ms),
            })
        }
        TelemetryPayload::Imu(value) => {
            proto::watch_telemetry_response::Payload::Imu(proto::ImuTelemetry {
                imu_name: value.imu.to_string(),
                orientation: Some(quaternion(value.orientation_xyzw)),
                angular_velocity: Some(vector3(value.angular_velocity_xyz)),
                linear_acceleration: Some(vector3(value.linear_acceleration_xyz)),
                device_timestamp_ms: value.device_timestamp_ms,
                sample_age_ms: value.sample_age_ms.saturating_add(transport_age_ms),
            })
        }
        TelemetryPayload::Group(value) => {
            proto::watch_telemetry_response::Payload::Group(proto::GroupTelemetry {
                group: value.group.to_string(),
                state: group_state(value.state),
                command_count: value.command_count,
                control_tick: value.control_tick,
                jitter_us: value.jitter_us,
                detail: value.detail,
                cause: group_cause(value.cause),
            })
        }
        TelemetryPayload::CommandPolicy(value) => {
            proto::watch_telemetry_response::Payload::CommandPolicy(proto::CommandPolicyTelemetry {
                group: value.group.to_string(),
                motor_name: value.motor.to_string(),
                check: command_check(value.check),
                behavior: command_behavior(value.behavior),
                detail: value.detail,
                command_modified: value.command_modified,
                command_rejected: value.command_rejected,
                fault_response: fault_response(value.response),
            })
        }
        TelemetryPayload::Health(value) => {
            proto::watch_telemetry_response::Payload::Health(proto::HealthTelemetry {
                component: value.component,
                device_status: Some(device_status(&value.status)),
                dropped_events: value.dropped_events,
            })
        }
    };
    proto::WatchTelemetryResponse {
        monotonic_ns: value.monotonic_ns,
        payload: Some(payload),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::domain::{GroupName, ImuName, ImuTelemetry, MotorTelemetry};

    #[test]
    fn telemetry_age_includes_time_waiting_for_the_subscriber() {
        let response = telemetry_event(
            TelemetryEvent {
                monotonic_ns: 1_000_000,
                payload: TelemetryPayload::Motor(MotorTelemetry {
                    group: GroupName::from("group"),
                    motor: MotorName::from("motor"),
                    position: 0.0,
                    velocity: 0.0,
                    torque: 0.0,
                    temperature: 25.0,
                    fault_bits: 0,
                    mode_status: 0,
                    feedback_age_ms: 2,
                }),
            },
            6_000_000,
        );
        let Some(proto::watch_telemetry_response::Payload::Motor(motor)) = response.payload else {
            panic!("expected motor telemetry");
        };
        assert_eq!(motor.feedback_age_ms, 7);
    }

    #[test]
    fn imu_arrays_have_fixed_shape_on_the_wire() {
        let response = telemetry_event(
            TelemetryEvent {
                monotonic_ns: 1,
                payload: TelemetryPayload::Imu(ImuTelemetry {
                    imu: ImuName::from("body"),
                    orientation_xyzw: [1.0, 2.0, 3.0, 4.0],
                    angular_velocity_xyz: [5.0, 6.0, 7.0],
                    linear_acceleration_xyz: [8.0, 9.0, 10.0],
                    device_timestamp_ms: 11,
                    sample_age_ms: 12,
                }),
            },
            1,
        );
        let Some(proto::watch_telemetry_response::Payload::Imu(imu)) = response.payload else {
            panic!("expected IMU telemetry");
        };
        assert_eq!(imu.orientation.unwrap().w, 4.0);
        assert_eq!(imu.angular_velocity.unwrap().z, 7.0);
        assert_eq!(imu.linear_acceleration.unwrap().x, 8.0);
    }

    #[test]
    fn group_causes_are_typed_on_the_wire() {
        assert_eq!(
            group_cause(GroupCause::Temperature),
            proto::GroupCause::Temperature as i32
        );
        assert_eq!(
            group_cause(GroupCause::Unspecified),
            proto::GroupCause::Unspecified as i32
        );
    }
}
