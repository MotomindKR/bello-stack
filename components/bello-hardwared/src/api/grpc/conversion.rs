use std::time::Duration;

use tonic::Status;

use crate::config::{CommandBehavior, FaultCause, FaultResponse};
use crate::control::{CommandPolicyCheck, GroupState};
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
    Ok(CommandBatch {
        group: value.group.into(),
        lease_token: lease_token(&value.lease_token)?,
        sequence: value.sequence,
        valid_for: (value.valid_for_ms != 0).then(|| Duration::from_millis(value.valid_for_ms)),
        commands: value
            .commands
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
        GroupState::Enabled => proto::GroupState::Enabled,
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

fn fault_cause(value: Option<FaultCause>) -> i32 {
    (match value {
        None => proto::FaultCause::Unspecified,
        Some(FaultCause::CommandLimit) => proto::FaultCause::CommandLimit,
        Some(FaultCause::CommandPositionStep) => proto::FaultCause::CommandPositionStep,
        Some(FaultCause::CommandTimeout) => proto::FaultCause::CommandTimeout,
        Some(FaultCause::SafePosition) => proto::FaultCause::SafePosition,
        Some(FaultCause::StaleFeedback) => proto::FaultCause::StaleFeedback,
        Some(FaultCause::Temperature) => proto::FaultCause::Temperature,
        Some(FaultCause::ReportedMotorFault) => proto::FaultCause::ReportedMotorFault,
        Some(FaultCause::ModeStatus) => proto::FaultCause::ModeStatus,
        Some(FaultCause::Communication) => proto::FaultCause::Communication,
        Some(FaultCause::Enable) => proto::FaultCause::Enable,
        Some(FaultCause::Start) => proto::FaultCause::Start,
        Some(FaultCause::Stop) => proto::FaultCause::Stop,
        Some(FaultCause::Transmission) => proto::FaultCause::Transmission,
        Some(FaultCause::LeaseLoss) => proto::FaultCause::LeaseLoss,
        Some(FaultCause::Disable) => proto::FaultCause::Disable,
        Some(FaultCause::Recover) => proto::FaultCause::Recover,
        Some(FaultCause::Exit) => proto::FaultCause::Exit,
        Some(FaultCause::Internal) => proto::FaultCause::Internal,
    }) as i32
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
                orientation: Some(proto::Quaternion {
                    x: value.orientation_xyzw[0],
                    y: value.orientation_xyzw[1],
                    z: value.orientation_xyzw[2],
                    w: value.orientation_xyzw[3],
                }),
                angular_velocity: Some(proto::Vector3 {
                    x: value.angular_velocity_xyz[0],
                    y: value.angular_velocity_xyz[1],
                    z: value.angular_velocity_xyz[2],
                }),
                linear_acceleration: Some(proto::Vector3 {
                    x: value.linear_acceleration_xyz[0],
                    y: value.linear_acceleration_xyz[1],
                    z: value.linear_acceleration_xyz[2],
                }),
                device_timestamp_ms: value.device_timestamp_ms,
                sample_age_ms: value.sample_age_ms.saturating_add(transport_age_ms),
            })
        }
        TelemetryPayload::Group(value) => {
            proto::watch_telemetry_response::Payload::Group(proto::GroupTelemetry {
                group: value.group.to_string(),
                state: group_state(value.state),
                fault_detail: value.fault_detail,
                command_count: value.command_count,
                control_tick: value.control_tick,
                jitter_us: value.jitter_us,
                fault_cause: fault_cause(value.fault_cause),
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
    fn fault_causes_are_typed_on_the_wire() {
        assert_eq!(
            fault_cause(Some(FaultCause::Temperature)),
            proto::FaultCause::Temperature as i32
        );
        assert_eq!(fault_cause(None), proto::FaultCause::Unspecified as i32);
    }
}
