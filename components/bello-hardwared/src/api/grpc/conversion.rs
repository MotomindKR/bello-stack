use std::time::Duration;

use tonic::Status;

use crate::domain::{
    CommandBatch, DeviceState, DeviceStatus, ImpedanceCommand, LeaseToken, MotorCommand, MotorName,
    TelemetryEvent, TelemetryPayload,
};
use crate::proto;

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
            DeviceState::Disabled => proto::DeviceState::Disabled,
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
                address: Some(proto::MotorAddress {
                    bus: value.address.bus.to_string(),
                    id: value.address.id as u32,
                }),
                position: value.position,
                velocity: value.velocity,
                torque: value.torque,
                temperature: value.temperature,
                fault_bits: value.fault_bits as u32,
                mode_status: value.mode_status as u32,
                feedback_age_ms: value.feedback_age_ms.saturating_add(transport_age_ms),
                status: Some(device_status(&value.status)),
            })
        }
        TelemetryPayload::Imu(value) => {
            proto::watch_telemetry_response::Payload::Imu(proto::ImuTelemetry {
                orientation_xyzw: value.orientation_xyzw.to_vec(),
                angular_velocity_xyz: value.angular_velocity_xyz.to_vec(),
                linear_acceleration_xyz: value.linear_acceleration_xyz.to_vec(),
                device_timestamp_ms: value.device_timestamp_ms,
                sample_age_ms: value.sample_age_ms.saturating_add(transport_age_ms),
                imu_name: value.imu.to_string(),
                serial_device: value.serial_device.to_string(),
                status: Some(device_status(&value.status)),
            })
        }
        TelemetryPayload::Group(value) => {
            proto::watch_telemetry_response::Payload::Group(proto::GroupTelemetry {
                group: value.group.to_string(),
                state: value.state.as_str().into(),
                fault_reason: value.fault_reason,
                lease_owner: value.lease_owner.unwrap_or_default(),
                command_count: value.command_count,
                control_tick: value.control_tick,
                jitter_us: value.jitter_us,
            })
        }
        TelemetryPayload::CommandPolicy(value) => {
            proto::watch_telemetry_response::Payload::CommandPolicy(proto::CommandPolicyTelemetry {
                group: value.group.to_string(),
                motor_name: value.motor.to_string(),
                check: value.check,
                severity: value.severity,
                detail: value.detail,
                command_modified: value.command_modified,
                command_rejected: value.command_rejected,
                response: value.response,
            })
        }
        TelemetryPayload::Health(value) => {
            let status = DeviceStatus {
                state: value.status,
                detail: value.detail.clone(),
                transition_count: value.transition_count,
            };
            proto::watch_telemetry_response::Payload::Health(proto::HealthTelemetry {
                component: value.component,
                dropped_events: value.dropped_events,
                device_status: Some(device_status(&status)),
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
    use crate::domain::{BusName, CommandPolicyTelemetry, GroupName, MotorAddress, MotorTelemetry};

    #[test]
    fn telemetry_age_includes_time_waiting_for_the_subscriber() {
        let response = telemetry_event(
            TelemetryEvent {
                monotonic_ns: 1_000_000,
                payload: TelemetryPayload::Motor(MotorTelemetry {
                    group: GroupName::from("group"),
                    motor: MotorName::from("motor"),
                    address: MotorAddress {
                        bus: BusName::from("can"),
                        id: 1,
                    },
                    status: DeviceStatus {
                        state: DeviceState::Online,
                        detail: "healthy motor feedback".into(),
                        transition_count: 2,
                    },
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
    fn device_status_is_strongly_typed_at_the_proto_boundary() {
        let status = device_status(&DeviceStatus {
            state: DeviceState::Reconnecting,
            detail: "retrying".into(),
            transition_count: 7,
        });
        assert_eq!(status.state, proto::DeviceState::Reconnecting as i32);
        assert_eq!(status.detail, "retrying");
        assert_eq!(status.transition_count, 7);
    }

    #[test]
    fn command_policy_event_preserves_action_and_response() {
        let response = telemetry_event(
            TelemetryEvent {
                monotonic_ns: 1,
                payload: TelemetryPayload::CommandPolicy(CommandPolicyTelemetry {
                    group: GroupName::from("group"),
                    motor: MotorName::from("motor"),
                    check: "command_limit".into(),
                    severity: "warning".into(),
                    detail: "clamped".into(),
                    command_modified: true,
                    command_rejected: false,
                    response: "damping".into(),
                }),
            },
            1,
        );
        let Some(proto::watch_telemetry_response::Payload::CommandPolicy(policy)) =
            response.payload
        else {
            panic!("expected command policy telemetry");
        };
        assert_eq!(policy.check, "command_limit");
        assert!(policy.command_modified);
        assert_eq!(policy.response, "damping");
    }
}
