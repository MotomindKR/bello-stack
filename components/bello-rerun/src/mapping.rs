use std::fmt::Debug;

use bello_hardware_api::{
    CommandBehavior, CommandPolicyCheck, DeviceState, FaultResponse, WatchTelemetryResponse,
    watch_telemetry_response,
};

#[derive(Clone, Debug, PartialEq)]
pub(crate) enum LogEntry {
    Scalars {
        path: String,
        names: Vec<&'static str>,
        values: Vec<f64>,
    },
    Text {
        path: String,
        text: String,
    },
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) struct MappedTelemetry {
    pub(crate) source_timestamp_ns: u64,
    pub(crate) entries: Vec<LogEntry>,
    pub(crate) warnings: Vec<String>,
}

pub(crate) fn map(response: WatchTelemetryResponse) -> MappedTelemetry {
    let mut mapped = MappedTelemetry {
        source_timestamp_ns: response.monotonic_ns,
        entries: Vec::new(),
        warnings: Vec::new(),
    };

    match response.payload {
        Some(watch_telemetry_response::Payload::Motor(value)) => {
            mapped.entries.push(LogEntry::Scalars {
                path: format!("motors/{}/{}", value.group, value.motor_name),
                names: vec![
                    "position",
                    "velocity",
                    "torque",
                    "temperature",
                    "fault_bits",
                ],
                values: vec![
                    value.position,
                    value.velocity,
                    value.torque,
                    value.temperature,
                    f64::from(value.fault_bits),
                ],
            });
        }
        Some(watch_telemetry_response::Payload::Imu(value)) => {
            let path = format!("imus/{}", value.imu_name);
            match value.orientation {
                Some(orientation) => mapped.entries.push(LogEntry::Scalars {
                    path: format!("{path}/orientation"),
                    names: vec!["x", "y", "z", "w"],
                    values: vec![orientation.x, orientation.y, orientation.z, orientation.w],
                }),
                None => mapped
                    .warnings
                    .push(format!("IMU {} has no orientation", value.imu_name)),
            }
            match value.angular_velocity {
                Some(angular_velocity) => mapped.entries.push(LogEntry::Scalars {
                    path: format!("{path}/angular_velocity"),
                    names: vec!["x", "y", "z"],
                    values: vec![angular_velocity.x, angular_velocity.y, angular_velocity.z],
                }),
                None => mapped
                    .warnings
                    .push(format!("IMU {} has no angular velocity", value.imu_name)),
            }
            match value.linear_acceleration {
                Some(linear_acceleration) => mapped.entries.push(LogEntry::Scalars {
                    path: format!("{path}/linear_acceleration"),
                    names: vec!["x", "y", "z"],
                    values: vec![
                        linear_acceleration.x,
                        linear_acceleration.y,
                        linear_acceleration.z,
                    ],
                }),
                None => mapped
                    .warnings
                    .push(format!("IMU {} has no linear acceleration", value.imu_name)),
            }
        }
        Some(watch_telemetry_response::Payload::Group(value)) => {
            mapped.entries.push(LogEntry::Scalars {
                path: format!("groups/{}", value.group),
                names: vec!["control_tick", "jitter_us", "command_count"],
                values: vec![
                    value.control_tick as f64,
                    value.jitter_us as f64,
                    value.command_count as f64,
                ],
            });
        }
        Some(watch_telemetry_response::Payload::CommandPolicy(value)) => {
            mapped.entries.push(LogEntry::Text {
                path: format!("command_policy/{}/{}", value.group, value.motor_name),
                text: format!(
                    "{} {}: {}; modified={}, rejected={}, response={}",
                    enum_debug::<CommandBehavior>(value.behavior),
                    enum_debug::<CommandPolicyCheck>(value.check),
                    value.detail,
                    value.command_modified,
                    value.command_rejected,
                    enum_debug::<FaultResponse>(value.fault_response),
                ),
            });
        }
        Some(watch_telemetry_response::Payload::Health(value)) => {
            if let Some(status) = value.device_status {
                let dropped = if value.dropped_events == 0 {
                    String::new()
                } else {
                    format!("; dropped_events={}", value.dropped_events)
                };
                mapped.entries.push(LogEntry::Text {
                    path: format!("health/{}", value.component),
                    text: format!(
                        "{}: {}{dropped}",
                        device_state_name(status.state),
                        status.detail,
                    ),
                });
            } else {
                mapped.warnings.push(format!(
                    "health event for {} has no device status",
                    value.component
                ));
            }
        }
        None => mapped
            .warnings
            .push("telemetry response has no payload".into()),
    }
    mapped
}

fn enum_debug<T>(value: i32) -> String
where
    T: Debug + TryFrom<i32>,
{
    T::try_from(value).map_or_else(
        |_| format!("Unknown({value})"),
        |known| format!("{known:?}"),
    )
}

fn device_state_name(value: i32) -> String {
    match DeviceState::try_from(value) {
        Ok(DeviceState::Unspecified) => "unspecified".into(),
        Ok(DeviceState::Starting) => "starting".into(),
        Ok(DeviceState::Connecting) => "connecting".into(),
        Ok(DeviceState::Online) => "online".into(),
        Ok(DeviceState::Degraded) => "degraded".into(),
        Ok(DeviceState::Reconnecting) => "reconnecting".into(),
        Ok(DeviceState::Fault) => "fault".into(),
        Ok(DeviceState::Shutdown) => "shutdown".into(),
        Err(_) => format!("unknown({value})"),
    }
}

#[cfg(test)]
mod tests {
    use bello_hardware_api::{
        CommandPolicyTelemetry, DeviceStatus, FaultCause, GroupState, GroupTelemetry,
        HealthTelemetry, ImuTelemetry, MotorTelemetry, Quaternion, Vector3,
    };

    use super::*;

    fn response(payload: watch_telemetry_response::Payload) -> WatchTelemetryResponse {
        WatchTelemetryResponse {
            monotonic_ns: 42,
            payload: Some(payload),
        }
    }

    #[test]
    fn motor_uses_the_existing_entity_path_and_series() {
        let mapped = map(response(watch_telemetry_response::Payload::Motor(
            MotorTelemetry {
                group: "legs".into(),
                motor_name: "left_hip".into(),
                position: 1.0,
                velocity: 2.0,
                torque: 3.0,
                temperature: 4.0,
                fault_bits: 5,
                mode_status: 6,
                feedback_age_ms: 7,
            },
        )));
        let LogEntry::Scalars { path, names, .. } = &mapped.entries[0] else {
            panic!("expected scalar motor telemetry");
        };
        assert_eq!(path, "motors/legs/left_hip");
        assert_eq!(
            &names[..5],
            [
                "position",
                "velocity",
                "torque",
                "temperature",
                "fault_bits"
            ]
        );
        assert!(mapped.warnings.is_empty());
    }

    #[test]
    fn incomplete_imu_skips_only_the_missing_field() {
        let mapped = map(response(watch_telemetry_response::Payload::Imu(
            ImuTelemetry {
                imu_name: "body".into(),
                orientation: Some(Quaternion {
                    x: 1.0,
                    y: 2.0,
                    z: 3.0,
                    w: 4.0,
                }),
                angular_velocity: None,
                linear_acceleration: Some(Vector3 {
                    x: 5.0,
                    y: 6.0,
                    z: 7.0,
                }),
                device_timestamp_ms: 8,
                sample_age_ms: 9,
            },
        )));
        assert_eq!(mapped.entries.len(), 2);
        assert_eq!(mapped.warnings.len(), 1);
        assert!(mapped.warnings[0].contains("angular velocity"));
    }

    #[test]
    fn group_command_policy_and_health_payloads_are_mapped() {
        let group = map(response(watch_telemetry_response::Payload::Group(
            GroupTelemetry {
                group: "legs".into(),
                state: GroupState::Running as i32,
                fault_detail: String::new(),
                command_count: 1,
                control_tick: 2,
                jitter_us: 3,
                fault_cause: FaultCause::Unspecified as i32,
            },
        )));
        assert_eq!(group.entries.len(), 1);

        let policy = map(response(watch_telemetry_response::Payload::CommandPolicy(
            CommandPolicyTelemetry {
                group: "legs".into(),
                motor_name: "left_hip".into(),
                check: CommandPolicyCheck::Limit as i32,
                behavior: CommandBehavior::Clamp as i32,
                detail: "clamped".into(),
                command_modified: true,
                command_rejected: false,
                fault_response: FaultResponse::Unspecified as i32,
            },
        )));
        assert_eq!(policy.entries.len(), 1);

        let health = map(response(watch_telemetry_response::Payload::Health(
            HealthTelemetry {
                component: "grpc_telemetry".into(),
                device_status: Some(DeviceStatus {
                    state: DeviceState::Degraded as i32,
                    detail: "subscriber lagged".into(),
                    transition_count: 0,
                }),
                dropped_events: 12,
            },
        )));
        let LogEntry::Text { path, text } = &health.entries[0] else {
            panic!("expected health text");
        };
        assert_eq!(path, "health/grpc_telemetry");
        assert!(text.contains("dropped_events=12"));
    }

    #[test]
    fn missing_payload_and_health_status_are_warnings() {
        let missing = map(WatchTelemetryResponse {
            monotonic_ns: 1,
            payload: None,
        });
        assert!(missing.entries.is_empty());
        assert_eq!(missing.warnings.len(), 1);

        let health = map(response(watch_telemetry_response::Payload::Health(
            HealthTelemetry {
                component: "can0".into(),
                device_status: None,
                dropped_events: 0,
            },
        )));
        assert!(health.entries.is_empty());
        assert_eq!(health.warnings.len(), 1);
    }
}
