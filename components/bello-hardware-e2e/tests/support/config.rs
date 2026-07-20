use std::net::SocketAddr;

pub fn hardwared(interface: &str, listen: SocketAddr) -> String {
    hardwared_with_control_rate(interface, listen, 250.0)
}

pub fn hardwared_with_control_rate(
    interface: &str,
    listen: SocketAddr,
    control_rate_hz: f64,
) -> String {
    hardwared_with_grpc(
        interface,
        &format!(
            "  listen: {{ type: tcp, address: {listen} }}\n  default_lease_ms: 1000\n  maximum_lease_ms: 10000"
        ),
        control_rate_hz,
    )
}

pub fn hardwared_with_grpc(interface: &str, grpc: &str, control_rate_hz: f64) -> String {
    format!(
        r#"runtime:
  telemetry_capacity: 4096

grpc:
{grpc}

buses:
  - name: e2e_can
    interface: {interface}
    host_id: 253
    bitrate: 1000000
    maximum_utilization: 0.75
    reconnect: {{ initial_delay_ms: 50, maximum_delay_ms: 500 }}

motors:
  - name: e2e_motor_1
    coordinate: e2e_joint_1
    bus: e2e_can
    id: 1
    model: ROBSTRIDE_03
    direction: 1
  - name: e2e_motor_2
    coordinate: e2e_joint_2
    bus: e2e_can
    id: 2
    model: ROBSTRIDE_03
    direction: -1

motor_groups:
  - name: e2e_group
    motors: [e2e_motor_1, e2e_motor_2]
    control_rate_hz: {control_rate_hz}
    command_timeout_ms: 200
    feedback_timeout_ms: 100
    enable_timeout_ms: 500
    enable_retries: 2
    fault_policy:
      default: damping
      overrides:
        temperature: disable
        reported_motor_fault: disable
        enable: disable
        disable: disable
        exit: disable
"#
    )
}

pub fn emulator(interface: &str) -> String {
    format!(
        r#"interface: {interface}
host_id: 253
feedback_rate_hz: 500.0

dynamics:
  position_response_hz: 20.0
  idle_velocity_decay_hz: 20.0

actuators:
  - {{ id: 1, model: ROBSTRIDE_03, direction: 1, initial_position: 0.0 }}
  - {{ id: 2, model: ROBSTRIDE_03, direction: -1, initial_position: 0.0 }}

events: []
"#
    )
}
