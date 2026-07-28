use std::fs;
use std::io;
use std::net::SocketAddr;
use std::path::Path;

pub fn write_hardwared(path: &Path, interface: &str, listen: SocketAddr) -> io::Result<()> {
    write_hardwared_with_control_rate(path, interface, listen, 250.0)
}

pub fn write_hardwared_with_control_rate(
    path: &Path,
    interface: &str,
    listen: SocketAddr,
    control_rate_hz: f64,
) -> io::Result<()> {
    write_hardwared_with_grpc(
        path,
        interface,
        &format!(
            "  listen: {{ type: tcp, address: {listen} }}\n  default_lease_ms: 1000\n  maximum_lease_ms: 10000"
        ),
        control_rate_hz,
    )
}

pub fn write_hardwared_with_grpc(
    path: &Path,
    interface: &str,
    grpc: &str,
    control_rate_hz: f64,
) -> io::Result<()> {
    let description_path = path.with_file_name("e2e-hardware.yaml");
    fs::write(&description_path, hardware_description(control_rate_hz))?;
    fs::write(
        path,
        format!(
            r#"description: {}

runtime:
  telemetry_capacity: 4096

grpc:
{grpc}

bus_interfaces: {{ e2e_can: {interface} }}

enabled_groups: [e2e_group]
"#,
            description_path
                .file_name()
                .expect("description path has a file name")
                .display(),
        ),
    )
}

fn hardware_description(control_rate_hz: f64) -> String {
    format!(
        r#"logical_buses:
  - name: e2e_can
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
