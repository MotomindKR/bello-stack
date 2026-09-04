use std::fs;
use std::io;
use std::net::SocketAddr;
use std::path::Path;

use anyhow::{Context, Result, bail};
use serde::Deserialize;
use serde_yaml::Value;

pub struct CanonicalLowerBodyFixture {
    pub motor_names: Vec<String>,
}

#[derive(Deserialize)]
struct CanonicalHardware {
    logical_buses: Vec<CanonicalBus>,
    motors: Vec<CanonicalMotor>,
    motor_groups: Vec<CanonicalGroup>,
}

#[derive(Deserialize)]
struct CanonicalBus {
    name: String,
    host_id: u8,
}

#[derive(Deserialize)]
struct CanonicalMotor {
    name: String,
    bus: String,
    id: u8,
    model: String,
    direction: i8,
}

#[derive(Deserialize)]
struct CanonicalGroup {
    name: String,
    motors: Vec<String>,
}

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

pub fn write_canonical_lower_body(
    deployment_path: &Path,
    emulator_configs: [(&Path, &str); 2],
    listen: SocketAddr,
) -> Result<CanonicalLowerBodyFixture> {
    let manifest = Path::new(env!("CARGO_MANIFEST_DIR"));
    let hardware_path = manifest.join("../../descriptions/bello/hardware.yaml");
    let robot_description = manifest.join("../../descriptions/bello/urdf/bello.urdf");
    let source = fs::read_to_string(&hardware_path)
        .with_context(|| format!("reading {}", hardware_path.display()))?;
    let identity: CanonicalHardware = serde_yaml::from_str(&source)?;
    let mut description: Value = serde_yaml::from_str(&source)?;
    let mapping = description
        .as_mapping_mut()
        .context("canonical hardware description must be a YAML mapping")?;
    mapping.insert(
        Value::String("robot_description".into()),
        Value::String(robot_description.display().to_string()),
    );
    mapping.insert(
        Value::String("serial_devices".into()),
        Value::Sequence(Vec::new()),
    );
    mapping.insert(Value::String("imus".into()), Value::Sequence(Vec::new()));

    let description_path = deployment_path.with_file_name("canonical-lower-body-hardware.yaml");
    fs::write(&description_path, serde_yaml::to_string(&description)?)?;
    let [(can0_path, can0_interface), (can1_path, can1_interface)] = emulator_configs;
    fs::write(
        can0_path,
        canonical_emulator(&identity, "lower_body", "can0", can0_interface)?,
    )?;
    fs::write(
        can1_path,
        canonical_emulator(&identity, "lower_body", "can1", can1_interface)?,
    )?;
    fs::write(
        deployment_path,
        format!(
            "description: {}\n\ngrpc:\n  listen: {{ type: tcp, address: {listen} }}\n  default_lease_ms: 1000\n  maximum_lease_ms: 10000\n\nbus_interfaces: {{ can0: {can0_interface}, can1: {can1_interface} }}\n\nenabled_groups: [lower_body]\n",
            description_path
                .file_name()
                .context("description path has no file name")?
                .display()
        ),
    )?;

    let lower = identity
        .motor_groups
        .iter()
        .find(|group| group.name == "lower_body")
        .context("canonical hardware has no lower_body group")?;
    Ok(CanonicalLowerBodyFixture {
        motor_names: lower.motors.clone(),
    })
}

fn canonical_emulator(
    identity: &CanonicalHardware,
    group_name: &str,
    bus_name: &str,
    interface: &str,
) -> Result<String> {
    let group = identity
        .motor_groups
        .iter()
        .find(|group| group.name == group_name)
        .with_context(|| format!("canonical hardware has no {group_name} group"))?;
    let bus = identity
        .logical_buses
        .iter()
        .find(|bus| bus.name == bus_name)
        .with_context(|| format!("canonical hardware has no {bus_name} bus"))?;
    let motors = group
        .motors
        .iter()
        .map(|name| {
            identity
                .motors
                .iter()
                .find(|motor| motor.name == *name)
                .with_context(|| format!("canonical group refers to unknown motor {name}"))
        })
        .collect::<Result<Vec<_>>>()?
        .into_iter()
        .filter(|motor| motor.bus == bus_name)
        .collect::<Vec<_>>();
    if motors.is_empty() {
        bail!("canonical group {group_name} has no motors on bus {bus_name}");
    }

    let mut output = format!(
        "interface: {interface}\nhost_id: {}\nfeedback_rate_hz: 500.0\n\ndynamics:\n  position_response_hz: 20.0\n  idle_velocity_decay_hz: 20.0\n\nactuators:\n",
        bus.host_id
    );
    for motor in motors {
        output.push_str(&format!(
            "  - {{ id: {}, model: {}, direction: {}, initial_position: 0.0 }}\n",
            motor.id, motor.model, motor.direction
        ));
    }
    output.push_str("\nevents: []\n");
    Ok(output)
}

fn hardware_description(control_rate_hz: f64) -> String {
    let robot_description = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/vcan.urdf");
    format!(
        r#"robot_description: {}

logical_buses:
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
"#,
        robot_description.display()
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
