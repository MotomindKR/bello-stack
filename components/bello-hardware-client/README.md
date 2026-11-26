# Bello hardware Rust client

Use `Robot` to discover motor groups, read telemetry, and command a group
through an automatically renewed control session.

## Connections

Plaintext TCP, HTTPS, and Unix-domain socket endpoints use the same API:

```rust,no_run
# use bello_hardware_client::Robot;
# async fn example() -> Result<(), bello_hardware_client::Error> {
let local = Robot::connect("http://127.0.0.1:50051").await?;
let remote = Robot::connect("https://robot.example:50051").await?;
let socket = Robot::connect("unix:///run/bello/hardwared.sock").await?;
# let _ = (local, remote, socket);
# Ok(())
# }
```

HTTPS uses native trusted roots by default. Add a private CA and client identity
for mTLS with `ConnectOptions`:

```rust,no_run
use bello_hardware_client::{ConnectOptions, Robot, TlsOptions};

# async fn example() -> Result<(), Box<dyn std::error::Error>> {
let tls = TlsOptions::new()
    .ca_certificate(std::fs::read("/etc/bello/tls/ca.crt")?)
    .identity(
        std::fs::read("/etc/bello/tls/client.crt")?,
        std::fs::read("/etc/bello/tls/client.key")?,
    );
let options = ConnectOptions::new("https://robot.example:50051").tls(tls);
let robot = Robot::connect_with(options).await?;
# let _ = robot;
# Ok(())
# }
```

## Motor-group control

Use `release()` to give up ownership and leave enabled motors damped. Use
`shutdown()` to stop and disable them. A latched fault requires explicit
`recover()` before another session can enable the group.

```rust,no_run
use std::time::Duration;

use bello_hardware_client::{ControlOptions, CoordinateTarget, Robot};

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let robot = Robot::connect("http://127.0.0.1:50051").await?;
    let group = robot.group("emulated_group")?;

    println!("coordinates: {:?}", group.coordinate_names().collect::<Vec<_>>());

    let initial = group
        .frame()
        .coordinate(
            "emulated_joint_1",
            CoordinateTarget::position(0.0).gains(20.0, 1.0),
        )?
        .coordinate(
            "emulated_joint_2",
            CoordinateTarget::position(0.0).gains(20.0, 1.0),
        )?
        .build()?;

    let mut control = group
        .control_with(
            ControlOptions::new("example-controller")
                .command_validity(Duration::from_millis(100)),
        )
        .await?;

    // Enables damping, then atomically starts with the complete initial frame.
    control.start(initial).await?;

    let next = group
        .frame()
        .coordinate(
            "emulated_joint_1",
            CoordinateTarget::position(0.1).gains(20.0, 1.0),
        )?
        .coordinate(
            "emulated_joint_2",
            CoordinateTarget::position(-0.1).gains(20.0, 1.0),
        )?
        .build()?;

    control.send(next).await?;

    control.shutdown().await?;
    Ok(())
}
```

The group and its members come from daemon inventory; there are no fixed body
slots or motor-count assumptions. Every frame must assign each coordinate in its
group exactly once. Unknown, duplicate, missing, non-finite, or invalid-gain
targets fail locally without transmitting a partial command.

`CoordinateTarget` supports position, torque, damping, and complete hybrid impedance
targets. Position, velocity, torque, `kp`, and `kd` use the daemon's SI-unit
contract.

## Hardware inventory

Inspect the inventory returned at connection:

```rust,no_run
# use bello_hardware_client::Robot;
# async fn example() -> Result<(), bello_hardware_client::Error> {
let robot = Robot::connect("http://127.0.0.1:50051").await?;

for bus in robot.buses() {
    println!("{}: {} host {}", bus.name(), bus.transport(), bus.host_id());
}
for serial in robot.serial_devices() {
    println!("{}: {} at {} baud", serial.name(), serial.port(), serial.baud_rate());
}
for imu in robot.imus() {
    println!(
        "{} uses {} and publishes {} from {}",
        imu.name(),
        imu.serial_device(),
        imu.output_frame(),
        imu.source_frame(),
    );
}
for motor in robot.motors() {
    println!(
        "{} torque range: {}..={} Nm",
        motor.name(),
        motor.limits().torque_min,
        motor.limits().torque_max,
    );
}
# Ok(())
# }
```

Each device exposes `initial_status()`, the status captured by `ListHardware` at
connection time. Live lifecycle changes are available through the health
snapshots described below.

## Robot state

Compare reported states with `EnumValue::Known(GroupState::Idle)` or call
`.known()` to obtain `Option<GroupState>`. `EnumValue::Unknown(i32)` preserves
values introduced by a newer daemon.

```rust,no_run
use bello_hardware_client::{Robot, RobotStateChange};

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let robot = Robot::connect("http://127.0.0.1:50051").await?;
    let mut states = robot.states().await?;

    while let Some(state) = states.changed().await? {
        match state.last_change() {
            Some(RobotStateChange::Motor(motor)) => {
                println!("updated {} at {} ns", motor.motor, motor.monotonic_ns);
            }
            _ => {}
        }
        if let Ok(coordinate) = state.coordinate("emulated_joint_1") {
            println!(
                "position={} velocity={} temperature={}",
                coordinate.position, coordinate.velocity, coordinate.temperature
            );
        }
        for health in state.health_snapshots() {
            println!("{}: {:?}", health.component, health.status);
        }
        for policy in state.command_policies() {
            println!("{}: {:?} {:?}", policy.motor, policy.check, policy.behavior);
        }
    }
    Ok(())
}
```

`RobotState` is a latest-value aggregate, not a time-synchronized sensor frame.
Each call to `changed` applies one incoming motor, group, IMU, health, or
command-policy event. `last_change()` identifies that exact entry without
assuming source timestamps are unique. Read-only telemetry retries transient
gRPC unavailability and clears dynamic state after reconnect. Use `generation()`
to identify the new stream. Normal server EOF remains visible as `None`.

## Control semantics

- `MotorGroup::control` acquires exclusive write ownership and immediately
  starts automatic renewal. `control_with` customizes lease duration, command
  validity, acknowledgement timeout, and bounded task shutdown.
- `ControlSession::start` enables the group into `Damped` and atomically starts
  it with the complete initial frame.
- `ControlSession::send` allocates the sequence, submits one bounded request,
  correlates the response, and returns only for `ACCEPTED`.
- `ControlSession::stop` leaves the group `Damped` while retaining ownership.
- `ControlSession::resume` atomically starts a damped group with a fresh
  complete frame.
- `ControlSession::disable` returns Ready, Running or Damped directly to `Idle`.
  For an application-managed torque fade, acknowledge the final zero-gain and
  zero-feedforward frame with `send()`, then `disable()` and `release()`. This
  avoids the damping step in `stop()` / `shutdown()`; the application decides
  when torque removal is appropriate.
- `ControlSession::recover` performs explicit recovery from a latched group
  fault.
- `ControlSession::shutdown` stops and disables when needed, bounds renewal-task
  shutdown, and releases ownership.

Motion is never transparently reacquired, reconnected, or replayed. A renewal,
lease, command-stream, or acknowledgement failure is returned to the caller.
A rejected command whose acknowledgement reports daemon state `Damped` moves
the session to `Damped`; other ambiguous or rejected running commands move it
to `Uncertain`. Both close the command stream.
Dropping `ControlSession` aborts its local renewal task but cannot perform async
network cleanup; call and await `shutdown` during normal operation. The daemon's
command-timeout and lease-loss damping remain the final safety backstop.

## Raw access

Performance probes and protocol-aware integrations can opt into the lower
layer explicitly:

```rust,no_run
use bello_hardware_client::raw::{HardwareClient, proto::WatchTelemetryRequest};

# async fn example() -> Result<(), bello_hardware_client::Error> {
let hardware = HardwareClient::connect("http://127.0.0.1:50051").await?;
let inventory = hardware.inventory().await?;
let telemetry = hardware.watch(WatchTelemetryRequest {
    include_motors: true,
    ..Default::default()
}).await?;
# let _ = (inventory, telemetry);
# Ok(())
# }
```

Generated protobuf types are available at `bello_hardware_client::raw::proto`.

See the repository [API guide](../../docs/api.md) and
[control and safety](../../docs/control-and-safety.md) for server behavior.
