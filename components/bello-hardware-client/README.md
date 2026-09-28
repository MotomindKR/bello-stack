# Bello hardware Rust client

`bello-hardware-client` is the application-facing Rust interface to
`bello-hardwared`. Normal applications work with a discovered `Robot`, named
motor groups and coordinates, complete command frames, automatically renewed control
sessions, and aggregated robot state. Lease tokens, command sequences,
acknowledgements, and protobuf conversion stay inside the crate.

The exact protobuf and lease-oriented primitives remain available under
`bello_hardware_client::raw` for diagnostics and specialized tooling.

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

`ControlSession::release()` releases ownership with the daemon's damping
reaction and never calls disable. Running groups become Damped; Fault remains
latched. Use `shutdown()` only when motor disable is intended. Failed starts
roll back to damping. Bello Mini configures damping for every fault cause;
stale host feedback does not prevent zero-Kp damping commands, and failed
transmission is reported without substituting disable. Drive protections and
power/transport availability still determine whether damping can be applied.

Session shutdown preserves an already latched daemon Fault. If stop or disable
is rejected because the group has faulted, the client confirms that state with
fresh inventory and releases the lease without automatic recovery. An explicit
`recover()` is required before another control session can enable the group.

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

    // Returns only after the daemon accepts the complete snapshot.
    control.send(next).await?;

    // Stops into Damped, disables, ends automatic renewal, and releases ownership.
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

The connected `Robot` retains the complete validated daemon inventory. Stable,
name-ordered iterators expose CAN bus, serial transport, motor, and IMU metadata:

```rust,no_run
# use bello_hardware_client::Robot;
# async fn example() -> Result<(), bello_hardware_client::Error> {
let robot = Robot::connect("http://127.0.0.1:50051").await?;

for bus in robot.buses() {
    println!("{}: {} host {}", bus.name(), bus.interface(), bus.host_id());
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
        motor.limits().torque_min(),
        motor.limits().torque_max(),
    );
}
# Ok(())
# }
```

Each device exposes `initial_status()`, the status captured by `ListHardware` at
connection time. Live lifecycle changes are available through the health
snapshots described below.

## Robot state

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
assuming source timestamps are unique. Health is retained latest-by-component,
while command policy is retained latest-by-motor-and-check (with one bounded
slot for future unknown checks); neither collection grows with repeated events.
Read-only telemetry retries transient gRPC unavailability on the existing tonic
channel and clears dynamic state after reconnect; `generation()` distinguishes
the new stream. Normal server EOF remains visible as `None`.

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
  complete frame; `disable` explicitly returns it to `Idle`.
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
use bello_hardware_client::raw::{HardwareClient, TelemetryFilter};

# async fn example() -> Result<(), bello_hardware_client::Error> {
let hardware = HardwareClient::connect("http://127.0.0.1:50051").await?;
let inventory = hardware.inventory().await?;
let telemetry = hardware.watch(TelemetryFilter::all()).await?;
# let _ = (inventory, telemetry);
# Ok(())
# }
```

The generated contract is `bello_hardware_client::raw::proto`. Raw lease and
command types retain their explicit, conservative semantics; the ergonomic API
uses those same implementations rather than duplicating them.

## Source layout

```text
src/robot.rs      connection, validated inventory, groups, and raw escape hatch
src/inventory.rs  daemon inventory validation and resolved motor metadata
src/frame.rs      coordinate targets and complete command-frame validation
src/control.rs    automatic renewal and ergonomic control lifecycle
src/state.rs      typed latest-value telemetry aggregation
src/client.rs     raw tonic client
src/lease.rs      raw lease ownership and renewal
src/command.rs    raw command sequencing and acknowledgements
src/telemetry.rs  raw event telemetry
src/endpoint.rs   TCP/HTTPS/Unix, TLS identity, timeout, and keepalive policy
```

See the repository [API guide](../../docs/api.md) and
[control and safety](../../docs/control-and-safety.md) for server behavior.
