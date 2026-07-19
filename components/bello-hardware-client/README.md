# Bello hardware Rust client

`bello-hardware-client` is the application-facing Rust interface to
`bello-hardwared`. Normal applications work with a discovered `Robot`, named
motor groups and joints, complete command frames, automatically renewed control
sessions, and aggregated robot state. Lease tokens, command sequences,
acknowledgements, and protobuf conversion stay inside the crate.

The exact protobuf and lease-oriented primitives remain available under
`bello_hardware_client::raw` for diagnostics and specialized tooling.

## Motor-group control

```rust,no_run
use std::time::Duration;

use bello_hardware_client::{ControlOptions, JointTarget, Robot};

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let robot = Robot::connect("http://127.0.0.1:50051").await?;
    let group = robot.group("emulated_group")?;

    println!("joints: {:?}", group.joint_names().collect::<Vec<_>>());

    let initial = group
        .frame()
        .joint(
            "emulated_joint_1",
            JointTarget::position(0.0).gains(20.0, 1.0),
        )?
        .joint(
            "emulated_joint_2",
            JointTarget::position(0.0).gains(20.0, 1.0),
        )?
        .build()?;

    let mut control = group
        .control_with(
            ControlOptions::new("example-controller")
                .command_validity(Duration::from_millis(100)),
        )
        .await?;

    // Enables the group, confirms the initial frame, then starts the controller.
    control.start(initial).await?;

    let next = group
        .frame()
        .joint(
            "emulated_joint_1",
            JointTarget::position(0.1).gains(20.0, 1.0),
        )?
        .joint(
            "emulated_joint_2",
            JointTarget::position(-0.1).gains(20.0, 1.0),
        )?
        .build()?;

    // Returns only after the daemon accepts the complete snapshot.
    control.send(next).await?;

    // Stops, disables, ends automatic renewal, and releases ownership.
    control.shutdown().await?;
    Ok(())
}
```

The group and its members come from daemon inventory; there are no fixed body
slots or motor-count assumptions. Every frame must assign each joint in its
group exactly once. Unknown, duplicate, missing, non-finite, or invalid-gain
targets fail locally without transmitting a partial command.

`JointTarget` supports position, torque, damping, and complete hybrid impedance
targets. Position, velocity, torque, `kp`, and `kd` use the daemon's SI-unit
contract.

## Robot state

```rust,no_run
use bello_hardware_client::Robot;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let robot = Robot::connect("http://127.0.0.1:50051").await?;
    let mut states = robot.states().await?;

    while let Some(state) = states.changed().await? {
        if let Ok(joint) = state.joint("emulated_joint_1") {
            println!(
                "position={} velocity={} temperature={}",
                joint.position, joint.velocity, joint.temperature
            );
        }
    }
    Ok(())
}
```

`RobotState` is a latest-value aggregate, not a time-synchronized sensor frame.
Each call to `changed` applies one incoming motor, group, IMU, or health event.
Read-only telemetry retries transient gRPC unavailability on the existing tonic
channel and clears dynamic state after reconnect; `generation()` distinguishes
the new stream. Normal server EOF remains visible as `None`.

## Control semantics

- `MotorGroup::control` acquires exclusive write ownership and immediately
  starts automatic renewal. `control_with` customizes lease duration, command
  validity, acknowledgement timeout, and bounded task shutdown.
- `ControlSession::start` performs enable, initial-frame acceptance, and start
  in the required order.
- `ControlSession::send` allocates the sequence, submits one bounded request,
  correlates the response, and returns only for `ACCEPTED`.
- `ControlSession::stop` stops and disables while retaining ownership, allowing
  another `start` on the same session.
- `ControlSession::recover` performs explicit recovery from a latched group
  fault.
- `ControlSession::shutdown` stops and disables when needed, bounds renewal-task
  shutdown, and releases ownership.

Motion is never transparently reacquired, reconnected, or replayed. A renewal,
lease, command-stream, or acknowledgement failure is returned to the caller.
An ambiguous or rejected running command moves the session to `Uncertain` and
closes its command stream, preventing an accidental retry before explicit
recovery or shutdown.
Dropping `ControlSession` aborts its local renewal task but cannot perform async
network cleanup; call and await `shutdown` during normal operation. The daemon's
lease expiry and fault policy remain the final safety backstop.

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
src/frame.rs      joint targets and complete command-frame validation
src/control.rs    automatic renewal and ergonomic control lifecycle
src/state.rs      typed latest-value telemetry aggregation
src/client.rs     raw tonic client
src/lease.rs      raw lease ownership and renewal
src/command.rs    raw command sequencing and acknowledgements
src/telemetry.rs  raw event telemetry
src/endpoint.rs   HTTP/HTTPS, roots, timeout, and keepalive policy
```

See the repository [API guide](../../docs/api.md) and
[control and safety](../../docs/control-and-safety.md) for server behavior.
