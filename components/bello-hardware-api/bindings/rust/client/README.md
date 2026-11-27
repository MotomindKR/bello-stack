# Bello hardware Rust client

## Read hardware

```rust
use bello_hardware_client::{Robot, RobotStateChange};

#[tokio::main]
async fn main() -> Result<(), bello_hardware_client::Error> {
    let robot = Robot::connect("http://127.0.0.1:50051").await?;
    println!("{:?}", robot.group_names().collect::<Vec<_>>());

    let mut states = robot.states(Default::default()).await?;
    while let Some(state) = states.changed().await? {
        if let Some(RobotStateChange::Motor(motor)) = state.last_change() {
            println!("{}: {} rad", motor.motor, motor.position);
        }
    }
    Ok(())
}
```

- Connection loads inventory. Use `robot.refresh_inventory().await?` on a mutable handle to pick up changes. Existing group handles and readers keep their snapshot.
- Each `changed()` applies one telemetry event. `generation()` changes when a reconnect clears cached state.
- Endpoints accept `http://`, `https://`, or `unix:///absolute/path`. For mTLS, pass `ConnectOptions::tls` a `TlsOptions` with a CA and client identity. [Transport configuration](../../../../../docs/configuration.md#grpc)

## Control a motor group

Replace `leg` with a configured group. A command frame takes one target per coordinate.

```rust
use bello_hardware_client::{CoordinateTarget, Robot};

#[tokio::main]
async fn main() -> Result<(), bello_hardware_client::Error> {
    let robot = Robot::connect("http://127.0.0.1:50051").await?;
    let group = robot.group("leg")?;
    let frame = group.frame(group.coordinate_names().map(|name| {
        (name, CoordinateTarget::damping(1.0))
    }))?;

    let mut control = group.control("example").await?;
    control.start(frame.clone()).await?;
    control.send(frame).await?;
    control.shutdown().await
}
```

| Operation | Behavior |
| --- | --- |
| `send(frame).await?` | Wait for capacity and daemon acknowledgement. |
| `try_submit(frame)?` | Return an awaitable receipt or `Error::QueueFull`. Default capacity is one. |
| `stop().await?` / `resume(frame).await?` | Hold damping or resume with fresh targets. Repeated stops keep the group damped. |
| `disable().await?` | Disable the motors directly and keep the lease. |
| `shutdown().await?` / `release().await?` | Shut down to disable and release. Use release alone to leave a running group damped. |

A rejection closes the command stream. Resume after confirmed damping. Check group state before recovery after an ambiguous result. [Control and safety](../../../../../docs/control-and-safety.md) · [Direct RPCs](../../../../../docs/api.md#direct-clients)
