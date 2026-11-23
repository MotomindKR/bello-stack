# Bello hardware daemon

`bello-hardwared` is the production monolithic hardware process. It owns
Robstride CAN actuators, EBIMU serial sensors, generic motor groups, safety,
leases, gRPC, and telemetry.

Enabled AGX power boards can hand the physical emergency button to hardwared:
pressing it revokes all leases and damps enabled motor groups. Orderly shutdown
restores the board's hardware cutoff and firmware latch. See the
[power-board guide](../../docs/power_board.md#physical-emergency-button-and-damping)
for startup, release, telemetry-loss, and crash behavior.

Higher-level robot behavior, kinematics, and pose control are outside this
component.

## Run

From the repository's Nix/direnv environment, validate configuration without
opening hardware:

```sh
cargo run -p bello-hardwared -- \
  --check --config components/bello-hardwared/config/bello.yaml
```

Run the daemon:

```sh
cargo run -p bello-hardwared -- \
  --config components/bello-hardwared/config/bello.yaml
```

`--config` is required: hardwared never selects a deployment implicitly. The
maintained Bello physical deployment is this component's `config/bello.yaml`.
It selects groups from
[`descriptions/bello/hardware.yaml`](../../descriptions/bello/hardware.yaml)
and maps that description's logical buses and serial devices to host names.
Stable bitrate, host ID, sensor rate, and timeout settings remain in the
description. Its referenced canonical URDF supplies direct and differential
joint position and velocity limits plus scalar feedforward-torque caps derived
from effort limits. It also supplies the fixed, coincident transform from each
IMU's physical source frame to the logical frame published by hardwared. The
daemon does not create SocketCAN links or change their bitrate. A `gsusb` transport is
configured directly over USB using its serial number and channel; see the
[configuration guide](../../docs/configuration.md#description-logical_buses).

The physical and vCAN configurations expose `lower_body` and `upper_body`.
Each arm has six motors, and the sphere end effectors are fixed to the wrist
links rather than assigned separate motor coordinates.

The Milly profile is a seven-actuator single arm adapted from the canonical
configuration in `motomind_platform`. Validate or run it with
`config/milly.yaml`; `config/milly-vcan.yaml` binds the same stable hardware
description to `vcan0` and pairs with the emulator's
`config/milly-vcan.yaml`. Both expose one atomic `arm` control group.

Visualization and recording are deliberately outside this process. External
tools can subscribe to the daemon's read-only gRPC telemetry stream without
affecting hardware ownership.

Validate the [Bello Mini physical](config/bello-mini.yaml) or [vCAN](config/bello-mini-vcan.yaml)
profile without opening hardware:

```sh
cargo run -p bello-hardwared -- \
  --check --config components/bello-hardwared/config/bello-mini-vcan.yaml
```

Before physical operation, complete the [Mini calibration checks](../../descriptions/bello_mini/motor_topology.yaml).

## Interfaces

- The protobuf contract is
  [`hardware.proto`](../bello-hardware-api/proto/bello/hardware/v1/hardware.proto).
- Rust applications can use the maintained
  [`bello-hardware-client`](../bello-hardware-client) wrapper.
- The maintained physical-hardware example is [`config/bello.yaml`](config/bello.yaml).
- The full Bello MuJoCo HIL example is
  [`config/bello-vcan.yaml`](config/bello-vcan.yaml).
- Bello motor identity and group membership live only in
  [`descriptions/bello/hardware.yaml`](../../descriptions/bello/hardware.yaml).
- Milly motor identity and group membership live only in
  [`descriptions/milly/hardware.yaml`](../../descriptions/milly/hardware.yaml).
- RobStride protocol limits live only in
  [`descriptions/robstride/models.yaml`](../../descriptions/robstride/models.yaml).
- The Rust library intentionally exposes only the `run` and `check` entrypoints;
  shared generated protobuf types live in [`bello-hardware-api`](../bello-hardware-api).

## Documentation

- [Architecture and ownership](../../docs/architecture.md)
- [Configuration reference](../../docs/configuration.md)
- [Hardware and reconnect behavior](../../docs/hardware.md)
- [Motor-group control and safety](../../docs/control-and-safety.md)
- [gRPC and telemetry](../../docs/api.md)
- [Operation and troubleshooting](../../docs/operations.md)

## Source layout

This is an abridged component map. The canonical ownership and worker map is in
the [architecture guide](../../docs/architecture.md#source-layout).

```text
src/
  app/                 assembly, ownership, and shutdown
  api/grpc/            tonic service, conversion, inventory, and streams
  config/              YAML model and validated topology resolution
  control/             group FSM, worker, commands, and safety
  domain/              typed identifiers, commands, telemetry, and errors
  hardware/can/        CAN queues, handles, sessions, and reconnect
  hardware/serial/     serial queues, handles, sessions, and reconnect
  hardware/robstride/  motor protocol device and feedback health
  hardware/imu/        EBIMU configuration, parser, and sensor worker
  hardware/power_board/ COG telemetry and optional AGX emergency handoff
  hardware/lifecycle   shared device state and reconnect backoff
  lease.rs             exclusive group lease store
  telemetry.rs         bounded event broadcast
```

The shared API crate's `build.rs` uses `protoc` from `PATH` and does not
download a compiler.
