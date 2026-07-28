# Bello hardware daemon

`bello-hardwared` is the production monolithic hardware process. It owns
Robstride CAN actuators, EBIMU serial sensors, generic motor groups, safety,
leases, gRPC, and telemetry.

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

The default configuration path is this component's `config/bello.yaml`.
SocketCAN interfaces and serial-device settings must already exist and match
the YAML. The daemon does not create CAN links or change their bitrate.

That physical configuration exposes `lower_body` and `upper_body` for normal
control, plus an independent `wrist_yaw` group containing `left_arm_7` and
`right_arm_7`. The separate group lets an application opt into holding those
otherwise-unactuated motors without changing the 12-coordinate `upper_body`
contract. The sphere-hand vcan configuration intentionally omits it because
its emulator does not publish those two motor IDs.

Visualization and recording are deliberately outside this process. External
tools can subscribe to the daemon's read-only gRPC telemetry stream without
affecting hardware ownership.

## Interfaces

- The protobuf contract is
  [`hardware.proto`](../bello-hardware-api/proto/bello/hardware/v1/hardware.proto).
- Rust applications can use the maintained
  [`bello-hardware-client`](../bello-hardware-client) wrapper.
- The maintained physical-hardware example is [`config/bello.yaml`](config/bello.yaml).
- The full Bello MuJoCo HIL example is
  [`config/bello-vcan.yaml`](config/bello-vcan.yaml).
- The minimal two-motor emulator example is [`config/vcan.yaml`](config/vcan.yaml).
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
  hardware/lifecycle   shared device state and reconnect backoff
  lease.rs             exclusive group lease store
  telemetry.rs         bounded event broadcast
```

The shared API crate's `build.rs` uses `protoc` from `PATH` and does not
download a compiler.
