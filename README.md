# Bello stack

Bello stack is a Linux-oriented Rust workspace for actuator and IMU hardware
integration. Its production hardware component is one monolithic daemon that
owns SocketCAN buses, Robstride actuators, serial devices, EBIMU sensors,
motor-group control, safety, gRPC, and telemetry. An optional standalone Rerun
observer visualizes that telemetry through the read-only gRPC API.

Robot-level kinematics, pose generation, and behavior policy are intentionally
outside this repository.

## Workspace

| Component | Role | Artifact |
| --- | --- | --- |
| [`bello-hardwared`](components/bello-hardwared) | Production hardware ownership, control, safety, gRPC, and telemetry | `bello-hardwared` binary and narrow Rust library entrypoint |
| [`bello-hardware-api`](components/bello-hardware-api) | Shared protobuf contract and generated tonic types | Rust library |
| [`bello-hardware-client`](components/bello-hardware-client) | Robot/group/joint control and aggregated state; raw gRPC access is an explicit escape hatch | Rust library |
| [`bello-rerun`](components/bello-rerun) | Optional read-only gRPC telemetry visualization and recording | `bello-rerun` binary |
| [`robstride-protocol`](components/robstride-protocol) | Robstride extended-CAN frames, codecs, and model limits | Rust library |
| [`robstride-emulator`](components/robstride-emulator) | Thin SocketCAN actuator emulator with deterministic faults | `robstride-emulator` binary |
| [`bello-hardware-e2e`](components/bello-hardware-e2e) | Process-level gRPC and vCAN scenarios | Cargo integration tests |

Shared Cargo, Nix, Buf, and CI configuration remains at the repository root.
Component-specific source and examples stay under `components/`.

## Quick start

The checked-in direnv configuration enters the pinned Nix development shell:

```sh
direnv allow
```

Validate the production example without opening hardware:

```sh
cargo run -p bello-hardwared -- \
  --check --config components/bello-hardwared/config/bello.yaml
```

Run it after the configured SocketCAN and serial devices are available:

```sh
cargo run -p bello-hardwared -- \
  --config components/bello-hardwared/config/bello.yaml
```

Optionally record its read-only telemetry from another shell:

```sh
cargo run -p bello-rerun -- \
  --config components/bello-rerun/config/bello.yaml
```

For a hardware-free actuator path, use the vCAN configuration and Robstride
emulator described in the [operations guide](docs/operations.md).

## Documentation

Start with the [documentation index](docs/README.md), or go directly to:

- [Architecture and runtime ownership](docs/architecture.md)
- [Configuration and validation](docs/configuration.md)
- [CAN, Robstride, serial, and EBIMU behavior](docs/hardware.md)
- [Motor-group control and safety](docs/control-and-safety.md)
- [gRPC API and telemetry](docs/api.md)
- [Deployment, emulation, and troubleshooting](docs/operations.md)
- [Development and testing](docs/development.md)

The protobuf contract is
[`hardware.proto`](components/bello-hardware-api/proto/bello/hardware/v1/hardware.proto).
Rust applications should normally start with the
[`bello-hardware-client` guide](components/bello-hardware-client/README.md).

## Repository checks

```sh
cargo fmt --all --check
cargo clippy --workspace --all-targets --all-features -- -D warnings
cargo test --workspace --all-features
buf format --diff --exit-code
buf lint
```

The host-vCAN end-to-end scenario is explicit because it requires a Linux CAN
interface:

```sh
cargo e2e
```

The hardware API and configuration are greenfield. Backward compatibility is
not required; obsolete fields are removed rather than retained as aliases or
migration scaffolding.
