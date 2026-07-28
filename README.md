# Bello stack

Bello stack is a Linux-oriented hardware workspace centered on a Rust actuator
and IMU daemon with Rust and Python clients. Its production hardware component
is one monolithic daemon that
owns SocketCAN buses, Robstride actuators, serial devices, EBIMU sensors,
motor-group control, safety, gRPC, and telemetry.

Behavior policy remains outside this repository. Robot-level local pose
generation is provided as a separate read-only service and never participates
in hardware command safety.

## Workspace

| Component | Role | Artifact |
| --- | --- | --- |
| [`bello-hardwared`](components/bello-hardwared) | Production hardware ownership, control, safety, gRPC, and telemetry | `bello-hardwared` binary and narrow Rust library entrypoint |
| [`bello-hardware-api`](components/bello-hardware-api) | Shared protobuf contract and generated tonic types | Rust library |
| [`bello-hardware-client`](components/bello-hardware-client) | Robot/group/coordinate control and aggregated state; raw gRPC access is an explicit escape hatch | Rust library |
| [`bello-hardware-python`](components/bello-hardware-python) | Thin blocking robot/group/coordinate client with background lease and telemetry I/O | Pure-Python package |
| [`bello-upper-body-teleop`](components/bello-upper-body-teleop) | Dynamixel teaching-arm input mapped to the daemon's leased upper-body group | Python application |
| [`bello-pose-api`](components/bello-pose-api) | Neutral streaming contract for external root-pose providers | Rust library and protobuf descriptor |
| [`bello-state-estimatord`](components/bello-state-estimatord) | URDF/IMU/encoder contact-aided local pose provider | `bello-state-estimatord` binary |
| [`bello-robot-description`](components/bello-robot-description) | Shared URDF hierarchy, visuals, joint limits, differential transmissions, and kinematics | Rust library |
| [`bello-visualizer`](components/bello-visualizer) | Native URDF monitor and toggle-controlled multi-group command UI | `bello-visualizer` binary |
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

For hardware-free checks, use the generated process-level vCAN test or the full
Bello emulator integration described in the [operations guide](docs/operations.md).

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
Python applications should start with the
[`bello-hardware` package guide](components/bello-hardware-python/README.md).
The native viewer and its single-file YAML contract are documented in the
[`bello-visualizer` guide](components/bello-visualizer/README.md).

## Repository checks

```sh
cargo fmt --all --check
cargo clippy --workspace --all-targets --all-features -- -D warnings
cargo test --workspace --all-features
buf format --diff --exit-code
buf lint
cd components/bello-hardware-python
ruff format --check .
ruff check .
mypy src/bello_hardware tests
pytest
cd ../bello-upper-body-teleop
ruff format --check .
ruff check .
mypy src/bello_upper_body_teleop tests
pytest
```

The host-vCAN end-to-end scenario is explicit because it requires a Linux CAN
interface:

```sh
cargo e2e
```

The hardware API and configuration are greenfield. Backward compatibility is
not required; obsolete fields are removed rather than retained as aliases or
migration scaffolding.
