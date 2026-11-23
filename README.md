# Bello stack

Bello stack provides a Rust actuator and IMU daemon for Linux, macOS, and
Windows, with Rust and Python clients. The daemon owns SocketCAN or GS-USB
buses, Robstride actuators, serial devices, EBIMU sensors, motor-group control,
safety, gRPC, and telemetry.

Behavior policy remains outside this repository. Robot-level local pose
generation is provided as a separate read-only service and never participates
in hardware command safety.

## Workspace

- [Hardware daemon](components/bello-hardwared) and its [Rust](components/bello-hardware-client) and [Python](components/bello-hardware-python) clients
- [Robot descriptions](descriptions), [MuJoCo simulator](components/bello-mujoco), and [pose estimator](components/bello-state-estimatord)
- gRPC contracts for [hardware](components/bello-hardware-api), [cameras](components/bello-camera-api), and [pose](components/bello-pose-api)

GS-USB adapters work on all three desktop platforms; SocketCAN is Linux-only.
The default Cargo build produces `bello-hardwared`.
See [desktop builds and USB setup](docs/desktop.md) for native build artifacts,
platform requirements, and the Milly GS-USB deployment. The MuJoCo host
integration and vCAN tests use Linux.

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

[Guides and troubleshooting](docs/README.md) · [Development checks](docs/development.md#normal-checks) · [Hardware protobuf](components/bello-hardware-api/proto/bello/hardware/v1/hardware.proto)

Cartesian teleoperation lives in the separate
[`MotomindKR/bello_torso_control`](https://github.com/MotomindKR/bello_torso_control) repository,
which pins this stack as a submodule and uses its leased hardware client.
