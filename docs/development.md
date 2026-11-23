# Development guide

## Nix and direnv

The full workspace and vCAN tests run on Linux. For native executable builds on
Linux, macOS, and Windows, see [desktop builds](desktop.md).

Enter the development shell with direnv or Nix:

```sh
direnv allow
```

or explicitly:

```sh
nix develop
```

The shell places Cargo artifacts under
`${XDG_CACHE_HOME:-$HOME/.cache}/bello-stack/cargo-target`, keeping the repository
and Nix source snapshots small.

## Protobuf toolchain policy

The API crates need `protoc` on `PATH`. The Nix shell supplies it.
After editing a `.proto`, run Buf and rebuild its producers and clients.
The hardware API and configuration can change incompatibly.

The Python package commits private generated bindings so wheels do not require
a protobuf compiler. Regenerate them after every contract change and review the
result with the handwritten conversions:

```sh
python components/bello-hardware-python/scripts/generate_proto.py
```

## Normal checks

Run formatting, lint, tests, and protobuf checks from the repository root:

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
python -m hatchling build
```

Configuration-only validation is quick and does not open
hardware:

```sh
cargo run -p bello-hardwared -- \
  --check --config components/bello-hardwared/config/bello.yaml
cargo run -p bello-hardwared -- \
  --check --config components/bello-hardwared/config/milly-sim.yaml
cargo run -p robstride-emulator -- --check
```

Do not hide warnings or make host hardware a prerequisite for ordinary unit and
workspace tests. Test behavior, protocol semantics, safety invariants, and
parsing boundaries. The URDF and hardware descriptions own joint geometry, limits,
and wiring. Exercise the description parser and kinematics with small test
fixtures. Use the daemon's `--check` command to validate a shipped profile.

## Host-vcan integration test

Run `cargo e2e` for gRPC and simulated motor behavior, including faults, recovery,
and coupled joints on two buses. These tests run in the normal workspace suite
without creating network interfaces. The fault scenario also runs the Python
client from the repository's development shell.

The Linux SocketCAN checks capture frames from the kernel interface. To run them:

```sh
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set dev vcan0 up
cargo e2e-vcan
```

Set `BELLO_E2E_CAN_INTERFACE` to use a different isolated vCAN interface.

To measure command-to-CAN and feedback-to-telemetry latency:

```sh
cargo test --release -p bello-hardware-e2e --test vcan \
  grpc_can_latency_distribution -- --ignored --nocapture
```

Set `BELLO_LATENCY_SAMPLES` and `BELLO_LATENCY_CONTROL_RATE_HZ` to change the
sample count and control frequency.
