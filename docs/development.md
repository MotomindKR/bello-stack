# Development guide

[Documentation index](README.md)

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
  --check --config components/bello-hardwared/config/bello-vcan.yaml
cargo run -p robstride-emulator -- \
  --check --config components/robstride-emulator/config/vcan.yaml
```

Do not hide warnings or make host hardware a prerequisite for ordinary unit and
workspace tests. Test behavior, protocol semantics, safety invariants, and
parsing boundaries. The URDF and hardware descriptions own joint geometry, limits,
and wiring. Exercise the description parser and kinematics with small test
fixtures. Use the daemon's `--check` command to validate a shipped profile.

## Host-vcan integration test

TLS, mTLS, and Unix-socket process tests run under normal `cargo test` without
host hardware. The vCAN control scenarios remain ignored Cargo integration
tests. To run the complete file, create and bring up two host interfaces:

```sh
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set dev vcan0 up
sudo ip link add dev vcan1 type vcan
sudo ip link set dev vcan1 up
cargo test -p bello-hardware-e2e --test vcan -- \
  --ignored --nocapture --test-threads=1
```

`cargo e2e` is an alias for the final command. The single test thread is
required because the scenarios exclusively own their shared interfaces. Set
`BELLO_E2E_CAN_INTERFACE=vcan2` to select the interface used by the single-bus
scenarios. Set `BELLO_E2E_CAN_INTERFACES=vcan2,vcan3` to select the ordered pair
used by the canonical multi-bus scenario. See the [E2E guide](../components/bello-hardware-e2e/README.md) for individual scenarios.
