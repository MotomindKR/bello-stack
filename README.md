# Bello stack

This repository is a workspace for Bello runtime, hardware, tooling, and bridge
components. Shared development infrastructure stays at the repository root;
each independently scoped component lives under `components/`.

## Components

- [`bello-hardwared`](components/bello-hardwared): monolithic Robstride and
  EBIMU hardware daemon with gRPC control and optional Rerun logging.
- [`robstride-protocol`](components/robstride-protocol): shared Robstride CAN
  wire types and codec.
- [`robstride-emulator`](components/robstride-emulator): thin SocketCAN/vcan
  actuator emulator with deterministic fault scenarios.
- [`bello-hardware-e2e`](components/bello-hardware-e2e): process-level gRPC and
  SocketCAN integration scenarios using the actual daemon and emulator binaries.

## Development environment

Direnv loads the pinned Nix development shell automatically:

```sh
direnv allow
```

The root is also a Cargo workspace, so checks can be run across all Rust
components:

```sh
cargo test --workspace --all-features
cargo clippy --workspace --all-targets --all-features -- -D warnings
buf format -d --exit-code
buf lint
```

The hardware end-to-end scenario has an explicit vcan prerequisite and therefore
runs separately from hermetic Cargo tests:

```sh
cargo e2e
```

Buf configuration is shared at the repository root. Protobuf modules use the
`STANDARD` lint profile and `FILE`-level breaking-change detection.
