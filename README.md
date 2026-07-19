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

The hardware end-to-end test has an explicit host vcan prerequisite. It is an
ignored Cargo integration test, so ordinary workspace tests remain hermetic:

```sh
cargo test -p bello-hardware-e2e --test vcan -- --ignored --nocapture
```

`cargo e2e` is an alias for that command.

Buf configuration is shared at the repository root and protobuf modules use the
`STANDARD` lint profile. The hardware API is greenfield; backward compatibility
is not required, so obsolete fields are removed instead of retained as aliases
or reserved migration scaffolding.
