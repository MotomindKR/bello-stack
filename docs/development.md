# Development guide

[Documentation index](README.md)

This document covers repository mechanics and review order. Runtime behavior is
documented in [`operations.md`](operations.md), the external contract in
[`api.md`](api.md), and deeper structure and configuration in
[`architecture.md`](architecture.md) and
[`configuration.md`](configuration.md).

## Nix and direnv

The checked-in Nix flake supplies Rust/Cargo, rustfmt, Clippy, system `protoc`,
Buf, `grpcurl`, can-utils, iproute2, and native build dependencies. Enter it
automatically:

```sh
direnv allow
```

or explicitly:

```sh
nix develop
```

The shell places Cargo artifacts under
`${XDG_CACHE_HOME:-$HOME/.cache}/bello-stack/cargo-target`, keeping the repository
and Nix source snapshots small. The workspace declares Rust 1.96 as its minimum
supported version; the concrete Nix inputs are pinned by
[`flake.lock`](../flake.lock). See [`flake.nix`](../flake.nix) and
[`.envrc`](../.envrc).

## Protobuf toolchain policy

Cargo does not download or vendor `protoc`. The shared API crate's build script
invokes `tonic-prost-build` with the `protoc` found on `PATH`; the Nix shell
provides it. This keeps compiler selection explicit and reproducible. See
[`build.rs`](../components/bello-hardware-api/build.rs).

Buf owns source formatting and linting under the repository-level
[`buf.yaml`](../buf.yaml). When changing the API, edit
[`hardware.proto`](../components/bello-hardware-api/proto/bello/hardware/v1/hardware.proto),
run Buf, then compile/test the Rust conversions and clients. The contract is
greenfield: backward compatibility is not currently required, but field types,
enum values, workflow changes, and downstream test updates should still be
reviewed together.

## Workspace and source layout

The Cargo workspace contains seven components:

- [`bello-hardware-api`](../components/bello-hardware-api): protobuf source and
  generated tonic client/server types;
- [`bello-hardware-client`](../components/bello-hardware-client): maintained
  robot/group/joint client with internal lease, command, and state machinery;
- [`bello-hardwared`](../components/bello-hardwared): daemon and `run`/`check`
  library entrypoints;
- [`bello-rerun`](../components/bello-rerun): optional read-only gRPC observer
  and Rerun output;
- [`robstride-protocol`](../components/robstride-protocol): Robstride wire model,
  frame layout, and codec helpers;
- [`robstride-emulator`](../components/robstride-emulator): thin SocketCAN
  actuator emulator;
- [`bello-hardware-e2e`](../components/bello-hardware-e2e): process-level,
  Cargo-driven vcan integration test.

Conceptually, configuration assembles transport and device workers, group
control consumes narrow hardware handles, and the API/observers present daemon
state. This is an ownership and data-flow model, not a claim that Rust module
dependencies are strictly one-way. The maintained directory map and layering rationale are in
[`architecture.md`](architecture.md). Avoid a generic transport abstraction
that erases CAN/serial queue and reconnect semantics; share small
lifecycle/backoff primitives instead.

## Normal checks

Run formatting, lint, tests, and protobuf checks from the repository root:

```sh
cargo fmt --all --check
cargo clippy --workspace --all-targets --all-features -- -D warnings
cargo test --workspace --all-features
buf format --diff --exit-code
buf lint
```

During iteration, use focused package/test commands first, then finish with the
workspace commands. Configuration-only validation is quick and does not open
hardware:

```sh
cargo run -p bello-hardwared -- \
  --check --config components/bello-hardwared/config/bello.yaml
cargo run -p bello-hardwared -- \
  --check --config components/bello-hardwared/config/vcan.yaml
cargo run -p robstride-emulator -- \
  --check --config components/robstride-emulator/config/vcan.yaml
cargo run -p bello-rerun -- \
  --check --config components/bello-rerun/config/bello.yaml
```

Do not hide warnings or make host hardware a prerequisite for ordinary unit and
workspace tests. Tests should assert behavior, protocol semantics, safety
invariants, or parsing boundaries—not merely prove that a refactor preserved a
type name or call path.

## Host-vcan integration test

The process-level scenario is an ignored Cargo integration test. Normal
`cargo test` compiles it but does not require `vcan0`. To run it, create and
bring up a host interface:

```sh
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set dev vcan0 up
cargo test -p bello-hardware-e2e --test vcan -- \
  --ignored --nocapture --test-threads=1
```

`cargo e2e` is an alias for the final command. The single test thread is
required because both scenarios exclusively own `vcan0`. Set
`BELLO_E2E_CAN_INTERFACE=vcan1` to select another interface; the test writes that
name into both generated runtime configurations. It also chooses a free
loopback gRPC port, controls fault phases by suspending/resuming the emulator,
and verifies child-process liveness. Details are in the
[E2E README](../components/bello-hardware-e2e/README.md) and
[`tests/vcan.rs`](../components/bello-hardware-e2e/tests/vcan.rs).

## CI

[GitHub Actions](../.github/workflows/ci.yml) runs two jobs:

1. hermetic formatting, workspace Clippy with warnings denied, all-feature
   workspace tests, and Buf format/lint;
2. after those pass, creation of `vcan0` and the ignored Cargo integration test.

Keep local commands aligned with CI. A change is not complete when only the
focused test passes; run the proportionate workspace checks before handoff.

## Suggested review order

For a first full review, read in this order:

1. [`hardware.proto`](../components/bello-hardware-api/proto/bello/hardware/v1/hardware.proto)
   and [`api.md`](api.md) for the external contract.
2. [`config/model.rs`](../components/bello-hardwared/src/config/model.rs),
   [`config/resolved.rs`](../components/bello-hardwared/src/config/resolved.rs),
   and the owning `spec.rs` modules under
   [`hardware/`](../components/bello-hardwared/src/hardware),
   [`control/`](../components/bello-hardwared/src/control), and
   [`api/grpc/`](../components/bello-hardwared/src/api/grpc) for accepted input,
   local validation, and resolved topology. The observer has its own independent
   configuration boundary under [`bello-rerun`](../components/bello-rerun).
3. [`domain/`](../components/bello-hardwared/src/domain) for IDs, commands,
   telemetry, and errors shared across boundaries.
4. [`app/run.rs`](../components/bello-hardwared/src/app/run.rs) and
   [`app/daemon.rs`](../components/bello-hardwared/src/app/daemon.rs) for startup,
   ownership, and shutdown order.
5. [`hardware/lifecycle.rs`](../components/bello-hardwared/src/hardware/lifecycle.rs),
   then [`hardware/can/`](../components/bello-hardwared/src/hardware/can) and
   [`hardware/serial/`](../components/bello-hardwared/src/hardware/serial) for
   transport supervision.
6. [`hardware/robstride/`](../components/bello-hardwared/src/hardware/robstride)
   and [`hardware/imu/`](../components/bello-hardwared/src/hardware/imu) for
   device behavior.
7. [`control/`](../components/bello-hardwared/src/control) for group FSM, safety
   policy, mailbox scheduling, and fixed-rate control.
8. [`api/grpc/`](../components/bello-hardwared/src/api/grpc),
   [`bello-hardware-client`](../components/bello-hardware-client), and
   [`telemetry.rs`](../components/bello-hardwared/src/telemetry.rs) for external
   presentation, followed by [`bello-rerun`](../components/bello-rerun) for one
   read-only observer implementation.
9. [`robstride-emulator`](../components/robstride-emulator) and the
   [`vcan` integration test](../components/bello-hardware-e2e/tests/vcan.rs) for
   executable examples of the full boundary.

For a focused change, start at the relevant public boundary and follow the same
dependency direction. Review safety reactions and shutdown effects whenever a
change touches queues, leases, lifecycle transitions, transport freshness, or
command validity.
