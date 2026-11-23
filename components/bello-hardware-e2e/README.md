# Bello hardware end-to-end tests

`bello-hardware-e2e` contains Cargo-native process integration tests for the
actual `bello-hardwared` and `robstride-emulator` implementations. The scenario
uses only their public gRPC and SocketCAN interfaces. Its canonical control
workflow uses `bello-hardware-client`; the latency probe intentionally keeps
the generated tonic client so the measurement path stays minimal. The scenario
also invokes the pure-Python package against those same daemon and emulator
processes.

## Tests

Ordinary workspace tests validate generated matching daemon and emulator
configurations and start the real daemon over server-authenticated TLS, mTLS,
and a Unix-domain socket without requiring host hardware. They verify private
CA trust, rejection without a client certificate, Rust and Python mTLS clients,
socket permissions, stale-socket replacement, and shutdown cleanup.

The ignored host-vCAN scenario exercises:

- the ergonomic `Robot`/`ControlSession` path, including automatic renewal and
  confirmed command frames;
- the blocking Python `Robot.states()` and `Robot`/`ControlSession` paths
  through generated gRPC, including a complete daemon state frame;
- lease acquisition, release, and expiry;
- motor enable, command-triggered control, and periodic retransmission;
- complete command streaming;
- the canonical 13-motor lower-body group across two buses, including exact
  differential-ankle command rejection;
- feedback-loss and command-watchdog faults;
- fault latching, explicit recovery, and re-enable;
- child-process liveness during fault injection.

The same test target also contains a release-mode latency probe. It correlates
unique command and feedback payloads to measure immediate `gRPC command -> vCAN
frame` and `vCAN feedback -> gRPC telemetry` distributions without adding
timestamps to the production path. The outbound probe fails when p95 delivery
approaches the periodic control-tick boundary instead of remaining
command-triggered.

## Run the vCAN scenario

Create and enable two CAN interfaces once per boot:

```sh
sudo ip link add dev vcan0 type vcan
sudo ip link set dev vcan0 up
sudo ip link add dev vcan1 type vcan
sudo ip link set dev vcan1 up
```

Then run:

```sh
cargo e2e
```

This expands to:

```sh
cargo test -p bello-hardware-e2e --test vcan -- \
  --ignored --nocapture --test-threads=1
```

Run only the latency probe with:

```sh
cargo test --release -p bello-hardware-e2e --test vcan \
  grpc_can_latency_distribution -- --ignored --nocapture
```

It collects 500 samples per direction by default. Override this with
`BELLO_LATENCY_SAMPLES`; use `BELLO_LATENCY_CONTROL_RATE_HZ` to measure another
motor-group frequency.

Set `BELLO_E2E_CAN_INTERFACE` to select another Linux CAN interface. The
canonical multi-bus scenario uses `vcan0,vcan1`; override that pair with
`BELLO_E2E_CAN_INTERFACES`. Tests fail explicitly when host prerequisites are
absent; ordinary `cargo test` does not execute ignored scenarios.

See [development and testing](../../docs/development.md) for the full check
matrix and [operations](../../docs/operations.md) for manual emulator use.
