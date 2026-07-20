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
- the blocking Python `Robot`/`ControlSession` path through generated gRPC;
- the headless visualizer `RuntimeHandle`, matching MJCF coordinates, and its
  configured complete-frame command rate without a renderer;
- lease acquisition, release, and expiry;
- motor enable and fixed-rate control;
- complete command streaming;
- feedback-loss and command-watchdog faults;
- fault latching, explicit recovery, and re-enable;
- child-process liveness during fault injection.

The same test target also contains a release-mode latency probe. It correlates
unique command and feedback payloads to measure `gRPC command -> vCAN frame`
and `vCAN feedback -> gRPC telemetry` distributions without adding timestamps
to the production path.

## Run the vCAN scenario

Create and enable a CAN interface once per boot:

```sh
sudo ip link add dev vcan0 type vcan
sudo ip link set dev vcan0 up
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

Set `BELLO_E2E_CAN_INTERFACE` to select another Linux CAN interface. The test
fails explicitly when its host prerequisite is absent; ordinary `cargo test`
does not execute the ignored scenario.

See [development and testing](../../docs/development.md) for the full check
matrix and [operations](../../docs/operations.md) for manual emulator use.
