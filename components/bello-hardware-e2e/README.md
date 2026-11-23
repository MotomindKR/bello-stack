# Bello hardware end-to-end tests

Process-level tests run hardwared and the Robstride emulator over gRPC and
SocketCAN. Run the transport tests without CAN hardware:

```sh
cargo test -p bello-hardware-e2e --test transports
```

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

Run only the latency probe with:

```sh
cargo test --release -p bello-hardware-e2e --test vcan \
  grpc_can_latency_distribution -- --ignored --nocapture
```

The probe measures command-to-CAN and feedback-to-telemetry latency. Set
`BELLO_LATENCY_SAMPLES` to change the sample count and `BELLO_LATENCY_CONTROL_RATE_HZ`
to change the control frequency.

Set `BELLO_E2E_CAN_INTERFACE` to select another Linux CAN interface. The
canonical multi-bus scenario uses `vcan0,vcan1`; override that pair with
`BELLO_E2E_CAN_INTERFACES`. Tests fail explicitly when host prerequisites are
absent; ordinary `cargo test` does not execute ignored scenarios.

See [development and testing](../../docs/development.md) for the full check
matrix and [operations](../../docs/operations.md) for manual emulator use.
