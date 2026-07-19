# Robstride emulator

`robstride-emulator` is a thin SocketCAN actuator emulator for exercising
`bello-hardwared` through the real CAN transport and Robstride protocol layers.
It is intended for integration and fault testing, not rigid-body simulation.

The emulator supports the operations currently used by hardwared: run-mode
writes, active-report configuration, enable, stop and clear-fault, and impedance
control. It returns bounded first-order feedback.

## Run

Create a vCAN interface once per boot:

```sh
sudo ip link add dev vcan0 type vcan
sudo ip link set dev vcan0 up
```

Validate and run the default example:

```sh
cargo run -p robstride-emulator -- --check \
  --config components/robstride-emulator/config/vcan.yaml
cargo run -p robstride-emulator -- \
  --config components/robstride-emulator/config/vcan.yaml
```

Omitting `--config` selects this component's `config/vcan.yaml` regardless of
the current directory. Run hardwared separately with its matching
[`config/vcan.yaml`](../bello-hardwared/config/vcan.yaml).

## Fault scenarios

Scheduled YAML events can:

- drop, delay, or jitter feedback;
- set reported motor fault bits;
- raise or clear temperature;
- set or clear the reported operating mode.

[`config/faults.yaml`](config/faults.yaml) contains a deterministic example.
Bring the vCAN link down separately to test transport loss and reconnection.

See [hardware behavior](../../docs/hardware.md), [operations](../../docs/operations.md),
and [development and testing](../../docs/development.md) for the complete workflow.
