# Robstride emulator

`robstride-emulator` runs simulated Robstride actuators on SocketCAN for hardwared
integration and fault testing. For rigid-body physics, use [bello-mujoco](../bello-mujoco/).

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
[`config/bello-vcan.yaml`](../bello-hardwared/config/bello-vcan.yaml).
For the seven-actuator Milly profile, use `config/milly-vcan.yaml` here and
[`config/milly-vcan.yaml`](../bello-hardwared/config/milly-vcan.yaml) in
hardwared.

## Fault scenarios

Use [config/faults.yaml](config/faults.yaml) to schedule feedback loss, delay,
jitter, drive faults, temperature changes, and operating-mode changes.
Bring the vCAN link down separately to test transport loss and reconnection.

See [hardware behavior](../../docs/hardware.md), [operations](../../docs/operations.md),
and [development and testing](../../docs/development.md) for the complete workflow.
