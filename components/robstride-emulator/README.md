# Robstride emulator

`robstride-emulator` is a thin SocketCAN actuator emulator for exercising
`bello-hardwared` through its real CAN transport and Robstride device layers. It
is not a dynamics simulator. It implements only the protocol operations used by
hardwared and returns bounded first-order actuator feedback.

## Run with vcan

Create a virtual CAN interface once per boot:

```sh
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

Validate and run the emulator:

```sh
cargo run -p robstride-emulator -- --check --config components/robstride-emulator/config/vcan.yaml
cargo run -p robstride-emulator -- --config components/robstride-emulator/config/vcan.yaml
```

Omitting `--config` selects that component-local `vcan.yaml` regardless of the
current working directory.

Run `bello-hardwared` in another shell with a bus
whose `interface` is `vcan0`. The emulator responds to run-mode writes, active
report configuration, enable, stop/clear-fault, and impedance-control frames.
Requests addressed to a host other than the configured `host_id` are ignored;
incoming traffic never mutates the emulator's host identity.

## Fault scenarios

The YAML `events` list applies deterministic changes at milliseconds after
startup. Supported actions are:

- `drop_feedback`
- `set_feedback_delay`
- `set_feedback_jitter`
- `set_fault_bits`
- `set_temperature` and `clear_temperature`
- `set_mode_status` and `clear_mode_status`

Use `ip link set vcan0 down` separately to exercise transport loss and SocketCAN
reconnection. Bringing it back with `ip link set vcan0 up` restores the transport;
hardwared still requires explicit group recovery before motion resumes.

`config/faults.yaml` is a ready-to-run scenario covering over-temperature,
feedback loss, reported fault bits, and wrong operating mode.
