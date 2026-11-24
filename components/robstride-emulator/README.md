# Robstride emulator

`robstride-emulator` simulates Robstride motors and protocol faults. For rigid-body
physics, use [bello-sim](../bello-sim/).

## Run

The default [Milly profile](config/milly.yaml) uses loopback UDP. Start the emulator
and hardwared in separate terminals:

```sh
cargo run -p robstride-emulator
cargo run -p bello-hardwared -- --config components/bello-hardwared/config/milly-sim.yaml
```

Use `--config PATH` to load another profile and `--check` to validate it. For
Linux SocketCAN tools, bring up `vcan0` and select `config/milly-vcan.yaml`
in both components.

## Fault scenarios

[config/faults.yaml](config/faults.yaml) exercises temperature, loss of feedback,
drive faults, and operating-mode changes on a two-motor UDP bus. Its host ID is
253. To exercise the default Milly profile, copy the `events` list into a local
copy of [config/milly.yaml](config/milly.yaml).
