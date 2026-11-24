# Motor emulator

The emulator simulates CAN motors and protocol faults. For rigid-body
physics, use [bello-sim](../bello-sim/).

## Run

The default [Milly profile](config/milly.yaml) uses loopback UDP. Run these commands
from the repository root in separate terminals:

```sh
cargo run --manifest-path components/motor-emulator/Cargo.toml
cargo run -p bello-hardwared -- --config components/bello-hardwared/config/milly-sim.yaml
```

Use `--config PATH` to load another profile and `--check` to validate it. For
Linux SocketCAN tools, follow the [vCAN setup](../../docs/operations.md#virtual-can-and-emulator)
to select matching profiles for both components.

## Fault scenarios

[config/faults.yaml](config/faults.yaml) exercises temperature, loss of feedback,
drive faults, and operating-mode changes on a two-motor UDP bus. Its host ID is
253. To exercise the default Milly profile, copy the `events` list into a local
copy of [config/milly.yaml](config/milly.yaml).
