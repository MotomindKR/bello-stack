# Bello hardware daemon

`bello-hardwared` controls motors and reads sensors over CAN and serial. Clients acquire a
lease to command a motor group through the [hardware API](../bello-hardware-api/).

On an AGX power board with emergency-button handoff enabled, pressing the button
revokes leases and damps enabled groups. See the [power-board guide](../../docs/power_board.md#physical-emergency-button-and-damping)
for release and power-cutoff behavior.

## Run

From the repository's Nix/direnv environment, validate configuration without
opening hardware:

```sh
cargo run -p bello-hardwared -- \
  --check --config components/bello-hardwared/config/bello.yaml
```

Run the daemon:

```sh
cargo run -p bello-hardwared -- \
  --config components/bello-hardwared/config/bello.yaml
```

Choose the [deployment profile](config/) for your robot and transport. Create
SocketCAN interfaces before startup. For direct USB CAN, configure the adapter
serial number and channel in the [bus configuration](../../docs/configuration.md#description-logical_buses).

Before running Bello Mini on hardware, complete the [calibration checks](../../descriptions/bello_mini/motor_topology.yaml).

## Documentation

- [Configuration](../../docs/configuration.md)
- [Control and safety](../../docs/control-and-safety.md)
- [Operation and troubleshooting](../../docs/operations.md)
- [Rust client](../bello-hardware-client/) and [Python client](../bello-hardware-python/)
