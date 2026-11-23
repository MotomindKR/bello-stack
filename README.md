# Bello stack

`bello-hardwared` controls motors and reads sensors on Bello and Milly robots.
Use the [Rust](components/bello-hardware-client) or [Python](components/bello-hardware-python) client to command a motor group and read telemetry.
On Linux, the [MuJoCo simulator](components/bello-mujoco) lets you use the same clients without a physical robot.

From the repository root, check the Milly arm configuration without connecting hardware:

```sh
cargo run -p bello-hardwared -- --check --config components/bello-hardwared/config/milly-gsusb.yaml
```

## Documentation

- [Build or install the daemon](docs/desktop.md)
- [Connect hardware or start a simulator](docs/operations.md)
- [Configure a robot](docs/configuration.md) · [Robot descriptions](descriptions)
- [Hardware API](docs/api.md) · [Motor control and safety](docs/control-and-safety.md)
