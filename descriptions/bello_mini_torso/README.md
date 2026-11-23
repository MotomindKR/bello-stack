# Bello Mini Torso

Both hip motors drive one pitch joint. Enable and command them together through the
`torso_pitch` group. Before physical bring-up, verify motor addresses, directions,
encoder zeros, travel stops, and load sharing on the installed assembly.

## Virtual bring-up

From the repository root in the Linux Nix shell, validate the profiles:

```sh
cargo run -p bello-hardwared -- --check \
  --config components/bello-hardwared/config/bello-mini-torso.yaml
cargo run -p bello-hardwared -- --check \
  --config components/bello-hardwared/config/bello-mini-torso-vcan.yaml
cargo run -p bello-mujoco -- check \
  --config components/bello-mujoco/config/bello_mini_torso.yaml
```

Set up the virtual CAN and UART devices using the [operations guide](../../docs/operations.md).
Start each process in its own terminal:

```sh
cargo run -p bello-mujoco -- run \
  --config components/bello-mujoco/config/emulator_mini_torso.yaml
cargo run -p bello-hardwared -- \
  --config components/bello-hardwared/config/bello-mini-torso-vcan.yaml
```

Use the [hardware API control workflow](../../docs/api.md#normal-control-workflow) to move the torso.

## Regenerate the grippers

```sh
uv run --with numpy --with scipy --with trimesh scripts/mini_torso_grippers.py
uv run --with numpy --with scipy --with trimesh \
  python -m unittest discover -s scripts -p test_mini_torso_grippers.py
```
