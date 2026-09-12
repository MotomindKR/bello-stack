# Native MuJoCo runtime

The headless physics adapter uses `mujoco-rs` and the existing
`robstride-emulator`/`robstride-protocol` engine. It does not implement a second
ankle conversion: native joint/tendon transmissions supply motor coordinates.
Impedance and torque limits are applied inside MuJoCo.

From the matching `bello_mujoco` source checkout on Linux:

```bash
nix develop .#rust-runtime
cargo test --manifest-path deps/bello-stack/Cargo.toml -p bello-mujoco
```

MuJoCo is pinned to **3.12.0**, matching training. Cargo uses the published
`mujoco-rs` 6.0.1 release; viewer, renderer, and automatic-download features are off.
Other build environments must set `MUJOCO_DYNAMIC_LINK_DIR` to a directory
containing `libmujoco.so` (Linux) or `libmujoco.dylib` (macOS), and configure the
loader path for the same installation. A version mismatch fails at startup.

Validate a robot-owned model without a policy or Python:

```bash
cargo run --manifest-path deps/bello-stack/Cargo.toml -p bello-mujoco -- \
  --config deps/bello-stack/components/bello-mujoco/config/bello_mini.yaml --check
```

The config selects canonical hardware/URDF inputs, startup pose, timestep, and
neighbor collision exclusions. Construction preserves link frames and inertias,
removes visual geometry, adds a free root and floor, and creates native motor
actuators, differential tendons, and source-frame IMU sensors. Startup rejects
incomplete motor ownership, invalid poses, and initial self-collisions. Change
the profile to `bello.yaml` for Bello. `--export-physics PATH` writes an optional
inspection MJCF; neither model construction nor simulation requires an artifact.
URDF transmission offsets are represented separately from native tendon lengths.

`Simulation::load` and `Simulation::from_xml` remain low-level native loaders;
loading a URDF alone does not create its differential transmissions.

The tested adapter boundary covers protocol commands, native physics, motor
feedback, and stop/disable behavior. SocketCAN process orchestration, IMU serial
emulation, pose publication, and the torso guide are not wired here yet. The
Python emulator remains the complete executable until those features are ported.
