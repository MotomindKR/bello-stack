# Native MuJoCo runtime

The physics adapter uses MuJoCo's installed C headers and the existing
`robstride-emulator`/`robstride-protocol` engine. It does not implement a second
ankle conversion: native joint/tendon transmissions supply motor coordinates.
Impedance and torque limits are applied inside MuJoCo.

From the matching `bello_mujoco` source checkout on Linux:

```bash
nix develop .#rust-runtime
cargo test --manifest-path deps/bello-stack/Cargo.toml -p bello-mujoco
```

Other build environments must set `MUJOCO_INCLUDE_DIR`, `MUJOCO_LIB_DIR`, and a
loader path for the same MuJoCo installation. Header/library version mismatches
fail at initialization. The caller supplies compiled MJCF and the canonical
hardware-to-actuator mapping; this crate does not compile or repackage URDFs.
`Simulation::from_xml` consumes the deployment bundle's `physics.xml` bytes
directly, with no Python, temporary extraction, or visual assets.

The tested adapter boundary covers protocol commands, native physics, motor
feedback, and stop/disable behavior. SocketCAN process orchestration, IMU serial
emulation, pose publication, and the torso guide are not wired here yet. The
Python emulator remains the complete executable until those features are ported.
