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

`Simulation::load` accepts native URDF/MJCF; `Simulation::from_xml` accepts
self-contained model XML without Python or extraction. The caller supplies
the canonical hardware-to-actuator mapping. URDF-to-emulator augmentation
(actuators, tendons, sensors, and world setup) is not wired here yet. Loading a
URDF alone does not create its differential transmissions. The emulator model
is robot-owned and must not depend on a policy bundle.

The tested adapter boundary covers protocol commands, native physics, motor
feedback, and stop/disable behavior. SocketCAN process orchestration, IMU serial
emulation, pose publication, and the torso guide are not wired here yet. The
Python emulator remains the complete executable until those features are ported.
