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
  check --config deps/bello-stack/components/bello-mujoco/config/bello_mini.yaml
```

The config selects canonical hardware/URDF inputs, startup pose, timestep, and
neighbor collision exclusions. Construction preserves link frames and inertias,
removes visual geometry, adds a free root and floor, and creates native motor
actuators, differential tendons, and source-frame IMU sensors. Startup rejects
incomplete motor ownership, invalid poses, and initial self-collisions. Change
the profile to `bello.yaml` for Bello. `--export-physics PATH` writes an optional
inspection MJCF; neither model construction nor simulation requires an artifact.
URDF transmission offsets are represented separately from native tendon lengths.

Model profiles optionally accept `payloads`, a list of `{frame, mass_kg, radius_m}`.
Each entry adds a spherical payload inertia at an existing inertial-free frame;
the radius describes the payload mass distribution and adds no collision geometry.
The builder rejects missing frames, duplicate entries, nonpositive dimensions,
or attempts to replace a body's existing inertia. Empty lists add no payload.
Use two entries for symmetric carried masses. Payloads are model settings,
independent of policy bundles, and are included in exported physics.

`Simulation::load` and `Simulation::from_xml` remain low-level native loaders;
loading a URDF alone does not create its differential transmissions.

Run the headless emulator on Linux after creating virtual CAN interfaces and an
IMU serial pair (the `bello_mujoco` Lima setup supplies these):

```bash
cargo run --manifest-path deps/bello-stack/Cargo.toml -p bello-mujoco -- \
  run --config deps/bello-stack/components/bello-mujoco/config/emulator_mini.yaml
```

Use `emulator_bello.yaml` for Bello. Runtime profiles select bus interfaces,
serial/pose endpoints, command latency, gain scaling, IMU noise, and an optional
vertical torso guide. Set both noise standard deviations and command latency to
zero for clean feedback. These settings never come from a policy checkpoint.
The native command currently accepts virtual CAN interfaces only.

Enter releases the guide; Enter again resets the pose and disarms every motor.
Ctrl-C/SIGTERM closes all endpoints with bounded shutdown. `--duration-seconds N`
provides a bounded run. Transport queues are bounded; stale simulation timing or
an unconsumed serial stream fails explicitly. Pose consumers receive the latest
sample with its age, not an unbounded history.

The opt-in Linux transport test requires **isolated** virtual interfaces and a
profile. It chooses its own PTY and pose port and never enables motors:

```bash
BELLO_TEST_CAN_INTERFACES=vcan_test0,vcan_test1 \
BELLO_TEST_EMULATOR_PROFILE=deps/bello-stack/components/bello-mujoco/config/emulator_mini.yaml \
cargo test --manifest-path deps/bello-stack/Cargo.toml -p bello-mujoco \
  --test vcan -- --ignored
```

The Python emulator remains available while policy-driven parity and the CPU
video-recording workflow are migrated. Do not run both on the same endpoints.
