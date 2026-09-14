# Native MuJoCo runtime

The maintained emulator profiles also expose a synthetic AGX COG power board.
Create the power UART pair alongside the IMU pair; see
[power-board configuration and limitations](../../docs/power_board.md).

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
Offline video uses its low-level rendering wrappers with software Mesa EGL;
no GPU or windowing backend is enabled.
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

The opt-in Linux transport/video test requires **isolated** virtual interfaces,
a profile, software Mesa and ffmpeg. It chooses its own PTY and pose port and
never enables motors:

```bash
BELLO_TEST_CAN_INTERFACES=vcan_test0,vcan_test1 \
BELLO_TEST_EMULATOR_PROFILE=deps/bello-stack/components/bello-mujoco/config/emulator_mini.yaml \
cargo test --manifest-path deps/bello-stack/Cargo.toml -p bello-mujoco \
  --test vcan -- --ignored
```

## Software video

Keep `video: null` to disable recording. In a local runtime profile, replace it
with an explicit recording configuration, for example:

```yaml
video:
  output: outputs/emulator.mp4
  width: 640
  height: 480
  fps: 30
  max_seconds: 300
  render_timeout_seconds: 600
  camera:
    body: pelvis_link
    distance_m: 3.0
    azimuth_degrees: 135
    elevation_degrees: -14
    lookat_offset_m: [0, 0, 0]
```

Poses are sampled only while the torso guide is released (or absent). Resetting
the guide pauses sampling. The explicit duration bounds retained pose memory,
which must fit within 64 MiB; reaching the bound fails the run instead of growing
memory indefinitely. The frame rate cannot exceed the simulation rate.

After CAN, serial and pose endpoints close, a separate process loads the canonical
visual meshes and renders with verified Mesa llvmpipe. Physics stays mesh-free.
The Linux `bello_mujoco` Rust shell supplies Mesa EGL and ffmpeg. Rendering has
an explicit timeout; Ctrl-C/SIGTERM cancels the renderer and encoder. All failures
are reported and fail the command. Native macOS/GPU rendering is unsupported.

The MP4 is published only after successful encoding and never overwrites a file.
`outputs/emulator.recording/` retains the compiled visual model, video settings,
and poses. These are disposable local artifacts, not robot sources or policy
bundles. Re-render with the same executable (change the output path in
`video.json` if its previous video exists):

```bash
cargo run --locked --manifest-path deps/bello-stack/Cargo.toml -p bello-mujoco -- \
  render --recording outputs/emulator.recording

cargo test --locked --manifest-path deps/bello-stack/Cargo.toml -p bello-mujoco \
  --test video -- --ignored
```

The render test uses synthetic geometry; it checks actual software pixels,
frame count, malformed recording rejection, and overwrite protection without
CAN devices or robot-specific assertions.
