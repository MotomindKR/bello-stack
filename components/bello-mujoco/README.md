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

For the fixed-base Bello Mini Torso, use `config/bello_mini_torso.yaml` for
`check` and `config/emulator_mini_torso.yaml` for `run`. Its model sets
`root: fixed` and `floor: false`: the invisible hip-pitch base stays at world
zero and the floor does not intersect below-pivot geometry. Existing profiles
default to `root: free` and `floor: true`. A fixed root rejects a torso guide.
Simple transmissions create one native motor-coordinate tendon per actuator;
multiple actuators can drive the same joint and their efforts add. See the
[torso bring-up guide](../../descriptions/bello_mini_torso/README.md).

Model profiles optionally accept `payloads`, a list of `{frame, mass_kg, radius_m}`.
Each entry adds a spherical payload inertia at an existing inertial-free frame;
the radius describes the payload mass distribution and adds no collision geometry.
The builder rejects missing frames, duplicate entries, nonpositive dimensions,
or attempts to replace a body's existing inertia. Empty lists add no payload.
Use two entries for symmetric carried masses. Payloads are model settings,
independent of policy bundles, and are included in exported physics.

`Simulation::load` and `Simulation::from_xml` remain low-level native loaders;
loading a URDF alone does not instantiate its actuator transmissions.

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

### Supported HIL startup

| Runtime field | Behavior |
| --- | --- |
| `startup_hold_until_commanded: false` | Default: unpowered physics advances normally. |
| `startup_hold_until_commanded: true` | One-shot kinematic fixture holds the initial pose with zero velocity while protocol/feedback clocks continue. |
| Release | Every mapped motor on every bus must be enabled in impedance mode with a received command; enable alone is insufficient. |
| After release | Ordinary dynamics, gravity, effort limits and fault behavior; stop/disable/reset never automatically reapply support. |

Use this opt-in fixture for parked robots that would fall before controller
preflight and telemetry initialization finish. It represents external startup
support, not a motor command, physical backrest or validated contact model.
The emulator logs activation and release. It does not change the controller's
travel, startup, collision or deadline checks. A missing motor command leaves
the fixture held; the integration test must enforce its own startup timeout.

The opt-in Linux transport/video test requires **isolated** virtual interfaces,
a profile, software Mesa and ffmpeg. It chooses its own PTY and pose port and
never enables motors:

```bash
BELLO_TEST_CAN_INTERFACES=vcan_test0,vcan_test1 \
BELLO_TEST_EMULATOR_PROFILE=deps/bello-stack/components/bello-mujoco/config/emulator_mini.yaml \
cargo test --manifest-path deps/bello-stack/Cargo.toml -p bello-mujoco \
  --test vcan -- --ignored
```

## Milly physics over virtual CAN

Use `config/milly.yaml` for `check` and `config/emulator_milly.yaml` for `run`.
Pair this with hardwared's `config/milly-vcan.yaml` and the visualizer's
`config/milly-physics.example.yaml`. The model has a fixed base, gravity, floor and
24 box/cylinder colliders, seven native motor actuators, and two native site-to-site
`connect` equalities for the gripper. Passive positions are initialized from the
URDF slider-crank definition; MuJoCo solves them dynamically after startup.
The physics viewer profile uses kp=100 and kd=3 to reduce gravity sag. The
native motor joint stops activate 0.02 rad before the URDF limit with a 4 ms
response time. This leaves room for solver compliance before hardwared's
unchanged safety envelope; using the default soft stop directly at that boundary
previously caused joint 2 to cross -0.1 rad during ordinary arm motion.
These are simulation contact settings, not changes to motor health checks.
See [MuJoCo joint-limit constraints](https://mujoco.readthedocs.io/en/stable/computation/#limit).
No IMU or power UART is required (`imu: null`, `power_board: null`). A configured
canonical IMU still requires a matching runtime endpoint.

```sh
cargo run -j 2 -p bello-mujoco -- check --config components/bello-mujoco/config/milly.yaml
cargo run -j 2 -p bello-mujoco -- run --config components/bello-mujoco/config/emulator_milly.yaml
```

Milly uses the primitive-only collision policy. The collider recipe is
`descriptions/milly/collision-primitives.json`; regenerate it into the URDF with
`python scripts/milly_collisions.py`. There are no collision mesh assets or
extra rod/carriage collision exclusions. The gripper uses compound exterior shapes for the motor housing, rail and
moving jaws, including rounded jaw tips. Crank and rods retain their moving links,
inertias and loop constraints but have no collision geometry. The normal -2.2
to 0 rad gripper sweep is checked for self-penetration at zero and bent arm poses.

Visual geometry is excluded from physics. Mass properties come from the pinned
SDK URDF at commit `4c70a0a`, transformed into the existing link frames by
`scripts/milly_mass_properties.py`. The nominal 35 g crank/rod partition is
subtracted from the SDK wrist aggregate; it is not added to the total. Native
mass is **5.449411285057828 kg**. All moving inertias now live in the canonical
URDF; `config/milly.yaml` supplies only simulation damping, armature and stop
settings. Joint/control conventions are retained. The linkage partition and
contact surfaces remain approximations, not calibrated payload/contact models.

Physics advances continuously, including gravity when motors are disabled.
The initial joint angles are zero; an unpowered arm can move from that pose.
The visualizer reconstructs passive geometry from measured crank feedback;
it does not display the small constraint compliance of the native simulation.

## Live camera streams

Set `cameras: /absolute/path/to/cameras_mini_torso.yaml` in the emulator profile.
Use `config/cameras_mini_torso.yaml` as a provisional three-view configuration;
measure mounting transforms and intrinsics before using calibrated data.

| Item | Behavior |
| --- | --- |
| Views | Bird's-eye and both canonical wrist camera optical frames |
| Output | JPEG 640x360 at 30 FPS through `bello-camera-api` over a private UDS |
| Physics boundary | Preallocated, nonblocking state snapshots; no rendering in the physics loop |
| Renderer | Separate child with its own visual model, MuJoCo data and offscreen OpenGL context |
| Appearance | Flat gray skybox; robot CAD GLB base colors preserved by material, with physics meshes/inertias unchanged |
| Context | First EGL device on Linux; accelerated CGL on macOS |
| Calibration | Optical +X right/+Y down/+Z forward; wxyz mount transform; independent fx/fy and principal point |
| Isolation | Linux renderer CPUs 0-3, normal scheduling; control/physics scheduling stays independent |
| Source identity | Original host snapshot time and simulation time retained; driver flags are zero |
| Failure | Media consumers report stream failure; camera failure does not stop physics |

All three views are rendered from one state snapshot. Lens distortion is not
emulated. The separate GStreamer media service in `bello_webxr_teleop` forwards
these frames to WebRTC viewers and the dataset recorder. Physical cameras expose
the same source contract.

The `camera_preview` example exercises this renderer without CAN or motor
commands, holding the initial model pose. Existing HIL profiles keep cameras
disabled until explicitly configured.

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
visual meshes and renders with verified Mesa llvmpipe. Primitive-only profiles keep physics mesh-free.
The Linux `bello_mujoco` Rust shell supplies Mesa EGL and ffmpeg. Rendering has
an explicit timeout; Ctrl-C/SIGTERM cancels the renderer and encoder. All failures
are reported and fail the command. This offline video command uses Linux software
rendering; live cameras use the separate native offscreen renderer described above.

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

## Configurable stereo rendering

Camera profiles may select `width`/`height` (defaults 640×360), any unique list
of named cameras (up to 16), and an optional TCP `listen` address in addition to
the Unix `socket`. All views use one physics qpos snapshot, sequence, host
monotonic capture timestamp and simulation timestamp. Each frame's additive
`metadata_json` contains `world_from_optical` (metres, XYZW quaternion) and the
snapshot's joint positions. It is exposure state, not live or commanded state.
Existing three-camera profiles and the strict camera API constructor retain
640×360 defaults; stereo publishers use `Frames::configured` explicitly.

Projection is pinhole with independent fx/fy and principal point. For a
calibrated fisheye stereo rig, render rectified P1/P2 intrinsics and place the
virtual optical frames using measured R/T and R1/R2. No lens distortion is
implicitly introduced or removed by the renderer.

Model profiles can add simulation-only `environment_boxes`, each containing
`name`, `position`, `half_size` (all metres) and `rgba`. These world-fixed
primitive boxes appear in physics and visual models and are suitable for
near/far stereo validation targets. They do not alter canonical descriptions.
