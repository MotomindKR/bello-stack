# Native MuJoCo runtime

Headless robot physics for hardware-in-the-loop testing. Run the following
commands from the `bello-stack` root in the Linux Nix shell. If building outside
Nix, set `MUJOCO_DYNAMIC_LINK_DIR` and the loader path to the matching MuJoCo library.

## Validate and run a model

```sh
cargo run -p bello-mujoco -- check --config components/bello-mujoco/config/bello_mini.yaml
cargo run -p bello-mujoco -- run --config components/bello-mujoco/config/emulator_mini.yaml
```

The shipped profiles use [loopback UDP for CAN](../../docs/operations.md#simulate-can-without-socketcan).
On Linux and macOS, MuJoCo supplies a fresh pseudo-terminal for each simulated IMU
and power board when hardwared connects. Run both processes as the same user.
Their profiles use matching names such as `pty:imu` and `pty:power`. Give each
stack its own names when running multiple instances. An explicit serial device
path still opens an externally provisioned port. To use Linux SocketCAN instead, set each logical bus
in a local runtime profile to its vCAN interface (for example, `bus_interfaces: {can0: vcan0}`). Use `check --export-physics PATH`
to inspect the compiled MJCF. Loading a URDF directly with `Simulation::load`
skips actuator transmissions. For the fixed-base Mini Torso, pair the [emulator](config/emulator_mini_torso.yaml)
with the [hardwared simulation profile](../bello-hardwared/config/bello-mini-torso-sim.yaml).

To model a carried mass, add a `payloads` entry with `frame`, `mass_kg`, and
`radius_m` to the model profile. The frame must have no existing inertia. Its
radius sets the mass distribution without adding collision geometry.

Press Enter to release the torso guide, and again to reset the pose and disarm
all motors. Use `--duration-seconds N` for a bounded run.

### Supported HIL startup

Set `startup_hold_until_commanded: true` to hold the starting pose while the
controller connects. Physics resumes once every mapped motor is in impedance
mode, enabled, and has received a command. A missing command leaves the
fixture held. Stopping or disabling motors after release does not reapply it.

The process integration test uses UDP and pseudo-terminals. On Linux it also checks video output with software Mesa and ffmpeg.

```sh
cargo test -p bello-mujoco --test simulation
```

## Milly physics

The [Milly emulator](config/emulator_milly.yaml) runs without serial devices.
Pair it with [hardwared's simulation profile](../bello-hardwared/config/milly-sim.yaml).
Gravity acts on the arm while motors are disabled.

```sh
cargo run -p bello-mujoco -- check --config components/bello-mujoco/config/milly.yaml
cargo run -p bello-mujoco -- run --config components/bello-mujoco/config/emulator_milly.yaml
```

To update the arm's collision geometry, edit the [primitive definitions](../../descriptions/milly/collision-primitives.json)
and run `python scripts/milly_collisions.py`.

## Live camera streams

Set `cameras` in the emulator profile to the path to a [camera configuration](config/cameras_mini_torso.yaml).
Measure the mounting transforms and intrinsics before using the images for calibrated work.
The cameras capture the same physics snapshot and publish JPEGs through
[the camera gRPC service](../bello-camera-api/proto/bello/camera/v1/camera.proto).
[bello-camerad](https://github.com/MotomindKR/bello_webxr_teleop/tree/main/crates/cameras) streams these frames or physical camera feeds over WebRTC. Lens distortion is not simulated.
A camera failure interrupts the stream while physics continues.

Preview the cameras without CAN devices:

```sh
cargo run -p bello-mujoco --example camera_preview -- preview \
  --model-config components/bello-mujoco/config/bello_mini_torso.yaml \
  --camera-config components/bello-mujoco/config/cameras_mini_torso.yaml
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

After the run, Linux software rendering requires Mesa EGL with llvmpipe and
ffmpeg. Ctrl-C or SIGTERM cancels rendering and encoding.

The MP4 is published only after successful encoding and never overwrites a file.
`outputs/emulator.recording/` retains the compiled visual model, video settings,
and poses. Re-render with the same executable (change the output path in
`video.json` if its previous video exists):

```bash
cargo run --locked -p bello-mujoco -- \
  render --recording outputs/emulator.recording

cargo test --locked -p bello-mujoco \
  --test video -- --ignored
```
