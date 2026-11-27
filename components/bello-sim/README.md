# bello-sim

The native simulator runs headless MuJoCo physics for hardware-in-the-loop testing. Run the following
commands from the `bello-stack` root in the Linux Nix shell. If building outside
Nix, set `MUJOCO_DYNAMIC_LINK_DIR` and the loader path to the matching MuJoCo library.

## Validate and run a model

```sh
cargo run -p bello-sim -- check --config components/bello-sim/config/bello_mini.yaml
cargo run -p bello-sim -- run --config components/bello-sim/config/emulator_mini.yaml
```

The shipped profiles use [loopback UDP for CAN](../../docs/operations.md#simulate-can-without-socketcan).
On Linux and macOS, start MuJoCo first. It creates the PTYs and publishes the
slave paths at `/tmp/bello-sim/imu` and `/tmp/bello-sim/power`. Start hardwared
under the same user. For concurrent simulations, choose distinct `pty_link`
paths in MuJoCo and matching `serial_ports` in hardwared. Use `device` in
MuJoCo to open an externally provisioned port. To use Linux SocketCAN instead, set each logical bus
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
cargo test -p bello-sim --test simulation
```

## Milly physics

The [Milly emulator](config/emulator_milly.yaml) runs without serial devices.
Pair it with [hardwared's simulation profile](../bello-hardwared/config/milly-sim.yaml).
Gravity acts on the arm while motors are disabled.

```sh
cargo run -p bello-sim -- check --config components/bello-sim/config/milly.yaml
cargo run -p bello-sim -- run --config components/bello-sim/config/emulator_milly.yaml
```

## Live camera streams

Set `cameras` in the emulator profile to the path to a [camera configuration](config/cameras_mini_torso.yaml).
Measure the mounting transforms and intrinsics before using the images for calibrated work.
Each named stream has either a `mono` lens or a `stereo` pair with `left` and `right` lenses.
The configured width and height apply to each eye. The simulator applies each lens’s optional
fisheye calibration, packs stereo left-to-right, then uses GStreamer to encode one JPEG.
`GetCameras` reports the encoded dimensions, layout and host-local GStreamer socket.
`WatchFrames` delivers the same JPEG with its capture timestamp and sequence number.
The dashboard handles WebRTC signaling. Physical Milly telepresence reads the USB camera directly.

A camera failure interrupts the stream while physics continues.

Preview the cameras without CAN devices from the Nix shell. Native installations need GStreamer 1.28 or later with the base, good, and bad plugins.

```sh
cargo run -p bello-sim --example camera_preview -- preview \
  --model-config components/bello-sim/config/bello_mini_torso.yaml \
  --camera-config components/bello-sim/config/cameras_mini_torso.yaml
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
cargo run --locked -p bello-sim -- \
  render --recording outputs/emulator.recording

cargo test --locked -p bello-sim \
  --test video -- --ignored
```

## Dual Milly stereo

Build the supervised worker with `cargo build -p bello-camera-worker` in the development shell. Start the dual 7-DOF fixture and its two calibrated views with the following commands in separate terminals. The simulator holds the initial pose until the motors receive commands.

```sh
cargo run -p bello-sim -- run --config components/bello-sim/config/emulator_milly_7dof_gimbal.yaml
cargo run -p bello-hardwared -- --config components/bello-hardwared/config/milly-7dof-gimbal-sim.yaml
```

The hardware daemon at `127.0.0.1:50051` publishes one `stereo` stream. Each 3200 × 1200 JPEG contains the left and right 1600 × 1200 exposures from one simulation snapshot. Use the [mono profile](config/cameras_milly_mono.yaml) for a single 1600 × 1200 stream from the left lens.

For high-resolution streams, allow 32 MiB per gRPC response (`MAX_FRAME_MESSAGE_BYTES` in Rust or `grpc.max_receive_message_length` in Python). Detailed scenes can exceed the common 4 MiB client default. GStreamer and gRPC carry the original encoded JPEG at the same quality.

To discover the local socket, run the following command from the repository root. Pass the returned `localSources.stereo.unixfdSocket` to `unixfdsrc` on the same host. Discover it again when the publisher restarts.

```sh
grpcurl -plaintext -import-path components/bello-hardware-api/proto \
  -proto bello/camera/v1/camera.proto 127.0.0.1:50051 \
  bello.camera.v1.CameraSourceService/GetCameras
gst-launch-1.0 unixfdsrc socket-path="$CAMERA_SOCKET" ! jpegdec ! videoconvert ! autovideosink
```

The camera profile uses measured per-eye intrinsics, OpenCV fisheye coefficients and the inverse stereo extrinsics to place the right sensor relative to the left. The left optical centre is anchored half the measured baseline from the CAD midpoint. The ELP calibration comes from `bello-vr-telepresence/config/stereo.json` at revision `bded0439` (SHA-256 `f86832f946c464d0300e0463b04a67cd0b0d6687cde2fc89569c94b585059ab3`). [Stereo camera profile](config/cameras_milly_stereo.yaml)

To configure another camera, set the dimensions and specify each optical pose as XYZ in metres followed by a WXYZ quaternion. Supply fx, fy, cx and cy in sensor pixels. Omit the four fisheye coefficients for pinhole projection. The model profile holds provisional auxiliary inertias for the gimbal and gripper linkages. [Simulation model](config/milly_7dof_gimbal.yaml)
