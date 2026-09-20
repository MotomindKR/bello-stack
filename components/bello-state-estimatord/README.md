# bello-state-estimatord

`bello-state-estimatord` reads direction-corrected motor and EBIMU telemetry
through `bello-hardware-client`, estimates a local tracked-frame pose with
measured-torque contact inference and stationary-foot constraints, and implements the existing
`bello.pose.v1.PoseService`.

Hardwared may start later or restart independently. The pose service stays up,
retries connection/telemetry transport failures and stream closure with bounded
backoff, and reloads/validates the inventory on each connection. No poses are
produced during an outage; new subscribers do not receive cached poses older
than `imu_stale_ms`. On recovery, fresh aligned motor/IMU data resumes the same
pose sequence without integrating outage time. Existing odometry is preserved,
so movement while telemetry is missing cannot be reconstructed. Invalid local
configuration, incompatible inventory/frames, and non-transport errors still
fail explicitly.

`imu_frame` names the hardwared IMU `output_frame`. Hardwared has already
rotated the physical sensor sample into those axes, while preserving the sensor
origin. The estimator runs its contact filter at that origin, then uses the
URDF's fixed transform to publish the configured `tracked_frame` pose and
twist. This keeps Bello's pelvis-aligned IMU and Bello Mini's offset, yawed IMU
on the same explicit contract. A missing link, non-descendant link, or movable
transform is rejected during configuration validation.

The daemon applies a quaternion SLERP low-pass filter to the EBIMU orientation
and vector low-pass filters to angular velocity and linear acceleration. Their
cutoff frequencies are configured independently. Translation and velocity are
estimated by a contact-aided linear Kalman filter. It is local odometry:
without an external absolute position source it can drift, especially during
foot slip, flight, rolling, or non-foot support.

The estimator currently requires `planar: true`: each acquired foot anchor is
constrained to `ground_height_m` in the reference frame, while pelvis height
remains free to change. Uneven terrain and stairs are not supported.

The support point follows the lowest surface of the foot link's URDF collision
primitives using measured joint angles and IMU orientation. Flat support uses
the center of the lowest primitive faces; pitched feet use their toe/heel
edges or rounded surfaces. When this material point changes, its existing
anchor moves by the corresponding rigid-body offset, avoiding a false base
translation. This uses gravity as the support normal, not a terrain estimate;
it does not infer hand or knee support.

Contact evidence is continuous rather than a binary threshold. Estimated load
and relative foot height produce smooth scores whose product is filtered with
separate rise and fall time constants. Only the initial pose bootstrap uses
relative height and vertical encoder/gyro motion before momentum history exists;
`contact_motion_*` thresholds apply to that initialization, not ongoing contact
classification. There is no selectable kinematic-only mode or fallback.

The filtered probability is cubed before it scales both sides of the contact
model. This makes medium-confidence contacts much weaker while preserving a
near-certain stance constraint. Contact measurement covariance is normalized by
the actual encoder update interval so its total information per second does not
increase with telemetry rate. High confidence keeps the anchor nearly fixed;
low confidence increases anchor process noise so a questionable anchor can move
rather than pinning the robot. Contacts below 0.95 probability cannot update the
accelerometer-bias state. Binary state remains only for anchor lifecycle: a new
anchor is created above an internal acquisition probability and retired only
after its already-weak confidence falls near zero. The daemon has no
gait-reference contact input.

## Required dynamics model

Load-aware estimation is the sole implementation in both shipped robot profiles.
Custom configs must supply the `load_observer` block; omission or null is an
error. Remove obsolete `contact_world_speed_*` settings, which are rejected.
For example, the Mini profile uses paths relative to the config:

```yaml
load_observer:
  model: ../../bello-mujoco/config/bello_mini.yaml
  cutoff_hz: 10.0
  velocity_cutoff_hz: 10.0
  zero_weight_fraction: 0.02
  full_weight_fraction: 0.10
```

The model and its native builder live entirely in this bello-stack repository's
`components/bello-mujoco`, with canonical inputs under `descriptions/`. No Python
training checkout, generated policy MJCF, or policy bundle is needed.
The model is built from the existing robot-owned physics configuration and must
resolve to the same canonical URDF. `tracked_frame` must be its free-root link.
This requires the pinned headless MuJoCo 3.12 runtime, all body motor feedback,
and `planar: true`. `--check` compiles and validates the observer model too.

The observer uses timestamp-aligned measured torque, joint position/velocity,
IMU orientation/gyro, and the filter's predicted root state. Native tendon
transmissions supply the motor-to-joint torque mapping. It does not read actuator
commands, emulator truth, simulated acceleration, or solved contact forces.
A bounded worker owns all MuJoCo data; computation time counts toward pose age.

Estimated vertical load and relative support height determine contact confidence.
Sole tilt is measured against world +Z using `world_from_imu * imu_from_sole`.
Horizontal anchor information fades between 3 and 15 degrees while vertical
support information remains available at the toe/heel. The selected foot link's
local +Z must be its sole normal. This does not estimate terrain orientation.

Missing/stale whole-body feedback withholds poses; there is no silent kinematic
fallback. Reconnection preserves odometry but resets the momentum history.
Wrong masses, encoder zeros, torque scaling, slip, hand/knee support, and telemetry
latency can still corrupt inferred loads. A good load estimate is not a direct
contact measurement. Making this the software default is not physical deployment
approval: Lima tests still show substantial, variable horizontal drift during
deep balance/lunge motions despite improved height estimates. Commission on
supported hardware before relying on this pose stream for motion control.

Motor samples are timestamped at their estimated source time and interpolated,
or velocity-extrapolated within `motor_stale_ms`, to each IMU source timestamp
before foot kinematics are evaluated. This avoids combining a current IMU
sample with an older, unadjusted encoder frame.

Anchor transitions are logged at the default info level with raw and effective
probability plus their motion inputs. Enable one-second state, velocity,
accelerometer-bias, contact-anchor, probability, and motor-alignment summaries
with:

```sh
RUST_LOG=bello_state_estimatord=debug cargo run -p bello-state-estimatord -- \
  --config components/bello-state-estimatord/config/bello.yaml
```

Run the robot's configuration; `bello-hardwared` may start before or after it:

```sh
cargo run -p bello-state-estimatord -- \
  --config components/bello-state-estimatord/config/bello.yaml
```

For Bello Mini, select its profile explicitly:

```sh
cargo run -p bello-state-estimatord -- \
  --config components/bello-state-estimatord/config/bello-mini.yaml
```

Use the same robot profile for physical hardware and emulation: the estimator
consumes gRPC telemetry and does not select CAN or serial transports. The
supplied profiles bind PoseService on `0.0.0.0:50053`; local
consumers connect to `127.0.0.1:50053` and request `odom` as the reference frame
and `pelvis_link` as the tracked frame.

The Mini profile uses the nominal IMU pose and ankle differential reductions in
the canonical URDF. Validate those values, encoder zeros, and motor directions
on supported hardware before relying on contact odometry.

The estimator has no separate estimator API and accepts no pose submissions.
Its only public output is `PoseService::WatchPose`; covariance and contact
classifier state stay internal. It runs whenever fresh IMU samples arrive, so
the effective rate is the configured sensor's actual telemetry rate rather
than a hard-coded 500 Hz.
