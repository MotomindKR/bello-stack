# bello-state-estimatord

`bello-state-estimatord` reads direction-corrected motor and EBIMU telemetry
through `bello-hardware-client`, estimates a local tracked-frame pose with
encoder-based stationary-foot constraints, and implements the existing
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

The required `planar` configuration field selects the ground model. With
`planar: true`, each acquired foot anchor is constrained to `ground_height_m`
in the configured reference frame. Set `planar: false` for uneven terrain,
stairs, or any deployment where contact height must remain unconstrained.

Contact evidence is continuous rather than a binary threshold. Relative foot
height and base-independent encoder kinematics each produce a smooth score
between zero and one; their product is filtered with separate rise and fall
time constants. In planar mode the motion score uses only vertical foot motion,
so normal horizontal stance motion does not reject a contact. Non-planar mode
uses the full base-independent kinematic speed.

Full world-foot speed supplies an additional score only when the opposite foot
is already both high-confidence and stationary. This lets a trusted stance foot
reject a low-clearance swing foot using horizontal motion. A foot cannot use its
own anchor to validate its speed, and world speed is ignored when neither foot
provides stationary opposite support, so drift cannot block contact recovery.

The filtered probability is cubed before it scales both sides of the contact
model. This makes medium-confidence contacts much weaker while preserving a
near-certain stance constraint. Contact measurement covariance is normalized by
the actual encoder update interval so its total information per second does not
increase with telemetry rate. High confidence keeps the anchor nearly fixed;
low confidence increases anchor process noise so a questionable anchor can move
rather than pinning the robot. Contacts below 0.95 probability cannot update the
accelerometer-bias state. Binary state remains only for anchor lifecycle: a new
anchor is created above an internal acquisition probability and retired only
after its already-weak confidence falls near zero. No force or torque
measurement is used. The daemon currently has no gait-reference contact input.

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
