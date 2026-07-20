# bello-state-estimatord

`bello-state-estimatord` reads direction-corrected motor and EBIMU telemetry
through `bello-hardware-client`, estimates the local `odom_from_pelvis`
transform with encoder-based stationary-foot constraints, and implements the
existing `bello.pose.v1.PoseService`.

The daemon uses the EBIMU quaternion directly for orientation. Translation and
velocity are estimated by a contact-aided linear Kalman filter. It is local
odometry: without an external absolute position source it can drift, especially
during foot slip, flight, rolling, or non-foot support.

Run the physical configuration after `bello-hardwared` is online:

```sh
cargo run -p bello-state-estimatord -- \
  --config components/bello-state-estimatord/config/bello.yaml
```

For the full vCAN emulator use `bello-vcan.yaml`. The PoseService listens on
`127.0.0.1:50052`; consumers request `odom` as the reference frame and
`pelvis_link` as the tracked frame.

The estimator has no separate estimator API and accepts no pose submissions.
Its only public output is `PoseService::WatchPose`; covariance and contact
classifier state stay internal. It runs whenever fresh IMU samples arrive, so
the effective rate is the configured sensor's actual telemetry rate rather
than a hard-coded 500 Hz.
