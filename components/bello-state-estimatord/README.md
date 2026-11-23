# bello-state-estimatord

`bello-state-estimatord` combines motor encoders and EBIMU readings to estimate
local odometry. It streams the result through [WatchPose](../bello-pose-api/).
Position can drift during foot slip, flight, or non-foot support.

Set `imu_frame` to hardwared's IMU output frame. It must be the tracked frame
or a descendant connected to it by fixed joints. Use `planar: true` on level ground
to fix foot anchors to `ground_height_m`. Use `planar: false` on stairs or uneven terrain.

The service stays available when hardwared disconnects. Pose updates resume
with fresh telemetry. Motion during the outage is lost.

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

Use the same profile for hardware and emulation. With the supplied profiles, connect
to `127.0.0.1:50053` and request `odom` as the reference frame and `pelvis_link` as
the tracked frame. Set `RUST_LOG=bello_state_estimatord=debug` to inspect contact
probabilities and motor alignment.

The Mini profile uses the nominal IMU pose and ankle differential reductions in
the canonical URDF. Validate those values, encoder zeros, and motor directions
on supported hardware before relying on contact odometry.
