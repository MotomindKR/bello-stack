# bello-visualizer

`bello-visualizer` is the native Kiss3d + egui monitor and command UI for
`bello-hardwared`. It renders the repository's URDF description, follows live
motor and IMU telemetry, reconstructs differential ankle joints, can command
explicitly configured motor groups, and serves normalized physical-controller
intent to remote policy runners.

Create a machine-local Bello configuration from the maintained example, then
run it with:

```sh
cp components/bello-visualizer/config/bello.example.yaml \
  components/bello-visualizer/config/bello.yaml
cargo run -p bello-visualizer -- \
  --config components/bello-visualizer/config/bello.yaml
```

The local `bello.yaml` is ignored by Git. The example uses localhost for its
hardware and pose clients and listens on port 50054 for remote-command
subscribers. Override the clients without editing the YAML when they run on
another host:

```sh
cargo run -p bello-visualizer -- \
  --config components/bello-visualizer/config/bello.yaml \
  --hardware-endpoint https://robot.local:50051 \
  --pose-endpoint https://robot.local:50053
```

`--pose-endpoint` requires `root.source: external_pose`. The configured
TLS material, pose frame names, and staleness remain YAML-owned when an
endpoint is overridden. The configured hardware and pose services must already
be reachable. The application starts monitor-only. Select groups and enable
**Commanding** to acquire their leases, enable them, and start control.
Disabling the toggle stops, disables, and releases every owned group while
monitoring continues. Fault recovery is always an explicit per-group action.

## Configuration

One YAML document defines:

- `urdf`: robot description used for rendering, hierarchy, joint limits, and
  differential transmissions;
- `hardware`: hardwared endpoint and optional TLS material;
- `remote`: plaintext TCP listen address, controller index, poll rate, and
  rescaled stick deadzone for `bello.remote.v1.RemoteCommandService`;
- `root`: a fixed transform or an external `PoseService`;
- `refresh_hz`: telemetry refresh frequency;
- `control_profiles`: the hardware groups that the UI may command and their
  command cadence, validity, staleness, slew rate, and optional conservative
  velocity, feed-forward torque, `kp`, and `kd` ranges.

Relative URDF and certificate paths resolve from the YAML file. Hardware
inventory is authoritative for group membership, coordinate names, motor
identity, and each coordinate's command limits. A control profile can only
narrow those live limits. Position bounds come directly from inventory. The
URDF must contain a scalar joint with the same name as each direct coordinate,
or a differential transmission whose actuator name matches the coordinate.
Unknown, duplicate, or underdetermined mappings are rejected.

Each command session seeds position from fresh feedback and restores the
profile defaults for velocity, feed-forward torque, and gains. When a group is
not commanded, both editor views remain read-only and follow fresh feedback.
The model-space velocity and effort sliders use conservative transmission-aware
bounds, so every slider combination remains inside all actuator limits.
Position is slew-limited in the background command loop.

## Remote controller stream

The visualizer owns the configured physical controller. Left-stick vertical
and horizontal publish normalized forward and lateral intent; right-stick
horizontal publishes normalized yaw. R1 publishes a level-sensitive abort
request. The stream does not contain policy-specific metres-per-second or
radians-per-second limits; consumers apply their own command bounds.

A newly connected controller publishes zero until every mapped stick has
returned inside the configured deadzone once. Disconnects also publish exact
zero. The gRPC service is read-only, supports multiple subscribers, and sends
the latest snapshot at `remote.poll_hz`. Version 1 is plaintext TCP intended
for a trusted operator/robot network. On Linux, the visualizer process needs
access to the selected `/dev/input/event*` device.

## Root pose and diagnostics

With `root.source: external_pose`, samples are
`reference_from_tracked`; `tracked_from_model_root` is composed only for
display. Configured quaternions are normalized when the YAML is loaded. A stale
or disconnected pose stream freezes the last root transform and never changes
command or safety behavior.

The controls window has a dedicated **Gamepad** tab showing the remote listen
address, controller identity, raw stick positions, normalized command axes,
neutral-arm state, R1 abort state, and any terminal service error.
The diagnostics window shows transport lifecycle, motor inventory and
limits, motor/group/IMU telemetry, command state, faults, sample ages, and a
bounded event history. Disconnected streams retain their last values but are
marked frozen and cannot seed control.

The minimal `config/vcan.yaml` and `config/vcan.urdf` pair is useful for a
two-joint UI smoke test. The normal test suite covers configuration,
transmission reconstruction, telemetry, command cadence, and lease cleanup
without opening a GPU window.
