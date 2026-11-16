# bello-visualizer

The **Power** diagnostics tab displays COG battery, rail and board status
telemetry from hardwared. See [power-board integration](../../docs/power_board.md).

`bello-visualizer` is the native Kiss3d + egui monitor and command UI for
`bello-hardwared`. It renders the repository's URDF description, follows live
motor and IMU telemetry, reconstructs differential and parallel-actuated joints, can command
explicitly configured motor groups. Policy operation uses the Electron console
and runner-owned operator service in `bello_mujoco`. This diagnostic viewer does
not open a controller or host a joystick command server. GLB node transforms
and baked materials are imported by Kiss3d; STL remains supported for other robots.

Create a machine-local Bello configuration from the maintained example, then
run it with:

```sh
cp components/bello-visualizer/config/bello.example.yaml \
  components/bello-visualizer/config/bello.yaml
cargo run -p bello-visualizer -- \
  --config components/bello-visualizer/config/bello.yaml
```

For the provisional Bello Mini vCAN deployment, copy its maintained example to
the ignored machine-local profile and run that profile:

```sh
cp components/bello-visualizer/config/bello-mini.example.yaml \
  components/bello-visualizer/config/bello-mini.yaml
cargo run -p bello-visualizer -- \
  --config components/bello-visualizer/config/bello-mini.yaml
```

This profile uses the reviewed Mini motor directions, while physical encoder
zeros and ankle reductions remain calibration-gated.

The local `bello.yaml` and `bello-mini.yaml` files are ignored by Git. Both
examples use localhost for their hardware clients. Both use their matching state estimator on port
50053. MuJoCo ground truth remains separately available on port 50052 during
emulator runs. Override the clients without editing the YAML when they run on
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

## Root pose and diagnostics

With `root.source: external_pose`, samples are
`reference_from_tracked`; `tracked_from_model_root` is composed only for
display. Configured quaternions are normalized when the YAML is loaded. A stale
or disconnected pose stream freezes the last root transform and never changes
command or safety behavior.

The diagnostics window shows transport lifecycle, motor inventory and
limits, motor/group/IMU telemetry, command state, faults, sample ages, and a
bounded event history. Disconnected streams retain their last values but are
marked frozen and cannot seed control.

The minimal `config/vcan.yaml` and `config/vcan.urdf` pair is useful for a
two-joint UI smoke test. The normal test suite covers configuration,
transmission reconstruction, telemetry, command cadence, and lease cleanup
without opening a GPU window.

`config/milly.example.yaml` is the maintained fixed-root visualizer profile for
the single-arm Milly hardware description. It connects to hardwared on
localhost and exposes the atomic `arm` group with conservative editor gains.

`config/milly-dual.example.yaml` loads two copies of the revised Milly arm and
exposes the atomic `dual_arm` group. Pair it with hardwared's `milly-dual.yaml`
(two physical CAN buses) or `milly-dual-vcan.yaml` (two virtual CAN buses).
The combined URDF is maintained directly. Apply shared single-arm geometry and
mass changes to both arm copies when updating the descriptions.
Each arm inherits the 24 primitive colliders, SDK inertias, camera frames and
its own gripper loop. The imported fixture retains 443 mm base spacing and the
center D405 frame, keepout box and support post. The dual description was imported from
`feat/milly-camera-streaming` at `81e4d8a`; camera streaming and remote-controller
services from that branch are separate from these hardware/visualizer profiles.
CAN transports use the current schema; out-of-range commands use `reject`,
matching the canonical single-arm policy (the older branch used `clamp`).
