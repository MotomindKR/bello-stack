# bello-visualizer

`bello-visualizer` is the native Kiss3d + egui monitor and command UI for
`bello-hardwared`. It renders the repository's URDF description, follows live
motor and IMU telemetry, reconstructs differential ankle joints, and can
command explicitly configured motor groups.

Run the maintained Bello configuration with:

```sh
cargo run -p bello-visualizer -- \
  --config components/bello-visualizer/config/bello-vcan.yaml
```

The configured hardware and pose services must already be reachable. The
application starts monitor-only. Select groups and enable **Commanding** to
acquire their leases, enable them, and start control. Disabling the toggle
stops, disables, and releases every owned group while monitoring continues.
Fault recovery is always an explicit per-group action.

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
Position is slew-limited in the background command loop.

## Root pose and diagnostics

With `root.source: external_pose`, samples are
`reference_from_tracked`; `tracked_from_model_root` is composed only for
display. A stale or disconnected pose stream freezes the last root transform
and never changes command or safety behavior.

The diagnostics window shows transport lifecycle, motor inventory and limits,
motor/group/IMU telemetry, command state, faults, sample ages, and a bounded
event history. Disconnected streams retain their last values but are marked
frozen and cannot seed control.

The minimal `config/vcan.yaml` and `config/vcan.urdf` pair is useful for a
two-joint UI smoke test. The normal test suite covers configuration,
transmission reconstruction, telemetry, command cadence, and lease cleanup
without opening a GPU window.
