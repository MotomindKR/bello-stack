# bello-visualizer

`bello-visualizer` is a Linux native Kiss3d + egui application. It loads one
local, flattened viewer MJCF and its referenced meshes, continuously monitors
`bello-hardwared`, reconstructs fixed-tendon model joints, and can command
explicitly profiled motor groups.

The orbit camera uses the model's MuJoCo Z-up convention, so the robot is
upright without modifying its model or telemetry transforms. A camera-relative
world-axis indicator remains in the lower-left corner: X is red, Y is green,
and Z is blue.

Prepare the production model in `bello_mujoco`:

```bash
uv run bello-prepare-assets --config-name visualizer
```

The maintained recipe generates sphere end effectors at
`assets/generated/visualizer/`. With the full Bello vCAN hardware daemon
running, start the local pose provider and then launch the matching
configuration:

```bash
cargo run -p bello-state-estimatord -- \
  --config components/bello-state-estimatord/config/bello-vcan.yaml
cargo run -p bello-visualizer -- \
  --config components/bello-visualizer/config/bello-vcan.yaml
```

The sphere morphology intentionally omits the two wrist-yaw coordinates and
their CAN motors. Hardware inventory supplies each group's coordinate names;
the matching MJCF position actuators supply coordinate position limits and
defaults. The `lower_body` and `upper_body` YAML entries only select commandable
groups and optionally override common command dynamics. Their default gains are
for the emulator and are not approved physical-hardware gains.

For a custom deployment, copy a configuration, replace `mjcf` with its viewer
path, and add an empty control profile (`group_name: {}`) for each commandable
hardware group. Every hardware coordinate must resolve to exactly one MJCF
`<position>` actuator with a `ctrlrange`; direct joints and fixed tendons are
both supported. Sensible common defaults are used for velocity, feed-forward
torque, gains, command rate, position slew, command validity, and feedback
staleness. These common values can be overridden per group. MJCF actuator
`kp`/`kv` simulation gains are never used for hardware commands.

Run the included minimal two-joint example with its matching hardwared vCAN
fixture and Robstride emulator:

```bash
cargo run -p bello-visualizer -- --config components/bello-visualizer/config/vcan.yaml
```

The application starts monitor-only. Select groups, then enable the
**Commanding** toggle to acquire leases, enable, and start them. Disable the
toggle to stop, disable, and release all owned groups while telemetry continues.
A red **Clear Fault** row appears directly beneath each faulted group and
performs a separate lease/recover/release cycle for only that group; faults are
never recovered automatically. Each new command session discards prior-session
targets, reseeds positions from fresh feedback, and restores the group defaults
for velocity, feed-forward torque, and gains before starting. While a group is
not running, both editor views continuously follow fresh live feedback and are
read-only. Position uses hold-to-jog controls and is slew-limited again in the
background command loop. Velocity, feed-forward torque, `kp`, and `kd` use
bounded sliders.

The separate **Bello diagnostics** window consumes the maintained
`bello-hardware-client` inventory and latest-state APIs. Its tabs expose:

- an overview of connection, pose, fault, stale-input, temperature, and
  telemetry-gap counts;
- every CAN bus and serial device with configured metadata, lifecycle state,
  detail, and transition count;
- every motor with coordinate/group/address/model/direction, live position,
  velocity, torque, temperature, raw fault bits, raw mode status, feedback age,
  source time, and live/frozen stream state; motor age coloring uses that
  group's configured control-staleness threshold;
- every motor group with daemon state, visualizer session state, configured
  rate, fault cause/detail, command count, control tick, jitter, and source
  time;
- every IMU with serial ownership, lifecycle state, quaternion, angular
  velocity, linear acceleration, device timestamp, sample age, and source time;
- a bounded, filterable event view for connection changes, lifecycle health,
  group state/fault changes, command-policy decisions, and telemetry gaps.

High-rate motor, IMU, and unchanged group timing samples remain latest-wins and
do not flood the event log. Repeated identical diagnostic events collapse into
one row with a count even when other components' events occur between them.
Disconnection marks streams frozen, freezes the last values, and continues to
increase their ages; frozen samples can never seed control. The daemon does not
currently publish motor voltage or current, CAN frame/error counters, or serial
byte/error counters, so the UI does not invent values for them.

Tracked roots use `root.source: external_pose`. The maintained provider is
`bello-state-estimatord`, which implements the neutral PoseService consumed by
the visualizer. Pose samples are
`reference_from_tracked`; `tracked_from_model_root` is composed on the display
path only. A stale or disconnected tracker freezes the last root pose and never
changes command or safety calculations.

## Manual smoke test

On a Linux desktop, run the vCAN example above and confirm the native window
opens, both links render, orbit controls work, the egui panel updates telemetry,
the **Commanding** toggle starts/stops the selected group, diagnostics show all
configured devices and live telemetry, event filters work, and monitoring
continues after release. This is the only verification step that requires a
GPU/window server.
