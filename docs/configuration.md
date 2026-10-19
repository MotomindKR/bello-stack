# Configuration

For opt-in COG serial telemetry and emulated power UARTs, see
[Power-board telemetry](power_board.md).

[Documentation index](README.md)

`bello-hardwared` reads a deployment YAML and the hardware description it
references, validates both without opening hardware, and resolves them into the
topology used for the lifetime of the process. The description owns stable
logical buses, motor identity, group membership, and a reference to the
canonical URDF. The deployment selects groups and binds transports. Neither
document nor TLS certificates are hot-reloaded.

Validate a file before deployment with:

```console
cargo run -p bello-hardwared -- \
  --check --config components/bello-hardwared/config/bello.yaml
```

`--check` and normal startup use the same parse and resolution path. Passing it
does not prove that an interface or device is present, accessible, or configured
to match the host. CAN and serial connectivity is opened asynchronously; an
unavailable transport normally appears as reconnecting after process startup.
`--config` is required for both paths; the daemon never selects a deployment
file implicitly.

The maintained examples are:

- [production hardware](../components/bello-hardwared/config/bello.yaml)
- [full Bello vCAN deployment](../components/bello-hardwared/config/bello-vcan.yaml)
- [canonical Bello hardware description](../descriptions/bello/hardware.yaml)
- [Milly single-arm hardware](../components/bello-hardwared/config/milly.yaml)
- [Milly single-arm vCAN deployment](../components/bello-hardwared/config/milly-vcan.yaml)
- [canonical Milly hardware description](../descriptions/milly/hardware.yaml)
- [provisional Bello Mini physical bindings](../components/bello-hardwared/config/bello-mini.yaml)
- [provisional Bello Mini vCAN deployment](../components/bello-hardwared/config/bello-mini-vcan.yaml)
- [Bello Mini hardware description](../descriptions/bello_mini/hardware.yaml)
- [Bello Mini Torso hardware](../components/bello-hardwared/config/bello-mini-torso.yaml)
- [Bello Mini Torso vCAN deployment](../components/bello-hardwared/config/bello-mini-torso-vcan.yaml)
- [Bello Mini Torso hardware description](../descriptions/bello_mini_torso/hardware.yaml)

Bello Mini exposes 28 motors in `lower_body` (legs and both waist axes) and
`upper_body` (seven motors per arm). These profiles use provisional motor
directions and ankle transmissions. See the
[calibration assumptions](../descriptions/bello_mini/ASSET_PROVENANCE.md) before
physical operation. The physical bindings use `can0`, `can1`, and the IMU's
stable `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5C93185491-if00` path;
vCAN uses `vcan0`, `vcan1`, and `/dev/ttyIMU`. Both use 115200 baud to match the
installed EBIMU, with 100 Hz output to fit the ASCII samples within the serial
bandwidth. The Mini profile warns at 60 C, faults at 95 C, and requires cooling
to 75 C before recovery; the fault threshold is not a continuous operating
target. Update the physical serial binding for a different sensor adapter.

The raw schema and defaults live in
[`config/model.rs`](../components/bello-hardwared/src/config/model.rs). The
validated boundary types live beside the subsystem that consumes them, and
cross-object checks live in
[`config/resolved.rs`](../components/bello-hardwared/src/config/resolved.rs).
Those files are the source of truth when this guide and the executable differ.

## Document structure

The deployment's top-level sections are:

```yaml
description: path/to/hardware.yaml
runtime: {}
grpc: {}
bus_transports: {}   # logical bus name -> SocketCAN or GS-USB transport
serial_ports: {}     # logical serial name -> host device path
enabled_groups: []   # explicit selection from the description
```

Unknown fields are rejected at every YAML object boundary. This catches
misspelled keys instead of silently applying a default. Relative description
paths resolve from the deployment file. Unknown or duplicate groups, missing or
unused bus mappings, and incomplete description ownership are rejected.

The hardware description must name its robot description:

```yaml
robot_description: urdf/bello.urdf
```

That path resolves from the hardware-description file. Every controlled
coordinate must resolve to either a movable URDF joint or one actuator
coordinate of a complete differential or simple transmission. Several simple
transmissions may drive one shared joint, with distinct actuator coordinates.
Every actuator of that joint must belong to the same motor group. Their scalar
effort envelopes divide the joint envelope into equal joint-torque shares,
accounting for each signed reduction.
Transmitted joints must be controlled through their declared actuator
coordinates; their joint names cannot serve as implicit direct-drive aliases.

### `runtime`

```yaml
runtime:
  telemetry_capacity: 4096
  motor_velocity_filter:
    cutoff_hz: 10.0
  imu_filter:
    orientation_cutoff_hz: 10.0
    gyro_cutoff_hz: 10.0
    acceleration_cutoff_hz: 10.0
```

`telemetry_capacity` is the capacity of the process-wide broadcast channel,
not a sampling rate and not a per-device history length. Every motor sample,
IMU sample, group timing sample, lifecycle transition, and command-policy event
shares it. Each subscriber receives events independently; a lagging subscriber
can drop old events without blocking hardware control.

- default: `4096`
- valid range: `64..=1000000`

`motor_velocity_filter` applies a timestamp-aware first-order low-pass filter
to every actuator-reported velocity sample in the Robstride feedback dispatcher
before publishing it:

```text
alpha = 1 - exp(-2 * pi * cutoff_hz * sample_interval)
published = previous_published + alpha * (current - previous_published)
```

The cutoff must be finite and positive and defaults to `10.0` Hz. Deriving the
coefficient from each receive interval keeps the physical cutoff stable when
feedback timing varies. The first sample, and the first sample after a
non-increasing timestamp or a gap longer than the motor group's feedback
timeout, passes through unchanged. Filtering affects motor values published by
`WatchTelemetry` and `GetStateFrame`; raw protocol feedback remains
authoritative for internal motor health and safety checks.

`imu_filter` applies independent timestamp-aware first-order low-pass filters
to every parsed EBIMU sample before hardwared updates the latest-sample cache
or publishes IMU telemetry. Orientation is interpolated on the quaternion
manifold; angular velocity and linear acceleration are filtered componentwise.
The coefficient uses the EBIMU device timestamp rather than the serial chunk
receive time, so several records delivered in one operating-system read retain
their original sample intervals.

All three cutoffs default to `10.0` Hz. Each must be finite, positive, and below
half the configured IMU output rate. The first sample passes through unchanged.
A non-increasing device timestamp, a gap longer than 100 ms, or a serial
reconnect resets the filter instead of blending across an invalid interval.
This host filter is separate from the EBIMU's persistent onboard
accelerometer-filter and vibration-cancellation settings.

### `grpc`

```yaml
grpc:
  listen: [{ type: tcp, address: 127.0.0.1:50051 }]
  default_lease_ms: 1000
  maximum_lease_ms: 5000
```

The `listen` field is a nonempty list of endpoints; its default is one TCP
listener on loopback. All endpoints share the same hardware and exclusive
control leases. All endpoints must bind successfully before hardware starts.
Use a list even when configuring a single endpoint. The lease values must
satisfy
`20 <= default_lease_ms <= maximum_lease_ms <= 300000`. Control leases are
exclusive per motor group; read-only telemetry streams do not require a
control lease.

For server-authenticated TLS, configure an absolute PEM certificate and private
key path:

```yaml
grpc:
  listen: [{ type: tcp, address: 0.0.0.0:50051 }]
  tls:
    certificate: /etc/bello/tls/server.crt
    private_key: /etc/bello/tls/server.key
  default_lease_ms: 1000
  maximum_lease_ms: 5000
```

Adding `client_ca` enables mutual TLS. A connection must then present a client
certificate signed by that CA:

```yaml
  tls:
    certificate: /etc/bello/tls/server.crt
    private_key: /etc/bello/tls/server.key
    client_ca: /etc/bello/tls/client-ca.crt
```

The hostname used by clients must appear in the server certificate's subject
alternative names. Keep private keys readable only by the service account.

For same-host clients, a Unix-domain socket avoids exposing a TCP port:

```yaml
grpc:
  listen:
    - type: unix
      path: /run/bello/hardwared.sock
      mode: "0660"
  default_lease_ms: 1000
  maximum_lease_ms: 5000
```

To expose TCP and a local Unix socket simultaneously:

```yaml
grpc:
  listen:
    - { type: tcp, address: 0.0.0.0:50051 }
    - { type: unix, path: /run/user/1000/bello-hardwared.sock, mode: "0600" }
```

Local clients use `unix:///run/user/1000/bello-hardwared.sock`; remote clients
use the TCP endpoint. Select the runtime directory for the service account's UID.

The socket path must be absolute and its parent directory must already exist.
The daemon applies the configured permission mode, removes a demonstrably stale
socket before binding, and removes only its own socket inode during shutdown.
Protect the parent directory as well, because its permissions control who can
reach the socket throughout creation. Run the service with the desired owning
user/group, or configure those through the service manager. TLS applies only
to TCP listeners; Unix sockets use filesystem permissions.
Configuring TLS requires at least one TCP listener.

Do not expose plaintext TCP on a non-loopback address. Server-only TLS encrypts
traffic and authenticates the robot but does not restrict which clients may
connect; use mTLS, firewall policy, or a protected Unix socket where control
access must be restricted.

See [`api/grpc/spec.rs`](../components/bello-hardwared/src/api/grpc/spec.rs)
for validation and the
[protobuf API](../components/bello-hardware-api/proto/bello/hardware/v1/hardware.proto)
for lease semantics.

### Description `serial_devices` and `imus`

A serial entry describes a transport. An IMU entry describes the sensor
protocol running on that transport:

```yaml
serial_devices:
  - name: imu_serial
    baud_rate: 921600
    timeout_ms: 100
    reconnect: { initial_delay_ms: 100, maximum_delay_ms: 5000 }

imus:
  - name: imu
    serial_device: imu_serial
    source_frame: pelvis_link
    output_frame: pelvis_link
    stale_timeout_ms: 500
    output_rate_hz: 200
```

The deployment supplies only the host binding:

```yaml
serial_ports: { imu_serial: /dev/ttyUSB0 }
```

Serial names and mapped physical ports must be unique. `baud_rate` must be positive,
and `timeout_ms` is in `1..=500`. The timeout bounds a write from queueing
through completion; it is also the OS read timeout.

The current EBIMU driver requires exclusive ownership: every configured serial
device is assigned to exactly one IMU, and every IMU names an existing serial
device. An unused serial entry or two sensors sharing one entry is rejected.

`source_frame` names the URDF link whose axes match the parsed EBIMU sample.
`output_frame` names the frame exposed through hardwared telemetry and state
frames. Both fields are required. The source must be rigidly attached below the
output frame, and their origins must coincide; hardwared applies the URDF
rotation but cannot compensate linear acceleration for a displaced origin. The
rotation is applied to orientation, angular velocity, and linear acceleration
before host filtering or publication. Set both names to the same link for an
identity transform.

`output_rate_hz` is in `1..=1000` and must divide 1000 exactly because the
device accepts an integer-millisecond interval. Consequently, the accepted
rates are `1, 2, 4, 5, 8, 10, 20, 25, 40, 50, 100, 125, 200, 250, 500, 1000`.
`stale_timeout_ms` is in `1..=60000` and must cover at least two output periods
and the 20 ms sensor poll period.

See [`hardware/serial/spec.rs`](../components/bello-hardwared/src/hardware/serial/spec.rs)
and [`hardware/imu/spec.rs`](../components/bello-hardwared/src/hardware/imu/spec.rs).

### Description `logical_buses`

```yaml
logical_buses:
  - name: can_a
    host_id: 253
    bitrate: 1000000
    maximum_utilization: 0.75
    reconnect: { initial_delay_ms: 100, maximum_delay_ms: 5000 }
```

The deployment binds that name to the host:

```yaml
bus_transports: { can_a: { type: socketcan, interface: can0 } }
```

For a GS-USB adapter, use its USB serial number and zero-based CAN channel:

```yaml
bus_transports: { can_a: { type: gsusb, serial: YOUR_ADAPTER_SERIAL, channel: 0 } }
```

The daemon opens a mapped SocketCAN interface without bringing the link up or
setting its bitrate. For GS-USB it finds the matching USB device and configures
the selected channel at the description's `bitrate`. Every GS-USB adapter must
have a readable, unique serial number. One adapter may be assigned to only one
logical bus; the current `gsusbio` socket claims the whole USB interface.
`bitrate` is a timing and validation input and must match the externally
configured SocketCAN interface when that backend is selected. It is in
`10000..=1000000`. `maximum_utilization` is finite and in `(0, 1]`.

Defaults are host ID `253` (`0xFD`), bitrate `1000000`, and maximum utilization
`0.75`. Bus names and transport assignments must both be unique. Each bus
must have at least one motor.

The resolver budgets two extended eight-byte frames per motor per transmission
at roughly 150 bits per frame. Periodic mode uses `command_refresh_rate_hz`;
`on_update` conservatively uses `control_rate_hz`. It rejects aggregate load above
`maximum_utilization` and a group batch that cannot fit within that mode's
transmission period. See [`hardware/can/spec.rs`](../components/bello-hardwared/src/hardware/can/spec.rs)
and the utilization check in
[`config/resolved.rs`](../components/bello-hardwared/src/config/resolved.rs).

### Hardware-description `motors`

```yaml
motors:
  - name: motor_a
    coordinate: coordinate_a
    bus: can_a
    id: 1
    model: ROBSTRIDE_03
    direction: 1
    limits:
      position_min: -2.0
      position_max: 2.0
    health:
      safe_position_min: -2.2
      safe_position_max: 2.2
      feedback_position_tolerance: 0.4
      temperature_warning_c: 60.0
      maximum_temperature_c: 80.0
      temperature_recovery_c: 75.0
      temperature_recovery_duration_ms: 2000
    command_policy:
      maximum_position_step: 0.5
      limit: reject
      position_step: fault
    fault_damping_kd: 10.0
```

`name` identifies the device, while `coordinate` is its external robot/coordinate name.
`bus` must name a declared logical bus. The `(bus, id)` address is unique, and a motor
ID cannot equal that bus's `host_id`. `direction` is `1` or `-1`.

The selected Robstride `model` supplies protocol velocity, torque, and gain
encoding ranges. The URDF supplies the robot-coordinate position, velocity,
and effort envelope. Hardwared intersects the position, velocity, and scalar
feedforward-torque sources, and optional `limits` may only narrow those
results. Position command limits must be ordered and remain within the Type-1
protocol position span. The configured safe feedback range must contain the
command position range; its effective range is also narrowed to the URDF
coordinate envelope. Inventory reports the effective position, velocity, and
feedforward-torque limits plus the model's gain encoding ranges.

`feedback_position_tolerance` is in radians and expands only the feedback
position fault boundary; it does not expand accepted command targets. It must
be finite and between one protocol quantization interval and `0.4`. Its
default is two protocol quantization intervals.

For an asymmetric model range, `direction: -1` reverses and negates the range
before it is exposed in robot coordinates. `velocity_abs` and `torque_abs`
then intersect each side independently, so an absolute cap may narrow only the
larger side.

Supported model names are `ROBSTRIDE_00` through `ROBSTRIDE_06`. Their
different velocity, torque, `kp`, and `kd` capabilities are defined centrally
in
[`descriptions/robstride/models.yaml`](../descriptions/robstride/models.yaml).

Temperature handling has separate warning, fault, and recovery thresholds.
The warning must be below the fault threshold. After a temperature fault, the
motor must stay at or below `temperature_recovery_c` continuously for
`temperature_recovery_duration_ms` before its device fault clears. The group
fault remains latched until explicit recovery.

Scalar `limit` and `position_step` policy behavior values are `reject` and
`fault`. Hardwared never repairs targets. Scalar limits apply to position,
velocity, and feedforward torque. Clients must constrain their requests before
submission, including coupled-joint bounds and maximum position steps.
Finite `kp` and `kd` values are passed through without a hardwared gain guard;
the Robstride Type-1 encoder saturates them to the model's representable range.

The fault damping gain must be finite and positive. Hardwared sends the
configured value without torque-envelope evaluation or group-wide scaling; the
Robstride Type-1 encoder applies only its physical gain encoding range.
`maximum_position_step` compares the first target of a lease generation
with fresh measured feedback and subsequent targets with the previously
accepted target. Bello's maintained hardware description sets this guard to
0.5 rad. Model-dependent bounds are intentionally documented in
[`hardware/robstride/spec.rs`](../components/bello-hardwared/src/hardware/robstride/spec.rs)
rather than duplicated here.

Direct joints use their URDF limits directly. Differential actuator
coordinates receive conservative independent ranges for inventory and scalar
admission, then every complete group command is transformed into joint space
and checked against the exact coupled position and velocity limits. Hardwared
does not compute impedance-derived torque or enforce a coupled differential
effort envelope. Feedforward torque remains subject to each actuator's scalar
clamp.

### Hardware-description `motor_groups`

```yaml
motor_groups:
  - name: group_alpha
    motors: [motor_a, motor_b]
    control_rate_hz: 250.0
    command_refresh_rate_hz: 50.0
    command_timeout_ms: 100
    feedback_timeout_ms: 100
    enable_timeout_ms: 500
    enable_retries: 3
    fault_policy:
      default: damping
      overrides:
        temperature: disable
        reported_motor_fault: disable
```

A group is an arbitrary synchronized control and ownership unit. It is not tied
to body regions or to a single CAN interface. Groups may span buses, and
different groups may share a bus. Every motor belongs to exactly one group.

- `control_rate_hz`: finite, `1..=2000` (default `500`)
- `command_refresh_rate_hz`: finite, `1..=control_rate_hz`; omission preserves
  the legacy behavior by using `control_rate_hz`
- `command_transmission`: `on_update` (default) sends accepted updates immediately
  and refreshes only when the producer pauses; `periodic` stages updates and
  sends the latest target on the CAN TX worker's `command_refresh_rate_hz` clock.
  Missed slots are counted and skipped without catch-up bursts. Four refresh
  periods without completed socket writes latch a transmission fault; updates
  do not reset this watchdog. Original target validity and lease checks apply.
- `periodic_feedback_during_control`: boolean (default `true`). When `false`,
  periodic motor reports are disabled only in Running; command replies provide
  feedback. Damping, stop, disable, faults, and shutdown restore periodic reports.
- `command_timeout_ms`: `1..=10000` (default `100`)
- `feedback_timeout_ms`: `1..=10000` (default `100`)
- `parallel_feedback_position_tolerance`: finite, `(0, 0.4]` (default `0.02`),
  in shared-joint units (radians for revolute joints). This is the maximum
  disagreement between encoder positions resolved through parallel simple
  transmissions. Targets must agree to numerical precision in both position
  and velocity; violating feedback follows the group's `safe_position` fault
  response.
- `enable_timeout_ms`: `1..=30000` (default `500`)
- `enable_retries`: `1..=100` (default `3`)

The fault policy chooses the group's safety reaction. The default response is
`damping`; an override maps a specific cause to `damping` or `disable`. Cause
keys are:

```text
command_limit             command_position_step     safe_position
stale_feedback            temperature               reported_motor_fault
mode_status               communication             enable
transmission              disable                   recover
exit                      internal
```

Per-motor health and command settings decide whether a motor reports a
condition; the group policy decides how the whole owning group reacts to that
condition. Command timeout and lease loss always move an enabled group to
`Damped` and are intentionally not configurable fault-policy causes.
Definitions remain authoritative in
[`config/model.rs`](../components/bello-hardwared/src/config/model.rs).

## Reconnect policy

Both CAN buses and serial devices use:

```yaml
reconnect:
  initial_delay_ms: 100
  maximum_delay_ms: 5000
```

Defaults are 100 ms and 5000 ms. The initial delay must be positive, the
maximum must be at least the initial delay, and the maximum is at most 60000
ms. The runtime uses bounded exponential backoff. See
[`hardware/lifecycle.rs`](../components/bello-hardwared/src/hardware/lifecycle.rs).

## Validation boundaries

Validation is deliberately distributed while still forming one startup pass:

1. YAML deserialization validates shape, scalar types, enum spellings, and
   rejects unknown keys via the raw types in
   [`config/model.rs`](../components/bello-hardwared/src/config/model.rs).
2. Each consuming subsystem converts raw data into a side-effect-free validated
   spec. This is where local ranges and semantic relationships are checked:
   gRPC, CAN, serial, IMU, motor, and motor-group settings.
3. [`ResolvedTopology`](../components/bello-hardwared/src/config/resolved.rs)
   checks global references, uniqueness, exclusive ownership, group membership,
   and CAN timing/utilization.
4. Only after resolution succeeds does startup open hardware and spawn workers.

This keeps hardware-specific rules close to the hardware boundary without
allowing partially validated configuration to reach runtime code.

## YAML anchors and aliases

YAML anchors are useful for repeated complete objects. The production example
uses direct aliases for motor health, command policy, and group fault policy:

```yaml
motors:
  - name: motor_a
    # ...
    health: &motor_health
      safe_position_min: -12.57
      safe_position_max: 12.57
      temperature_warning_c: 60.0
      maximum_temperature_c: 80.0
      temperature_recovery_c: 75.0
      temperature_recovery_duration_ms: 2000
  - name: motor_b
    # ...
    health: *motor_health
```

The anchor labels a YAML node and each alias reuses that whole node before Rust
deserialization. Validation is unchanged: aliased content still has to satisfy
the target object schema. Prefer direct aliases as above. YAML merge keys
(`<<`) are not part of this configuration contract and should not be relied on.

## Reading the examples

The [hardware description](../descriptions/bello/hardware.yaml) defines Bello's
logical buses, 25 physical motors, command policy, and two ownership groups. The
[production deployment](../components/bello-hardwared/config/bello.yaml)
enables all groups and maps `can0`/`can1` to physical interfaces. The
[vCAN deployment](../components/bello-hardwared/config/bello-vcan.yaml)
preserves those logical assignments while mapping `can0`/`can1` to
`vcan0`/`vcan1`.

## Visualizer YAML

The visualizer accepts one independent YAML file containing a local URDF path,
hardware endpoint/TLS settings, fixed or externally tracked root configuration,
refresh rate, and optional group control profiles. Relative URDF and certificate
paths resolve from the YAML directory. The maintained full-Bello example is
[`config/bello.example.yaml`](../components/bello-visualizer/config/bello.example.yaml).
The provisional Mini vCAN example is
[`config/bello-mini.example.yaml`](../components/bello-visualizer/config/bello-mini.example.yaml);
it uses the reviewed motor directions while encoder zeros and ankle reductions
remain calibration-gated.
Copy the full-size example to the ignored local `config/bello.yaml`, or the
Mini example to the ignored local `config/bello-mini.yaml`. Their hardware
endpoints default to localhost. Both use the matching estimator PoseService on
port 50053; MuJoCo ground truth remains available separately on port 50052.
Use `--hardware-endpoint` and `--pose-endpoint` to redirect those services
without editing the YAML. The latter is valid only with
`root.source: external_pose`.
The adjacent [`config/vcan.yaml`](../components/bello-visualizer/config/vcan.yaml)
and `vcan.urdf` are a minimal two-joint UI fixture.

Control is unavailable unless a selected group has an explicit profile.
Hardwared inventory supplies group membership, coordinate names, and hard
position, velocity, torque, `kp`, and `kd` bounds. Profiles may narrow the
non-position bounds and configure their defaults, plus group command rate,
validity, slew, and feedback-staleness limits. Defaults are 50 Hz, 100 ms, and
200 ms respectively. The URDF supplies rendering, hierarchy, joint limits, and
direct or differential coordinate mappings; it never supplies hardware gains.
See the
[`bello-visualizer` guide](../components/bello-visualizer/README.md).

The Bello Mini profile uses 300 Hz safety ticks and TX-owned periodic CAN transmission,
with `periodic_feedback_during_control: false`. Its policy still supplies new
50 Hz targets; each target is retained between updates without extending its
original validity deadline. Idle/enabling/damped monitoring retains periodic
feedback. At seven motors per bus, 300 command/reply exchanges per motor per
second use approximately 63--67% of a 1 Mbps bus. Measure the active hardware
before treating that estimate as an observed rate.
