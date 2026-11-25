# Configuration

For COG serial telemetry, `power_boards[].emergency_stop` (`hardware` or
`damping`), and emulated power UARTs, see
[Power-board telemetry](power_board.md).

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

Start with the robot's [hardware description](../descriptions/) and select a
[deployment profile](../components/bello-hardwared/config/).

Before running Bello Mini on hardware, check the outstanding calibration work in
[motor_topology.yaml](../descriptions/bello_mini/motor_topology.yaml) and set the IMU serial path for the installed adapter.

The raw schema and defaults live in
[`config/model.rs`](../components/bello-hardwared/src/config/model.rs). The
validated boundary types live beside the subsystem that consumes them, and
cross-object checks live in
[`config/resolved.rs`](../components/bello-hardwared/src/config/resolved.rs).

The [seven-axis Milly](../descriptions/milly-7dof/) and [dual stereo fixture](../descriptions/milly-7dof-gimbal/) have physical, loopback simulation and vCAN deployments.

Both [dual Milly O6](../descriptions/milly-7dof-o6/) and [O6 with stereo gimbal](../descriptions/milly-7dof-o6-gimbal/) pair the arm deployment with a [standalone Linux hand driver](../components/bello-hand/). The hand driver reads the same SocketCAN interface bindings as hardwared.

## Document structure

Unknown fields are rejected at every YAML object boundary. This catches
misspelled keys instead of silently applying a default. Relative description
paths resolve from the deployment file. Unknown or duplicate groups, missing or
unused bus mappings, and incomplete description ownership are rejected.

Each shipped robot has a `descriptions/<model>/hardware.yaml` and a
`urdf/<model>.urdf` in the same model directory. The deployment's
`description` field points to the hardware YAML, which names the URDF.
Relative URDF paths resolve from the hardware description. Every controlled
coordinate must resolve to either a movable URDF joint or one actuator
coordinate of a complete differential or simple transmission. Several simple
transmissions may drive one shared joint, with distinct actuator coordinates.
Every actuator of that joint must belong to the same motor group. Their scalar
effort envelopes divide the joint envelope into equal joint-torque shares,
accounting for each signed reduction. Shipped URDFs use
`<mechanicalReduction>` for both simple and differential transmissions.
Transmitted joints must be controlled through their declared actuator
coordinates; their joint names cannot serve as implicit direct-drive aliases.

### `runtime`

`telemetry_capacity` is the capacity of the process-wide broadcast channel,
not a sampling rate and not a per-device history length. Every motor sample,
IMU sample, group timing sample, lifecycle transition, and command-policy event
shares it. Each subscriber receives events independently; a lagging subscriber
can drop old events without blocking hardware control.

`motor_velocity_filter` filters actuator-reported velocity. Deriving the
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

Each IMU cutoff must be finite, positive, and below half the configured output
rate. The first sample passes through unchanged.
A non-increasing device timestamp, a gap longer than 100 ms, or a serial
reconnect resets the filter instead of blending across an invalid interval.
This host filter is separate from the EBIMU's persistent onboard
accelerometer-filter and vibration-cancellation settings.

### `grpc`

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

Set `simulation: true` once at the top of a simulated deployment. It applies to
all serial ports and defaults to `false`. Each entry in `serial_ports` is a
device path. On macOS, simulation opens those paths at baud zero to skip the
hardware speed ioctl. Other opens use the baud rate from the hardware
description. CAN transport selection stays in `bus_transports`.

Start MuJoCo before hardwared. If the simulator exits, restart MuJoCo before
restarting hardwared. The daemon stops using a disconnected simulated serial
port because the OS can reassign its PTY path. Physical serial ports retry.

Each serial device belongs to exactly one IMU or power board.

`source_frame` names the URDF link whose axes match the parsed EBIMU sample.
`output_frame` names the frame exposed through hardwared telemetry and state
frames. Both fields are required. The source must be rigidly attached below the
output frame, and their origins must coincide; hardwared applies the URDF
rotation but cannot compensate linear acceleration for a displaced origin. The
rotation is applied to orientation, angular velocity, and linear acceleration
before host filtering or publication. Set both names to the same link for an
identity transform.

`output_rate_hz` is in `1..=1000` and must divide 1000 exactly because the
device accepts an integer-millisecond interval.
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

For a simulated bus, use `{ type: sim, daemon_port: 19100, simulator_port: 19101 }`.
See the [simulator setup](operations.md#simulate-can-without-socketcan) for the peer configuration.

GS-USB is supported on Linux, macOS, and Windows. SocketCAN is Linux-only;
selecting it on another platform fails configuration checking and startup.
The maintained `milly-gsusb.yaml` deployment is a portable example.

The daemon opens a mapped SocketCAN interface without bringing the link up or
setting its bitrate. For GS-USB it finds the matching USB device and configures
the selected channel at the description's `bitrate`. Every GS-USB adapter must
have a readable, unique serial number. One adapter may be assigned to only one
logical bus; the current `gsusbio` socket claims the whole USB interface.
`bitrate` is a timing and validation input and must match the externally
configured SocketCAN interface when that backend is selected. It is in
`10000..=1000000`. `maximum_utilization` is finite and in `(0, 1]`.

Bus names and transport assignments must both be unique. Each bus
must have at least one motor.

The resolver budgets two extended eight-byte frames per motor per transmission
at roughly 150 bits per frame. Periodic mode uses `command_refresh_rate_hz`;
`on_update` conservatively uses `control_rate_hz`. It rejects aggregate load above
`maximum_utilization` and a group batch that cannot fit within that mode's
transmission period. See [`hardware/can/spec.rs`](../components/bello-hardwared/src/hardware/can/spec.rs)
and the utilization check in
[`config/resolved.rs`](../components/bello-hardwared/src/config/resolved.rs).

### Hardware-description `motors`

`name` identifies the device, while `coordinate` is its external robot/coordinate name.
`bus` must name a declared logical bus. The `(bus, id)` address is unique, and a motor
ID cannot equal that bus's `host_id`. `direction` is `1` or `-1`.

The selected motor `model` supplies protocol velocity, torque, and gain
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

Use `RS_00` through `RS_06` for the motor model. The
[model catalog](../descriptions/motors/models.yaml) lists each model's velocity,
torque, `kp`, and `kd` ranges. Inventory reports the short model names.
When upgrading, change existing profiles to these IDs and update the daemon and
Python client together.

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
the Type-1 encoder saturates them to the model's representable range.

The fault damping gain must be finite and positive. Hardwared sends the
configured value without torque-envelope evaluation or group-wide scaling; the
Type-1 encoder applies only its physical gain encoding range.
`maximum_position_step` compares the first target of a lease generation
with fresh measured feedback and subsequent targets with the previously
accepted target.

Direct joints use their URDF limits directly. Differential actuator
coordinates receive conservative independent ranges for inventory and scalar
admission, then every complete group command is transformed into joint space
and checked against the exact coupled position and velocity limits. Hardwared
does not compute impedance-derived torque or enforce a coupled differential
effort envelope. Feedforward torque remains subject to each actuator's scalar
clamp.

### Hardware-description `motor_groups`

A group is an arbitrary synchronized control and ownership unit. It is not tied
to body regions or to a single CAN interface. Groups may span buses, and
different groups may share a bus. Every motor belongs to exactly one group.

- Omit `command_refresh_rate_hz` to refresh at `control_rate_hz`.
- `command_transmission`: `on_update` (default) sends accepted updates immediately
  and refreshes only when the producer pauses; `periodic` stages updates and
  sends the latest target on the CAN TX worker's `command_refresh_rate_hz` clock.
  Missed slots are counted and skipped without catch-up bursts. Four refresh
  periods without completed socket writes latch a transmission fault; updates
  do not reset this watchdog. Original target validity and lease checks apply.
- `periodic_feedback_during_control`: boolean (default `true`). When `false`,
  periodic motor reports are disabled only in Running; command replies provide
  feedback. Damping, stop, disable, faults, and shutdown restore periodic reports.
- `parallel_feedback_position_tolerance`: finite, `(0, 0.4]` (default `0.02`),
  in shared-joint units (radians for revolute joints). This is the maximum
  disagreement between encoder positions resolved through parallel simple
  transmissions. Targets must agree to numerical precision in both position
  and velocity; violating feedback follows the group's `safe_position` fault
  response.

The fault policy chooses the group's safety reaction. The default response is
`damping`; an override maps a specific cause to `damping` or `disable`.

Per-motor health and command settings decide whether a motor reports a
condition; the group policy decides how the whole owning group reacts to that
condition. Command timeout and lease loss always move an enabled group to
`Damped` and are intentionally not configurable fault-policy causes.
Definitions remain authoritative in
[`config/model.rs`](../components/bello-hardwared/src/config/model.rs).

## Reconnect policy

CAN and serial transports reconnect with bounded exponential backoff. See
[`hardware/lifecycle.rs`](../components/bello-hardwared/src/hardware/lifecycle.rs).

## Validation boundaries

The [YAML types](../components/bello-hardwared/src/config/model.rs) reject unknown fields.
Each subsystem checks its settings, then the [topology resolver](../components/bello-hardwared/src/config/resolved.rs)
checks references, exclusive ownership and CAN capacity before startup opens hardware.

## YAML anchors and aliases

Use direct aliases to share a complete object, as in [Bello’s hardware description](../descriptions/bello/hardware.yaml).
YAML merge keys (`<<`) are unsupported.
