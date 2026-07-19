# Configuration

[Documentation index](README.md)

`bello-hardwared` reads one YAML document, validates it without touching the
hardware, and resolves it into the topology used for the lifetime of the
process. Configuration is not hot-reloaded.

Validate a file before deployment with:

```console
cargo run -p bello-hardwared -- \
  --check --config components/bello-hardwared/config/bello.yaml
```

`--check` and normal startup use the same parse and resolution path. Passing it
does not prove that an interface or device is present, accessible, or configured
to match the host. CAN and serial connectivity is opened asynchronously; an
unavailable transport normally appears as reconnecting after process startup.

The maintained examples are:

- [production hardware](../components/bello-hardwared/config/bello.yaml)
- [SocketCAN virtual-bus development](../components/bello-hardwared/config/vcan.yaml)

The raw schema and defaults live in
[`config/model.rs`](../components/bello-hardwared/src/config/model.rs). The
validated boundary types live beside the subsystem that consumes them, and
cross-object checks live in
[`config/resolved.rs`](../components/bello-hardwared/src/config/resolved.rs).
Those files are the source of truth when this guide and the executable differ.

## Document structure

The top-level sections are:

```yaml
runtime: {}          # optional; process-wide buffering
grpc: {}             # optional; server and lease settings
serial_devices: []   # optional only when no serial sensor is used
imus: []             # optional only when no IMU is used
buses: []            # required, with at least one entry
motors: []           # required, with at least one entry
motor_groups: []     # required, with at least one entry
```

Unknown fields are rejected at every YAML object boundary. This catches
misspelled keys instead of silently applying a default.

### `runtime`

`telemetry_capacity` is the capacity of the process-wide broadcast channel,
not a sampling rate and not a per-device history length. Every motor sample,
IMU sample, group timing sample, lifecycle transition, and command-policy event
shares it. Each subscriber receives events independently; a lagging subscriber
can drop old events without blocking hardware control.

- default: `4096`
- valid range: `64..=1000000`

### `grpc`

```yaml
grpc:
  listen: 127.0.0.1:50051
  default_lease_ms: 1000
  maximum_lease_ms: 5000
```

The listen address defaults to loopback. The lease values must satisfy
`20 <= default_lease_ms <= maximum_lease_ms <= 300000`. Control leases are
exclusive per motor group; read-only telemetry streams do not require a
control lease. Do not expose the server on a non-loopback address unless the
deployment provides the required network isolation and transport security.

See [`api/grpc/spec.rs`](../components/bello-hardwared/src/api/grpc/spec.rs)
for validation and the
[protobuf API](../components/bello-hardware-api/proto/bello/hardware/v1/hardware.proto)
for lease semantics.

### `serial_devices` and `imus`

A serial entry describes a transport. An IMU entry describes the sensor
protocol running on that transport:

```yaml
serial_devices:
  - name: imu_serial
    port: /dev/ttyUSB0
    baud_rate: 921600
    timeout_ms: 100
    reconnect: { initial_delay_ms: 100, maximum_delay_ms: 5000 }

imus:
  - name: imu
    serial_device: imu_serial
    stale_timeout_ms: 500
    output_rate_hz: 500
```

Serial names and physical ports must be unique. `baud_rate` must be positive,
and `timeout_ms` is in `1..=500`. The timeout bounds a write from queueing
through completion; it is also the OS read timeout.

The current EBIMU driver requires exclusive ownership: every configured serial
device is assigned to exactly one IMU, and every IMU names an existing serial
device. An unused serial entry or two sensors sharing one entry is rejected.

`output_rate_hz` is in `1..=1000` and must divide 1000 exactly because the
device accepts an integer-millisecond interval. Consequently, the accepted
rates are `1, 2, 4, 5, 8, 10, 20, 25, 40, 50, 100, 125, 200, 250, 500, 1000`.
`stale_timeout_ms` is in `1..=60000` and must cover at least two output periods
and the 20 ms sensor poll period.

See [`hardware/serial/spec.rs`](../components/bello-hardwared/src/hardware/serial/spec.rs)
and [`hardware/imu/spec.rs`](../components/bello-hardwared/src/hardware/imu/spec.rs).

### `buses`

```yaml
buses:
  - name: can_a
    interface: can0
    host_id: 253
    bitrate: 1000000
    maximum_utilization: 0.75
    reconnect: { initial_delay_ms: 100, maximum_delay_ms: 5000 }
```

The daemon opens an existing SocketCAN interface; it does not bring the link up
or set its bitrate. `bitrate` is a timing and validation input and must match
the externally configured classic-CAN interface. It is in
`10000..=1000000`. `maximum_utilization` is finite and in `(0, 1]`.

Defaults are host ID `253` (`0xFD`), bitrate `1000000`, and maximum utilization
`0.75`. Bus names and physical interface names must both be unique. Each bus
must have at least one motor.

The resolver's current timing model budgets two extended eight-byte frames per
motor per group tick at roughly 150 bits per frame. It rejects both aggregate load
above `maximum_utilization` and a group batch that cannot fit within its control
period. See [`hardware/can/spec.rs`](../components/bello-hardwared/src/hardware/can/spec.rs)
and the utilization check in
[`config/resolved.rs`](../components/bello-hardwared/src/config/resolved.rs).

### `motors`

```yaml
motors:
  - name: motor_a
    joint: joint_a
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
      temperature_warning_c: 60.0
      maximum_temperature_c: 80.0
      temperature_recovery_c: 75.0
      temperature_recovery_duration_ms: 2000
    command_policy:
      maximum_position_step: 0.5
      limit: clamp
      position_step: fault
    fault_damping_kd: 2.0
```

`name` identifies the device, while `joint` is its external robot/joint name.
`bus` must name an existing bus. The `(bus, id)` address is unique, and a motor
ID cannot equal that bus's `host_id`. `direction` is `1` or `-1`.

The selected Robstride `model` supplies physical velocity, torque, and gain
caps. Optional `limits` may only narrow those caps. Position command limits
must be ordered and remain within the Type-1 protocol position span. The
configured safe feedback range must contain the command position range.

Supported model names are `ROBSTRIDE_00` through `ROBSTRIDE_06`. Their
different velocity, torque, `kp`, and `kd` capabilities are defined centrally
in
[`robstride-protocol/src/model.rs`](../components/robstride-protocol/src/model.rs).

Temperature handling has separate warning, fault, and recovery thresholds.
The warning must be below the fault threshold. After a temperature fault, the
motor must stay at or below `temperature_recovery_c` continuously for
`temperature_recovery_duration_ms` before its device fault clears. The group
fault remains latched until explicit recovery.

Command policy behavior values are `clamp`, `reject`, and `fault`. The fault
damping gain must be positive and no greater than the effective motor `kd`
limit. Model-dependent bounds are intentionally documented in
[`hardware/robstride/spec.rs`](../components/bello-hardwared/src/hardware/robstride/spec.rs)
rather than duplicated here.

### `motor_groups`

```yaml
motor_groups:
  - name: group_alpha
    motors: [motor_a, motor_b]
    control_rate_hz: 250.0
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
- `command_timeout_ms`: `1..=10000` (default `100`)
- `feedback_timeout_ms`: `1..=10000` (default `100`)
- `enable_timeout_ms`: `1..=30000` (default `500`)
- `enable_retries`: `1..=100` (default `3`)

The fault policy chooses the group's safety reaction. The default response is
`damping`; an override maps a specific cause to `damping` or `disable`. Cause
keys are:

```text
command_limit             command_position_step     command_timeout
safe_position             stale_feedback            temperature
reported_motor_fault      mode_status               communication
enable                    start                     stop
transmission              lease_loss                disable
recover                   exit                      internal
```

Per-motor health and command settings decide whether a motor reports a
condition; the group policy decides how the whole owning group reacts to that
condition. Definitions remain authoritative in
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

## Rerun observer configuration

`bello-rerun` reads its own YAML document; Rerun settings are not accepted in
the hardware-daemon configuration. The maintained observer example is
[`config/bello.yaml`](../components/bello-rerun/config/bello.yaml):

```yaml
hardware_endpoint: http://127.0.0.1:50051

reconnect:
  initial_delay_ms: 100
  maximum_delay_ms: 5000

output:
  mode: save
  path: bello_hardware.rrd
```

To send events to an existing Rerun listener instead, replace `output` with:

```yaml
output:
  mode: connect
  address: 127.0.0.1:9876
```

Validate the observer independently without contacting hardwared or opening an
output:

```sh
cargo run -p bello-rerun -- \
  --check --config components/bello-rerun/config/bello.yaml
```

Unknown fields are rejected. The hardware endpoint must be a valid gRPC HTTP or
HTTPS URI. Reconnect delays must be positive, `maximum_delay_ms` must be at least
`initial_delay_ms`, and the maximum is limited to 60000 ms. A save path or
connect address must be non-empty for its respective mode. The connect address
is a bare `host:port` with a port in `1..=65535`; schemes, paths, queries, and
credentials are rejected. `hardware_endpoint` and `output` are required;
omitting `reconnect`
selects 100 ms initially and 5000 ms maximum. There is no `off` mode: omit the
optional process when visualization is not wanted.

HTTPS hardware endpoints use the host's native certificate roots and HTTP/2.
Connections use a five-second connect timeout plus HTTP/2 keepalive every 10
seconds, with a three-second keepalive timeout even while otherwise idle.

The observer-to-writer queue is deliberately not configurable. It holds 4096
events and drops on overflow so Rerun output cannot backpressure gRPC or signal
handling. This queue is separate from the daemon's configurable
`runtime.telemetry_capacity` broadcast ring.

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

The [production file](../components/bello-hardwared/config/bello.yaml) shows two
physical CAN interfaces, a serial EBIMU, motors split across both buses, groups
that span those buses, and anchors for shared policy objects. Its device paths,
IDs, model names, directions, limits, and safety policy are deployment inputs,
not universal defaults.

The [vcan file](../components/bello-hardwared/config/vcan.yaml) is a minimal
host-only topology: two emulated Robstride motors share `vcan0`, one has
reversed direction, and the group runs at 250 Hz. It deliberately omits serial
and IMU sections. Its `bitrate` remains present because scheduling and load
validation need nominal timing even though a virtual CAN interface has no
physical bitrate.
