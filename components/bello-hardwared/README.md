# Bello hardware daemon

`bello-hardwared` is a single Rust process that owns Robstride CAN buses, EBIMU
serial input, motor-group state machines, safety watchdogs, gRPC control leases,
telemetry, and optional Rerun recording. It deliberately contains no robot policy,
kinematics, or pose controller.

## Build and run

With direnv installed, approve the project once and the pinned Nix environment
will load whenever you enter the directory:

```sh
direnv allow
```

Without direnv, enter it manually:

```sh
cd components/bello-hardwared
nix develop
cargo test
cargo run -- --check --config config/bello.yaml
cargo run -- --config config/bello.yaml
```

Protobuf generation uses the `protoc` executable on `PATH`; the pinned Nix
development shell provides it. Cargo does not download a vendored `protoc`
binary.

The daemon always uses its configured transports. For development without
physical actuators, use the external Robstride emulator on `vcan`; production
code contains no simulated transport or device branches. Build Rerun support
explicitly:

```sh
cargo run --features rerun -- --config config/bello.yaml
```

For transport-level integration without physical actuators, run the workspace's
`robstride-emulator` on `vcan0` and use `config/vcan.yaml`. This exercises
SocketCAN, Robstride frame encoding and decoding,
feedback watchdogs, and transport/device fault propagation end to end.

`rerun.mode` controls where hardware telemetry is sent:

| Mode | Behavior | Setting used |
| --- | --- | --- |
| `off` | Disables Rerun telemetry. | None |
| `save` | Writes telemetry to an `.rrd` file for later viewing. Relative paths are resolved from the daemon's working directory. | `rerun.path` |
| `connect` | Streams telemetry to an existing Rerun gRPC server or viewer at `rerun+http://<address>/proxy`. | `rerun.address` |

Every mode except `off` requires the binary to be built with `--features rerun`.
The daemon never embeds a viewer: use an external Rerun viewer or open the
`.rrd` file produced by `save` mode.

## Ownership and control

Motor groups are generic named sets. A group may span
several buses, and several groups may share a bus. Configuration validation
requires every motor to belong to exactly one group, so no actuator has two
command owners.

Each group has an independent FSM:

```text
Idle --enable--> Enabled --start--> Running
  ^                 ^                 |
  |                 +------stop-------+
  +----recover---- Fault <--- watchdog / lease loss / explicit fault
```

Enable configures MIT/impedance mode, enables active feedback, enables motors,
and waits for healthy feedback from every motor in the group.
Starting requires a complete, fresh, safe command batch. While Running, every
control tick retransmits the latest batch. A command timeout, stale feedback,
motor fault, over-temperature condition, or lease loss applies the configured
fault response and enters `Fault`. Recovery is explicit and returns to `Idle`.
It succeeds only after clear-fault stop frames are delivered and every motor has
an online CAN transport, fresh healthy feedback, and no latched motor fault. A
rejected recovery leaves the group fault-latched.

Physical transports and individual devices use the shared typed lifecycle FSM:

```text
Starting -> Connecting -> Online -> Degraded -> Fault -> Reconnecting
                ^                                  |           |
                +----------------------------------+-----------+
Any state -------------------------------------------------> Shutdown
```

The SocketCAN layer owns only raw frame transport, priority queues, interface
health, and reconnects. The Robstride layer owns protocol encoding/decoding,
one `MotorController` per configured address, latest feedback, reported motor
faults, intrinsic feedback health, and the per-motor feedback watchdog. Intrinsic
health includes CAN availability, fault bits, temperature, feedback position,
and the expected operating mode. A motor derives transport loss from the status
of its referenced CAN bus. The group controller deals only in motor controllers
and never constructs a CAN frame, parses Robstride feedback, or classifies device
health. It retains synchronized command validation, leases, command freshness,
group-wide fault responses, and the fault-latched motion FSM.

Group responses are selected independently in `motor_groups[].fault_responses`.
The typed causes are command limit, command position step, command timeout, safe
position, stale feedback, temperature fault, reported motor fault bits, mode
status, communication, enable, start, stop, transmission, lease loss, manual
fault, disable, and process exit. A fault is latched and published before its
response is executed.

Responses are `none`, `damping`, or `disable`. `none` sends no new hardware
command and stops the faulted group's periodic command stream. `damping` sends a
finite burst of position-hold damping commands using
each motor's latest measured position and `motors[].fault_damping.kd`. The burst
count and rate come from `motor_groups[].damping_response`. `disable` sends
Robstride stop frames.

Temperature has separate warning and fault thresholds. At or above
`temperature_warning_c`, the motor remains `Online`, but its status detail and
health telemetry report a warning. Temperature faults use hysteresis. A motor faults at or above
`maximum_temperature_c`, remains thermally faulted while cooling, and becomes
device-healthy only after staying at or below `temperature_recovery_c` for
`temperature_recovery_duration_ms`. Any sample above the recovery threshold
restarts the cool-down timer. Motor-device recovery never clears the group fault
latch.

An unavailable CAN bus therefore moves dependent motor device FSMs through
fault and reconnect states. Every enabled or running group that depends on a
unavailable motor applies its configured group fault response and enters
its own fault-latched state. The CAN supervisor rejects queued safety frames,
discards queued periodic commands, and reopens fresh SocketCAN sockets with
bounded exponential backoff. A successful transport or motor reconnect never
resumes motion: affected groups still require explicit recovery, enable, a
fresh command, and start.

Named serial transports own port opening, raw byte I/O, lifecycle state, and
reconnect backoff. Named sensor drivers reference a transport and own framing,
device configuration, decoding, and sample-validity rules. The EBIMU driver is
the first sensor implementation: it accumulates newline-delimited ASCII frames,
marks malformed input degraded, and requests a transport reconnect when valid
samples become stale. This covers tty-backed USB serial devices; it is not a
generic libusb device layer.

The two hardware paths use the same layered topology:

```text
raw CAN transport -----> Robstride motor device -----> motor group policy/FSM
raw serial transport --> IMU sensor device
```

Several serial devices and IMUs may be configured. A serial byte stream is
exclusive, so validation prevents two enabled sensors from claiming the same
serial transport.

The only motion command surface is impedance control: position, velocity,
feed-forward torque, `kp`, and `kd`. The command-policy evaluator implements the same basic
checks as the Motomind stack:

| Check | Condition | Default behavior in the example |
| --- | --- | --- |
| Command limit | A command field is non-finite or outside its configured/model range. | Finite out-of-range values are clamped and reported as warnings; non-finite values always fault and reject. |
| Command position step | Target position differs from the latest measured position by more than `maximum_position_step`. | Reject and fault. |
| Safe position | Measured position is outside `safe_position_min..=safe_position_max`. | Motor and group fault. |
| Command timeout | No accepted fresh batch before `command_timeout_ms` or its shorter `valid_for_ms`. | Group fault. |
| Stale feedback | An enabled motor has no fresh feedback within `feedback_timeout_ms`. | Motor and group fault. |
| Temperature warning/fault | Temperature reaches the configured warning or fault threshold. | Warning remains online; fault latches with hysteresis. |
| Motor fault | Robstride reports any nonzero fault bits. | Motor and group fault. |
| Mode status | Fresh operating feedback does not report mode status `2`. | Motor and group fault. |

Command-limit and step policies are independently configured per motor with
`*_action` (`clamp` or `reject`) and `*_severity` (`warning` or `fault`). A
warning may therefore clamp and accept, or reject without faulting. A fault
always applies its cause-specific group response and latches the group fault.
Complete group batches, monotonically increasing sequences, and freshness
deadlines are enforced separately from this command policy.

## gRPC API

The protobuf contract is
[proto/bello/hardware/v1/hardware.proto](proto/bello/hardware/v1/hardware.proto).
A client:

1. calls `ListHardware` to discover buses, motors, arbitrary groups, and state;
2. acquires an exclusive lease for one group and renews it before expiry;
3. sends `EnableGroup`, a complete command batch, then `StartControl`;
4. streams complete command batches while consuming acknowledgements and
   `WatchTelemetry`;
5. stops and disables the group, then releases its lease.

`ListHardware` includes named CAN buses, motors, serial devices, and IMUs with
typed `DeviceStatus` values. Motor and IMU samples also carry the device status
captured with that sample. `WatchTelemetry(include_health = true)` publishes
every transport and device transition. IMU telemetry may be filtered by name
through `WatchTelemetryRequest.imus`.
`WatchTelemetry(include_command_policy_events = true)` publishes command-policy
warnings and faults, including whether the command was modified or rejected and
which response was selected. The same command-policy events
are written as text logs when Rerun is enabled.

The protobuf lifecycle uses `DeviceState` rather than free-form state strings.
Removed field numbers and names are reserved so obsolete fields cannot be
accidentally reused in a later schema revision.

Leases are exclusive per group, not per bus. The server is intentionally
insecure for an isolated robot LAN; deploy network isolation or a TLS proxy when
crossing trust boundaries. A ROS migration bridge should be a separate process
that translates existing messages to this gRPC API, keeping this binary ROS-free.

## Runtime threads

- one reconnecting supervisor per physical CAN bus, with transmit and receive
  session workers while the interface is online;
- one Robstride receive dispatcher that routes raw frames by `(bus, motor ID)`
  and maintains the individual motor device FSMs;
- one fixed-rate controller per logical group;
- one reconnecting byte-stream supervisor per serial device;
- one protocol/framing worker per enabled IMU;
- Tokio tasks for gRPC streams, lease expiry, telemetry fan-out, and Rerun.

The bus transmit worker arbitrates groups sharing an interface. Each bus has a
high-priority channel for safety and lifecycle frames and a normal channel for
periodic control frames; the writer always selects safety traffic first.
Configuration rejects an estimated aggregate CAN load over each bus's
utilization budget.

Each group controller has separate inputs for lifecycle operations and the latest
command snapshot. An incoming command replaces an older command that is still
waiting to be validated, so network bursts cannot build a stale-command backlog.
Lease verification and command-mailbox insertion share the lease expiry gate,
so a command cannot be queued through a release or expiry race.
Control ticks run before ordinary requests whenever their deadline is due. Lease
loss and process shutdown are atomic, latched signals; they cannot be dropped
when the lifecycle queue is full. Motor enable checks those signals throughout
its feedback wait and retry periods. Damping, stop, enable, and other safety or
lifecycle CAN frames use the priority enqueue path. Normal periodic control
frames remain non-blocking and fault the group if the bus queue is saturated.

Worker ownership is RAII-based. A partial startup failure signals and joins every
worker that was already created. Normal shutdown stops and joins group workers
before dropping transport supervisors, allowing final motor-stop frames to be
transmitted when their buses are online.

## Source layout

```text
src/
  app/                process assembly and worker ownership
  api/grpc/           thin tonic service plus inventory and stream handlers
  config/             serialized model and cross-boundary topology resolution
  control/            group handle, worker, controller, FSM, command policy
  domain/             transport-independent IDs, commands, telemetry, errors
  hardware/can/       protocol-agnostic SocketCAN frame transport and bus FSM
  hardware/imu/       EBIMU framing, configuration, parsing, and sensor worker
  hardware/lifecycle  shared physical-device FSM and reconnect policy
  hardware/robstride/ protocol codec, motor controllers, device FSMs, parameters
  hardware/serial     generic byte-stream transport and reconnect supervisor
```

The gRPC layer converts generated protobuf messages at the API boundary. Control
and hardware code use domain types and do not depend on protobuf.

Raw `*Config` structures only describe the YAML format. Serial and CAN reconnect
settings use `initial_delay_ms` and `maximum_delay_ms`; the delay doubles after
each failed attempt and is capped at the configured maximum.
`imus[].stale_timeout_ms` sets the maximum interval without a valid decoded
sample. Motor configuration is deliberately separated by ownership:
`motors[].health` contains feedback-derived health thresholds,
`motors[].command_policy` contains incoming-command validation policy, and
`motors[].fault_damping` contains the per-motor gain used only when the group
selects a damping response. `motor_groups[].fault_responses` maps typed causes
to coordinated responses, while `motor_groups[].damping_response` controls the
burst timing. Topology validation requires the safe position range to contain
the command position range, the warning and recovery temperatures to be
strictly below the trip temperature, and a positive recovery duration. Each owning boundary
converts its raw configuration into a side-effect-free `*Spec`: `RuntimeSpec`,
`GrpcSpec`, `RerunSpec`, `BusSpec`, `SerialSpec`, `MotorSpec`, `GroupSpec`, and
`ImuSpec`.
Those conversions validate local values without opening devices or starting
threads. `ResolvedTopology::load` parses, validates, and resolves exactly once.
The topology resolver validates relationships across boundaries:
unique names and CAN addresses, bus/motor and serial/sensor references,
exclusive group and serial-stream ownership, and aggregate bus utilization.
Only a fully resolved topology can be passed to runtime workers.

Group state changes are published immediately. Periodic group timing statistics
are decimated to 25 Hz; motor and IMU telemetry remain event-driven at their
configured or device feedback rates. Per-motor health transitions use
`motor/<name>`, independently of `can/<name>` and the group state. gRPC adds
subscriber transport delay to the reported sample age and reports broadcast lag
as health telemetry. Rerun reports lag and continues instead of silently
terminating its subscriber.
