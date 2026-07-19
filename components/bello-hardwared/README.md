# Bello hardware daemon

`bello-hardwared` is one Rust process that owns Robstride CAN actuators, EBIMU
serial sensors, motor-group control, safety policy, gRPC, telemetry, leases, and
optional Rerun logging. Robot policy, kinematics, and pose control remain outside
this component.

The API and configuration are intentionally greenfield. Backward compatibility
is not a design constraint; obsolete fields and aliases are removed instead of
being carried as migration scaffolding.

## Build and run

The repository's direnv configuration loads a pinned Nix shell containing Rust,
`protoc`, and Buf:

```sh
direnv allow
cargo test --workspace
cargo run -p bello-hardwared -- --check --config components/bello-hardwared/config/bello.yaml
cargo run -p bello-hardwared -- --config components/bello-hardwared/config/bello.yaml
```

Cargo uses `protoc` from `PATH`; `build.rs` does not download a compiler. The
production daemon contains no simulated hardware path. Use
`robstride-emulator` with `config/vcan.yaml` for virtual integration.

Rerun is optional at compile time:

```sh
cargo run -p bello-hardwared --features rerun -- --config components/bello-hardwared/config/bello.yaml
```

Its YAML is mode-specific:

```yaml
rerun: { mode: off }
# rerun: { mode: save, path: bello_hardware.rrd }
# rerun: { mode: connect, address: 127.0.0.1:9876 }
```

The Rerun task is owned and joined by the application so save recordings are
closed during orderly shutdown.

## Layering

```text
SocketCAN transport --> Robstride motor device --> motor-group policy/FSM
serial transport ----> EBIMU sensor device
                                      |
                                      +--> telemetry --> gRPC / optional Rerun
```

Transport supervisors own raw I/O, health, bounded queues, and reconnects.
Device controllers own protocol parsing and device-specific health. A group
controller owns synchronized commands and coordinated motion reactions. CAN and
serial intentionally share lifecycle and reconnect primitives, but not a generic
transport trait: their queues and session failure semantics differ.

The daemon opens existing SocketCAN interfaces; it does not configure link
bitrate. Each bus `bitrate` is a classic CAN nominal bitrate in 10 kbit/s through
1 Mbit/s used for timing and utilization validation, and must exactly match the
interface configured externally with `ip link` or equivalent tooling.

All transports and devices use this lifecycle:

```text
Starting -> Connecting -> Online -> Degraded -> Fault -> Reconnecting
                ^                                  |           |
                +----------------------------------+-----------+
Any state -------------------------------------------------> Shutdown
```

Rejected lifecycle transitions are logged. Reconnect uses bounded exponential
backoff. CAN write deadlines and serial read/write timeouts are bounded during
configuration validation, giving shutdown a bounded transport-I/O wait.

## Motor groups and ownership

Groups are generic named motor sets. They may span buses, and buses may be
shared by several groups. Every motor belongs to exactly one group.

```text
Idle --enable--> Enabled --start--> Running
  ^                 ^                 |
  |                 +------stop-------+
  +----recover---- Fault <--- watchdog / lease loss / safety fault
```

Enable configures impedance mode and waits for fresh, healthy feedback from
every motor. Start requires a complete, fresh command from the current lease
generation. Disable, fault, recovery, lease loss, and shutdown clear all cached
commands and sequence state, so a new ownership epoch always requires a new
command.

Leases are exclusive per group. Every queued lifecycle operation and command is
tagged with the lease generation and checked by the group worker at execution
time. Release and expiry invalidate that generation before waking the worker.
`ReleaseLease` waits until the group has applied its lease-loss reaction and
returns the resulting typed state. Nonzero requested durations are clamped to
20 ms through the configured maximum; expiry is polled every 5 ms.

The group worker accepts a small bounded queue of lifecycle operations and one
latest-wins command snapshot. Lifecycle operations are always considered before
the command mailbox, so command traffic cannot starve stop, disable, or recover.
Complete command validation uses a name-indexed map and scales linearly with
group size.

## Safety policy

Each motor defines feedback health, command behavior, and a damping gain:

```yaml
health:
  safe_position_min: -12.57
  safe_position_max: 12.57
  temperature_warning_c: 60.0
  maximum_temperature_c: 80.0
  temperature_recovery_c: 75.0
  temperature_recovery_duration_ms: 2000
command_policy:
  maximum_position_step: 0.5
  limit: clamp          # clamp | reject | fault
  position_step: fault  # clamp | reject | fault
fault_damping_kd: 2.0
```

`Clamp`, `Reject`, and `Fault` are complete behaviors; invalid action/severity
combinations cannot be represented. Non-finite commands always fault. Thermal
faults latch at `maximum_temperature_c` and clear at the device layer only after
remaining below `temperature_recovery_c` for the configured duration. The group
fault remains latched until explicit recovery.

Group reaction configuration uses one default plus sparse typed overrides:

```yaml
fault_policy:
  default: damping
  overrides:
    temperature: disable
    reported_motor_fault: disable
    exit: disable
```

Supported responses are `damping` and `disable`. There is no no-op fault
response. Damping and stop are sent as one batch per bus. The controller submits
the safety batch to every affected bus before waiting for any completion. A
safety batch is a queue barrier: the transmit worker discards pending ordinary
controls before writing it. Safety uses a dedicated queue ahead of both
configuration and control traffic. A partial or late batch is treated as an
indeterminate transmission and reconnects the bus. Thus a queued motion command
cannot run after a stop or damping response.

The CAN control queue is deliberately shallow and non-blocking. Saturation
faults the affected group instead of accumulating old control ticks. Motor
feedback routing is immediate. The feedback watchdog scan period is derived
from the shortest configured feedback timeout and clamped to 1--20 ms, so it is
responsive without scanning once per received frame.

## Serial and EBIMU

Named serial transports own byte I/O and reconnect. Every configured IMU is an
active EBIMU sensor and exclusively owns one serial stream; omit an IMU from YAML
to disable it. The driver always configures and parses one complete schema:
quaternion, angular velocity, acceleration, and device timestamp. Missing
measurements are rejected instead of represented by zero-filled optional fields.

Malformed samples make the sensor degraded. Stale samples request a serial
transport reconnect. Samples are event telemetry only; the daemon does not keep
an unused latest-sample cache.

## gRPC and telemetry

The protobuf contract is
[hardware.proto](proto/bello/hardware/v1/hardware.proto). Protobuf enums carry
group state, device state, command results, fault causes, command-policy
checks/behaviors, and fault responses. IMU vectors use fixed-shape messages
rather than variable-length arrays. Unary lifecycle failures use gRPC status
codes; successful responses contain the resulting typed state.

The default listener is loopback-only. The service currently provides neither
TLS nor client authentication, so binding it to a non-loopback address emits an
explicit warning and should only be done behind a trusted network boundary.

`ListHardware` reports topology and current status. It does not mirror local
safety thresholds or policy YAML. High-rate motor and IMU telemetry contains
dynamic measurements plus logical identity; static addresses and status are
available through inventory and health events.

Telemetry has one bounded Tokio broadcast ring. gRPC subscribers stream directly
from it without a second per-client queue. Lag is reported as a health event.
Group timing telemetry is published at 25 Hz; motor and IMU samples are emitted
at their received rates. Optional Rerun consumes the same broadcast source.

SIGINT or SIGTERM first stops command and lease admission, then requests the
configured exit response from every group concurrently while CAN and motor
workers are still alive. gRPC receives a two-second drain window and the lease
watchdog joins. Transport and device workers are then destroyed while Rerun is
still consuming telemetry; finally Rerun drains pending events and receives its
own bounded join window. A panicked group worker enters an internal fault,
attempts the configured reaction, rejects pending work, and emits terminal
status instead of leaving callers waiting forever.

## Source layout

```text
src/
  app/                 assembly, ownership, shutdown
  api/grpc/            tonic service, conversions, inventory, streams
  config/              YAML model and cross-boundary topology resolution
  control/             group FSM, handle, worker, policy, fixed-rate loop
  domain/              typed IDs, commands, telemetry, errors
  hardware/can/        CAN handle/manager, spec, and session supervisor
  hardware/serial/     serial handle/manager, spec, and session supervisor
  hardware/robstride/  motor device controller and feedback dispatcher
  hardware/imu/        EBIMU codec and sensor worker
  hardware/lifecycle   shared device FSM, backoff, interruptible wait
```

The public Rust library surface is intentionally narrow: generated protobuf
types and the daemon `run`/`check` entrypoints. Runtime implementation modules
remain private. `robstride-protocol` is a separate dependency with `model`,
`frame`, and `codec` modules; its public API exposes semantic frame helpers, not
raw header bit manipulation.
