# Architecture

[Documentation index](README.md)

`bello-hardwared` is one production process with explicit internal ownership
boundaries. It owns the physical transports, protocol devices, sensor drivers,
motor-group controllers, leases, telemetry fan-out, and gRPC server. The
public read-only gRPC stream supports external monitoring; higher-level motion
generation, kinematics, and robot behavior remain outside the daemon.

The process is monolithic in deployment, not in implementation: each layer has
one kind of policy and communicates through narrow handles and bounded queues.
The adjacent API and Robstride protocol libraries contain shared wire-format
knowledge. The high-level Rust and Python clients bind application-side leases
and command streams without entering the daemon, while the emulator and
end-to-end test components remain outside the hardware process.

`bello-visualizer` is also a separate client process. Kiss3d, egui, scene nodes,
and the camera stay on its main thread. A background Tokio runtime continuously
drains hardware and optional pose streams, publishes latest-wins snapshots at
60 Hz, and owns lease/command sessions. Rendering therefore cannot backpressure
telemetry or set the command cadence. External root tracking modifies display
transforms only; command conversion and safety use actuator/model coordinates.
The visualizer uses the maintained `bello-hardware-client` for validated
inventory, aggregated telemetry, reconnect boundaries, and control sessions.
It does not implement a parallel gRPC client. Its diagnostic publisher retains
latest motor/IMU/group/device values and a bounded low-rate event history;
ordinary sample traffic never enters the history.

## Boundaries

| Boundary | Owns | Does not own |
| --- | --- | --- |
| Application | Startup order, long-lived managers, shutdown coordination | Device protocols or control policy |
| gRPC API | Request conversion, stream lifetime, API error mapping | Lease truth or group state |
| Configuration | YAML decoding, local specification validation, cross-device topology validation | Runtime recovery |
| Control | Group state, command timing, motion safety policy, fault reaction | CAN framing or sockets |
| CAN and serial transports | Physical sessions, queues, deadlines, reconnect | Motor or IMU meaning |
| Robstride and EBIMU devices | Protocol configuration, feedback parsing, device health | Group ownership or client sessions |
| Telemetry | Timestamped in-process event fan-out | Durable storage or backpressure on producers |
| Hardware clients | Application-side channels, lease renewal, command sequencing, and telemetry access | Automatic motion recovery, server safety policy, or hardware ownership |

The assembly boundary is [`app/daemon.rs`](../components/bello-hardwared/src/app/daemon.rs).
The API receives an `Arc<HardwareDaemon>` and calls its methods rather than
reaching through to transport workers. Group control receives cloned
`MotorController` handles and does not construct Robstride frames. Sensor code
receives a `SerialHandle` and does not open a port itself.

## Startup assembly

Configuration resolution, listener binding, and worker creation are fail-fast
and follow dependency order. Hardware connectivity is supervised
asynchronously: an absent CAN interface or serial port normally leaves its
supervisor reconnecting rather than failing process assembly.

1. The CLI selects a configuration path and either validates it or starts the
   async runtime. See [`main.rs`](../components/bello-hardwared/src/main.rs).
2. Raw YAML is decoded, each subsystem converts its section into a validated
   specification, and `ResolvedTopology` verifies cross-references and physical
   ownership. No hardware is opened during `--check`. See
   [`config/resolved.rs`](../components/bello-hardwared/src/config/resolved.rs).
3. The gRPC socket is bound before transport and device workers start. This
   avoids discovering a port conflict after hardware ownership has begun.
   Signal handlers, telemetry, and shutdown watches are then prepared. See
   [`app/run.rs`](../components/bello-hardwared/src/app/run.rs).
4. `HardwareDaemon::start` constructs managers in dependency order:
   `BusManager`, `MotorManager`, `SerialManager`, IMU workers, and group workers.
   The lease manager is created after every configured group handle exists. See
   [`app/daemon.rs`](../components/bello-hardwared/src/app/daemon.rs).
5. The lease watchdog is spawned and the tonic server begins serving.

If assembly fails partway through, already-created managers are dropped and
join their workers. The production daemon has no in-process simulation branch;
hardware-free actuator testing uses the separate
[`robstride-emulator`](../components/robstride-emulator) process over vCAN.

## Ownership and execution model

The Tokio runtime handles network-facing and coordination tasks. Device and
fixed-period group loops use named operating-system threads; this isolates
blocking I/O but does not make them hard real-time.

| Execution context | Cardinality | Responsibility |
| --- | --- | --- |
| Tokio runtime | One runtime, multiple tasks | tonic server/RPC streams, lease watchdog, signals |
| Group worker | One thread per motor group | FSM, command mailbox, control clock, active-group motor status polling, safety reaction |
| CAN supervisor | One persistent thread per bus | Open/reopen SocketCAN sessions and own reconnect state |
| CAN session workers | One TX and one RX thread while a bus session is open | Prioritized batch transmission and frame reception |
| Robstride feedback worker | One shared thread | Route received frames by `(bus, motor id)` and scan motor health |
| Serial supervisor | One persistent thread per serial device | Open/reopen the port and own reconnect state |
| Serial session workers | One writer and one reader thread while a session is open | Deadline-bound writes and byte-chunk reception |
| IMU worker | One thread per IMU | Configure EBIMU, frame and parse samples, detect stale input |

The group topology is generic: a group can contain any non-empty set of motors,
can span multiple buses, and has no body-position semantics. A bus can serve
multiple groups. Configuration guarantees that each motor belongs to exactly
one group and each serial stream belongs to exactly one sensor.

Worker creation and joining live in
[`control/group_worker.rs`](../components/bello-hardwared/src/control/group_worker.rs),
[`hardware/can/mod.rs`](../components/bello-hardwared/src/hardware/can/mod.rs),
[`hardware/serial/mod.rs`](../components/bello-hardwared/src/hardware/serial/mod.rs),
[`hardware/robstride/manager.rs`](../components/bello-hardwared/src/hardware/robstride/manager.rs),
and [`hardware/imu/mod.rs`](../components/bello-hardwared/src/hardware/imu/mod.rs).

### Communication primitives

The code uses different channel semantics deliberately:

- Lifecycle operations use a bounded synchronous queue and a one-shot reply.
  They are serialized by the group thread.
- Commands use a one-slot, latest-value mailbox. Replacing an unprocessed
  command explicitly reports the older command as superseded.
- The one-slot group wake queue is only a notification; the actual state lives
  in atomics, the operation queue, and the command mailbox, so coalesced wakeups
  do not lose work.
- CAN uses separate bounded safety, configuration, and control queues per bus.
  Safety has highest receive priority, followed by configuration and control.
- Received CAN frames share one bounded queue consumed by the Robstride
  feedback dispatcher.
- Serial uses bounded outbound and received-chunk queues. The IMU consumes the
  chunks; the supervisor drains stale chunks when replacing a transport
  session.
- Telemetry uses a bounded Tokio broadcast ring. Each gRPC subscriber has an
  independent cursor; a slow subscriber can lose its own old events without
  blocking producers or other subscribers.

The concrete queue ownership is visible in
[`control/group_handle.rs`](../components/bello-hardwared/src/control/group_handle.rs),
[`hardware/can/mod.rs`](../components/bello-hardwared/src/hardware/can/mod.rs),
[`hardware/serial/mod.rs`](../components/bello-hardwared/src/hardware/serial/mod.rs),
and [`telemetry.rs`](../components/bello-hardwared/src/telemetry.rs).

## Control and data flow

### Commands to motors

```text
gRPC command stream
  -> HardwareDaemon verifies lease and captures its generation
  -> group latest-value mailbox
  -> group worker validates age, sequence, completeness, and motor policy
  -> group control tick reuses the latest accepted command
  -> Robstride ControlBatcher creates one batch per affected bus
  -> CAN control queue
  -> SocketCAN TX worker
```

The API acknowledgement means the group accepted and staged the batch; it does
not mean a CAN frame has already reached a motor. Periodic transmission begins
only in the group's `Running` state. The detailed timing and safety rules are in
[Control and safety](control-and-safety.md). The implementation path is
[`api/grpc/command_stream.rs`](../components/bello-hardwared/src/api/grpc/command_stream.rs),
[`app/daemon.rs`](../components/bello-hardwared/src/app/daemon.rs),
[`control/group.rs`](../components/bello-hardwared/src/control/group.rs), and
[`hardware/robstride/controller.rs`](../components/bello-hardwared/src/hardware/robstride/controller.rs).

CAN transmission is queue-driven, not a separate fixed-frequency CAN loop.
Group control ticks create the periodic control traffic. A complete logical
group tick is accumulated and submitted as one queue item per bus. Frames are
atomic with respect to queue ordering on one bus, but separate physical buses
cannot be made electrically atomic. The TX worker rejects work that cannot
finish before its freshness deadline. See
[`hardware/can/supervisor.rs`](../components/bello-hardwared/src/hardware/can/supervisor.rs).

### Motor feedback to state and telemetry

```text
SocketCAN RX worker
  -> timestamped raw frame queue
  -> shared Robstride feedback dispatcher
  -> controller selected by (bus, motor id)
  -> protocol decode and motor health classification
  -> per-motor latest feedback and lifecycle state
  -> motor telemetry broadcaster
  -> active group polls motor availability/fault on its next control tick
```

Feedback is retained per motor as the latest timestamped sample; it is not
averaged. Every decoded sample also produces a motor telemetry event. The group
reads device status rather than consuming the telemetry stream. See
[`hardware/robstride/manager.rs`](../components/bello-hardwared/src/hardware/robstride/manager.rs)
and [`hardware/robstride/controller.rs`](../components/bello-hardwared/src/hardware/robstride/controller.rs).

### Serial input to IMU telemetry

```text
serial reader thread
  -> timestamped byte chunks
  -> IMU worker line framing
  -> EBIMU parse, unit conversion, quaternion normalization
  -> per-IMU latest sample
  -> one telemetry event per valid sample
```

Malformed lines are discarded and move an online IMU to `Degraded`; a later
valid sample returns it to `Online`. Stale valid data causes the IMU to request
a serial reconnect. Samples from multiple configured IMUs remain independent
events and are neither averaged nor collapsed into a shared state. See
[`hardware/imu/mod.rs`](../components/bello-hardwared/src/hardware/imu/mod.rs)
and [`hardware/imu/ebimu.rs`](../components/bello-hardwared/src/hardware/imu/ebimu.rs).

### Telemetry consumers

Transports, motors, sensors, command policy, and groups publish timestamped
domain events to one broadcaster. gRPC applies per-subscriber filters and
reports subscriber-local lag as a health event. External diagnostic consumers
use `WatchTelemetry`. Control-loop consumers use client-paced `GetStateFrame`
requests, which sample the latest motor, IMU, and group caches into one complete
response with per-sample timing without draining the event broadcaster. The
daemon does not own visualization or durable storage. Telemetry capacity is the
shared raw-event ring-retention window, not a state-frame queue or sampling
rate. See
[`api/grpc/telemetry_stream.rs`](../components/bello-hardwared/src/api/grpc/telemetry_stream.rs),
[`api/grpc/state_frame.rs`](../components/bello-hardwared/src/api/grpc/state_frame.rs)
and [`telemetry.rs`](../components/bello-hardwared/src/telemetry.rs).

## Shared device lifecycle

CAN buses, serial transports, motors, and IMUs all expose the same device-state
vocabulary. The common recovery path is:

```text
Starting -> Connecting -> Online
                         |   |
                         |   v
                         | Degraded
                         |   |
                         v   v
                         Fault -> Reconnecting -> Connecting

any non-Shutdown state -------------------------> Shutdown
```

The full legal edges also allow startup or validation to enter `Fault`,
`Degraded` to return directly to `Online`, and reconnect attempts to return to
`Fault` before trying again.

Semantic operations construct legal multi-step paths. For example, a failure
from `Online` publishes `Degraded` and then `Fault`; reconnect then publishes
`Reconnecting`, `Connecting`, and eventually `Online`. A same-state detail
update does not increment `transition_count`. `Shutdown` is terminal and
idempotent. State changes and their ordered health events are committed under
one lifecycle lock, so another thread cannot observe a partial multi-step
operation.

This device lifecycle is transport/device health, not the motor-group control
FSM. Device reconnect can be automatic; a group fault is latched and requires
explicit recovery. See
[`hardware/lifecycle.rs`](../components/bello-hardwared/src/hardware/lifecycle.rs)
and [Control and safety](control-and-safety.md#device-to-group-fault-propagation).

## Shutdown

Shutdown is intentionally ordered so the path needed for the final safety
reaction stays alive:

1. SIGINT or SIGTERM requests the lease watchdog to stop.
2. Daemon admission closes. Every group clears pending commands, applies its
   configured `exit` response, enters `Shutdown`, publishes that final group
   event, and joins while CAN and motor workers remain available.
3. The server and telemetry-stream shutdown watch then close. Each telemetry
   stream drains already-queued selected events; tonic receives up to two
   seconds to complete this graceful drain before it is forced closed.
4. The watchdog is joined and dropping the daemon stops the IMU, Robstride,
   CAN, and serial workers. Their terminal transport/device states occur after
   gRPC closes and therefore are not guaranteed to reach remote observers.

The two-second deadline bounds the gRPC drain, not the preceding hardware exit
reaction or group-worker joins.

If the tonic server terminates independently, the same hardware safety steps
still run, but the server has already ended and cannot provide the graceful
telemetry drain.

`HardwareDaemon::shutdown` is idempotent. Group worker and manager `Drop`
implementations also request stop and join, providing cleanup on partial
startup and error paths. The ordering is implemented in
[`app/run.rs`](../components/bello-hardwared/src/app/run.rs),
[`app/daemon.rs`](../components/bello-hardwared/src/app/daemon.rs), and the
worker modules linked above.

## Source layout

```text
components/bello-hardwared/
  src/main.rs              CLI and Tokio runtime entry
  src/lib.rs               narrow run/check library surface
  src/app/                 assembly, ownership, signals, shutdown
  src/api/grpc/            tonic service, streams, conversion, inventory
  src/config/              raw YAML model and resolved topology
  src/control/             group FSM, worker, command policy, safety
  src/domain/              typed identifiers, commands, states, telemetry
  src/hardware/lifecycle.rs shared device lifecycle
  src/hardware/can/        SocketCAN queues, sessions, reconnect
  src/hardware/serial/     serial queues, sessions, reconnect
  src/hardware/robstride/  motor protocol boundary and feedback health
  src/hardware/imu/        EBIMU configuration, framing, parsing
  src/lease.rs             exclusive group lease store
  src/telemetry.rs         in-process event broadcaster

components/bello-hardware-client/
  src/client.rs            connection, inventory, and lease acquisition
  src/lease.rs             group-bound lifecycle and restricted renewal
  src/command.rs           command sequencing and acknowledgements
  src/telemetry.rs         filters and event stream

components/bello-hardware-python/
  src/bello_hardware/      blocking robot, control, and latest-state API
  tests/                   gRPC behavior and vCAN smoke driver

components/bello-visualizer/
  src/transmission.rs      URDF-derived forward/inverse mappings
  src/runtime.rs           telemetry, pose, leases, and command sessions
  src/scene.rs             main-thread URDF Kiss3d hierarchy

components/bello-robot-description/
  src/lib.rs               shared URDF model, visuals, transmissions, kinematics

components/bello-pose-api/
  proto/                   neutral external-pose streaming contract
```

Workspace-level supporting components are
[`bello-hardware-api`](../components/bello-hardware-api),
[`bello-hardware-client`](../components/bello-hardware-client),
[`bello-hardware-python`](../components/bello-hardware-python),
[`robstride-protocol`](../components/robstride-protocol),
[`robstride-emulator`](../components/robstride-emulator), and
[`bello-hardware-e2e`](../components/bello-hardware-e2e). Configuration fields
and validation limits are documented in [Configuration](configuration.md); the
wire API is documented in [API](api.md).
