# Hardware gRPC API

See [hardware.proto](../components/bello-hardware-api/proto/bello/hardware/v1/hardware.proto)
for the wire contract. The [Rust](../components/bello-hardware-client/README.md)
and [Python](../components/bello-hardware-python/README.md) client guides show how to open
a control session, renew its lease, and shut it down.

## Access model

`ListHardware`, `WatchTelemetry`, and `GetStateFrame` are read-only and
require no lease. Any number of observers may use them concurrently, subject
to server and network capacity. There is no separate read-only lease.

Motion authority is an exclusive bearer lease for one named motor group.
Different groups may be leased concurrently, but one group has at most one
write lease. The token is required by every lifecycle operation and every
command batch. Treat it as a short-lived credential: it is not bound to a
transport connection, and releasing or allowing it to expire invalidates its
ownership generation. Lease behavior is implemented in
[`lease.rs`](../components/bello-hardwared/src/lease.rs) and admission is
serialized in
[`daemon.rs`](../components/bello-hardwared/src/app/daemon.rs).

## Normal control workflow

1. Call `ListHardware` and select a configured group and its motor names.
2. Open `WatchTelemetry` before motion so group faults and health changes are
   not missed.
3. Call `AcquireLease` with a non-empty `client_name`. A zero duration selects
   the configured default; a nonzero request is clamped to 20 ms through the
   server maximum.
4. Start renewing the lease immediately and continue until release. Renew well
   before expiry, including while waiting for lifecycle RPCs: enable and
   recovery can block for their configured retry/timeout windows.
5. Call `EnableGroup`. The server configures every actuator and waits for fresh,
   healthy feedback, applies configured damping, and returns
   `GROUP_STATE_DAMPED`.
6. Call `StartControl` with `valid_for_ms` and one complete `MotorCommand` for
   every motor. The server atomically validates and installs this initial
   command while moving `Damped` to `Running`.
7. Open `StreamCommands` and continue sending complete batches with increasing
   sequence numbers. Stream commands are accepted only in `Running`.
8. Finish with `StopControl` (`Running` to `Damped`). Either resume with another
   atomic `StartControl`, or call `DisableGroup` (`Damped` to `Idle`) and then
   `ReleaseLease`.

If a lease expires during enable or recovery, the generation is invalidated and
the group worker interrupts the operation rather than allowing the former owner
to complete it. Release or expiry while `Running` applies damping and leaves the
group in `Damped` with cause `lease_loss`; lease loss while already `Damped`
retains damping.

If the group enters `Fault`, it remains latched. Correct the underlying device
condition, then call `RecoverGroup`; successful recovery returns the group to
`Idle`, after which enable/start must be performed again. The exact state
machine and safety reactions are documented in
[`control-and-safety.md`](control-and-safety.md).

Lifecycle transition errors are returned as gRPC statuses rather than encoded
as successful response bodies. Common mappings include `INVALID_ARGUMENT` for
malformed tokens/selectors, `NOT_FOUND` for an unknown group,
`PERMISSION_DENIED` for an expired or non-owning token,
`FAILED_PRECONDITION` for an invalid state transition,
`RESOURCE_EXHAUSTED` for a full lifecycle queue, and `UNAVAILABLE` when lease
acquisition/renewal is closed or a controller queue has stopped. The current
untyped lifecycle-admission error after daemon shutdown maps to `INTERNAL`. See
[`service.rs`](../components/bello-hardwared/src/api/grpc/service.rs) for the
exact mapping.

## Command stream

`StreamCommands` is bidirectional. Each `StreamCommandsRequest` contains:

- `group` and `lease_token`;
- a monotonically increasing `sequence` while the current command state is
  retained; ownership and lifecycle operations that clear command state also
  reset the server-side sequence comparison;
- optional `valid_for_ms`, measured from server receipt; zero uses the group's
  command timeout and nonzero is capped by it;
- exactly one `MotorCommand` for every motor in the group.

Commands are impedance targets: position, velocity, torque feed-forward, `kp`,
and `kd`. Missing, duplicate, unknown, stale, expired, unsafe, or superseded
batches are not silently accepted, and stream commands are legal only while the
group is `Running`. Every well-formed request receives a
`StreamCommandsResponse` with the same group/sequence, a typed `CommandResult`,
human-readable `detail`, and the observed `GroupState`. Acceptance triggers an
immediate checked control tick and defers periodic CAN refresh; the response
still acknowledges daemon admission rather than physical receipt by a motor. A
domain rejection does not close the stream; malformed input such as an
invalid token encoding is a gRPC stream error. The server uses a latest-wins
command mailbox, so clients must use responses and telemetry rather than
assuming every submitted snapshot was applied. See
[`command_stream.rs`](../components/bello-hardwared/src/api/grpc/command_stream.rs)
and [`conversion.rs`](../components/bello-hardwared/src/api/grpc/conversion.rs).

## Inventory and telemetry

`ListHardware` returns buses, serial devices, IMUs, motors, and groups with
typed current states. IMU inventory includes the configured source and output
URDF frames. It is an inventory/status snapshot, not a copy of the local
safety-policy YAML. Each bus includes a `transport` label such as
`socketcan:can0` or `gsusb:SERIAL:0`.

`WatchTelemetryRequest` filters one shared event stream:

- an empty `groups` list selects every group; otherwise it limits motor, group,
  and command-policy events to the named groups;
- group events are included for selected groups regardless of
  `include_motors`;
- `include_motors`, `include_imu`, `include_health`, and
  `include_command_policy_events` enable those payload classes;
- an empty `imus` list selects every configured IMU when `include_imu` is true;
  a non-empty `imus` list is invalid when `include_imu` is false;
- empty or unknown group/IMU selectors fail the RPC with `INVALID_ARGUMENT`.

Samples are neither averaged nor replaced with a
single latest state: every selected broadcast event is forwarded while the
subscriber keeps pace. Motor and IMU events follow their received rates. Group
state/cause/detail changes publish immediately. `cause` describes normal
transitions such as `enable`, `stop`, `command_timeout`, and `lease_loss` as
well as hard faults. Unchanged timing snapshots are rate-limited to 25 Hz and
can be slower when the group control rate is below 25 Hz.

`monotonic_ns` is response publication time. Motor `feedback_age_ms` and IMU
`sample_age_ms` are measured relative to it and include server queueing once.
Subtract the age from this timestamp to estimate sensor acquisition time;
client transport delay still contributes to the sample's age at receipt.
Sub-millisecond server queueing is rounded up, so serialization cannot renew a sample.

State frames and telemetry use the same [motor and IMU filters](configuration.md#runtime).
Motor health checks use raw feedback.

The telemetry ring is bounded by `runtime.telemetry_capacity`. A slow
subscriber loses overwritten events rather than blocking hardware control. On
lag, the stream forcibly emits a `HealthTelemetry` event for component
`grpc_telemetry`, even if `include_health` is false; `dropped_events` reports the
loss. The stream then resumes with available events. Server shutdown or closure
ends the stream. During graceful daemon shutdown, group workers publish their
final `Shutdown` events before the telemetry watch closes. The server then
drains already-queued events selected by each subscriber and normally ends the
stream with EOF. This drain shares the server's two-second shutdown deadline,
so a forced deadline expiry can truncate the tail. Transport/device terminal
events happen later and are not guaranteed over gRPC. These semantics live in
[`telemetry_stream.rs`](../components/bello-hardwared/src/api/grpc/telemetry_stream.rs)
and [`telemetry.rs`](../components/bello-hardwared/src/telemetry.rs).

`GetStateFrame` is the client-paced control-loop interface. Its request selects
groups and IMUs using the same empty-means-all convention. The daemon samples
its latest producer caches directly. Until every selected motor, IMU, and group
has a sample, it returns `FAILED_PRECONDITION`; the Python state reader retries
that startup condition within the caller's timeout. Each successful response
contains the complete selected set, daemon-relative capture and per-sample
timestamps, the oldest sample age, and the span between the oldest and newest
samples.

Every successful request assembles one response from the current caches, so
there is no server or client frame stream that can accumulate behind a slow
Python consumer. Clients conservatively add the complete read duration,
including any startup retries, to the daemon-reported sample ages. Control
clients should reject stale group samples as well as stale motor and IMU
samples, and bound `sample_span_ns` so a response assembled from widely
separated producer cycles cannot be treated as coherent. `WatchTelemetry`
remains the event and diagnostics API when individual changes, health events,
command-policy decisions, or drop counts are required. The state-frame
semantics live in
[`state_frame.rs`](../components/bello-hardwared/src/api/grpc/state_frame.rs).

## Security boundary

The daemon can listen on plaintext TCP, TLS TCP, mutually authenticated TLS, or
a Unix-domain socket. Plaintext TCP defaults to loopback. A non-loopback
plaintext binding warns that it has neither encryption nor authentication;
server-only TLS on a non-loopback address warns that client authentication is
absent.

TLS protects token confidentiality in transit and authenticates the robot.
Configuring a client CA additionally rejects connections without a client
certificate signed by that CA. A Unix socket relies on filesystem ownership and
permissions for same-host access control. Certificate identities and Unix users
are not mapped to per-RPC roles: every connected client can read topology and
telemetry and can attempt to acquire an available group. Use separate network
or process boundaries if read-only and control clients require different
permissions. The lease mechanism coordinates motion ownership; it is not an
authorization system.

Transport configuration, certificate requirements, and Unix socket behavior
are documented in [`configuration.md`](configuration.md#grpc).

For startup, hardware, diagnostics, and shutdown procedures, see
[`operations.md`](operations.md). For build and review conventions, see
[`development.md`](development.md).

## External pose stream

[`pose.proto`](../components/bello-pose-api/proto/bello/pose/v1/pose.proto)
defines `WatchPose`. The transform is `reference_from_tracked`; orientation
uses normalized `xyzw` quaternions and angular velocity uses tracked-frame axes.

[`bello-state-estimatord`](../components/bello-state-estimatord) is the
maintained local provider for Bello and Bello Mini. It consumes the high-level
`bello-hardware-client`, uses the EBIMU quaternion for orientation, and applies
encoder/URDF stationary-foot corrections to IMU-propagated translation. The
configured `imu_frame` must be rigidly attached below `tracked_frame`; the
estimator runs at the IMU origin and uses that URDF transform to publish the
requested tracked-frame pose and twist.
