# Hardware gRPC API

[Documentation index](README.md)

The authoritative contract is
[`hardware.proto`](../components/bello-hardware-api/proto/bello/hardware/v1/hardware.proto).
This guide describes the intended workflow and runtime semantics; generated
client types should be used instead of duplicating the schema by hand. Rust
applications should normally use the robot-oriented
[`bello-hardware-client`](../components/bello-hardware-client). Generated tonic
types and lease-oriented primitives remain available under its explicit `raw`
module for diagnostics and specialized integrations.

The protobuf package is `bello.hardware.v1` and the fully qualified service is
`bello.hardware.v1.HardwareService`. It uses proto3 conventions: enum value zero
is `UNSPECIFIED`, repeated fields default to empty, and telemetry payloads use a
`oneof`. Group/device states, command results, fault causes/responses, and
command-policy decisions are typed enums. IMU orientation and vectors are
fixed-shape messages rather than variable-length arrays.

## Maintained Rust client

The [`bello-hardware-client` README](../components/bello-hardware-client/README.md)
contains the complete application workflow. `Robot` loads and validates daemon
inventory, `MotorGroup` creates complete joint-oriented command frames,
`ControlSession` owns automatic lease renewal and synchronous acknowledgement
handling, and `RobotStates` aggregates event telemetry into typed latest-value
state.

These helpers do not duplicate daemon safety limits or hide motion ambiguity:
an active motion session is never transparently reacquired, reconnected, or
replayed. Explicit `shutdown` performs stop, disable, renewal-task termination,
and release; `Drop` cannot perform asynchronous network cleanup.

Specialized tooling can use `bello_hardware_client::raw`. Its `HardwareClient`,
`GroupLease`, `LeaseRenewer`, command stream, and telemetry filter preserve the
protocol-oriented behavior. `raw::proto` exposes the generated wire contract
and `HardwareClient::grpc_client` returns the tonic stub.

## Access model

`ListHardware` and `WatchTelemetry` are read-only and require no lease. Any
number of observers may use them concurrently, subject to server and network
capacity. There is no separate read-only lease.

Motion authority is an exclusive bearer lease for one named motor group.
Different groups may be leased concurrently, but one group has at most one
write lease. The token is required by every lifecycle operation and every
command batch. Treat it as a short-lived credential: it is not bound to a
transport connection, and releasing or allowing it to expire invalidates its
ownership generation. Lease behavior is implemented in
[`lease.rs`](../components/bello-hardwared/src/lease.rs) and admission is
serialized in
[`daemon.rs`](../components/bello-hardwared/src/app/daemon.rs).

| RPC | Lease required | Purpose |
| --- | --- | --- |
| `ListHardware` | No | Read configured topology and current component/group states. |
| `WatchTelemetry` | No | Subscribe to selected dynamic events. |
| `AcquireLease` | No | Obtain exclusive write ownership for one group. |
| `RenewLease` | Token | Extend that token's expiry. |
| `ReleaseLease` | Token | Invalidate ownership and wait for the group reaction. |
| `EnableGroup`, `StartControl`, `StopControl`, `DisableGroup`, `RecoverGroup` | Group token | Drive the motor-group lifecycle. |
| `StreamCommands` | Token in every request | Submit complete impedance-command snapshots and receive typed results. |

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
   healthy feedback before returning `GROUP_STATE_ENABLED`.
6. Open `StreamCommands` and submit a complete batch containing every motor in
   the group. Wait for `COMMAND_RESULT_ACCEPTED`.
7. Call `StartControl`. Start is accepted only from `Enabled` with a fresh
   command from the current lease generation.
8. Continue sending complete batches with increasing sequence numbers.
9. Finish with `StopControl` (`Running` to `Enabled`) and `DisableGroup`
   (`Enabled` to `Idle`), then `ReleaseLease`.

If a lease expires during enable or recovery, the generation is invalidated and
the group worker interrupts the operation rather than allowing the former owner
to complete it.

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
batches are not silently accepted. Every well-formed request receives a
`StreamCommandsResponse` with the same group/sequence, a typed `CommandResult`,
human-readable `detail`, and the observed `GroupState`. A domain rejection does
not close the stream; malformed input such as an invalid token encoding is a
gRPC stream error. The server uses a latest-wins command mailbox, so clients
must use responses and telemetry rather than assuming every submitted snapshot
was applied. See
[`command_stream.rs`](../components/bello-hardwared/src/api/grpc/command_stream.rs)
and [`conversion.rs`](../components/bello-hardwared/src/api/grpc/conversion.rs).

## Inventory and telemetry

`ListHardware` returns buses, serial devices, IMUs, motors, and groups with
typed current states. It is an inventory/status snapshot, not a copy of the
local safety-policy YAML.

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

Each response carries daemon-relative `monotonic_ns` and exactly one payload:
motor feedback, fixed-shape IMU data, group state/timing, component health, or a
command-policy decision. Samples are neither averaged nor replaced with a
single latest state: every selected broadcast event is forwarded while the
subscriber keeps pace. Motor and IMU events follow their received rates. Group
state/detail changes publish immediately; unchanged timing snapshots are
rate-limited to 25 Hz and can be slower when the group control rate is below
25 Hz.

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

The optional [`bello-rerun`](../components/bello-rerun) process is a normal
read-only client of this API. It requests all telemetry classes, has no private
in-process access to daemon state, and decouples its gRPC task from Rerun output
through a separate bounded writer queue. Observer-local queue drops are marked
in the Rerun recording rather than reported as `grpc_telemetry` events.

## Security boundary

The server provides no TLS, client authentication, authorization policy, or
token confidentiality. Its default listener is loopback-only; a non-loopback
binding logs a warning. Run plaintext gRPC only inside a trusted host/network
boundary, or place an authenticated encrypted proxy in front of it and restrict
direct access with host firewall rules. Anyone who can reach the endpoint can
read topology and telemetry, acquire an unleased group, and use a captured
lease token. The lease mechanism coordinates motion ownership; it is not a
security system.

For startup, hardware, diagnostics, and shutdown procedures, see
[`operations.md`](operations.md). For build and review conventions, see
[`development.md`](development.md).
