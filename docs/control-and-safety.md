# Control and safety

[Documentation index](README.md)

Motor groups are the unit of control ownership, synchronous command admission,
state transition, and fault reaction. A group is generic and may contain any
non-empty set of motors across one or more CAN buses. Device health is tracked
below it for each bus and motor; while active, the group polls that state and
converts an observed device failure into one latched group fault.

The core invariants are:

- only the current lease generation can change or command a group;
- every accepted command covers every group motor exactly once;
- command validity starts when the daemon begins submission and continues
  through lease admission, the mailbox, and group-thread processing;
- only `Running` transmits the staged command periodically;
- command timeout and loss of ownership move an enabled group to `Damped`;
- a required motor that is still unavailable when an enabled group polls it
  faults the group;
- a fault response is attempted for every group motor, including motors on
  other buses;
- automatic transport recovery never silently clears a group fault once
  latched.

The main implementation is
[`control/group.rs`](../components/bello-hardwared/src/control/group.rs), with
the pure state machine in
[`control/fsm.rs`](../components/bello-hardwared/src/control/fsm.rs).

## Leases and generations

A lease is exclusive write/control ownership of one motor group. At most one
lease can exist for a particular group, but different groups can be leased at
the same time, including by the same client. Leases are not scoped to a CAN bus,
so groups sharing a bus retain independent ownership. Read-only operations and
telemetry do not require a lease; there is no separate read-only lease type.

Each acquisition creates a random token and takes the next value from an
incrementing nonzero generation counter. The counter wraps while skipping zero.
The token authorizes an API request. The generation travels with
queued lifecycle work and commands so work admitted under an old owner cannot
execute after release, expiry, and reacquisition. Zero is reserved to mean no
active generation. See [`lease.rs`](../components/bello-hardwared/src/lease.rs)
and [`control/group_handle.rs`](../components/bello-hardwared/src/control/group_handle.rs).

The daemon protects lease verification, command enqueue, lifecycle enqueue,
release, and expiry notification with one short `lease_gate`. It releases that
gate before awaiting a group-thread reply. This makes admission atomic with
ownership changes without letting a slow hardware operation block all lease
bookkeeping. A Tokio watchdog checks expiry every 5 ms using skip-on-missed-tick
behavior. Requests also verify expiry directly, so correctness does not depend
on the watchdog running at an exact instant. See
[`app/daemon.rs`](../components/bello-hardwared/src/app/daemon.rs).

Release and expiry set the active generation to zero and wake the group. Any
queued operation rechecks its generation on the group thread; any queued command
does the same before staging. Lease loss clears all staged command state. It
moves `Running` to `Damped` with the configured damping command.
`Damped` stays damped and records `lease_loss` as its latest cause. `Idle` and
an existing `Fault` do not change state. Lease loss during enable stops the
motors and leaves the group `Idle`; a failed stop instead latches `Fault`.

## Motor-group FSM

The group FSM is deliberately small and has no `Degraded` state:

| Current state | Operation/event | Next state | Additional condition or action |
| --- | --- | --- | --- |
| `Idle` | enable | `Damped` | Configure and enable every motor, require fresh healthy feedback, then send the configured damping command |
| `Damped` | enable | `Damped` | Idempotently adopt the already-damped group under the current lease |
| `Damped` | start with initial command | `Running` | Atomically validate and install one complete, safe, unexpired command |
| `Running` | stop | `Damped` | Clear command state and send one damping safety batch |
| `Running` | command timeout | `Damped` | Clear command state and send one damping safety batch |
| `Damped`, `Running` | lease loss | `Damped` | Apply or retain damping and record cause `lease_loss` |
| `Idle`, `Damped`, `Running` | disable | `Idle` | Enabled states clear commands and send stop frames; `Idle` is idempotent |
| Any state except `Shutdown` | hard fault | `Fault` | Latch cause/detail and apply the configured fault response |
| `Fault` | recover | `Idle` | Request motor fault clearing, then require every motor and bus to be healthy |
| Any state | process shutdown | `Shutdown` | Apply the configured exit response first |

All other requested transitions are rejected without changing state. Every
lifecycle operation must carry a valid lease, including stop, disable, and
recover. `Fault` is latched even if the underlying transport later reconnects.
`Damped` has no escalation timer and persists until start, disable, a hard
fault, or shutdown. `Shutdown` is terminal for the worker. Transition legality is defined in
[`control/fsm.rs`](../components/bello-hardwared/src/control/fsm.rs); hardware
guards and reactions are in
[`control/group.rs`](../components/bello-hardwared/src/control/group.rs).

### Lifecycle operations

Lifecycle requests enter a bounded queue and execute serially on the group
thread. Their one-shot response is sent only after the hardware action or guard
has completed.

- **Enable** clears old command state, sends Robstride run-mode, active-report,
  and enable configuration, then waits for new healthy feedback. It retries the
  whole configured sequence, sends configured damping, and returns `Damped` only
  after damping succeeds. Shutdown or lease loss interrupts this wait and
  attempts to stop the motors.
- **Start** sends no special start frame. Its request carries the initial
  complete motor command and validity. The group validates and installs that
  command while `Damped`, then changes to `Running` in the same serialized
  operation. An invalid, incomplete, unsafe, or expired initial command leaves
  the group `Damped` unless the configured command policy itself requests a
  hard fault.
- **Stop** clears the staged command and sends the configured damping command
  once. It leaves the motors enabled and returns the group to `Damped`.
  Stop is idempotent in `Damped` and preserves the existing cause and detail.
  If fresh finite position feedback is unavailable, damping cannot be formed: the
  daemon sends motor stop frames and latches the group in `Fault`.
- **Disable** sends Robstride stop frames from active states and returns to
  `Idle`. It does not request fault clearing.
- **Recover** sends stop frames with the Robstride clear-fault flag, waits for
  all buses and motors to report online with fresh feedback and no motor fault,
  then returns to `Idle`.

The group worker and queue mechanics are in
[`control/group_worker.rs`](../components/bello-hardwared/src/control/group_worker.rs)
and [`control/group_handle.rs`](../components/bello-hardwared/src/control/group_handle.rs).

## Command mailbox and timing

The command stream is latest-value control, not a FIFO trajectory buffer. Each
group has one mailbox slot. A new submission replaces an older submission that
the group thread has not yet processed; the old request receives `Superseded`.
Once accepted, the command is stored per motor and triggers an immediate
control tick. That tick runs the same health, feedback, mechanical, and watchdog
checks as a periodic tick. With the default `command_transmission: on_update`,
it also transmits the command immediately. The `control_rate_hz` safety
schedule stays fixed and performs those checks without necessarily transmitting
a CAN command. Each immediate command defers periodic CAN refresh for one
`command_refresh_rate_hz` period plus one safety tick. If no newer command
arrives, hardwared then retransmits the retained command at the refresh rate.
The one-tick grace prevents the refresh deadline from racing a policy command
expected at the same nominal frequency. `Running` retains the command until a
newer complete batch replaces it, or until stop, disable, recover, lease loss,
fault, or shutdown clears it.
With `command_transmission: periodic`, the group encodes each accepted target
set once and publishes it into one latest-value slot per group per bus. The CAN
TX worker owns the `command_refresh_rate_hz` clock and reads that slot at each
send; it does not wait for the group to assemble and enqueue another repeat.
Updates replace pending targets without resetting the clock. Missed slots are
counted and skipped, with no catch-up replay. Each repeat retains the original
command deadline and lease generation. TX checks validity before a burst and
lease ownership, shutdown, and revocation before every frame. Stop, disable,
recover, lease loss, fault, and shutdown revoke the activation before submitting
ordered safety barriers; an old activation cannot resume after a later Start.

Periodic mode allows isolated missed slots. Four refresh periods without a
completed SocketCAN write burst revoke the targets and latch a transmission
fault through the group's configured safety response. At 300 Hz this is about
13.33 ms. The group safety tick also checks this watchdog, so fresh policy
updates cannot conceal a stalled TX worker. This watches host socket writes;
it is not a motor acknowledgement or a hard real-time guarantee. Skipped-slot
counts and lateness are logged when a slot is missed. Actual write errors or
expiry after a partial burst remain indeterminate transport failures.
TX checks the silence guard between frames and on completion, so a pause during
a burst cannot continue the remaining frames or hide the gap by completing late.

The default `on_update` path still has a separate delivery deadline of one
`control_rate_hz` period, measured from assembly of its first frame. At 300 Hz
this is 3.33 ms. Seven extended frames at 1 Mbps reserve 2.10 ms, leaving about
1.23 ms for assembly and enqueue scheduling. Enqueue expiry faults that group;
TX discards a batch expired before its first write. Periodic mode instead
reserves the same wire budget against the original target validity deadline,
without a new one-period assembly/handoff deadline on every repeat. A moderate
average bus load does not guarantee Linux scheduling deadlines.

`StartControl` installs the first command directly; stream commands are accepted
only in `Running`. Expired commands are rejected at admission. When a retained
target expires, TX stops repeating it and the group moves to `Damped` with the
command-timeout response. Repetition never extends validity in either mode.

The daemon records `submitted_at` before taking the lease gate. The effective
validity is the smaller of the batch's requested validity and the group's
configured command timeout; omission uses the configured timeout. Therefore
time spent waiting for lease verification, mailbox access, or the group thread
consumes the validity window. A batch already expired when processed is
rejected. Time in the network or tonic input stream before daemon submission is
not included. See [`app/daemon.rs`](../components/bello-hardwared/src/app/daemon.rs)
and [`control/group.rs`](../components/bello-hardwared/src/control/group.rs).

Command admission on the group thread verifies, in order:

1. the lease generation is still current and nonzero;
2. the group is `Damped` for the initial `StartControl` command or `Running`
   for a stream command;
3. the command deadline is still in the future;
4. the sequence is newer than the previous retained sequence for the current
   command state;
5. the batch names every group motor exactly once and no other motor;
6. every motor command passes numeric, effective-limit, and position-step
   policy;
7. direct and differential targets resolve inside the canonical URDF joint
   position/velocity envelope.

Validation is staged before mutating per-motor command state, so a partially
valid batch never replaces part of the current command. Sequence tracking is
cleared with the lease or group command state, so a new lease generation begins
a new sequence domain. Command types and typed rejection reasons live in
[`domain/command.rs`](../components/bello-hardwared/src/domain/command.rs) and
[`domain/error.rs`](../components/bello-hardwared/src/domain/error.rs).

### Control clock and CAN batching

Each group thread maintains its own fixed-period deadline from the configured
control rate. When late, it records positive scheduling jitter; if it has missed
more than one period, it schedules the next tick one full period from the
current time instead of executing a burst of catch-up ticks. Group telemetry is
published immediately on state/detail changes. Unchanged timing snapshots are
rate-limited to one every 40 ms and are evaluated on control ticks, so their
steady-state rate is capped at 25 Hz but quantized to group ticks and can be
lower even when the control rate is above 25 Hz.

`Damped` polls each motor's already-classified status and the exact coupled
differential feedback pose, but does not send periodic control frames.
`Running` performs the same availability and feedback-position checks, then
checks target expiry and periodic transport health. Target position and velocity
are validated against the mechanical envelope once during admission; unchanged
retained targets are not rebuilt and revalidated on each safety tick.

For `on_update`, `ControlBatcher` holds frames until it has seen every motor in
the logical group tick, then enqueues one atomic batch on each affected bus.
All buses are attempted before an enqueue error is returned. These batches
expire one control period after assembly begins. Expiry before any write
is discarded; write errors, partial expiry, or late completion cause reconnect.
Periodic mode publishes each new target set once and repeats it directly from
the TX worker's latest-value slot under the original target deadline. Normal
per-frame lease/revocation and stall-deadline checks use atomics and a captured
deadline; the slot is locked only when a stall must be latched, or for batch
scheduling/completion. Queue ordering is atomic on one bus; separate physical
CAN interfaces cannot be made simultaneous. See
[`hardware/robstride/controller.rs`](../components/bello-hardwared/src/hardware/robstride/controller.rs),
[`hardware/can/mod.rs`](../components/bello-hardwared/src/hardware/can/mod.rs),
and [`hardware/can/supervisor.rs`](../components/bello-hardwared/src/hardware/can/supervisor.rs).

## Validation boundaries

Static YAML validation is distributed to each owning subsystem and assembled
into one fail-fast topology resolution before hardware opens; the complete
model is documented in [Configuration](configuration.md#validation-boundaries).
Runtime command validation remains in the group and motor-policy boundary
because it depends on current ownership, state, command age, and feedback. See
[`control/command_policy.rs`](../components/bello-hardwared/src/control/command_policy.rs).

The scalar policy runs before the mechanical-envelope policy. Direct URDF
limits narrow the scalar ranges. Differential commands are then transformed as
a complete pair, so individually valid ankle-motor targets cannot violate
ankle roll/pitch position or velocity limits. A coupled target violation is
never independently clamped; it rejects the batch unless any affected motor
selects fault behavior. Motor feedback position checks allow the configured
feedback-position tolerance at a limit; it defaults to two protocol
quantization intervals, and differential feedback propagates that tolerance
through its transmission.

Parallel simple transmissions resolve each actuator onto the same joint.
Complete command batches must request matching joint positions and velocities;
contradictory targets are rejected through the coupled-limit policy. Feedback
checks each resolved position against the joint envelope and compares the
encoder span with `parallel_feedback_position_tolerance`. Disagreement faults
the shared group with cause `safe_position`. The torso profile selects disable
for that cause, affecting both hip motors. Reductions also transform velocity
and effort envelopes; the joint effort allowance is divided into equal
joint-torque contributions across its motors.

Out-of-range feedforward torque is rejected by the scalar command policy. Hardwared does not
estimate torque generated by position or velocity error and gains, and it does
not clamp or reject finite `kp` or `kd` values. Safety damping sends each
motor's configured `fault_damping_kd` without group-wide torque scaling. Its
measured position is only an inert target (`kp = 0`), so an already
out-of-range pose does not prevent damping. Robstride Type-1 serialization
still saturates gains to the physical model encoding range.

## Motor health and hysteresis

One shared Robstride feedback thread routes frames to motor controllers and
runs the health scan. Its scan period is one quarter of the smallest configured
group feedback timeout, clamped to 1--20 ms. A motor stores its latest sample
and receive timestamp; feedback is not averaged. See
[`hardware/robstride/manager.rs`](../components/bello-hardwared/src/hardware/robstride/manager.rs).

Motor health considers:

| Check | Behavior |
| --- | --- |
| CAN availability | An unavailable bus gives the motor a communication fault and reconnecting lifecycle state |
| Feedback freshness | Once operating feedback is expected, missing or stale feedback faults after the group feedback timeout |
| Reported fault bits | Any nonzero Robstride fault bits fault immediately |
| Temperature | Warning changes online detail; the maximum threshold latches an over-temperature fault |
| Safe feedback position | A measured position outside the configured health range faults |
| Mode status | While operating feedback is expected, a non-running mode faults |

Reported fault bits take precedence over thermal, safe-position, and mode
classification for the same sample. Communication and stale-feedback checks
occur outside that per-sample classification. The implementation is split
between
[`hardware/robstride/controller.rs`](../components/bello-hardwared/src/hardware/robstride/controller.rs)
and [`hardware/robstride/health.rs`](../components/bello-hardwared/src/hardware/robstride/health.rs).

Temperature uses a latch with hysteresis and a continuous cooldown requirement:

1. reaching or exceeding the maximum temperature latches the fault;
2. cooling merely below the maximum does not clear it;
3. the temperature must stay at or below the lower recovery threshold for the
   full recovery duration;
4. any sample above the recovery threshold resets the cooldown timer.

A temperature warning below the fault threshold leaves the motor lifecycle
`Online`; it changes health telemetry detail, while motor telemetry carries the
measured temperature. The group has no warning or degraded state of its own.

## Command safety policy

Every impedance field must be finite. Non-finite input always uses fault
behavior. Configured scalar command limits and maximum position-target step
each select one of two behaviors:

- `reject` publishes an event and rejects the batch without faulting the group;
- `fault` publishes an event, latches the corresponding group fault, applies
  its configured response, and rejects the batch.

The first position target in a new lease generation is measured from fresh
motor feedback, preventing an initial jump. Every later position step in that
generation is measured from the previously accepted target, so normal tracking
error does not become a command-step fault. Scalar limit checks cover position,
velocity, and feedforward torque. Gains must be finite but have no hardwared
command limit. No staged command is committed until every motor succeeds; a
rejecting or faulting issue exits admission early, and fault behavior
deliberately transitions the group immediately. See
[`control/command_policy.rs`](../components/bello-hardwared/src/control/command_policy.rs).

## Fault propagation and reactions

The group polls all member motor lifecycle states on every control tick while
`Damped` or `Running`. A motor still non-online at that poll becomes a group
fault. Feedback classification and watchdog scanning happen independently in
the shared Robstride worker, so a short device fault that clears on healthy
feedback before the group poll is not guaranteed to latch at group level. A
typed motor fault observed by the group maps to its matching cause; otherwise
the cause is `communication`. Direct control-transmission errors fault the group
without waiting for a later device-status poll.
Group telemetry records a cause for every state transition, not only faults.
The major cause families are:

| Family | Causes |
| --- | --- |
| Normal lifecycle | `enable`, `start`, `stop`, `command_timeout`, `disable`, `recover` |
| Command fault | `command_limit`, `command_position_step` |
| Motor/device | `safe_position`, `stale_feedback`, `temperature`, `reported_motor_fault`, `mode_status`, `communication` |
| Delivery/ownership | `transmission`, `lease_loss` |
| Process integrity | `exit`, `internal` |

The public `GroupCause` enum covers all of these transitions. Fault-response
configuration accepts only causes that can produce a hard fault; it does not
accept `start`, `stop`, `command_timeout`, or `lease_loss`. Detection and
mapping live in
[`control/group.rs`](../components/bello-hardwared/src/control/group.rs).
`exit` is a reaction selector for orderly shutdown: it does not latch `Fault`,
and the group proceeds to `Shutdown` after attempting the response.

### Device-to-group fault propagation

A CAN session failure rejects queued work, moves the bus into reconnect, and
retries with bounded exponential backoff. The Robstride health worker mirrors
the unavailable bus into each attached motor as a communication fault. On its
next tick, an active group containing one of those motors enters `Fault` if the
condition remains observable and applies its response. If the transport and
motor recover before that poll, the transient device fault may not become a
group fault. Once a group fault is latched, later CAN recovery and healthy
feedback do not clear it; recovery must be requested explicitly.

Serial and IMU lifecycles are independent of motor groups. Serial failure makes
the sensor wait for the transport, reconfigure against the new transport
generation, and resume telemetry. Malformed input degrades IMU health; stale
valid data also requests a serial reconnect. Neither faults a motor group. The
shared device lifecycle is implemented in
[`hardware/lifecycle.rs`](../components/bello-hardwared/src/hardware/lifecycle.rs);
the propagation paths are in
[`hardware/robstride/controller.rs`](../components/bello-hardwared/src/hardware/robstride/controller.rs)
and [`hardware/imu/mod.rs`](../components/bello-hardwared/src/hardware/imu/mod.rs).

### Configurable reaction

Each group has a default fault response and may override it by hard-fault
cause. The same response resolver is used for detected faults,
lifecycle-operation failures, worker panic, and process exit. Command timeout
and lease loss always use the dedicated transition to `Damped`; they are not
fault-policy override keys.

- **Damping** sends zero velocity/torque/`kp` and the latest measured position
  as an inert target. The configured `kd` values are scaled together as needed
  so motor torque and direct or differential URDF effort remain valid for every
  model-representable feedback velocity. Fresh finite position and velocity
  feedback are required. Missing, stale, or non-finite feedback, or a damping
  transmission failure, falls back to Robstride stop frames rather than an
  unvalidated damping command.
- **Disable** sends Robstride stop frames without requesting fault clearing and
  marks operating feedback as no longer expected.

The dedicated transitions to `Damped` use the same validated damping
construction. If it cannot be formed or transmitted, the daemon sends stop
frames and latches `Fault`; if that fallback also fails, the fault detail
records both failures. The group never reports `Damped` unless damping was
successfully established or was already retained there.

Both responses are grouped per bus into safety-barrier batches. The group
submits every bus before waiting for any completion, so one failed bus does not
prevent later buses from receiving the response. When the TX worker selects its
next batch, safety wins over queued configuration and control and purges queued
controls. It cannot interrupt a batch whose frames are already being written.
There is no physical atomicity across separate buses. See
[`control/safety.rs`](../components/bello-hardwared/src/control/safety.rs) and
[`hardware/can/supervisor.rs`](../components/bello-hardwared/src/hardware/can/supervisor.rs).

Entering a group fault first clears commands, latches and publishes cause and
detail, and only then performs the bounded safety response. If response delivery
fails, the group stays in `Fault` and appends the response failure to its detail.
A panic in a group worker is caught and routed through the `internal` response;
if that handler also panics, the snapshot is forced to `Fault` and the process
logs that safety state is unknown. See
[`control/group_worker.rs`](../components/bello-hardwared/src/control/group_worker.rs).

## Recovery

Transport and device recovery are automatic where possible; group recovery is
explicit. A client must hold the current group lease and request recovery from
`Fault`. The group then:

1. clears pending and staged commands;
2. sends stop frames with fault clearing enabled;
3. waits until every motor's bus and motor lifecycle are `Online`, no typed
   motor fault remains, and feedback is fresh;
4. returns to `Idle` on success.

Failure leaves the group in `Fault`, records cause `recover`, and applies that
cause's configured response. Success does not resume motion: the client must
enable again, submit a new complete command, and start control. This prevents a
reconnected device or stale pre-fault command from causing automatic movement.

During process shutdown, normal recovery is bypassed. Each group clears
commands, applies the `exit` response while CAN is still alive, transitions to
`Shutdown`, and is joined before transport teardown. See
[`app/daemon.rs`](../components/bello-hardwared/src/app/daemon.rs).
