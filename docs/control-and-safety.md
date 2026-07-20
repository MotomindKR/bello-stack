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
- losing ownership faults an active group; a required motor that is still
  unavailable when the active group polls it also faults the group;
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
faults a group in `Enabled` or `Running`, applies the configured `lease_loss`
response, and is otherwise acknowledged without changing `Idle` or an existing
`Fault`.

## Motor-group FSM

The group FSM is deliberately small and has no `Degraded` state:

| Current state | Operation/event | Next state | Additional condition or action |
| --- | --- | --- | --- |
| `Idle` | enable | `Enabled` | Configure and enable every motor; wait for fresh healthy feedback |
| `Enabled` | start | `Running` | A complete, safe, unexpired command from the current generation must already be staged |
| `Running` | stop | `Enabled` | Clear command state and send one damping safety batch |
| `Idle`, `Enabled`, `Running` | disable | `Idle` | Active states clear commands and send stop frames; `Idle` is idempotent |
| `Enabled`, `Running` | lease loss | `Fault` | Apply the configured lease-loss response |
| Any state except `Shutdown` | detected fault | `Fault` | Latch cause/detail and apply the configured response |
| `Fault` | recover | `Idle` | Request motor fault clearing, then require every motor and bus to be healthy |
| Any state | process shutdown | `Shutdown` | Apply the configured exit response first |

All other requested transitions are rejected without changing state. Every
lifecycle operation must carry a valid lease, including stop, disable, and
recover. `Fault` is latched even if the underlying transport later reconnects.
`Shutdown` is terminal for the worker. Transition legality is defined in
[`control/fsm.rs`](../components/bello-hardwared/src/control/fsm.rs); hardware
guards and reactions are in
[`control/group.rs`](../components/bello-hardwared/src/control/group.rs).

### Lifecycle operations

Lifecycle requests enter a bounded queue and execute serially on the group
thread. Their one-shot response is sent only after the hardware action or guard
has completed.

- **Enable** clears old command state, sends Robstride run-mode, active-report,
  and enable configuration, then waits for new healthy feedback. It retries the
  whole configured sequence. Shutdown or lease loss interrupts this wait and
  attempts to stop the motors.
- **Start** sends no special start frame. It changes the group to `Running` only
  when the current lease generation has already submitted a complete command
  whose deadline is still in the future. A missing or expired initial command
  from an otherwise valid `Enabled` state faults the group with cause `start`.
- **Stop** clears the staged command and sends a damping command once. It leaves
  the motors enabled and returns the group to `Enabled`.
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
Once accepted, the command is stored per motor. `Running` reuses it on every
control tick until a newer complete batch replaces it, or until enable, stop,
active-state disable, recover, lease loss, fault, or shutdown clears it. Start
retains the staged command. A command already expired when a running tick
evaluates it is not submitted and faults the group; after tick admission, the
resulting CAN batch is governed by its own one-period freshness deadline.

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
2. the group is `Enabled` or `Running`;
3. the command deadline is still in the future;
4. the sequence is newer than the previous retained sequence for the current
   command state;
5. the batch names every group motor exactly once and no other motor;
6. every motor command passes numeric, configured-limit, and position-step
   policy.

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

`Enabled` polls each motor's already-classified status but does not send
periodic control frames. `Running` first performs the same availability poll and
checks the command watchdog, then submits one control frame per motor.
`ControlBatcher` holds those frames until it has seen every motor in the logical
group tick, then enqueues one atomic batch on each affected bus. All buses are
attempted before an enqueue error is returned.

The CAN TX workers are event-driven rather than frequency-driven. A control
batch expires after one group control period measured from collection of its
first motor frame. If the bus can determine before writing that the complete
batch will not fit, it discards the batch without reconnecting. A write error,
expiry after partial transmission, or completion after the deadline is
indeterminate and causes transport failure and reconnect. Queue ordering is
atomic on one bus; no software can make separate physical CAN interfaces
simultaneous. See
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
behavior. Configured command limits and maximum position-target step each
select one of three behaviors:

- `clamp` modifies the offending value, publishes a command-policy event, and
  continues validation; the batch is admitted if no other issue rejects or
  faults;
- `reject` publishes an event and rejects the batch without faulting the group;
- `fault` publishes an event, latches the corresponding group fault, applies
  its configured response, and rejects the batch.

The first position target in a new lease generation is measured from fresh
motor feedback, preventing an initial jump. Every later position step in that
generation is measured from the previously accepted target, so normal tracking
error does not become a command-step fault. Limit checks cover position,
velocity, torque, `kp`, and `kd`. No
staged command is committed until every motor succeeds; a rejecting or faulting
issue exits admission early, and fault behavior deliberately transitions the
group immediately. See
[`control/command_policy.rs`](../components/bello-hardwared/src/control/command_policy.rs).

## Fault propagation and reactions

The group polls all member motor lifecycle states on every control tick while
`Enabled` or `Running`. A motor still non-online at that poll becomes a group
fault. Feedback classification and watchdog scanning happen independently in
the shared Robstride worker, so a short device fault that clears on healthy
feedback before the group poll is not guaranteed to latch at group level. A
typed motor fault observed by the group maps to its matching cause; otherwise
the cause is `communication`. Direct control-transmission errors fault the group
without waiting for a later device-status poll.
The major cause families are:

| Family | Causes |
| --- | --- |
| Command | `command_limit`, `command_position_step`, `command_timeout` |
| Motor/device | `safe_position`, `stale_feedback`, `temperature`, `reported_motor_fault`, `mode_status`, `communication` |
| Lifecycle operation | `enable`, `start`, `stop`, `disable`, `recover` |
| Delivery/ownership | `transmission`, `lease_loss` |
| Process integrity | `exit`, `internal` |

The complete enum and configurable selection live in
[`config/model.rs`](../components/bello-hardwared/src/config/model.rs); detection
and mapping live in
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

Each group has a default fault response and may override it by cause. The same
response resolver is used for detected faults, lifecycle-operation failures,
lease loss, worker panic, and process exit.

- **Damping** sends zero velocity/torque/`kp`, the configured damping `kd`, and
  the latest measured position as target. If no feedback exists, the target
  falls back to zero.
- **Disable** sends Robstride stop frames without requesting fault clearing and
  marks operating feedback as no longer expected.

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
