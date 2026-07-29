# Hardware transports and devices

[Documentation index](README.md)

`bello-hardwared` separates transport ownership from device protocols:

```text
SocketCAN supervisor -> Robstride motor controller -> motor-group controller
serial supervisor    -> EBIMU sensor worker         -> telemetry
```

Each transport and device publishes a common lifecycle status, but recovery is
owned at the layer that has enough context to perform it. CAN and serial
supervisors reopen operating-system resources. Motor and IMU workers validate
protocol data. A motor group owns coordinated control, fault reactions, and
manual recovery.

The common transport lifecycle and reconnect policy are in
[`hardware/lifecycle.rs`](../components/bello-hardwared/src/hardware/lifecycle.rs).
Typical states are `Starting`, `Connecting`, `Online`, `Degraded`, `Fault`,
`Reconnecting`, and `Shutdown`. These device states are distinct from the motor
group FSM states `Idle`, `Damped`, `Running`, `Fault`, and `Shutdown` in
[`control/fsm.rs`](../components/bello-hardwared/src/control/fsm.rs).

## SocketCAN transport

### Responsibilities

For every configured logical bus, the CAN manager creates one supervisor and
three bounded transmit queues. Each online session opens two sockets on the
already configured interface: a writer with a 20 ms write timeout and a reader
with a 100 ms read timeout. The daemon does not create the interface, bring it
up, or set its bitrate. See
[`hardware/can/mod.rs`](../components/bello-hardwared/src/hardware/can/mod.rs)
and
[`hardware/can/supervisor.rs`](../components/bello-hardwared/src/hardware/can/supervisor.rs).

The session then owns two worker threads:

- RX blocks on SocketCAN, accepts only eight-byte frames, timestamps them, and
  publishes them to one process-wide bounded frame queue of 8192 entries.
- TX is event-driven. It wakes when a batch arrives; it does not spin at a
  separately configured CAN frequency.

The supervisor checks the workers' failure channel every 20 ms. A reported
socket error, receive-queue overflow, disconnected failure channel, or
indeterminate transmission ends the entire session and schedules a reconnect.

### Queue ordering and deadlines

The three transmit classes are, in strict priority order:

1. safety barriers
2. configuration batches
3. control batches

The TX worker uses biased selection, so pending safety and configuration work
is chosen ahead of normal control. A safety barrier additionally purges every
queued control batch before its safety frames are written. A configuration
batch preserves queued control traffic.

Control queue capacity is the number of motor groups using that bus, with a
minimum of one. A group contributes at most one atomic batch per bus per
control tick. Configuration and safety queues each have capacity
`max(control_capacity, 4)`. Producers use bounded, nonblocking submission so a
stalled transport becomes an explicit error instead of unbounded latency.

Every control batch has an absolute freshness deadline: collection time of its
first motor frame plus one group period. Submission and transmission both check
that the full batch still fits its estimated wire-time budget. A batch that
expires before its first frame is discarded. Expiry after any frame was written,
a write error, or completion after the deadline makes the outcome indeterminate
and forces reconnection.

Configuration and safety batches use an absolute delivery deadline derived
from estimated wire time, a two-times transmission margin, a 20 ms scheduling
margin, and a 50 ms queue margin. Their callers receive an explicit completion
result and wait for success or failure. The estimates are in
[`hardware/can/spec.rs`](../components/bello-hardwared/src/hardware/can/spec.rs).

Safety reactions spanning multiple buses submit all bus batches before waiting
for any completion. Thus one failed interface does not prevent submission to
later interfaces, although separate physical buses can never be transactionally
atomic. See
[`control/safety.rs`](../components/bello-hardwared/src/control/safety.rs).

### CAN reconnect behavior

On session failure, the supervisor:

1. rejects queued configuration and safety work with a transmission error;
2. discards queued controls rather than replaying stale commands;
3. marks the bus `Reconnecting`;
4. waits using bounded exponential backoff; and
5. reopens both sockets and creates fresh RX/TX workers.

The bus becomes `Online` after the sockets open. It does not claim that every
motor is healthy. Shutdown rejects remaining work and moves the lifecycle to
`Shutdown`.

## Robstride motor devices

### Startup is not enable

At daemon startup, the motor manager constructs one logical
`MotorController` for each configured motor, binds it to its bus, and starts a
single `robstride-rx` feedback dispatcher. It does **not** send enable or motor
configuration commands. The device lifecycle waits for valid feedback, which
may be received before the group is enabled. See
[`hardware/robstride/manager.rs`](../components/bello-hardwared/src/hardware/robstride/manager.rs).

An `EnableGroup` operation performs the motor-side setup. For each motor, the
controller sends one configuration-priority CAN batch in this order:

1. write parameter `0x7005` to value `0` (impedance run mode);
2. enable active feedback reports; and
3. send the motor enable frame.

The motor then clears old cached feedback, arms its operating-feedback
expectation, and waits for fresh healthy feedback. The group retries up to
`enable_retries`, with each attempt bounded by `enable_timeout_ms`. The frame
construction and feedback boundary are in
[`hardware/robstride/controller.rs`](../components/bello-hardwared/src/hardware/robstride/controller.rs);
the group procedure is in
[`control/group.rs`](../components/bello-hardwared/src/control/group.rs).

Hardwared does not configure the motor ID, CAN bitrate, encoder zero,
calibration, stored gains, firmware limits, or a motor-side communication
watchdog. Those values must already be correct on the device. Limits, command
policy, feedback freshness, and fault reactions in the daemon are host-side
behavior. Recovery sends stop/clear-fault frames; the full run-mode/reporting
sequence is sent again on the next enable.

### Control batches

The motor-group worker runs at `control_rate_hz`. On each tick it calls every
motor controller once. A shared `ControlBatcher` holds those frames until all
motors in the group have contributed, partitions them by CAN bus, and enqueues
one control batch per bus with the same logical tick deadline. This makes the
queue item atomic on each bus and supports groups that span buses. If the same
motor contributes twice before the batch completes, the incomplete batch is
discarded and reported as interrupted.

The configured rate drives control production; CAN TX itself is queue-driven.
CAN RX is also event-driven by incoming frames. Therefore neither has an
independent fixed polling frequency.

### Feedback routing and watchdogs

The shared dispatcher extracts the motor ID from each protocol feedback frame,
combines it with the receiving bus, checks that the frame targets the configured
host ID, and routes it by `(bus, motor_id)`. Valid feedback updates the motor's
latest sample and emits one telemetry event; samples are not averaged.

The feedback health scan runs at one quarter of the smallest configured group
feedback timeout, clamped to `1..=20` ms. A motor's stale-feedback watchdog is
armed only after an enable attempt establishes that operating feedback is
expected, and only while its CAN bus is online. This prevents an idle motor or
an already-lost transport from generating a misleading stale-sample fault.

Valid feedback is checked for:

- nonzero motor-reported fault bits;
- temperature warning and over-temperature fault;
- position outside the configured safe feedback range;
- non-operating mode status while operating feedback is expected; and
- staleness relative to `feedback_timeout_ms`.

Temperature warning is a lifecycle detail, not a fault. At the maximum
threshold the temperature fault latches. It clears at the motor-device layer
only after temperature remains at or below the recovery threshold for the
configured continuous recovery duration. Going above the recovery threshold
restarts that timer. See
[`hardware/robstride/health.rs`](../components/bello-hardwared/src/hardware/robstride/health.rs).

If the CAN lifecycle leaves `Online`, each bound motor records a
`communication` fault and enters reconnecting behavior. Once the transport is
restored, the motor waits for healthy feedback before returning online. Device
recovery does not silently clear the owning group's fault latch.

### Fault propagation and reaction

The failure path is:

```text
socket/queue/deadline failure
  -> CAN lifecycle unavailable
  -> motor communication fault
  -> active owning group observes the persistent fault on a control tick
  -> configured group safety reaction
```

Motor health faults follow the same last two steps when they remain present for
a `Damped` or `Running` group to observe. A transient device fault may clear
on healthy feedback before that group poll and therefore may not latch at group
level. Control enqueue failures and failed priority batches can fault the group
directly as transmission or lifecycle-operation failures.

The group maps the cause through its `fault_policy` and attempts either damping
or disable as a safety barrier. That reaction, its cross-bus delivery rules,
the latched group FSM, and explicit recovery are the control layer's policy and
are documented in [Control and safety](control-and-safety.md#fault-propagation-and-reactions).

## Serial transport

### Responsibilities and queues

Every configured serial transport has one supervisor. An online session opens
the port with the configured baud rate and timeout, clones it into reader and
writer handles, and runs separate RX/TX threads. See
[`hardware/serial/mod.rs`](../components/bello-hardwared/src/hardware/serial/mod.rs)
and
[`hardware/serial/supervisor.rs`](../components/bello-hardwared/src/hardware/serial/supervisor.rs).

- The outbound command queue holds 16 requests. Each request has an absolute
  deadline and a one-shot completion result.
- The inbound queue holds 1024 chunks. Reads use a 1024-byte buffer and retain
  the time the chunk arrived.
- The sensor-to-transport reconnect signal has capacity one, because additional
  requests while one reconnect is pending carry no extra work.

`SerialHandle::send` first requires an online transport. Its configured timeout
covers waiting for queue capacity and waiting for the writer to complete
`write_all` plus `flush`. Both waits observe shutdown in at most 5 ms steps.
The writer skips cancelled or expired work instead of transmitting it late.

The reader is driven by bytes arriving from the OS. OS read timeouts are normal
and merely return to the loop. Inbound queue saturation is treated as a session
failure because silently dropping a byte chunk would corrupt protocol framing.

### Serial reconnect behavior

A reported read or write error, disconnected failure channel, queue overflow,
or explicit sensor request ends the serial session. The supervisor rejects
queued writes, clears queued inbound chunks, marks `Reconnecting`, waits with
bounded exponential backoff, and opens a new transport generation. Commands
and bytes from an old generation are never replayed into the new one. The
supervisor polls session failure and reconnect signals every 20 ms.

The topology currently assigns each serial transport exclusively to one sensor.
The transport remains protocol-neutral: it moves deadline-bound byte vectors
out and timestamped byte chunks in; EBIMU framing and health belong to the IMU
worker.

## EBIMU sensor

### Configuration sequence

The EBIMU worker waits for its serial transport to be online. On every new
transport generation it sends the following commands in order. It waits up to
two seconds for each `<ok>` acknowledgement because physical EBIMU devices may
persist a setting before replying:

```text
<soc1>               ASCII output
<sof2>               quaternion orientation
<sog1>               gyroscope output enabled
<soa2>               local linear acceleration
<som0>               magnetometer output disabled
<sot0>               temperature output disabled
<sots1>              device timestamp enabled
<sor{interval_ms}>   output interval in milliseconds
```

Together these commands select the exact ASCII output layout consumed by this
driver and set `interval_ms = 1000 / output_rate_hz`. Treat the sequence and
parser as one schema: changing output fields or order requires changing both.
The definitions are in
[`hardware/imu/ebimu.rs`](../components/bello-hardwared/src/hardware/imu/ebimu.rs),
and orchestration is in
[`hardware/imu/mod.rs`](../components/bello-hardwared/src/hardware/imu/mod.rs).

The serial layer opens the configured baud rate but does not change the EBIMU's
stored baud setting; the two must already match. The sequence also does not set
gyro/accelerometer range, accelerometer filtering, magnetometer operating mode,
or calibration. A rejected command, missing acknowledgement, or failure to
produce a valid stream triggers the reconnect path.

### Wire schema and units

Samples are newline-delimited ASCII. A data line begins with `*` and contains
exactly 11 finite comma-separated numbers:

```text
*q0,q1,q2,q3,gx,gy,gz,ax,ay,az,timestamp_ms
```

The parser:

- converts the quaternion to application axes as `[q2, -q1, -q0, q3]` and
  normalizes it;
- converts angular velocity from degrees/s to radians/s;
- converts acceleration from g to m/s² using `9.80665`; and
- rounds the device timestamp, clamps it to nonnegative, and reports integer
  milliseconds.

Non-finite fields, the wrong field count, invalid UTF-8, and a near-zero
quaternion are rejected. Lines may span transport chunks. To bound memory after
lost framing, more than 64 KiB without a newline clears the frame buffer and is
reported as malformed input.

Every valid sample is emitted individually at the received rate. Multiple IMUs
remain distinct by name; their samples are neither averaged nor combined. The
telemetry channel is event-based rather than a cached latest-state publisher.

### Health and reconnect

The IMU worker polls its serial input at 20 ms intervals. A malformed line is
discarded. If the IMU was `Online`, this moves it to `Degraded`; before the
first valid sample it updates the current connecting detail instead. The next
valid sample returns it to `Online`.

After device configuration, the worker requires valid samples. If no valid
sample arrives within `stale_timeout_ms`, it marks the IMU reconnecting,
requests a serial reconnect, clears partial framing and timestamps, and waits
for the serial transport's transition count to prove that a new online
generation exists. It then sends the entire configuration sequence again. A
serial transport loss follows the same reconfiguration path. This generation
check prevents a request raised against an old session from being mistaken for
a successful reconnect.

Unlike a motor, an IMU does not belong to a motor group and therefore does not
directly invoke a group safety reaction. Its serial and IMU lifecycle events
are published for supervision, while valid samples continue into the public
gRPC telemetry stream.

## Rates and timing at a glance

| Activity | Timing source | Behavior |
| --- | --- | --- |
| Motor group control | `control_rate_hz`, 1–2000 Hz | Periodic group worker; produces one frame per motor per tick |
| CAN TX | Queue arrivals and batch deadlines | Event-driven, strict safety/configuration/control priority |
| CAN RX | SocketCAN input | Blocking/event-driven with a 100 ms OS timeout |
| Motor health scan | `feedback_timeout_ms / 4`, clamped to 1–20 ms | Checks bus state and armed feedback watchdogs |
| Serial TX | Sensor configuration requests | Event-driven; bounded by `timeout_ms` |
| Serial RX | Port input | Blocking/event-driven with configured OS timeout |
| Serial supervisor | Fixed 20 ms monitor poll | Detects worker failures and reconnect requests |
| EBIMU output | `output_rate_hz` | Device-generated samples at an exact integer-ms interval |
| EBIMU worker | Fixed 20 ms receive poll | Frames/parses input and evaluates stale timeout |

These are requested or supervisory timings, not real-time guarantees. OS
scheduling, driver buffering, CAN arbitration, bus load, serial latency, and
device firmware all contribute to observed latency and jitter. Group telemetry
reports control-tick jitter; raw transport deadlines keep stale queued work from
being mistaken for current control.
