# Bello hardware for Python

Synchronous Python client for reading hardwared telemetry and controlling motor groups.

## Install

With Python 3.10 or newer, install the package from the repository root:

```sh
python -m pip install ./components/bello-hardware-python
```

## Connect and inspect the robot

Use `Robot` as a context manager so the gRPC channel and any owned control
sessions are always closed:

```python
from bello_hardware import Robot

with Robot.connect("http://127.0.0.1:50051") as robot:
    print("groups:", robot.group_names)

    for motor in robot.motors:
        print(motor.name, "coordinate=", motor.coordinate, "torque=", motor.limits.torque_max)

    for imu in robot.inventory.imus:
        print(
            "IMU:",
            imu.name,
            "source_frame=",
            imu.source_frame,
            "output_frame=",
            imu.output_frame,
        )
```

`Robot.connect()` defaults to `http://127.0.0.1:50051` and validates the inventory
before returning. Check each IMU's `output_frame` before using its readings.

## Read state

`Robot.states()` creates a synchronous reader. Each `read()` returns one
immutable frame built from the latest samples for every selected motor, IMU,
and group. If daemon state is still initializing, the reader waits for a
complete frame within the caller's timeout.

```python
with Robot.connect("http://127.0.0.1:50051") as robot:
    reader = robot.states(
        groups=("leg",),
        imus=("body-imu",),
    )
    while True:
        frame = reader.read(timeout=1.0)
        print(frame.motors, frame.imus, frame.groups)
```

Empty group or IMU selectors select all configured devices. Frames expose
daemon-side sample ages and span, the local receive timestamp, and the complete
RPC round-trip duration. A control loop can conservatively add that duration
and any time since receipt to its freshness checks, covering transport and
Python scheduling delays without synchronized clocks. Apply those checks to
motor, IMU, and group ages, and reject a sample span wider than the control
loop's coherence budget. The raw `WatchTelemetry` protocol stream remains
available to protocol-level diagnostics that require event history, health, or
command-policy events.

For temporary debugging, `examples/log_hardware.py` records complete state
frames as newline-delimited JSON at 50 Hz by default:

```sh
PYTHONPATH=components/bello-hardware-python/src \
  python3 components/bello-hardware-python/examples/log_hardware.py \
  --endpoint http://127.0.0.1:50051
```

It creates `hardware-<UTC timestamp>.jsonl` in the current directory. Use
`--output -` for stdout, `--rate 100` to change the sampling rate, or
`--duration 30` for a bounded capture. The first line contains inventory; later
lines contain complete motor, group, and IMU frames with sample and transport
timing.

Turn a capture into a self-contained interactive browser dashboard with no
additional Python dependencies:

```sh
python3 components/bello-hardware-python/examples/view_hardware_log.py \
  hardware-20260722T120000Z.jsonl
```

The dashboard plots selectable motor and IMU channels and summarizes group and
fault transitions. It writes `LOG.jsonl.html` and opens it in the default
browser; pass `--no-open` when running headless.

## Control a motor group

Control authority is acquired per motor group. Only one client can hold the
write lease for a particular group, although different groups can be controlled
concurrently.

```python
from bello_hardware import CoordinateTarget, Robot

with Robot.connect("http://127.0.0.1:50051") as robot:
    leg = robot.group("leg")
    print("expected coordinates:", leg.coordinates)
    print("control rate:", leg.control_rate_hz, "Hz")

    initial = {
        "hip": CoordinateTarget.position(0.0, kp=20.0, kd=1.0),
        "knee": CoordinateTarget.position(0.0, kp=20.0, kd=1.0),
    }

    target = {
        "hip": CoordinateTarget.position(0.2, kp=20.0, kd=1.0),
        "knee": CoordinateTarget.position(-0.4, kp=20.0, kd=1.0),
    }

    with leg.control("walking-controller") as control:
        control.start(initial)
        control.send(target)
```

`control.start` enables the group into `DAMPED`, then atomically starts the
daemon controller with the initial complete coordinate frame. `control.send`
blocks until the daemon acknowledges that exact command. `control.stop()` leaves
the motors damped under the current lease; restart with
`control.resume(fresh_frame)` or call `control.disable()` to return to `IDLE`.
Leaving the control context performs stop if needed, disables the group, stops
automatic lease renewal, and releases the lease.

Safety-critical applications that require an operator-confirmed transition to
`IDLE` can call `control.close(preserve_damping=True)`. This stops a running
group when possible and releases the lease without sending `DisableGroup`;
daemon lease-loss handling retains damping. Call `control.disable()` explicitly
only after the operator has confirmed that disabling the group is safe.

Latency-sensitive fixed-rate loops can use a bounded pipeline so delayed
acknowledgements do not directly schedule the next command:

```python
from bello_hardware import CommandPipeline

pipeline = CommandPipeline(control, max_pending=4)
pipeline.send(target)
pipeline.poll()
pipeline.drain()
```

`pipeline.send` uses `control.send_async` and waits only when the explicit
pending bound is full. Recent submit-to-acknowledgement measurements are
available through `pipeline.acknowledgement_latencies_ms`, while submission
queue delay is exposed by `last_submission_latency_ms` and
`maximum_submission_latency_ms`. Applications must call `poll()` every control
tick and `drain()` during orderly shutdown. Asynchronous submission does not
weaken the daemon's command-validity watchdog.

Every `start` and `send` call must contain exactly one target for every coordinate in
`leg.coordinates`. A partial command is rejected before it reaches the daemon.

All target values use SI units. The available constructors are:

```python
CoordinateTarget.position(0.25, kp=20.0, kd=1.0)
CoordinateTarget.torque(1.5)
CoordinateTarget.damping(kd=1.0)
CoordinateTarget.hybrid(
    position=0.25,
    velocity=0.0,
    torque_feedforward=0.2,
    kp=20.0,
    kd=1.0,
)
```

The daemon applies accepted commands immediately and refreshes them at
`command_refresh_rate_hz` while valid. Lease expiry or command timeout damps
an enabled group if the client disconnects.

## Handle failures

All package-defined errors inherit from `BelloError`. Catch specific errors
when your application can make a meaningful recovery decision:

```python
from bello_hardware import (
    CommandRejectedError,
    ControlRpcError,
    CoordinateTarget,
    LeaseLostError,
)

try:
    control.send(
        {
            "hip": CoordinateTarget.damping(kd=1.0),
            "knee": CoordinateTarget.damping(kd=1.0),
        }
    )
except CommandRejectedError as error:
    print("command rejected:", error.result, error.detail)
except LeaseLostError:
    print("control authority was lost")
except ControlRpcError as error:
    print(error.operation, error.code.name, error.detail)
```

The client deliberately does not reacquire control or replay a motion command
after an ambiguous transport failure. If a rejected command acknowledgement
reports daemon state `DAMPED`, the session also becomes `DAMPED` and can be
resumed with a fresh frame. If a command might have reached the daemon but its
acknowledgement was lost, the session becomes `UNCERTAIN`. Resolve the
authoritative daemon state before choosing recovery or closing the session.

## Connection endpoints

`Robot.connect` accepts these endpoint forms:

```python
Robot.connect("127.0.0.1:50051")  # plaintext gRPC
Robot.connect("http://127.0.0.1:50051")  # plaintext gRPC
Robot.connect("https://robot.example")  # TLS with default roots
Robot.connect("unix:///run/bello/hardwared.sock")
```

For a private CA or mutual TLS, load the PEM files once and pass explicit
credentials:

```python
from bello_hardware import Robot, TlsCredentials

tls = TlsCredentials.from_files(
    ca_certificate="/etc/bello/tls/ca.crt",
    client_certificate="/etc/bello/tls/client.crt",
    client_private_key="/etc/bello/tls/client.key",
)

with Robot.connect("https://robot.example:50051", tls=tls) as robot:
    print(robot.group_names)
```

For server-only TLS, omit both client identity fields. TLS credentials are
rejected for plaintext and Unix endpoints.

Use the `timeout` argument to bound the initial connection and inventory call:

```python
Robot.connect("http://robot.local:50051", timeout=3.0)
```

## Development

Application users do not need Rust, `protoc`, or `grpcio-tools`; generated
protobuf modules are included in the package.

To work on the client itself from the repository environment:

```sh
cd components/bello-hardware-python
ruff format --check .
ruff check .
mypy src/bello_hardware tests
pytest
python -m hatchling build
```

After changing the canonical protobuf contract, regenerate the private Python
modules from the repository root:

```sh
python components/bello-hardware-python/scripts/generate_proto.py
```

For daemon configuration, safety semantics, and operations, see the main
[Bello stack documentation](../../README.md#documentation).
