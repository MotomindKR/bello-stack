# Bello hardware for Python

`bello-hardware` is the supported Python interface to `bello-hardwared`. Use it
to discover the configured robot, read motor and IMU telemetry, and control
motor groups from a normal synchronous Python program.

The package is a small pure-Python layer over gRPC. Applications work with
Python objects such as `Robot`, `MotorGroup`, `CoordinateTarget`, and
`HardwareStateFrame`; they do not need to construct protobuf messages or manage
lease tokens.

## Before you start

You need:

- Python 3.10 or newer;
- a running `bello-hardwared` reachable over the network; and
- the daemon's configured motor-group and coordinate names.

The Python package does not open CAN or serial devices itself. The process
boundary is:

```text
Python application -> gRPC -> bello-hardwared -> CAN and serial devices
```

## Install

Create a virtual environment for your application and install the package from
this repository:

```sh
python3 -m venv .venv
. .venv/bin/activate
python -m pip install -e /path/to/bello_stack/components/bello-hardware-python
```

The `-e` editable installation is convenient while developing both projects:
client source changes take effect without reinstalling it. For an ordinary
non-editable installation, omit `-e`:

```sh
python -m pip install /path/to/bello_stack/components/bello-hardware-python
```

`grpcio` and `protobuf` are installed automatically. Install application
dependencies such as NumPy into the same virtual environment:

```sh
python -m pip install numpy pyyaml
```

A local application's `requirements.txt` can record the editable dependency:

```text
-e /path/to/bello_stack/components/bello-hardware-python
numpy
pyyaml
```

The distribution name uses a hyphen (`bello-hardware`), while Python imports
use an underscore:

```python
import bello_hardware
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
        print("IMU:", imu.name)
```

The inventory is fetched and validated when `Robot.connect` returns. This
makes the daemon configuration the source of truth instead of duplicating the
robot topology in the Python application. Inconsistent bus or serial
references, unused transports, duplicate physical addresses, invalid CAN IDs,
unsupported Robstride models, and invalid device metadata fail the connection
instead of producing a partially usable client.

The default endpoint is `http://127.0.0.1:50051`, so this is also valid:

```python
robot = Robot.connect()
robot.close()
```

Prefer the context-manager form whenever possible.

## Read state

`Robot.states()` creates a synchronous reader. Each `read()` returns one
immutable frame built from the latest samples for every selected motor, IMU,
and group. If daemon state is still initializing, the reader waits for a
complete frame within the caller's timeout. There is no Python telemetry
aggregator, background receiver, or frame queue.

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
pipeline.poll()  # surfaces completed acknowledgement errors without waiting
# ...
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

The Python application sends new targets when it chooses; it does not need to
reproduce the group's fixed-rate loop. While the group is running, each accepted
command triggers an immediate checked control tick. The daemon keeps its fixed
periodic safety schedule independently. Immediate commands defer periodic CAN
refresh; if the command stream goes quiet, the daemon reapplies the latest
accepted command at `command_refresh_rate_hz`. Command-validity and safety
policies remain enforced at `control_rate_hz`.

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
    control.send({
        "hip": CoordinateTarget.damping(kd=1.0),
        "knee": CoordinateTarget.damping(kd=1.0),
    })
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

## Safety and lifecycle behavior

The daemon remains authoritative for command timing, configured limits, device
health, group state, fault reactions, and lease-loss behavior. The Python
client adds application-side validation and reliable resource ownership, but it
is not the safety controller.

Keep these rules in mind:

- use context managers or call `close()` explicitly;
- send complete group targets rather than individual-coordinate updates;
- treat rejected, timed-out, or ambiguous commands as real failures;
- monitor complete group state while commanding motion and use protocol-level
  diagnostics when component-health events are required; and
- rely on daemon-side lease expiry and command timeout to transition an enabled
  group to `DAMPED` if the Python process or network disappears.

Read-only inventory and state reads do not acquire a lease or start a client
worker. Active control uses a lease renewal thread and an acknowledgement
receiver; CAN, serial, device FSMs, and fixed-rate motor control stay in
`bello-hardwared`.

## Connection endpoints

`Robot.connect` accepts these endpoint forms:

```python
Robot.connect("127.0.0.1:50051")          # plaintext gRPC
Robot.connect("http://127.0.0.1:50051")   # plaintext gRPC
Robot.connect("https://robot.example")    # TLS with default roots
Robot.connect("unix:///run/bello/hardwared.sock")
```

For a private CA or mutual TLS, load the PEM files once and pass explicit
credentials:

```python
from bello_hardware import Robot, TlsCredentials

tls = TlsCredentials.from_files(
    ca_certificate="/etc/bello/tls/ca.crt",
    client_certificate="/etc/bello/tls/client.crt",       # omit both for TLS
    client_private_key="/etc/bello/tls/client.key",       # without mTLS
)

with Robot.connect("https://robot.example:50051", tls=tls) as robot:
    print(robot.group_names)
```

Client certificates and keys must be configured together. TLS credentials are
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
[Bello stack documentation](../../docs/README.md).
