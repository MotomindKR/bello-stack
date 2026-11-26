# Bello hardware Python client

Install from the repository root with Python 3.10 or newer.

```sh
python -m pip install ./components/bello-hardware-api ./components/bello-hardware-python
```

## Read hardware

```python
from bello_hardware import Robot

with Robot.connect() as robot:
    print(robot.inventory.group_names)
    frame = robot.states().read(timeout=1.0)
    for motor in frame.motors:
        print(motor.coordinate, motor.position)
```

| Call | Result |
| --- | --- |
| `robot.refresh_inventory()` | Fetch and cache new inventory. Existing groups and readers keep their snapshot. |
| `robot.states(groups=..., imus=...).read(timeout=...)` | One complete frame for the selection. Check sample ages, round-trip time, and sample span before using it for control. |
| `robot.power().read(timeout=...)` | A snapshot per board. Use `board.is_stale(sample)` for freshness. Optional readings such as battery charge use `None`. |

The default endpoint is `http://127.0.0.1:50051`. Pass an HTTPS URL or Unix socket path such as `unix:///run/bello/hardwared.sock` to `Robot.connect()`. For mTLS, pass `tls=TlsCredentials.from_files(...)`. [Transport configuration](../../docs/configuration.md#grpc)

## Control a motor group

Replace `leg` with a configured group. Each command supplies one target per coordinate.

```python
from bello_hardware import CoordinateTarget, Robot

with Robot.connect() as robot:
    group = robot.group("leg")
    targets = {name: CoordinateTarget.damping(kd=1.0) for name in group.coordinates}
    with group.control("example") as control:
        control.start(targets)
        control.send(targets)
```

| Operation | Behavior |
| --- | --- |
| `control.send(targets)` | Wait for capacity and daemon acknowledgement. |
| `control.try_submit(targets)` | Return a receipt or raise `CommandQueueFullError`. Call `receipt.result()` to wait. Default capacity is one. |
| `control.stop()` / `control.resume(targets)` | Keep the lease while damped, then restart with fresh targets. |
| Exit the control context | Stop, disable, and release. Use `control.close(preserve_damping=True)` to release with damping. |

A rejection closes the command stream. Resume after confirmed damping. Check group state before recovery after an ambiguous result. [Control and safety](../../docs/control-and-safety.md)

[Record telemetry](examples/log_hardware.py) · [View a recording](examples/view_hardware_log.py) · [Direct RPCs](../../docs/api.md#direct-clients) · [Development](../../docs/development.md)
