# Bello stack documentation

## Suggested reading paths

| Goal | Start here | Continue with |
| --- | --- | --- |
| Understand the complete process | [Architecture](architecture.md) | [Hardware](hardware.md), [Control and safety](control-and-safety.md) |
| Prepare a robot configuration | [Configuration](configuration.md) | [Operations](operations.md) |
| Build a Rust gRPC client | [`bello-hardware-client`](../components/bello-hardware-client) | [API](api.md), [Control and safety](control-and-safety.md) |
| Build a Python client | [`bello-hardware-python`](../components/bello-hardware-python) | [API](api.md), [Control and safety](control-and-safety.md) |
| Diagnose CAN, motor, serial, or IMU faults | [Hardware](hardware.md) | [Operations](operations.md) |
| Run or test the stack | [Operations](operations.md) | [Development](development.md) |
| Run on Linux, macOS, or Windows | [Desktop builds](desktop.md) | [Operations](operations.md) |
| Review or modify the code | [Architecture](architecture.md) | [Development](development.md) |

See [power-board telemetry](power_board.md) for power monitoring and the emergency button.

## Core terminology

- A **bus** is one named CAN transport bound to a SocketCAN interface or a
  GS-USB adapter channel.
- A **serial device** is one named serial transport and reconnect boundary.
- A **motor** is one Robstride protocol device at a `(bus, id)` address.
- An **IMU** is one EBIMU device that exclusively consumes one serial stream.
- A **motor group** is a generic, exclusive control and safety unit containing
  one or more motors. Groups may span buses; buses may serve several groups.
- A **lease** is temporary exclusive write ownership of one motor group.
- A **coordinate** is the hardware-facing position named by daemon inventory;
  it maps to a URDF joint directly or through a differential transmission.
- **Telemetry** is the event stream produced by transports, devices, and group
  controllers and exposed through gRPC.
