# Bello stack documentation

These guides describe the current Bello hardware stack as implemented. They
separate stable concepts and operational behavior from crate-level READMEs and
source comments.

## Suggested reading paths

| Goal | Start here | Continue with |
| --- | --- | --- |
| Understand the complete process | [Architecture](architecture.md) | [Hardware](hardware.md), [Control and safety](control-and-safety.md) |
| Prepare a robot configuration | [Configuration](configuration.md) | [Operations](operations.md) |
| Build a Rust gRPC client | [`bello-hardware-client`](../components/bello-hardware-client) | [API](api.md), [Control and safety](control-and-safety.md) |
| Visualize or record telemetry | [Operations](operations.md#rerun-observation) | [API](api.md#inventory-and-telemetry) |
| Diagnose CAN, motor, serial, or IMU faults | [Hardware](hardware.md) | [Operations](operations.md) |
| Run or test the stack | [Operations](operations.md) | [Development](development.md) |
| Review or modify the code | [Architecture](architecture.md) | [Development](development.md) |

## Guide map

- [Architecture](architecture.md) explains process boundaries, ownership,
  workers, data flow, lifecycle, and shutdown.
- [Configuration](configuration.md) describes the hardware-daemon and observer
  YAML models, validation boundaries, topology rules, and examples.
- [Hardware](hardware.md) covers SocketCAN, Robstride devices, serial sessions,
  EBIMU configuration, timing, and reconnect behavior.
- [Control and safety](control-and-safety.md) covers leases, the motor-group FSM,
  command handling, watchdogs, health checks, fault responses, and recovery.
- [API](api.md) explains the protobuf service, normal client workflow, typed
  results, telemetry filters, and security boundary.
- [`bello-hardware-client`](../components/bello-hardware-client) documents the
  maintained high-level Rust client, explicit renewal, and command streams.
- [Operations](operations.md) provides startup, vCAN emulation, standalone
  Rerun observation, diagnostics, and shutdown procedures.
- [Development](development.md) documents the toolchain, source layout, checks,
  integration tests, and a code-review order.

## Core terminology

- A **bus** is one named SocketCAN transport bound to an existing Linux CAN
  interface.
- A **serial device** is one named serial transport and reconnect boundary.
- A **motor** is one Robstride protocol device at a `(bus, id)` address.
- An **IMU** is one EBIMU device that exclusively consumes one serial stream.
- A **motor group** is a generic, exclusive control and safety unit containing
  one or more motors. Groups may span buses; buses may serve several groups.
- A **lease** is temporary exclusive write ownership of one motor group.
- **Telemetry** is the event stream produced by transports, devices, and group
  controllers and exposed through gRPC. The optional Rerun process is one
  read-only gRPC client; its dedicated bounded writer queue isolates gRPC from
  visualization backpressure.

## Sources of truth

Documentation is explanatory. The authoritative machine-readable interfaces
remain:

- [`hardware.proto`](../components/bello-hardware-api/proto/bello/hardware/v1/hardware.proto)
  for the gRPC contract;
- [`config/model.rs`](../components/bello-hardwared/src/config/model.rs) and the
  validated specification modules for YAML fields and limits;
- [`config/bello.yaml`](../components/bello-hardwared/config/bello.yaml) and
  [`config/vcan.yaml`](../components/bello-hardwared/config/vcan.yaml) for
  maintained example configurations.

The project is greenfield. Documentation describes the current interface only;
it does not preserve historical aliases or compatibility behavior.
