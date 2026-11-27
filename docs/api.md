# Hardware gRPC API

Managed SDKs · [Rust](../components/bello-hardware-api/bindings/rust/client/README.md) · [Python](../components/bello-hardware-api/bindings/python/client/README.md) · [Wire contract](../components/bello-hardware-api/proto/bello/hardware/v1/hardware.proto).

The [hardware bindings](../components/bello-hardware-api/bindings) are grouped by language. Each language has an `api/` package for generated gRPC types and a `client/` package for the managed SDK. Both use the same wire contract.

## Direct clients

**Rust** · `bello-hardware-api` crate, inside an async function

```rust
use bello_hardware_api::{ListHardwareRequest, hardware_service_client::HardwareServiceClient};

let mut client = HardwareServiceClient::connect("http://127.0.0.1:50051").await?;
let inventory = client.list_hardware(ListHardwareRequest {}).await?.into_inner();
```

**Python** · Install from the repository root with `python -m pip install ./components/bello-hardware-api/bindings/python/api`.

```python
import grpc
from bello_hardware_api import hardware_pb2, hardware_pb2_grpc

with grpc.insecure_channel("127.0.0.1:50051") as channel:
    client = hardware_pb2_grpc.HardwareServiceStub(channel)
    inventory = client.ListHardware(hardware_pb2.ListHardwareRequest(), timeout=5)
```

## Read hardware

These calls require no lease.

| RPC | Result |
| --- | --- |
| `ListHardware` | Current inventory and device status. |
| `GetStateFrame` | Latest cached samples for the selected motors, groups, and IMUs. Returns `FAILED_PRECONDITION` until every selected source has reported. Check sample ages and span before using a frame for control. |
| `WatchTelemetry` | Stream of individual events. A `grpc_telemetry` health event reports how many events a slow subscriber lost. |

## Normal control workflow

1. **Own the group.** Call `AcquireLease` for one motor group. Renew with `RenewLease` before expiry, including while lifecycle calls are pending.
2. **Start.** `EnableGroup` enters `Damped`. `StartControl` enters `Running` with one initial target per motor.
3. **Send.** Each `StreamCommands` batch carries the group, lease token, increasing sequence, and one target per motor. Match replies by group and sequence. An accepted reply confirms daemon admission. Use telemetry for motor feedback. Domain rejections arrive in the reply. Malformed requests can terminate the stream.
4. **Finish.** `StopControl` returns to `Damped`. Resume with a fresh `StartControl`, or call `DisableGroup` and `ReleaseLease`. Lease loss while running applies damping.

[Fault recovery and state transitions](control-and-safety.md#motor-group-fsm) · [TLS and Unix sockets](configuration.md#grpc) · [Re-zeroing and firmware updates](operations.md#re-zero-a-motor-or-update-its-firmware) · [Pose service](../components/bello-pose-api/proto/bello/pose/v1/pose.proto)
