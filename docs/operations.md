# Operations guide

[Documentation index](README.md)

This guide covers deployment and diagnostics. The complete configuration and
safety behavior are described in [`configuration.md`](configuration.md) and
[`control-and-safety.md`](control-and-safety.md); the checked examples are
[`bello.yaml`](../components/bello-hardwared/config/bello.yaml) and
[`bello-vcan.yaml`](../components/bello-hardwared/config/bello-vcan.yaml).

> **Warning:** enabling or starting a physical group can move hardware. Verify
> the emergency stop, mechanical clearance, actuator IDs/directions, limits,
> fault responses, and command source before granting motion authority.

## Prerequisites and configuration check

The runtime is Linux-oriented and expects existing SocketCAN interfaces and,
when configured, accessible serial device paths. The daemon does not create CAN
links or set their bitrate. For physical hardware, ensure the service account
has CAN access and read/write permission for the serial device (prefer stable
`/dev/serial/by-id/...` names).

Enter the repository environment and validate YAML without opening hardware:

```sh
direnv allow
cargo run -p bello-hardwared -- \
  --check --config components/bello-hardwared/config/bello.yaml
```

Validation is strict: unknown YAML fields, duplicate physical interfaces/ports,
invalid references, unsupported actuator limits, unsafe timing, and excessive
CAN utilization are rejected. The configured CAN `bitrate` is used for timing
and utilization calculations and must match the externally configured physical
link. The `--config` argument is always required so deployment selection is
explicit.

## Physical startup

Configure the CAN link before starting hardwared. Adjust interface, bitrate,
and restart policy for the actual adapter and network:

```sh
sudo scripts/setup-can.sh
```

The script configures both canonical Bello interfaces, `can0` and `can1`, for
classic CAN at 1 Mbit/s with a 100 ms automatic restart and a 32-frame transmit
queue. To configure one link manually instead:

```sh
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 restart-ms 100
sudo ip link set can0 txqueuelen 32
sudo ip link set can0 up
ip -details -statistics link show can0
```

Confirm serial identity and permissions when using EBIMU:

```sh
ls -l /dev/serial/by-id/
test -r /dev/serial/by-id/YOUR_IMU && test -w /dev/serial/by-id/YOUR_IMU
```

Then run the checked configuration:

```sh
RUST_LOG=info cargo run -p bello-hardwared -- \
  --config components/bello-hardwared/config/bello.yaml
```

The process binds gRPC before opening hardware, so address conflicts fail
without partially starting devices. Wait for the `Bello hardware daemon ready`
log, then use `ListHardware` and health telemetry to inspect the buses, serial
devices, and IMUs. A Robstride motor may remain `CONNECTING` until `EnableGroup`
turns on active feedback, so motor readiness is established by the enable
operation rather than being a prerequisite for acquiring a lease. The API
workflow is in [`api.md`](api.md#normal-control-workflow).

## Virtual CAN and emulator

Create `vcan0` once per boot:

```sh
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set dev vcan0 up
```

The small checked emulator configuration is a protocol fixture:

```sh
cargo run -p robstride-emulator -- \
  --check --config components/robstride-emulator/config/vcan.yaml
cargo run -p robstride-emulator -- \
  --config components/robstride-emulator/config/vcan.yaml
```

The process-level vCAN tests generate a matching strict hardware description
and deployment for this fixture; there is no second checked-in robot topology.
Use the full `bello-vcan.yaml` deployment only with the Bello emulator flow
later in this guide, which supplies both logical buses and the IMU endpoint.

The RobStride emulator is intentionally a thin protocol/integration model, not
a robot dynamics simulator. Its strict `events` list can inject feedback drop/delay/
jitter, temperature, fault bits, and mode-status changes. Use the ready-made
[`faults.yaml`](../components/robstride-emulator/config/faults.yaml), or follow
the action syntax in the
[emulator README](../components/robstride-emulator/README.md#fault-scenarios).
Events are relative to emulator startup. Group faults remain latched after the
injected condition clears and require `RecoverGroup`.

To exercise transport reconnect rather than actuator faults:

```sh
sudo ip link set vcan0 down
sudo ip link set vcan0 up
```

Hardwared supervises and reopens its CAN sockets. The emulator does not have a
SocketCAN reconnect loop and may exit when the interface goes down, so check its
process after restoring the link and restart it when necessary. Hardwared can
restore the bus while its group remains fault-latched; feedback and explicit
group recovery are still required.

The Cargo-native end-to-end test owns its phases deterministically and is the
preferred regression check; see
[`development.md`](development.md#host-vcan-integration-test).

## Bello visualization

The maintained visualizer reads the canonical URDF under
`descriptions/bello/`, hardware inventory from port 50051, and an optional
neutral `PoseService` on port 50053. Inspect a configured pose provider with:

```sh
grpcurl -plaintext \
  -import-path components/bello-pose-api/proto \
  -proto bello/pose/v1/pose.proto \
  -d '{"referenceFrame":"odom","trackedFrame":"pelvis_link"}' \
  127.0.0.1:50053 bello.pose.v1.PoseService/WatchPose
```

When the configured SocketCAN, IMU, hardware, and pose providers are available,
start hardwared and the visualizer in separate terminals:

```sh
cargo run -p bello-hardwared -- \
  --config components/bello-hardwared/config/bello-vcan.yaml
cargo run -p bello-visualizer -- \
  --config components/bello-visualizer/config/bello-vcan.yaml
```

Both visualizer service endpoints default to localhost. When the providers run
on another host, use TLS and append `--hardware-endpoint https://HOST:50051`
and `--pose-endpoint https://HOST:50053`.

If hardware or pose telemetry disconnects or becomes stale, the visualizer
freezes the last root transform and shows the condition; root tracking never
participates in command or safety decisions.

## Shutdown behavior

Send `SIGINT` (Ctrl-C) or `SIGTERM`; do not use `SIGKILL` during routine
operation. The daemon then:

1. requests the lease watchdog to stop;
2. closes admission, asks every group to apply its configured `exit` response
   while CAN and motor workers remain alive, publishes final group `Shutdown`
   events, and joins the group workers;
3. closes the server/telemetry watch and gives selected queued telemetry up to
   two seconds to drain through gRPC;
4. joins the watchdog, then stops device and transport ownership.

The two-second deadline applies only to gRPC draining after group shutdown has
completed. It does not abort the configured hardware exit reaction or a group
worker join.

The shutdown path is idempotent. A forced kill, kernel failure, or power loss
cannot perform the configured exit reaction, so system-level power removal and
emergency-stop design remain necessary. See
[`run.rs`](../components/bello-hardwared/src/app/run.rs) and
[`daemon.rs`](../components/bello-hardwared/src/app/daemon.rs).

## Diagnostics and troubleshooting

Use the protobuf file explicitly because the server does not enable gRPC
reflection:

```sh
PROTO_ROOT=components/bello-hardware-api/proto
PROTO=bello/hardware/v1/hardware.proto

grpcurl -plaintext -import-path "$PROTO_ROOT" -proto "$PROTO" \
  -d '{}' 127.0.0.1:50051 \
  bello.hardware.v1.HardwareService/ListHardware

grpcurl -plaintext -import-path "$PROTO_ROOT" -proto "$PROTO" \
  -d '{"groups":["emulated_group"],"includeMotors":true,"includeHealth":true}' \
  127.0.0.1:50051 bello.hardware.v1.HardwareService/WatchTelemetry
```

Useful CAN observations:

```sh
# Absolute timestamps and a replayable log format.
candump -ta -L can0

# Per-frame delta timestamps: inspect observed inter-frame spacing.
candump -td can0

# Link state, error counters, drops, and CAN controller state.
ip -details -statistics link show can0

# Approximate bus load; use the real nominal bitrate.
canbusload can0@1000000
```

Group control-loop timing is exposed as `GroupTelemetry.control_tick` and
`jitter_us`; despite its signed protobuf type, the current value is nonnegative
deadline lateness. `MotorTelemetry` and `ImuTelemetry` expose `feedback_age_ms`
and `sample_age_ms`, including time spent waiting in the subscriber path.

`candump -td` reports deltas between observed frames. It is useful for
inter-arrival jitter but does not measure end-to-end command latency by itself.
For command-to-feedback latency, capture both directions and correlate a
command with the corresponding motor feedback by CAN identifier and timestamp;
then compare that result with group deadline lateness and telemetry age.

Common failure checks:

- **Process exits before becoming ready:** run `--check`, verify the gRPC address
  is available, and inspect worker-creation errors. Missing hardware normally
  leaves a transport reconnecting rather than failing configuration resolution.
- **Bus reconnecting/faulted:** inspect CAN error counters and physical wiring,
  termination, power, bitrate, and duplicate IDs. Reconnect uses bounded
  exponential backoff; stale queued controls are discarded.
- **Motor enable fails:** confirm emulator/actuator host ID, motor ID, model,
  direction, operating mode support, and fresh feedback. Inspect motor health
  plus `fault_bits`, `mode_status`, temperature, and age telemetry.
- **IMU degraded/reconnecting:** verify the stable serial path, baud rate,
  exclusive ownership, permissions, expected EBIMU schema/rate, and malformed
  or stale-sample health detail.
- **Commands rejected:** inspect the typed `CommandResult` and `detail`, group
  state, lease expiry, batch completeness, sequence, validity window, and
  command-policy telemetry.
- **Telemetry drops:** increase `runtime.telemetry_capacity` only after checking
  subscriber throughput. A lag event reports the exact dropped count; control
  continues independently.

Set `RUST_LOG` for more detail, for example
`RUST_LOG=bello_hardwared=debug,robstride_emulator=debug`. Avoid debug logging on
a production control path until its timing impact has been evaluated.
