# Operations guide

This guide covers deployment and diagnostics. The complete configuration and
safety behavior are described in [`configuration.md`](configuration.md) and
[`control-and-safety.md`](control-and-safety.md); the checked examples are
[`bello.yaml`](../components/bello-hardwared/config/bello.yaml) and
[`bello-sim.yaml`](../components/bello-hardwared/config/bello-sim.yaml).

> **Warning:** enabling or starting a physical group can move hardware. Verify
> the emergency stop, mechanical clearance, actuator IDs/directions, limits,
> fault responses, and command source before granting motion authority.

## Prerequisites and configuration check

The daemon supports Linux, macOS, and Windows with accessible GS-USB adapters
and configured serial device paths. SocketCAN is available only on Linux;
`--check` rejects a SocketCAN deployment on other platforms. See
[desktop setup](desktop.md) for native executables and USB driver setup. The daemon
does not create SocketCAN links or set their bitrate. For physical hardware,
ensure the service account can access the selected CAN transport and has
read/write permission for serial devices (prefer stable
`/dev/serial/by-id/...` names).

Enter the repository environment and validate YAML without opening hardware:

```sh
direnv allow
cargo run -p bello-hardwared -- \
  --check --config components/bello-hardwared/config/bello.yaml
```

The configured CAN bitrate must match the physical link.

## Physical startup

For SocketCAN, configure the link before starting hardwared. Adjust interface,
bitrate, and restart policy for the actual adapter and network:

```sh
sudo scripts/setup-can.sh
```

The script configures both canonical Bello interfaces, `can0` and `can1`, for
classic CAN at 1 Mbit/s with a 100 ms automatic restart and a 64-frame transmit
queue. Its normal mode takes both interfaces down before configuring them; do
not run it while hardwared or the robot is operating. Inspect the existing
configuration without changing either interface with:

```sh
scripts/setup-can.sh --check
```

The read-only check requires both interfaces to be up, use classic-CAN MTU 16,
have CAN FD disabled, and report a 1 Mbit/s nominal bitrate. A standard-ID hand
controller may share `can1` without changing this link configuration. To
configure one link manually instead:

```sh
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 restart-ms 100 fd off
sudo ip link set can0 txqueuelen 64
sudo ip link set can0 up
ip -details -statistics link show can0
```

For direct GS-USB access, set `bus_transports` to `type: gsusb` with the
adapter's serial number and channel. The daemon configures that channel's
nominal bitrate. `gsusbio` detaches the Linux `gs_usb` driver while it owns the
USB interface, so do not also bind the same adapter through SocketCAN.

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

Select `bello-mini.yaml` in both component config directories for physical
Bello Mini. Once hardwared is ready, start its pose estimator separately:

```sh
RUST_LOG=info cargo run -p bello-state-estimatord -- \
  --config components/bello-state-estimatord/config/bello-mini.yaml
```

The process binds gRPC before opening hardware, so address conflicts fail
without partially starting devices. Wait for the `Bello hardware daemon ready`
log, then use `ListHardware` and health telemetry to inspect the buses, serial
devices, and IMUs. A motor may remain `CONNECTING` until `EnableGroup`
turns on active feedback, so motor readiness is established by the enable
operation rather than being a prerequisite for acquiring a lease. The API
workflow is in [`api.md`](api.md#normal-control-workflow).

## Simulate CAN without SocketCAN

With the pinned MuJoCo 3.12.0 library installed, run these in separate terminals from
the repository root. The Linux Nix shell supplies the library; on macOS, point
`MUJOCO_DYNAMIC_LINK_DIR` and `DYLD_LIBRARY_PATH` at the directory containing
`libmujoco.dylib`.

```sh
cargo run -p bello-sim -- run --config components/bello-sim/config/emulator_milly.yaml
cargo run -p bello-hardwared -- --config components/bello-hardwared/config/milly-sim.yaml
```

Each simulated bus uses two IPv4 loopback UDP ports, one for hardwared and one for
the simulator. Configure the same pair on both sides and give each bus its own
ports. Either process can start first. Motor feedback tells you the peer is alive;
an online CAN transport only means the daemon has bound its local socket.
Use vCAN for tests that need SocketCAN tools such as `candump`. Use physical CAN
for bus arbitration, electrical faults, or wire timing measurements.

To run the standalone motor emulator with the same daemon profile, start
`cargo run --manifest-path components/motor-emulator/Cargo.toml` in place of MuJoCo. Its
[events](../components/motor-emulator/README.md#fault-scenarios) can inject
actuator faults and delayed feedback.

## Virtual CAN and emulator

Use vCAN on Linux to inspect frames with `candump` or exercise SocketCAN link
recovery. Bring up a virtual interface:

```sh
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set dev vcan0 up
```

Run the matching SocketCAN profiles in separate terminals:

```sh
cargo run --manifest-path components/motor-emulator/Cargo.toml -- --config components/motor-emulator/config/milly-vcan.yaml
cargo run -p bello-hardwared -- --config components/bello-hardwared/config/milly-vcan.yaml
```

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

The [Linux SocketCAN tests](development.md#host-vcan-integration-test) capture frames on vCAN to check command expiry and measure latency.

## Bello and Bello Mini emulation

Follow the [native MuJoCo runtime instructions](../components/bello-sim/README.md)
to start `emulator_bello.yaml`
or `emulator_mini.yaml`. The simulator provides the IMU and power-board serial ports to hardwared automatically on Linux and macOS. Start MuJoCo first, then hardwared under the same user. After restarting MuJoCo, restart hardwared to open the new PTYs.
For Bello, run hardwared and the pose estimator in
separate terminals:

```sh
cargo run -p bello-hardwared -- \
  --config components/bello-hardwared/config/bello-sim.yaml
cargo run -p bello-state-estimatord -- \
  --config components/bello-state-estimatord/config/bello.yaml
```

For Bello Mini simulation, copy the estimator profile to `bello-mini.local.yaml`
in the same directory so its relative URDF path still resolves. Set
`hardware_endpoint: http://127.0.0.1:50051` in the copy to match the simulated
daemon's TCP listener. Start both processes in separate terminals:

```sh
cargo run -p bello-hardwared -- \
  --config components/bello-hardwared/config/bello-mini-sim.yaml
cargo run -p bello-state-estimatord -- \
  --config components/bello-state-estimatord/config/bello-mini.local.yaml
```

The Mini description remains provisional until encoder-zero and ankle-reduction
calibration is complete. On the physical Mini Torso, verify motor addresses,
directions, zeros, travel stops, and load sharing before enabling the hips.
Both motors drive one joint and must be commanded together through `torso_pitch`.

Inspect the estimator's `PoseService` on port 50053:

```sh
grpcurl -plaintext \
  -import-path components/bello-pose-api/proto \
  -proto bello/pose/v1/pose.proto \
  -d '{"referenceFrame":"odom","trackedFrame":"pelvis_link"}' \
  127.0.0.1:50053 bello.pose.v1.PoseService/WatchPose
```

Use the same request on port 50052 for MuJoCo ground truth.
See the [pose estimator README](../components/bello-state-estimatord/README.md)
for the estimator's configuration and frames.

## Shutdown behavior

Send `SIGINT` (Ctrl-C) or `SIGTERM`; do not use `SIGKILL` during routine
operation. The daemon then:

1. requests the lease watchdog to stop;
2. closes admission, asks every group to apply its configured `exit` response
   while CAN and motor workers remain alive, publishes final group `Shutdown`
   events, and joins the group workers;
   enabled AGX emergency-damping workers then restore category 0 and firmware
   latching before relinquishing the UART;
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

Use `RUST_LOG=bello_hardwared=debug` for daemon diagnostics. For emulator
diagnostics, set `RUST_LOG=debug` in the emulator terminal. Avoid debug logging on
a production control path until its timing impact has been evaluated.

## Re-zero a motor or update its firmware

Support the mechanism at its intended motor-zero pose before disabling torque. Disable and release every group sharing the target CAN bus. Stop other CAN tools and hand drivers on that bus. Hardwared keeps its adapter open throughout maintenance. A group spanning two buses stays unavailable for motion while either bus is reserved.

Use `MotorConfigurationService` on a loopback TCP, Unix socket, or mutually authenticated TLS listener. For example, from the repository root with the Milly simulator running on localhost:

```sh
motor_config() {
  grpcurl -plaintext -import-path components/bello-hardware-api/proto \
    -proto bello/hardware/v1/hardware.proto -d "$2" 127.0.0.1:50051 \
    "bello.hardware.v1.MotorConfigurationService/$1"
}
motor_config PrepareMotorConfiguration '{"motor":"joint_1","kind":"MOTOR_CONFIGURATION_KIND_ZERO"}'
```

Use the returned ID in `GetMotorConfiguration`. At `READY`, review the motor UUID and shaft angle. Commit sets the motor's current position to zero. For firmware, prepare with `MOTOR_CONFIGURATION_KIND_FIRMWARE` and review the installed and target versions. The server caches the setup tool’s pinned release catalog and verifies the image digest for the detected motor family. The simulated CAN transport uses a small fixture image and the emulator keeps its calibration and firmware version in memory.

```sh
motor_config GetMotorConfiguration '{"id":"<operation ID>"}'
motor_config CommitMotorConfiguration '{"id":"<operation ID>"}'
motor_config GetMotorConfiguration '{"id":"<operation ID>"}'
```

A prepared operation expires after two minutes. Use `CancelMotorConfiguration` to release it earlier. After committing, poll the same ID until the result is terminal. Disconnecting the client leaves the write running. Check the group state before acquiring a new motion lease.

If the result is `UNVERIFIED`, stop hardwared and leave the motor powered. Run the following command under the same service account and state directory. It checks the recorded UUID, firmware, and stopped feedback. Re-zero also requires a near-zero reading. It clears the pending write only after verification succeeds.

```sh
bello-hardwared --config <deployment.yaml> --verify-motor-configuration
```

Hardwared stores pending writes under `bello-hardwared` in `XDG_STATE_HOME`, `LOCALAPPDATA`, or `~/.local/state`. Restarting with another profile using the same transport still checks that record. If the motor cannot boot its application after an interrupted flash, use the vendor's bootloader recovery procedure before running verification again.
