# Power-board telemetry

The COG integration is monitoring-only: it does not switch rails, shut down
the host, clear an emergency, or change motor safety policy. Battery current
retains the firmware's sign; GPIO values are raw electrical levels. Do not
infer logical emergency state from a generic nonzero GPIO mask.

Protocol source: Motomind's
[COG monitor implementation](https://github.com/MotomindKR/motomind_platform/tree/5d2ea60634c1d35a88e45961cce98143b55e4b0d/platform/ros2/src/motomind_ros_bringup).
The shared Rust `cog-protocol` crate supports AGX (COBS, u8 length, CRC16-CCITT)
and Nano/NX (COBS, u16 length, STM32 CRC32). AGX has battery SOC, GPIO,
info/warning/critical masks, environment, local rails, motor rail and tick.
Nano/NX has no SOC, and exposes its legacy GPIO/fault/encoder fields instead.
Unknown valid packets are counted and ignored; malformed/nonfinite packets
are rejected. Unknown flag bits retain explicit names and their raw mask.

## Physical deployment

Bello and Mini descriptions declare `power_board` on `power_board_serial`.
The Mini physical deployment enables it on the verified AGX Orin UART
`/dev/ttyTHS1` at 921600 baud. Stop any existing COG monitor on that port
before starting hardwared; hardwared now owns telemetry and the sync heartbeat.
The original Bello physical deployment still leaves it disabled. To enable
it on another deployment, verify the board/profile/port and add both entries:

```yaml
enabled_power_boards: [power_board]
serial_ports:
  # Retain the existing IMU mapping too.
  power_board_serial: /dev/serial/by-id/REPLACE_WITH_VERIFIED_DEVICE
```

Validate with `bello-hardwared --check --config PATH` before opening hardware.
The canonical declaration selects AGX at 921600 baud, a 1000 ms sync period,
and a 3000 ms stale timeout. Confirm these against the installed board.
AGX sends only `sync\r\n`, immediately after connection and then periodically;
the reference host-valid timeout is five seconds. Set `sync_period_ms: 0`
in the description for passive monitoring. Nano/NX is read-only (period zero).
Heartbeat-loss effects on physical power are firmware-dependent and are not
assumed safe. One configured sensor owns each `SerialHandle`; power boards use
the same serial reconnect infrastructure as the IMU, not a second port opener.

## API and visualization

`ListHardware` includes board identity, transport, profile and status.
`WatchTelemetry` uses `include_power_boards` and optional `power_boards`
selectors. Each snapshot contains at most one sample per packet type, its own
age/stale flag, connection generation and error counts. Fresh tick packets do
not refresh old battery data. Reconnection clears prior samples.
Motor/IMU `GetStateFrame` and command acceptance are unchanged.

Rust `Robot::power_boards()` returns metadata; `Robot::states()` includes latest
board snapshots. Python `robot.power().read(timeout=1)` returns bounded
snapshots without acquiring control. The native visualizer's **Power** tab
shows battery SOC, signed current, rails, GPIO, named status masks and sample
age. It keeps latest values, not an ever-growing measurement history.

## Native emulation

The emulator is **Rust `components/bello-mujoco`**, not the retired Python
emulator. Maintained runtime profiles enable a synthetic AGX fixture at
`/dev/ttyPOWER-mujoco`; hardwared's vCAN profiles read `/dev/ttyPOWER`.
The paired UARTs are created by `bello_mujoco/scripts/setup_emulator.sh start`.
They are separate from the existing IMU pair and require no TCP port forwarding.
The native Mac visualizer receives power data over its existing hardwared RPC.

The runtime's `power_board` block explicitly sets battery voltage/current/SOC,
motor rail, environment, six local-rail values (voltage/current pairs), raw GPIO,
and IWC masks. Defaults are a synthetic 48 V battery at 75% charge, not an
electrical model or measured robot consumption. Use masks to test warning UI;
`mute_telemetry: true` tests stale/disconnected telemetry. A `null` block disables
the emulator endpoint; also disable the board/remove its binding in the
matching hardwared deployment.

Packets are emitted once per wall-clock second, independently of simulation
pose resets. Fragmented sync commands update the five-second internal
host-valid state. No undocumented flags, relay behavior, or simulated motor
interlocks are invented. UART input work and output queues are bounded; an
unconsumed output queue fails explicitly rather than growing indefinitely.

Validation covers reference CRC vectors, profile mismatch, corruption,
fragmentation, real PTY I/O, heartbeat, stale individual samples, reconnect,
client validation and visualizer disconnect/reconnect. Physical board testing
is still required before commissioning.
