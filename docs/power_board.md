# Power-board telemetry

COG carries power-board telemetry. The optional AGX `emergency_stop: damping`
mode also hands the physical emergency button to hardwared while it is running.
The default `emergency_stop: hardware` monitors without changing category or latch.
Neither mode clears software emergency requests or directly writes power rails.
Battery current retains the firmware's sign; GPIO values are raw electrical
levels. Do not infer logical emergency state from a generic nonzero GPIO mask.

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
before starting hardwared; hardwared owns telemetry and the sync heartbeat.
The Bello physical deployment leaves it disabled. To enable
it on another deployment, verify the board/profile/port and add both entries:

```yaml
enabled_power_boards: [power_board]
serial_ports:
  # Retain the existing IMU mapping too.
  power_board_serial: /dev/serial/by-id/REPLACE_WITH_VERIFIED_DEVICE
```

Validate with `bello-hardwared --check --config PATH` before opening hardware.
Confirm the serial speed, sync period and stale timeout against the installed board.
In hardware mode it sends only `sync\r\n`, immediately after connection and
then periodically;
the reference host-valid timeout is five seconds. Set `sync_period_ms: 0`
in the description for passive monitoring. Nano/NX is read-only (period zero).
Heartbeat-loss effects on physical power are firmware-dependent and are not
assumed safe. One configured sensor owns each `SerialHandle`; power boards use
the same serial reconnect infrastructure as the IMU, not a second port opener.

## API and telemetry

`ListHardware` includes board identity, transport, profile and status.
`WatchTelemetry` uses `include_power_boards` and optional `power_boards`
selectors. Each snapshot contains at most one sample per packet type, its own
age/stale flag, connection generation and error counts. Fresh tick packets do
not refresh old battery data. Reconnection clears prior samples.
Each decoded known COG packet publishes a snapshot immediately, including
unchanged readings with a new receive time. Packets received together are
published individually so intermediate GPIO and IWC transitions are preserved.
There is no fixed publication-rate cap for incoming data. When no known packets
arrive, snapshots continue every 250 ms to report age and connection status.
The serial receive wait wakes on incoming bytes; its 20 ms timeout is not an
added delay per packet. Delivery still depends on host scheduling and subscriber
capacity; lagging gRPC subscribers receive a dropped-event report.

The 10 Hz sync requests GPIO and IWC snapshots; it does not configure the
firmware's periodic sensor sampling or transmission rates. Forwarding more
frequently cannot make battery, rail, or temperature measurements fresher than
the board provides. Verify the installed firmware's sustained response rate on
hardware before relying on a particular update frequency.

Rust `Robot::power_boards()` returns metadata; `Robot::states()` includes latest
board snapshots. Python `robot.power().read(timeout=1)` returns bounded
snapshots without acquiring control. Rust sample readings use the generated
`PowerReading` variants, such as `PowerReading::Battery(battery)`. Python
`sample.reading` holds a Python reading object, such as `PowerBattery`, or an
integer tick count. `PowerBattery.soc_percent` and `PowerGpi.second_bits`
are `None` when the board omits them.

## Physical emergency button and damping

The Bello and Mini hardware descriptions select `emergency_stop: damping`.
It applies only when that power board is enabled in the deployment. At most one
board can own this mode; it requires AGX, a nonzero sync period, and a stale
timeout below the firmware's five-second host-valid timeout.

On each UART connection hardwared inhibits new leases, requests
`iwc emergency stop_category 0`, and sets `iwc emergency latch 0`. This prevents
a physical press from also latching a firmware software-stop request. Hardwared
does not send `iwc emergency release` or clear an existing software stop.
Only fresh IWC with all three emergency bits clear permits the request
`iwc emergency stop_category 1` and new leases. Heartbeats then keep the
hardware cutoff path bypassed. UART write completion is not firmware
acknowledgement: the documented protocol exposes no acknowledgement or readback
of the category/latch settings. Verify this handoff with the installed firmware.

Any effective, hardware, or software emergency bit revokes every group lease,
invalidates retained periodic CAN targets, and wakes every group controller.
Enabled groups clear their command state and apply their existing damping
response. Idle motors remain disabled; faulted groups retain their fault while
attempting damping. This includes every motor group enabled in hardwared, not
independent hand controllers or devices owned by another process. Damping
delivery failures remain explicit faults; successful host transmission does
not prove physical braking.

Release never resumes old commands. Once status is fresh and released and
controllers have handled the stop, clients must acquire new leases and
explicitly start control. A missing/disconnected board or stale IWC also
revokes leases and requests damping. Fresh battery/tick packets do not hide
stale IWC. Reconnection repeats the guarded handoff.

Orderly shutdown first stops group control, then sends
`iwc emergency stop_category 0` before closing the UART. Startup failures and
worker panic unwinding also attempt restoration. Cleanup additionally sends
`iwc emergency latch 1` to restore latched firmware emergency behavior.
If the restoration write fails, hardwared logs the failure and stops
heartbeats. SIGKILL, process/host failure, and loss of UART cannot guarantee an
immediate restoration command: the documented board behavior re-enables the
hardware cutoff path when host validity expires, up to five seconds after the
last sync. This re-enables the button's cutoff path; it is not an unconditional
power-off on daemon loss. An independent verified emergency-stop mechanism is
required when using host-controlled damping as an operational stop.

## Native emulation

[bello-sim](../components/bello-sim/) publishes synthetic AGX readings on
the power UART. See the [operations guide](operations.md) for virtual-device setup.

Use masks to test warning
telemetry. `mute_telemetry: true` tests stale or disconnected telemetry. A
`null` block disables the emulator endpoint. To use it, disable the board
and remove its binding in the matching hardwared deployment.

Packets are emitted once per wall-clock second, independently of simulation
pose resets. Fragmented sync commands update the five-second internal
host-valid state.
