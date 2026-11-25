# Bello hand component

`bello-hand` is a standalone Linux SocketCAN client. Its first driver supports the Linker O6. It reads the existing arm deployment, follows its `description` path, and loads the sibling [hand inventory](../../descriptions/milly-7dof-o6/hands.yaml). The arm deployment supplies the actual interface names for both physical and virtual CAN. The hand process opens filtered sockets on interfaces that the host has already brought up.

## Run

From the repository root:

```sh
python3 -m pip install ./components/bello-hand
deployment=components/bello-hardwared/config/milly-7dof-dual-o6.yaml
bello-hand --config "$deployment" check
bello-hand --config "$deployment" status left
```

`check` resolves the endpoints without opening a socket. `status` queries six positions and six motor fault bytes. Each response includes the host monotonic receipt time in nanoseconds. The standard CAN defaults are left `0x28` on `can0` and right `0x27` on `can1`. Confirm these addresses against the installed firmware.

`set SIDE P0 P1 P2 P3 P4 P5` accepts six raw position bytes in the vendor's order: thumb flexion, thumb spread, index, middle, ring and little finger flexion. All values are integers from 0 to 255. Each write requires a fresh all-zero fault response. `target_queued` reports that the kernel accepted the frame. Use a subsequent `status` to observe hand motion. The [vendor O6 driver](https://github.com/linker-bot/linkerhand-python-sdk/blob/0cc0585b97214b2cc4a9a5afcc84aee9f414e0e8/LinkerHand/core/can/linker_hand_o6_can.py) defines the registers and axis order.

## Shared-bus behavior

The component sends traffic on demand and spaces its outgoing frames at least 20 ms apart per hand. A status or set operation uses two outgoing frames. At 50 outgoing frames per second plus one reply per frame, the existing 150-bit-per-frame budgeting convention gives an additional 1.5% per 1 Mbit/s bus. This estimate excludes unsolicited traffic. Measure bus load and arm feedback latency with the installed hands.

An abstract Unix socket gives one component session ownership of each hand address on its interface. An I/O failure invalidates the session. Reopening starts idle and requires a fresh target. Other programs using raw CAN sockets can still transmit on the bus. The hand can retain its last target after the process exits or a bus fails. Coordinated stop, arm leases, combined bus scheduling and hardware qualification are tracked in [issue #73](https://github.com/MotomindKR/bello-stack/issues/73).

## Linux virtual CAN test

Run `sudo bash components/bello-hand/tests/run-socketcan.sh` with Python 3.10+, PyYAML, iproute2 and the Linux vxcan module available. The test creates an isolated network namespace with two pairs of CAN interfaces. Each simulated hand replies through its peer endpoint so the driver can distinguish device responses from locally echoed commands.
