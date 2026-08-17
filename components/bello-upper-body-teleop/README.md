# Bello upper-body teleoperation

`bello-upper-body-teleop` reads the two Dynamixel teaching arms and commands the
`upper_body` motor group through the supported `bello-hardware` Python client.
It does not open CAN or talk to Robstride motors directly:

```text
Dynamixel teaching arms
  -> this process
  -> bello-hardware gRPC client
  -> bello-hardwared upper_body lease -> 12 tracked Robstride motors on can1
```

This program can move physical hardware. Keep the robot supported, clear its
workspace, keep an emergency stop within reach, and validate the teaching-arm
readings with `--dry-run` before using `--enable`.

## Supported morphology

The checked-in mapping matches the six-joint-per-arm sphere-end-effector
topology in `components/bello-hardwared/config/bello.yaml`:

| Teaching-arm ID | Daemon coordinate |
| ---: | --- |
| 21 | `left_shoulder_pitch_joint` |
| 22 | `left_shoulder_roll_joint` |
| 23 | `left_shoulder_yaw_joint` |
| 24 | `left_elbow_pitch_joint` |
| 25 | `left_elbow_yaw_joint` |
| 26 | `left_wrist_pitch_joint` |
| 11 | `right_shoulder_pitch_joint` |
| 12 | `right_shoulder_roll_joint` |
| 13 | `right_shoulder_yaw_joint` |
| 14 | `right_elbow_pitch_joint` |
| 15 | `right_elbow_yaw_joint` |
| 16 | `right_wrist_pitch_joint` |

The configured default and weak-feedback pose is zero for ten mapped joints
and `−π/2` for both elbow-yaw joints. These symmetric defaults remain inside
their shared asymmetric joint limits and map back through the same teaching-arm
calibration.

The original teaching rig also has three movable joints that do not map to the
daemon's 12-coordinate `upper_body` group. They are managed as local
configured-position locks:

| Teaching-rig ID | Local behavior |
| ---: | --- |
| 9 | lock waist yaw at calibrated zero (raw 512) |
| 27 | lock left auxiliary wrist 180° opposite calibrated zero (raw 4096, modulo raw 0) |
| 17 | lock right auxiliary wrist 180° opposite calibrated zero (raw 4096, modulo raw 0) |

These three positions never become robot commands. The two gripper Dynamixels
(IDs 28 and 18) remain unmanaged. Startup rejects any daemon group whose
coordinate set does not exactly match the 12 mapped joints above.
Because encoder position is periodic, the controller chooses the nearest
4096-count equivalent of each listed goal. For example, raw 0, 4096, and 8192
are the same physical auxiliary target, so the controller avoids commanding an
unnecessary full turn from a nearby multi-turn reading.

Offsets, directions, and action limits come from the previous
`BelloJointTeleopControllerConfig`. At startup, an editable checkout parses the
repository's single canonical robot description,
`descriptions/bello/full_body/urdf/bello_full_body.urdf`, and intersects each
teaching limit with that joint's URDF position range. It fails startup if that
file is missing, malformed, or does not define a mapped revolute joint. Every
sample is clamped to the resulting range, and target-frame construction rejects
positions outside it before they can become robot commands.
Standalone distributions embed that same canonical URDF at build time and
parse the packaged snapshot together with its package-specific provenance
notice. The notice records the exact `bello-stack` commit and URDF digest, and
the distribution contains no adjacent mesh files. Building fails if the
checked-out URDF differs from the recorded commit. Wheels rebuilt from a source
distribution preserve its original snapshot and provenance rather than
searching for an external asset checkout.
Before reading the teaching arms or acquiring a lease, startup also requires
each effective range to fit within the matching motor position range reported
by the live daemon inventory.

The robot impedance gains match MotoMind's deployed upper-body hold tuning:
shoulder pitch/roll use `kp=70, kd=2`, shoulder yaw `20/1`, elbow pitch `40/2`,
elbow yaw `20/1`, and wrist pitch `10/1`. Both arms use the same gains.

## Install

Create a dedicated environment and install both adjacent components:

```sh
cd /path/to/bello_stack
python3 -m venv .venv-teleop
. .venv-teleop/bin/activate
python -m pip install \
  -e components/bello-hardware-python \
  -e components/bello-upper-body-teleop
```

Editable installations read the canonical URDF directly from the complete
`bello_stack` checkout. A built wheel is self-contained and carries the exact
URDF snapshot present when the wheel was built.

This installs the Robotis `dynamixel-sdk` dependency. The default teaching-arm
connection is `/dev/ttyUSB0` at 1,000,000 baud.

When working inside the repository's Nix/direnv shell, reload the shell after
pulling changes to `flake.nix`:

```sh
deactivate 2>/dev/null || true
direnv reload
. .venv-teleop/bin/activate
```

The shell supplies the C++ runtime required by pip's binary `grpcio` wheel. If
an already-open shell reports `ImportError: libstdc++.so.6`, the reload fixes it
without recreating or reinstalling the virtual environment.

## Validate teaching-arm input

The dry run never connects to `bello-hardwared`:

```sh
bello-upper-body-teleop --dry-run --duration 5
```

It prints calibrated positions as JSON. Confirm that all 12 positions move in
the expected direction and that `clipped_coordinates` is normally empty.
Communication failure is fatal; there is no fake-device fallback.

`--dry-run` keeps the teaching rig passive by default. The reader sends one
broadcast torque-disable and then reads every managed torque-enable register
back, requiring all 15 values to be zero.

`--enable` turns on weak teaching-arm centering torque by default. Startup first
disables torque, verifies current-based position mode on every Dynamixel,
checks that each configured hardware current limit permits the 800-unit cap,
and captures all 15 present positions into Goal Position. It then replaces any
stale gains, writes Goal Current 800, enables torque, and verifies every
torque-enable register.

After torque-on, every acquisition cycle writes the constant configured
robot-space default pose: both elbow-yaw joints at `−π/2`, with the other ten
mapped joints at zero. The weak `P=10` gain is used inside
the configured range. At either joint limit it switches to the original 10×
`P=100` virtual-stop gain while Goal Position remains at that joint's default,
resisting any further travel beyond the limit. In the same transaction, ID 9
continually receives calibrated zero while IDs 27 and 17 receive their
180°-opposite wrist targets, all with `P=250`. All gains use `I=D=0`. Torque is
broadcast-disabled and verified when the program exits. The overrides are:

```sh
# Keep the control run passive.
bello-upper-body-teleop --enable --disable-master-torque

# Exercise weak configured-pose centering without connecting to the robot daemon.
bello-upper-body-teleop --dry-run --enable-master-torque --duration 5

# Expert-only: do not read, change, or manage the existing torque state.
bello-upper-body-teleop --dry-run --leave-master-torque --duration 5
```

## Control the upper body

Start `bello-hardwared` with the physical configuration, then explicitly
prepare the interactive handoff:

```sh
bello-upper-body-teleop \
  --enable \
  --endpoint http://127.0.0.1:50051
```

The startup and handoff sequence is:

1. connects to the daemon, safely configures and enables all 15 managed
   teaching-rig Dynamixels, locks the three local-only joints at their
   configured positions, and reads one complete 12-joint command sample;
2. verifies that the daemon's `upper_body` coordinates exactly match the
   teaching-arm mapping;
3. waits for the operator to type `HOLD`;
4. obtains fresh feedback from every upper-body motor, acquires the exclusive
   group lease, and starts control at those measured positions with zero
   proportional gain;
5. continuously commands those same poses while ramping proportional gains
   over five seconds;
6. keeps streaming the held pose while waiting for the operator to type
   `TELEOP`;
7. snapshots the teaching-arm pose at that second confirmation and linearly
   interpolates to it over at least five seconds; and
8. begins live tracking only after the interpolation completes.

Typing anything other than the requested token cancels the operation. The
second prompt is non-blocking internally: command acknowledgements and hold
frames continue at the configured control rate while the application waits.
The transition duration is automatically extended when necessary to respect
the 0.5 rad/s per-joint velocity limit.

Useful overrides include:

```sh
bello-upper-body-teleop --enable \
  --port /dev/ttyUSB1 \
  --rate 30 \
  --gain-ramp 8 \
  --transition-duration 10 \
  --max-velocity 0.25 \
  --duration 60
```

Press Ctrl-C to stop. Any Dynamixel, gRPC, lease, acknowledgement, or command
failure unwinds the control session; the client disables teaching-arm torque,
then stops both active control groups, disables them, and releases their
leases. The daemon remains authoritative for command validity, motor health,
timeouts, and fault reactions.

## Development

The unit tests use an injected fake Dynamixel SDK and never open hardware:

```sh
cd components/bello-upper-body-teleop
ruff format --check .
ruff check .
mypy src/bello_upper_body_teleop tests
pytest
python -m hatchling build
```
