# Bello Mini Torso

The fixed-base torso retains both seven-joint arms, waist yaw/roll, pelvis,
head, camera, and IMU. The two active hip-pitch motors drive one shared pitch
joint. Including the two grippers, 20 actuators control 19 independent joints
and eight passive linkage joints. The user specified an invisible
base at the hip-pitch origin, upright zero, 60 degrees forward travel, and
30 degrees backward travel. The hands are replaced by articulated Milly grippers,
including Milly's wrist camera brackets, collision shapes and optical
frames on both sides. Modeled moving mass is approximately 28.177 kg with the provisional
gripper mass allocation described in the provenance.

Geometry, coordinate conventions, inherited calibration assumptions, and torque
sharing are recorded in [ASSET_PROVENANCE.md](ASSET_PROVENANCE.md).

## Gripper control

| Coordinate and motor group | Logical bus | Motor ID |
| --- | --- | --- |
| `left_gripper` | `can2` | 38 |
| `right_gripper` | `can3` | 48 |

Each gripper has its own lease and uses Milly's motor configuration
(`ROBSTRIDE_05`, direction -1) and 100 Hz control/refresh rates. Its commanded
range is -2.3 to 0 rad; -2.2 rad is the nominal open pose and 0 rad is closed.
The emulator initializes both hands open. The physical daemon reads their
actual positions; declaring the IDs does not program motor addresses or zeros.
Confirm the installed motor model, CAN address, direction, and zero before use.

Each URDF `bello:slider_crank` declaration preserves Milly's crank radius,
rod length, slider offset, phase and reference. Only the crank is a motor
coordinate; both fingers and rods are passive. The viewer and native emulator
resolve these linkages independently for each hand. Arm IK consumers can hold
the gripper coordinates at the measured or desired opening.

## Validate without hardware

From the stack root in its Linux Nix environment:

```sh
cargo run -p bello-hardwared -- --check \
  --config components/bello-hardwared/config/bello-mini-torso.yaml
cargo run -p bello-hardwared -- --check \
  --config components/bello-hardwared/config/bello-mini-torso-vcan.yaml
cargo run -p bello-mujoco -- check \
  --config components/bello-mujoco/config/bello_mini_torso.yaml
```

The model uses `root: fixed` and `floor: false`; the base root is at world zero
and has no visual or collision geometry. There is no free-root joint or torso
guide. Both hip actuators have signed unit mappings onto `torso_pitch_joint`.
Native torque saturation retains the intersection of the URDF motor effort
envelope and the protocol model range through emulator updates.

## Virtual bring-up

Prepare `vcan0` through `vcan3`, an IMU UART pair at `/dev/ttyIMU` and
`/dev/ttyIMU-mujoco`, and a power UART pair at `/dev/ttyPOWER` and
`/dev/ttyPOWER-mujoco`, following [operations](../../docs/operations.md).
Run the emulator and daemon in separate Linux terminals:

```sh
cargo run -p bello-mujoco -- run \
  --config components/bello-mujoco/config/emulator_mini_torso.yaml
cargo run -p bello-hardwared -- \
  --config components/bello-hardwared/config/bello-mini-torso-vcan.yaml
```

Open the viewer on the same host, or edit the hardware endpoint in a local
copy of the example when using a remote/VM daemon:

```sh
cargo run -p bello-visualizer -- \
  --config components/bello-visualizer/config/bello-mini-torso.example.yaml
```

The `torso_pitch` editor exposes one logical pitch angle and distributes joint
torque to both hip motors. Gains remain per-motor values; its defaults are
50/2 per hip. Both motors must enable and move together. A contradictory
position or velocity command is rejected. Encoder disagreement above 0.02 rad
faults the group and disables both hips. Missing/stale feedback follows the
existing group health and watchdog behavior. The waist and arms have separate
leases and editors.

## Physical commissioning and visual review

The physical profile inherits Mini's CAN addresses, motor directions, IMU USB
serial binding, and power UART. Confirm those bindings and encoder zeros on
the installed assembly. Configuration checks do not establish physical encoder
calibration or motor load sharing. Review the selected per-motor gains and
encoder-disagreement tolerance before enabling the physical group.

Local review images are written to the ignored
`outputs/bello_mini_torso_review/` directory. `review.png` shows front, side,
and three-quarter views plus the forward/backward limits. The review uses the
canonical GLB visuals, including the detailed head and open grippers. Native MuJoCo
video uses the retained companion STLs as described in the
[Mini provenance](../bello_mini/ASSET_PROVENANCE.md). Generated images and
inspection MJCF files are review artifacts rather than source descriptions.
