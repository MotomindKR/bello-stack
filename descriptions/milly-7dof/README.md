# Milly 7-DOF profiles

These profiles add one RS10P upper-arm roll joint between the original Milly's
shoulder joint 2 and elbow joint 3. Seven arm coordinates plus one gripper motor
are exposed per arm. The original `milly` and `milly-dual` profiles are preserved.

| Original coordinate / CAN ID | New coordinate / CAN ID | Motor |
| --- | --- | --- |
| joint_1 / 1 | joint_1 / 1 | Original RS06 |
| joint_2 / 2 | joint_2 / 2 | Original RS03 |
| Added rotation | joint_3 / 3 | RS10P |
| joint_3 / 3 | joint_4 / 4 | Original RS06 |
| joint_4 / 4 | joint_5 / 5 | Original RS00 |
| joint_5 / 5 | joint_6 / 6 | Original RS00 |
| joint_6 / 6 | joint_7 / 7 | Original RS05 |
| gripper / 7 | gripper / 8 | Original RS05 |

The dual profile applies the same ordering on `can0` and `can1`, with `left_`
and `right_` prefixes and one atomic `dual_arm` group. It inherits the existing
dual fixture mounting and center D405 geometry; this change does not add the
future stereo-camera gimbal or update the workcell's measured geometry.
The separate [combined gimbal profile](../milly-7dof-gimbal/README.md) uses the
user-requested 450 mm arm-base spacing and a centred gimbal mounting plate.

## Kinematics and inherited assets

The separately supplied `Milly 7dof all in 1.STEP` was inspected read-only.
Its SHA-256 is `6e1c9937388fddd1f1f73b7dbbdb62402267c39aa77c835aa695c78f28abcb64`.
The RS10P and adjoining shoulder cylinders identify an upper-arm roll axis
perpendicular to the shoulder/elbow hinges. In the existing `link_2` frame the
nominal axis is `(-1, 0, 0)` through `(0, 0, 0.097892511)` metres. The origin is
chosen on the axis at its shoulder projection; choosing another point along
the same line does not change the resulting revolute motion.

`joint_3` drives `link_3`, which now carries the upper-arm cover from the
7-DOF STEP. The old elbow and downstream links are renumbered and attached
below it. With `joint_3 = 0` and `joint_7 = 0`, all original link poses and
passive gripper closure are preserved, including the wrist camera.

J7 now rotates about the motor shaft: the nominal line in `link_6` has axis
`(1, 0, 0)` through `(-0.068805556, 0, 0.0235)` metres. The inherited axis
was parallel but offset by about 1.935 mm. `joint_7` drives `link_7_rotor`;
a fixed mount retains `link_7` as the original payload/inertia reference frame.
Existing poses are preserved at zero J7, while nonzero motion follows the
corrected shaft. Measurements and sources are in `kinematic-alignment.json`.

The arm's visual meshes come from the exterior parts in that STEP:
base case and mount, connector, shoulder spacer, shoulder/RS10P case,
roll-driven upper-arm case, forearm covers, elbow spacer and wrist covers.
The exposed elbow and wrist frames, actuator housings and roll output connector
are included: selecting covers alone left these joints visibly disconnected.
The J2 aluminum frames, shoulder mounting plates, plastic case holders and
RS10P support bracket are included. The plastic holders between the elbow
covers and aluminum frames are on `link_3`, the stationary side of J4.
J7's motor housing is on `link_6`, with its output plate and limit frame on
the rotating payload.

The plastic J6 button cap comes from the newly supplied
`Milly_Assem_0914_Case Only.step`, SHA-256
`dec06437d1785fb4f75cf56fc3306d4bf5ed0d29e5a6da4ed5e61a4a62fa753f`.
It is part of that file's wrist-cover body; subtracting the aligned 7-DOF
cover isolates the cap at the square aperture. The same 6-DOF source supplies
the missing J7 motor exterior. The PCB and board-mounted switch visuals were
removed. Motor internals, bearings, fasteners and electronics are omitted.

Both gripper carriages, rigid finger mounts and DHAS-GF-60 soft pads now use
the 7-DOF STEP geometry. Their undeformed CAD shapes move rigidly with the
existing finger slider links. The existing gripper body, rail, linkage and
wrist camera visuals are retained. Both single- and dual-arm descriptions
share the new assets under `meshes/visual/covers/`, `meshes/visual/exterior/`
and `meshes/visual/gripper/`.

Meshes are tessellated at 0.10 mm linear / 0.15 rad angular deflection and
stored in metres in their owning link's zero-pose frame. The source assembly
is registered using the base yaw and shoulder/RS10P axes. Part labels, source
paths, transforms and mesh hashes are recorded in `visual-assets.json`.

All collision geometry uses primitives; meshes are used only for visuals.
The old upper-arm collision layout has been replaced by a shoulder cylinder
and roll housing box on `link_2`, plus one upper-arm box on the rotating
`link_3`. The inherited 69 mm shoulder motor cylinder on `link_1` is
omitted because it duplicates the coaxial, same-radius 96 mm cylinder on
`link_2`. The remaining cylinder is symmetric about J2, so it covers that
housing throughout J2 travel. The two J4 fork side boxes are omitted at the user's request; the
elbow motor cylinder on `link_4` is extended to 90 mm along its axis to cover
the full elbow width. Each wrist camera has only its
camera housing box, with no stalk or mounting collider. Other collision
shapes are inherited. A complete inventory is recorded in
`collision-primitives.json`. Small overlaps between adjoining primitives
approximate continuous housings; these are approximate exterior envelopes,
not calibrated contact surfaces.
Each new finger assembly has one box envelope; compliant pad deformation is
not modeled.

Arm mass, centre of mass and inertia tensors follow the teammate's
`milly-urdf-with-meshes.zip` delivery, including the added roll link. Each arm
has a modelled mass of 6.803799249 kg, including its fixed base. Source hashes,
per-link values and import scope are recorded in `physical-parameters.json`.
Superseded auxiliary gripper mass estimates have been removed; auxiliary frames
without supplied inertia remain massless. The J7 payload inertia stays in the
rigidly attached `link_7` frame.

Only inertial elements were imported. Existing kinematics, visuals, primitive
collisions, SDK limits and the nonlinear gripper driver convention are retained.
The delivery's different finger coordinate convention and J7 limits are not
part of this physical-parameter update. Native simulation adapters live outside
`descriptions/`; this directory contains authored description data only.

The URDF and hardware YAML files are maintained directly; `descriptions/`
contains no generator scripts. Shared geometry and motor ordering must be kept
consistent across single- and dual-arm descriptions when editing them. Native
tests outside this directory verify the kinematic insertion and motor ordering.

## SDK position limits

The original six joints and gripper follow the
[Milly SDK 1.1.2 position ranges](https://github.com/MotomindKR/motomind_milly_sdk_release/blob/5ffc17f525535b433241a28252b5e685278d51e8/milly_description/README.md),
pinned to commit `5ffc17f525535b433241a28252b5e685278d51e8`.
The SDK publishes motor-raw radians. These profiles retain their existing
`direction` values and use `q_urdf = direction * q_motor_raw`, sorting the
transformed endpoints. SDK joints 3..6 map to joints 4..7 after the insertion.
Provenance, raw values and converted values are in `sdk-limits.json`.

| Coordinate | Command range (rad) | Measured-position range (rad) |
| --- | --- | --- |
| joint_1 | -2.80 to 2.80 | -2.95 to 2.95 |
| joint_2 | 0.00 to 3.20 | -0.15 to 3.35 |
| joint_3 (added) | -pi/2 to pi/2 | -pi/2 to pi/2 |
| joint_4 | 0.00 to 2.75 | -0.15 to 2.90 |
| joint_5 | -2.42 to 2.42 | -2.57 to 2.57 |
| joint_6 | -1.70 to 1.84 | -1.85 to 1.99 |
| joint_7 | -1.55 to 1.55 | -1.70 to 1.70 |
| gripper | -2.11 to 0.00 | -2.26 to 0.15 |

Hardware command limits use the narrower column; position health checks use
the measured-position column. URDF hard limits retain the measured envelope
because bello-hardwared intersects health limits with URDF limits. Using the
command range there would silently remove the SDK's 0.15 rad feedback margin.
The local preview sliders use hardware command bounds. Existing velocity,
feedforward torque, gain, temperature and command policies are unchanged.

## Commissioning values

Joint 3 has user-specified travel of **-90 to +90 degrees** (`[-pi/2, pi/2]`
rad). The URDF, hardware command limits and position health limits use this
range on each arm.

The added J3 motor uses `direction: -1` in the single-arm, dual-arm and combined
gimbal profiles, corrected from the original provisional +1 after hardware
commissioning. Hardwared applies this sign to feedback and commands; the URDF
axis and mechanical encoder zero do not change.
The 0.2 rad/s command velocity, 2 N m feedforward torque cap and other initial
control settings remain commissioning placeholders, not total closed-loop
torque guarantees. Confirm zero, direction, gains and damping before enabling
hardware.

The shifted CAN IDs are configuration changes only. Existing physical motors
must be assigned their new IDs separately; starting a profile does not readdress
devices. All original motor models and directions are retained after shifting.

RS10P support uses the manufacturer's CAN 2.0 **private extended-CAN protocol**,
not CAN-FD or the separate standard-ID MIT protocol. The shared model catalog
uses velocity +/-13 rad/s, torque +/-42 N m, Kp 0..500 and Kd 0..5. Electrical
metadata is 125 rpm no-load speed, 48 V rated voltage and 19 A peak phase current.
The existing shared +/-12.57 rad position encoding convention is retained.
Source: [RS10P CAN 2.0 manual 260907, sections 1.2/1.3 and 4.1.2](https://github.com/RobStride/Product_Information/blob/40d43fdd15d397a446c70e550d1fd0a3700c2c44/Product%20Literature/CAN2.0/10P/RS10PUser%20Manual260907.pdf).

## Entry points

| Component | Single arm | Dual arm |
| --- | --- | --- |
| Description | `hardware.yaml` | `hardware-dual.yaml` |
| URDF | `urdf/milly-7dof.urdf` | `urdf/milly-7dof-dual.urdf` |
| Hardware daemon | `config/milly-7dof.yaml` | `config/milly-7dof-dual.yaml` |
| vCAN daemon | `config/milly-7dof-vcan.yaml` | `config/milly-7dof-dual-vcan.yaml` |
| Visualizer | `config/milly-7dof.example.yaml` | `config/milly-7dof-dual.example.yaml` |
| Protocol emulator | `config/milly-7dof-vcan.yaml` | `config/milly-7dof-dual-can0-vcan.yaml`, `config/milly-7dof-dual-can1-vcan.yaml` |

Daemon, visualizer and emulator paths are relative to their respective
`components/bello-hardwared`, `components/bello-visualizer` and
`components/robstride-emulator` directories.

Validate without opening hardware:

```sh
cargo run -p bello-hardwared -- --check --config components/bello-hardwared/config/milly-7dof.yaml
cargo run -p bello-hardwared -- --check --config components/bello-hardwared/config/milly-7dof-dual.yaml
```
