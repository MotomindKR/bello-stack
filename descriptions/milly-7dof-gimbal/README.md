# Dual 7-DOF Milly with stereo gimbal

Combined description of both accepted 7-DOF arms and the three-axis stereo
gimbal. All visual meshes are referenced from the standalone descriptions;
this directory contains no duplicate meshes or generators.

The user requested 450 mm between the arm bases and the gimbal at their
midpoint. The inherited arm mounting rotations are retained. In
`dual_milly_base`, the arm base frames are `(0, -0.225, 0)` and
`(0, 0.225, 0)`, each with yaw `pi/2`.

The physical arm mounting plates are offset from those URDF base frames:
their underside centres are `(0.045, -0.225, 0.004)` and
`(0.045, 0.225, 0.004)` metres. The gimbal base is mounted at
`(0.0775, 0, 0.423)` with yaw `pi`: it is centred 419 mm above the arm mounting
plane. The level stereo optical frame is at `z=0.604 m`, 600 mm above that
plane, with the camera facing the arm workspace (-X at zero).
`mounting.json` records these frames.
Both J1 axes are at `x=0.0375 m`, `y=+/-0.225 m`, `z=0.072 m`.
The gimbal base centre is 40 mm behind their transverse centreline, along +X
(the camera faces -X). Its support and safety envelope follow this offset.
The square support pole is a 60 x 60 mm box from ground `z=0` to the gimbal
plate underside at `z=0.423 m`, with a matching box collider. Its cross-section
is provisional; no mass or inertia is assigned.
The combined workcell also includes `ground_link`, a fixed 2 x 2 m ground
surface represented by a 20 mm thick visual and collision box. Its top is
`z=0` and its bottom is `z=-0.020 m` in `dual_milly_base`. This fixture is kept
in the combined model so the standalone gimbal can be mounted elsewhere.
The old centre D405, keepout and support post are omitted. Arm-mounted wrist
cameras remain.

The optional coarse gimbal safety sphere is defined in
`../stereo-gimbal/safety-envelope.json` and its companion proxy URDF. It has a
132 mm radius, is fixed to `gimbal_base_link` at `(0, 0, 0.174)`, and therefore
has centre `(0.0775, 0, 0.597)` in `dual_milly_base`. It covers the moving
assembly throughout the three confirmed joint ranges, with at least 3.48 mm
clearance to the current model. The fixed base plate/yaw stator and support
pole retain their own primitives. Use the sphere in place of only its covered
moving exterior bodies; this proxy does not check internal gimbal collisions.

The browser preview can display all four stereo camera frames, or isolate the
body, center optical, left optical or right optical frame. The markers use the
URDF transforms and move with the gimbal; X is red, Y green and Z blue. The
Focus camera button zooms to them without changing the joint angles.

The hardware description exposes 19 motor coordinates in two independent
atomic groups:

| Group | Coordinates | Bus / IDs |
| --- | --- | --- |
| dual_arm | 7 arm joints and 1 gripper per arm | left: can0 / 1..8; right: can1 / 1..8 |
| gimbal | yaw, pitch, roll | can0 / 11, 12, 13 |

One hardware daemon owns both buses. The gimbal inherits the confirmed +/-90
degree travel and commissioning caps from its standalone profile. Arm position
bounds retain the SDK mapping described in `../milly-7dof/README.md`.

All 19 `fault_damping_kd` values are half their previous commissioning values.
The three gimbal motors now use 0.25, matching the standalone gimbal profile;
the arm values are documented in `../milly-7dof/README.md`. Normal tracking
gains are configured by each controller.

All three gimbal motors use the user-confirmed `direction: -1` mapping from
actuator raw angles to URDF joint angles. Encoder zeros remain provisional.
No gimbal masses or inertias have been assigned. The CAD check in
`../stereo-gimbal/clearance-check.json` finds camera-case interference with the
rear yaw support at pitch +90 degrees and roll +/-90 degrees. The independent
joint bounds do not enforce a collision-free coupled workspace. Resolve that
interference with coupled motion bounds or mechanical changes before full-range
head tracking.

Both arms use the supplied mass/COM/inertia values documented in
`../milly-7dof/physical-parameters.json`; each arm is 6.803799249 kg including
its fixed base. Arm-only dynamics can freeze the unmodelled gimbal without
inventing gimbal inertials. Simulation adapters belong outside this description
directory.

Repository entry points:

- `descriptions/milly-7dof-gimbal/urdf/milly-7dof-gimbal.urdf`
- `descriptions/milly-7dof-gimbal/hardware.yaml`
- `components/bello-hardwared/config/milly-7dof-gimbal.yaml`
- `components/bello-hardwared/config/milly-7dof-gimbal-vcan.yaml`
- `components/bello-visualizer/config/milly-7dof-gimbal.example.yaml`
- `components/robstride-emulator/config/milly-7dof-gimbal-can0-vcan.yaml`
- `components/robstride-emulator/config/milly-7dof-gimbal-can1-vcan.yaml`
