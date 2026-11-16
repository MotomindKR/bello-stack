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
`(0.045, 0, 0.304)` with yaw `pi`: it is centred 300 mm above the arm mounting
plane, with the stereo camera facing the arm workspace (-X at zero).
`mounting.json` records these frames.
The square support pole is a 60 x 60 mm box from ground `z=0` to the gimbal
plate underside at `z=0.304 m`, with a matching box collider. Its cross-section
is provisional; no mass or inertia is assigned.
The combined workcell also includes `ground_link`, a fixed 2 x 2 m ground
surface represented by a 20 mm thick visual and collision box. Its top is
`z=0` and its bottom is `z=-0.020 m` in `dual_milly_base`. This fixture is kept
in the combined model so the standalone gimbal can be mounted elsewhere.
The old centre D405, keepout and support post are omitted. Arm-mounted wrist
cameras remain.

The optional coarse gimbal safety sphere is defined in
`../stereo-gimbal/safety-envelope.json` and its companion proxy URDF. It has a
160 mm radius, is fixed to `gimbal_base_link` at `(0, 0, 0.147)`, and therefore
has centre `(0.045, 0, 0.451)` in `dual_milly_base`. It covers the complete
gimbal throughout the three confirmed joint ranges, with at least 4.4 mm
clearance to the current model. The support pole retains its separate box
collider. Future coarse solving should use the sphere in place of the gimbal
exterior primitives; it is not a self-collision body or an active control guard.

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

All three gimbal motors use the user-confirmed `direction: -1` mapping from
actuator raw angles to URDF joint angles. Encoder zeros remain provisional.
No gimbal masses or inertias have been assigned. The CAD check in
`../stereo-gimbal/clearance-check.json` finds camera-case interference with the
rear yaw support at pitch +90 degrees and roll +/-90 degrees. The independent
joint bounds do not enforce a collision-free coupled workspace. Resolve that
interference with coupled motion bounds or mechanical changes before full-range
head tracking.

Repository entry points:

- `descriptions/milly-7dof-gimbal/urdf/milly-7dof-gimbal.urdf`
- `descriptions/milly-7dof-gimbal/hardware.yaml`
- `components/bello-hardwared/config/milly-7dof-gimbal.yaml`
- `components/bello-hardwared/config/milly-7dof-gimbal-vcan.yaml`
- `components/bello-visualizer/config/milly-7dof-gimbal.example.yaml`
- `components/robstride-emulator/config/milly-7dof-gimbal-can0-vcan.yaml`
- `components/robstride-emulator/config/milly-7dof-gimbal-can1-vcan.yaml`
