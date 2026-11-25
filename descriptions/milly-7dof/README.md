# Milly 7-DOF arms

The single and dual configurations add upper-arm roll at J3. Each wrist carries a parallel gripper and camera.

## Single arm

[URDF](urdf/milly-7dof.urdf) · [Hardware inventory](hardware.yaml)

![Single seven-axis Milly arm, front and rear views of the visual mesh and collision geometry with joint axes](images/milly-7dof.png)

## Dual arms

The dual fixture adds a central D405 camera between the arms.

[URDF](urdf/milly-7dof-dual.urdf) · [Hardware inventory](hardware-dual.yaml)

![Dual seven-axis Milly arms, front and rear views of the visual mesh and collision geometry with joint axes](images/milly-7dof-dual.png)

Both configurations share the [shaft alignment](kinematic-alignment.json), [collision geometry](collision-primitives.json), [mesh sources](visual-assets.json), and [inertia sources](physical-parameters.json). Verify encoder zeros against the [joint limits](sdk-limits.json) before enabling motors.
