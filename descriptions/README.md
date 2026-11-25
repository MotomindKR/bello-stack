# Robot descriptions

Each model page pairs exterior views with the URDF collision shapes and movable joint axes.

| Robot | Configurations |
| --- | --- |
| Bello | [Full size](bello/README.md) · [Mini](bello_mini/README.md) · [Mini Torso with grippers](bello_mini_torso/README.md) |
| Milly 6-DOF | [Single arm](milly/README.md) · [Dual arms](milly-dual/README.md) |
| Milly 7-DOF | [Single arm](milly-7dof/README.md#single-arm) · [Dual arms](milly-7dof/README.md#dual-arms) · [Dual arms with stereo gimbal](milly-7dof-gimbal/README.md) |
| Milly 7-DOF with Linker O6 hands | [Dual arms](milly-7dof-o6/README.md) · [Dual arms with stereo gimbal](milly-7dof-o6-gimbal/README.md) |
| Stereo gimbal | [Three-axis camera mount](stereo-gimbal/README.md) · [Static collision envelope](stereo-gimbal/README.md#collision-envelope) |

In the collision views, each dot marks a joint origin and its line points along the positive axis. Orange marks rotation and blue marks translation.

The Milly previews show shoulder and elbow pitch at 1.15 rad. For the parallel grippers on Milly and Mini Torso, the drive is at −1.10 rad. The URDF’s `bello:slider_crank` linkage sets the rod angles and finger positions. The workcell previews omit the ground plate to show the arm and gimbal mounts.

[Hardware and simulator deployment configuration](../docs/configuration.md)
