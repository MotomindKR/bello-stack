# Robot descriptions

Choose a configuration to compare its visual mesh, collision geometry, and movable joint axes.

| Configuration | Equipment |
| --- | --- |
| [Bello](bello/README.md) | Full-size humanoid with separate upper- and lower-body motor groups. |
| [Bello Mini](bello_mini/README.md) | Humanoid with a two-axis waist and differential ankle transmissions. |
| [Bello Mini Torso](bello_mini_torso/README.md) | Upper-body fixture with a coupled hip drive for torso pitch and a Milly gripper on each arm. |
| [Milly arm](milly/README.md) | Six-axis arm with a parallel gripper and wrist camera. |
| [Dual Milly arms](milly-dual/README.md) | Two six-axis arms with parallel grippers, wrist cameras, and a central D405 camera. |
| [Milly 7-DOF arm](milly-7dof/README.md#single-arm) | Seven-axis arm with upper-arm roll at J3 and a wrist camera. |
| [Dual Milly 7-DOF arms](milly-7dof/README.md#dual-arms) | Two seven-axis arms with parallel grippers and a central D405 camera. |
| [Dual Milly 7-DOF with stereo gimbal](milly-7dof-gimbal/README.md) | Two seven-axis arms with parallel grippers and wrist cameras flank a three-axis stereo gimbal. |
| [Dual Milly 7-DOF with Linker O6 hands](milly-7dof-o6/README.md) | Two seven-axis arms carry rigid O6 hands and share a central D405 camera. |
| [Dual Milly 7-DOF with Linker O6 hands and stereo gimbal](milly-7dof-o6-gimbal/README.md) | Rigid O6 hands on both arms and a gimbal-mounted stereo pair. |
| [Stereo camera gimbal](stereo-gimbal/README.md) | Three joints drive yaw, pitch, and roll. Includes an optional [collision envelope](stereo-gimbal/README.md#collision-envelope). |

The two columns show front and rear three-quarter views at the same scale. The upper row shows the visual mesh. The lower row shows collision geometry and movable joint axes. Each dot marks a joint origin, and its line follows the positive joint axis.

The Milly arms use a bent pose with shoulder and elbow pitch at 1.15 rad. The other movable joints are at zero. The stereo workcell images omit the ground plate so the robots fill the frame.

[Deployment configuration](../docs/configuration.md)
