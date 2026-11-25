# Milly 7-DOF arms

The single arm and dual fixture use an RS_10P upper-arm roll at J3. The elbow, wrist and gripper move to CAN IDs 4–8. The dual fixture uses IDs 1–8 on each arm bus, with the left arm on can0 and the right on can1. Its wrist cameras and centre D405 are included. The [stereo gimbal fixture](../milly-7dof-gimbal/) has a different base.

| Profile | Hardware inventory | Group |
| --- | --- | --- |
| Single | [hardware.yaml](hardware.yaml) | arm |
| Dual | [hardware-dual.yaml](hardware-dual.yaml) | dual_arm |

Assign the physical CAN IDs and verify encoder zeros before enabling motors. The J3 sign is −1 on both arms and its travel is ±90°. The hardware inventories specify command and feedback limits separately. [SDK limit mapping](sdk-limits.json)

## Model sources

The shaft alignment comes from the seven-axis STEP assembly. J7 drives a rotor on the measured shaft and carries the gripper and wrist camera. The single and dual descriptions share meshes. Visuals use STEP exterior parts and collisions use primitives. [Shaft alignment](kinematic-alignment.json) · [Mesh provenance](visual-assets.json) · [Collision inventory](collision-primitives.json)

Each arm including its fixed base has 6.803799249 kg of supplied link inertias. Auxiliary links without supplied mass properties are recorded separately in simulation fixtures. [Inertia sources](physical-parameters.json)

The RS_10P uses the manufacturer's CAN 2.0 private protocol. Its wire limits are defined in the [shared motor catalogue](../motors/models.yaml). [Manufacturer manual, sections 1.2, 1.3 and 4.1.2](https://github.com/RobStride/Product_Information/blob/40d43fdd15d397a446c70e550d1fd0a3700c2c44/Product%20Literature/CAN2.0/10P/RS10PUser%20Manual260907.pdf)
