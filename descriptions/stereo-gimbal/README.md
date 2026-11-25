# Stereo camera gimbal

Three joints drive yaw, pitch, and roll. At zero the mounting frame points X forward, Y left, and Z up.

![Stereo camera gimbal exterior from two angles](images/stereo-gimbal.png)

[URDF](urdf/stereo-gimbal.urdf) · [Hardware inventory](hardware.yaml)

**Collision geometry and joint axes**

![Stereo camera gimbal collision shapes and movable joint axes from the same two angles](images/stereo-gimbal-collision.png)

The camera housing intersects the rear yaw support at pitch +90° combined with roll ±90°. Check [coupled clearance](clearance-check.json) and cable travel on the installed assembly.

## Collision envelope

The optional 132 mm radius sphere encloses the moving gimbal. Keep the fixed base, yaw stator, and support collisions when using it. This proxy has no visual geometry or movable joints.

[Proxy URDF](urdf/gimbal-safety-envelope.urdf) · [Envelope definition](safety-envelope.json) · [Detailed collisions](collision-primitives.json)

![Front and rear views of the spherical gimbal collision envelope](images/gimbal-safety-envelope.png)
