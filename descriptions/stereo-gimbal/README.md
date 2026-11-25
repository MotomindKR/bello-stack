# Stereo camera gimbal

The three RS_05 motors drive yaw (+Z), pitch (+Y) and roll (+X) on can0 IDs 11, 12 and 13. Each has ±90° of individual travel and direction −1. Encoder zeros and cable clearance require commissioning. [Hardware inventory](hardware.yaml)

At zero, the standalone mounting frame has X forward, Y left and Z up. The ELP-3DGS1200P01-H120 lens front planes are 59.2 mm apart in CAD. The optical frames use Z forward, X image-right and Y image-down. Visual meshes come from the supplied gimbal STEP assembly. [Mesh provenance](visual-assets.json)

The camera case intersects the rear yaw support at pitch +90° and roll ±90° in the checked CAD poses. Individual joint bounds alone allow these combinations. The 132 mm sphere in the optional proxy covers the moving exterior across joint travel. The fixed yaw stator, base plate and support retain their own collisions. Use the detailed geometry for internal collisions and check cables separately. [Clearance measurements](clearance-check.json) · [Proxy definition](safety-envelope.json) · [Detailed collision primitives](collision-primitives.json)

The standalone URDF supplies kinematics. The [dual-arm fixture](../milly-7dof-gimbal/) supplies its mounting pose. Provisional gimbal inertias for MuJoCo live in the simulator profile.
