# Bello Mini asset provenance

This directory is the canonical source for the Bello Mini robot description.
The URDF and reduced exterior meshes were generated from the reviewed assembly
STEP file whose SHA-256 digest is
`1161b7d63f14b735c82f2eb3a06bbe30ddd6e43558708607e748cc0731106d4b`.
The source assembly used millimetres and was classified with review contract
`bello_mini_resolved_link_ownership_v1`.

The checked-in description has 28 exterior visual meshes and primitive-only
collision geometry. Its modeled total mass is 43.503025077021036 kg:
19.529025077021032 kg of material-derived structure, 20.544 kg of catalog
RobStride motors, 3.4 kg of measured torso assemblies, and 0.03 kg for the
Orbbec Astra Stereo S U3 camera. Detailed hands are deliberately absent and
each wrist ends in a massless anonymous 25 mm sphere.

The camera sensor tree uses nominal mechanical optical origins rather than
factory calibration. The IR/depth origin is 2 mm behind the CAD front face,
the right IR origin is 40 mm toward camera-right, and RGB is 10 mm toward
camera-right and 2 mm forward of depth. Robot-state-publisher and the camera
driver must not both publish the same sensor transforms.

`imu_link` is a massless fixed child of `pelvis_link`. Its origin is 50 mm below
the exact midpoint of the reviewed left/right hip-pitch joint origins in pelvis
coordinates: `[-0.0099999999999894115, 0, -0.230000001143162]` m. Bello Mini's
pelvis axes are +X-forward, +Y-left, +Z-up, while original Bello's pelvis/IMU
axes are +X-right, +Y-forward, +Z-up. The fixed joint therefore has yaw
`-1.5707963267948966` rad so `imu_link` reproduces original Bello's IMU-axis
convention. This pose is a user-specified nominal mounting location rather than
a sensor-derived CAD datum.

`motor_topology.yaml` records the reviewed motor names, models, couplings, and
logical joints. CAN IDs follow increasing kinematic-chain order: left/right
arms use `31..37` / `41..47` on `can1`, waist yaw/roll use `51,52` on `can1`,
and left/right legs use `61..66` / `71..76` on `can0`.

The topology is intentionally marked `calibration_required`. Its geometric
direction estimate is the alignment between each CAD motor-frame `+Z` axis and
the reviewed joint `+` axis; RobStride documentation does not establish that
STEP-frame `+Z` is protocol-positive. Live command directions and encoder zero
offsets therefore remain null. Physical travel limits and ankle differential
coefficients/signs also require bench validation. This is not a deployable
`bello-hardwared` configuration until those blockers are resolved.

The descriptions and meshes are proprietary. This repository does not grant
permission to redistribute them. External distribution requires a separate
written agreement with the applicable rights holder.
