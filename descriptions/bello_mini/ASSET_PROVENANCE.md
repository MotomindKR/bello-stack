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
arms use `31..37` / `41..47` on `can1`, waist yaw/roll use `51,52` on `can0`,
and left/right legs use `61..66` / `71..76` on `can0`. Arm IDs 35/45 are the
left/right wrist-yaw motors and joints.

The URDF joint limits are the designer-provided nominal ranges dated
2026-09-03. User visual review established that hip pitch, hip roll, shoulder
roll, and elbow pitch use the opposite sign from the reported left-side
convention when expressed on the Mini URDF axes; all other left-side signs keep
the designer mapping. The right side is derived geometrically across the URDF
`y=0` plane: pitch keeps the left URDF sign while roll/yaw reverse. Encoder-zero
correspondence and physical stops remain pending validation. The ankle
pitch/roll limits are logical output envelopes, not independent limits for the
coupled ankle motors; the corners of that rectangular envelope are not yet
guaranteed reachable.

The topology is intentionally marked `calibration_required`. Its geometric
direction estimate is the alignment between each CAD motor-frame `+Z` axis and
the reviewed joint `+` axis; RobStride documentation does not establish that
STEP-frame `+Z` is protocol-positive. Operator review on 2026-09-06 confirmed
command direction -1 for waist yaw, both hip roll motors, left hip pitch,
all four ankle motors, and left shoulder pitch, and +1 for right hip pitch
and right shoulder pitch. Other command directions
and encoder zero offsets remain null in the topology. Designer-provided travel
limits still need physical-stop
confirmation, and ankle differential coefficients/signs require
bench validation. This is not a deployable `bello-hardwared` configuration
until those blockers are resolved.

`hardware.yaml` provides a provisional `bello-hardwared` profile for all 28
motors, using the topology's exact names, models, buses, and CAN IDs. Both waist
motors join the legs in `lower_body`; both seven-motor arms form `upper_body`.
Waist yaw, both hip roll motors, and left hip pitch use the confirmed direction -1;
right hip pitch uses the operator-confirmed direction +1. Shoulder pitch uses
the operator-confirmed directions -1 on the left and +1 on the right. Other direct-drive
directions use the topology's CAD candidate signs, conditional on
protocol-positive matching CAD motor-frame +Z. All four ankle motors use the
operator-confirmed direction -1; ankle coupling still requires validation.
The URDF now includes provisional unit differential transmissions on both sides:
`motor_1 = pitch + roll`, `motor_2 = pitch - roll`, with zero offsets. These
equations are simulation conventions, not measured linkage coefficients.
Physical calibration fields other than these operator-confirmed command
directions remain null in `motor_topology.yaml`.

The component's `config/bello-mini.yaml` and `config/bello-mini-vcan.yaml` bind
this description to physical and simulated transports respectively. Bus,
health, damping, and control timing settings follow the existing Bello profile,
with damping gains selected for Mini's motor models. The installed EBIMU uses
115200 baud; output is set to 100 Hz to fit ASCII samples within that bandwidth.
The physical binding identifies its USB adapter by serial number
`5C93185491`. Hardwared does not support per-motor
encoder offsets: physical encoder zeros must be aligned to the URDF convention
before operation. Validate motor directions, encoder zeros, physical stops,
and differential signs/reductions before using the physical bindings.
`--check` validates the software configuration, not those physical assumptions.

The descriptions and meshes are proprietary. This repository does not grant
permission to redistribute them. External distribution requires a separate
written agreement with the applicable rights holder.
