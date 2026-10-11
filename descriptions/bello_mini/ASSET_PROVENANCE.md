# Bello Mini asset provenance

This directory is the canonical source for the Bello Mini robot description.
The URDF and reduced exterior meshes were generated from the reviewed assembly
STEP file whose SHA-256 digest is
`1161b7d63f14b735c82f2eb3a06bbe30ddd6e43558708607e748cc0731106d4b`.
The source assembly used millimetres and was classified with review contract
`bello_mini_resolved_link_ownership_v1`.

The checked-in description has 28 exterior visual meshes and primitive-only
collision geometry. Its modeled total mass is **48.580 kg**, reconciled to
assembly measurements on 2026-09-22. Hand physical proxies remain 25 mm radius spheres carrying 0.470 kg each.
The fixed head retains its nominal sphere collision and inertia approximation.
Their separate proxy visuals are removed because the new GLBs include them.

## Measured mass reconciliation

| Assembly | Mass (kg) | Allocation |
| --- | ---: | --- |
| Left arm including hand/covers | 5.800 | 5.330 across seven arm links; 0.470 at the end-effector sphere |
| Right arm including hand/covers | 5.800 | Same measured target, scaled independently from its CAD baseline |
| Left leg | 10.200 | 5.640 above knee; 4.560 knee-down |
| Right leg | 10.200 | Same measured target |
| Central body, excluding head | 15.900 | Pelvis and waist yaw masses unchanged; excess allocated to torso |
| Head | 0.680 | Separate fixed `head_link` |
| **Total** | **48.580** | Sum of measured assemblies, not rounded to 48.5 |

The arm measurements exclude the shoulder-pitch motors already owned by the
torso; leg measurements exclude the hip-pitch motors already owned by the
pelvis. These root motors must not be counted twice. The 0.500 kg cover estimate
per arm is included in the 5.800 kg target, not an extra payload.

For each arm, all seven original masses and COM inertia tensors are scaled by
one common factor to total 5.330 kg, initially retaining the individual COMs.
The left/right symmetry pass below is then applied. Each 0.470 kg hand is added
at its existing sphere center (local
`[0, 0, -0.025]` m), with solid-sphere COM inertia
`diag(0.0001175, 0.0001175, 0.0001175)` kg m^2. This is an explicit inertia
approximation, not a measurement of the physical hand.

The hip-pitch, hip-roll and hip-yaw link masses and inertia tensors are scaled
proportionally to total 5.640 kg per leg. The knee-down target includes the knee
motor. Its 0.097690723121 kg excess is assigned only to `*_knee_link`, whose mass
becomes 3.533698537613 kg; its inertia initially scales by the same ratio about
its existing COM. Ankle pitch/roll masses are unchanged. Central-body excess
1.258867006766 kg is assigned only to `torso_link`, bringing it to
11.053965802291 kg with proportional inertia scaling. These allocations are
followed by the symmetry pass below.

These allocations deliberately approximate the distribution of unidentified
mass. The original 43.503025077021036 kg CAD baseline comprised
19.529025077021032 kg of structure, 20.544 kg of catalog motors, 3.400 kg of
measured torso assemblies and 0.030 kg of camera. The torso assemblies already
included the 2.650 kg battery and 0.750 kg electronics; neither is added again.
The old material/catalog breakdown is historical provenance, not an exact
decomposition of the reconciled inertials.

### Separate head placement

No head or neck geometry was present in the reviewed Mini model. The fixed
mount is inferred from the upper torso, not from an identified mounting hole:
`torso_link -> head_link` has `xyz="0 0 0.333"`, `rpy="0 0 0"`. This is centered
on the robot's sagittal plane above the actual torso visual mesh, whose local
bounds are approximately `[-0.074001, -0.114258, -0.080528]` to
`[0.070000, 0.116202, 0.332759]` m. The shoulder-pitch axes are at torso-local
z = 0.228 m.

The head retains a 0.060 m radius sphere for collision and inertia purposes. Its COM is `[0, 0, 0.060]` m in `head_link`, hence
`[0, 0, 0.393]` m in `torso_link`. Its mass is measured (0.680 kg), but its
position, dimensions and solid-sphere inertia
`diag(0.0009792, 0.0009792, 0.0009792)` kg m^2 are nominal approximations.
The head is fixed, not an additional actuated joint. Existing joint limits,
mesh transforms, camera and IMU frames are unchanged.

### Left/right symmetry

After matching the measured assembly totals, each left/right pair is averaged
in a common reflected frame using `S = diag(1, -1, 1)`. Mass is the arithmetic
mean, COM is the mass-weighted mean, and COM inertia includes the parallel-axis
terms needed to average the two mass distributions. The right link receives
`S * COM` and `S * I * S`. Thus `ixy` and `iyz` change sign, while `ixz` does
not. This preserves the measured totals rather than copying one side's mass
to the other. Pelvis, waist-yaw and torso inertials are similarly averaged with
their own reflections, placing their COMs on `y=0` and zeroing `ixy`/`iyz`.
These symmetry assumptions replace the small CAD-derived asymmetries.

Paired joint origins are averaged after reflection to remove sub-nanometre
CAD roundoff. Axes and reviewed signed limits are retained: mirrored pitch
motions use the same sign, while roll/yaw use opposite signs. The wrist-roll
collision sphere centers had a 3 mm sagittal mismatch and are replaced by
their reflected mean. Other paired collision shapes already agree within
numerical precision (the two heel cylinders exchange +/-Y roles). Original
CAD visual meshes are retained; small asymmetric mesh details are not used to
derive the symmetrized inertials.

Runtime validation with MuJoCo 3.12.0 confirmed 48.580 kg in both physics and
visual models, 28 actuators, positive physically valid inertia tensors, mirrored
paired inertials/collisions and sign-adjusted joint limits, and zero lateral
whole-body COM in the symmetric home pose. The existing neutral-pose collision
check passed. This validates the description, not a trained policy's behavior
under the new mass distribution; hardware deployment remains separate.

## Sensor and motor conventions

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

`imu_sensor_link` is a coincident fixed child of `imu_link` and represents the
installed EBIMU's raw axes. Its roll is `3.141592653589793` rad: the sensor is
mounted upside down about logical +X. The hardware description declares
`imu_sensor_link` as the source and `imu_link` as the output, so hardwared
applies this fixed rotation before filtering or publishing samples. Keeping the
logical link separate preserves the policy and state-estimator frame contract.
The roll axis is inferred from the installed orientation and should be
confirmed with deliberate forward and lateral tilt checks before armed use.

`motor_topology.yaml` records the reviewed motor names, models, couplings, and
logical joints. CAN IDs follow increasing kinematic-chain order: left/right
arms use `31..37` on `can2` / `41..47` on `can3`, waist yaw uses `51` on
`can1` and waist roll uses `52` on `can0`, and left/right legs use `61..66` on `can0` / `71..76` on `can1`. Arm IDs 35/45 are the
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
all four ankle motors, left shoulder pitch, and left wrist pitch, and +1 for
right hip pitch, right shoulder pitch, and right wrist pitch. On 2026-09-07 the
operator reported on both feet that, under the old mapping, toe-up reconstructed
as positive roll and positive physical roll reconstructed as toe-down (positive
URDF pitch). With Mini's +X-forward, +Y-left axes, this supports the same
normalized sign/order candidate used by Bello: `motor_1 = -pitch + roll` and
`motor_2 = pitch + roll`. Supported feedback-delta and low-amplitude command
tests must confirm the corrected mapping before armed use. Other command
directions and encoder zero offsets remain null in the topology.
Designer-provided travel limits still need physical-stop confirmation, and the
ankle differential reduction magnitudes still require bench validation. This is
not a deployable `bello-hardwared` configuration until those blockers are
resolved.

`hardware.yaml` provides a provisional `bello-hardwared` profile for all 28
motors, using the topology's exact names, models, buses, and CAN IDs. Both waist
motors join the legs in `lower_body`; both seven-motor arms form `upper_body`.
Waist yaw, both hip roll motors, and left hip pitch use the confirmed direction -1;
right hip pitch uses the operator-confirmed direction +1. Shoulder pitch and
wrist pitch use the operator-confirmed directions -1 on the left and +1 on the
right. Other direct-drive
directions use the topology's CAD candidate signs, conditional on
protocol-positive matching CAD motor-frame +Z. All four ankle motors use the
operator-confirmed direction -1. The URDF includes the observation-derived ankle
differential sign/order candidate on both sides:
`motor_1 = -pitch + roll`, `motor_2 = pitch + roll`, with provisional unit
reductions and zero offsets. The normalized coefficient matrices are recorded
in `motor_topology.yaml`; their magnitudes are not measured linkage reductions.
Physical calibration fields other than the confirmed command directions and
ankle sign/order observations remain null.

The component's `config/bello-mini.yaml` and `config/bello-mini-vcan.yaml` bind
this description to physical and simulated transports respectively. Bus,
health, damping, and control timing settings follow the existing Bello profile,
with damping gains selected for Mini's motor models. The installed EBIMU uses
115200 baud; output is set to 100 Hz to fit ASCII samples within that bandwidth.
The MuJoCo emulator samples `imu_sensor_link`, so the vCAN path exercises the
same hardwared source-to-output rotation as physical hardware.
The physical binding identifies its USB adapter by serial number
`5C93185491`. Hardwared does not support per-motor
encoder offsets: physical encoder zeros must be aligned to the URDF convention
before operation. Validate the corrected ankle sign/order, remaining motor
directions, encoder zeros, physical stops, and differential reduction magnitudes
before using the physical bindings.
`--check` validates the software configuration, not those physical assumptions.

The descriptions and meshes are proprietary. This repository does not grant
permission to redistribute them. External distribution requires a separate
written agreement with the applicable rights holder.

## GLB visual update (2026-09-29)

The 28 GLBs were supplied in the `bello_mini_urdf` package. They replace only
matching STL visual references and use the existing `0.001 0.001 0.001` scale:
vertices are in millimetres. Embedded glTF materials own visual colors; the old
per-visual URDF material overrides are removed. The torso GLB contains the head
and each wrist-roll GLB contains its hand, so only the redundant `head_link`
and end-effector-sphere visual elements are removed. All physical links, joints,
transmissions, collisions, inertials, limits and sensor frames are unchanged.

The companion STL meshes are retained for MuJoCo import. Both the stack native
emulator and bello_mujoco's Python asset pipeline create a temporary URDF with
explicit STL references; neither rewrites the canonical GLB URDF. Kiss3d and
the Electron console use the GLBs and their embedded materials directly. The
companion STLs retain their earlier geometry, so MuJoCo diagnostics do not show
the newly detailed head and hands. Physical mass and collision geometry match
across import paths.
