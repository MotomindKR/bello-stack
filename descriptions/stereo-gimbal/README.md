# Stereo camera gimbal

Three-axis visual/kinematic description from the supplied `gimbal.step`, SHA-256
`aa016f58329188274fbf2032a41e9e52aa4c86037b7fc4b07e3cc19a3fc258cf`.
The model contains three `RobStride_05` actuator housings and the
`ELP-3DGS1200P01-H120` camera assembly.

## Confirmed configuration

| Coordinate | Model | Logical bus / CAN ID | Travel |
| --- | --- | --- | --- |
| gimbal_yaw | ROBSTRIDE_05 | can0 / 11 | -90 to +90 degrees |
| gimbal_pitch | ROBSTRIDE_05 | can0 / 12 | -90 to +90 degrees |
| gimbal_roll | ROBSTRIDE_05 | can0 / 13 | -90 to +90 degrees |

The user supplied this travel and ID ordering and confirmed on the actuators
that all three raw positive rotations oppose the URDF joint axes. Each motor
therefore uses `direction: -1`: `q_joint = -q_motor_raw`, assuming aligned zeros.
Position, velocity and torque are sign-mapped once at the protocol boundary;
gains retain their sign. The URDF axes remain +Z yaw, +Y pitch and +X roll.
The browser sliders use joint angles and also show the corresponding actuator
angles with zero offset assumed. Encoder zeros and cable clearances still need
commissioning; the sign confirmation does not calibrate the zeros.
The 0.2 rad/s velocity, 1 N m feedforward cap, 0.1 rad maximum position step
and damping values are initial control settings, not measured physical ratings.
No actuator commands are issued by the browser model viewer.

## Geometry and frames

`gimbal_base_link` is at the yaw-axis projection onto the underside of the
mounting plate. At the CAD zero pose, X points forward through the camera,
Y points left and Z points up. The raw CAD is in millimetres; meshes are baked
in metres into their owning link frames.

| Joint | Parent | Origin in parent (m) | Positive axis |
| --- | --- | --- | --- |
| gimbal_yaw | gimbal_base_link | (0, 0, 0.053) | +Z |
| gimbal_pitch | gimbal_yaw_link | (-0.001, 0, 0.128) | +Y |
| gimbal_roll | gimbal_pitch_link | (0.075, 0, 0) | +X |

These shaft lines come from the motor housing cylinders. The yaw motor stator
belongs to the base; the pitch stator belongs to the yaw frame; the roll stator
belongs to the pitch carrier. Camera cases and optics follow the roll output.
The black yaw travel stop stays with the yaw stator on the base. The black
roll travel stop stays with the roll stator on the pitch carrier; neither
limiter follows its actuator's rotating output.
The ten structure solids from the unlabeled CAD compound are assigned to their
apparent mounting stages; review the pitch bearing collar ownership in the
browser before treating this as a commissioned mechanism.

All ten printed structure pieces, actuator housings, exposed lens bodies,
lenses and USB connector are included. The camera PCB and IC are omitted.
`visual-assets.json` records source paths/solid indices, transforms and hashes.
Only boxes and cylinders are used for collisions; the camera has one box.
The envelopes in `collision-primitives.json` are approximate. Independent joint
travel does not establish clearance for every combination of joint angles.
The selected-pair exact CAD check in `clearance-check.json` detects camera-case
interference with the rear yaw support at pitch +90 degrees and roll +/-90
degrees. Coupled motion bounds or a mechanical revision are needed before
full-range head tracking. The check covers only the recorded pair and poses.

`stereo_camera_link` is at the midpoint of the two lens front planes in the
CAD. The central, left and right optical frames use Z forward, X image-right
and Y image-down. The CAD lens spacing is 59.2 mm. These are geometric reference
frames; optical centres, intrinsics, distortion, stereo baseline and image
packing must be determined from the actual camera/calibration for WebXR.
The browser preview displays these four frames using their URDF transforms,
with a selector to isolate one and a Focus camera button. Frame axes use
red X, green Y and blue Z; camera frame markers are independent of joint axes.

No masses or inertia tensors are estimated in this visual/kinematic phase.
The URDF, YAML and description data are maintained directly; this directory
contains no generators.

`safety-envelope.json` defines an optional coarse safety sphere for later
arm/environment collision solving. Its centre is `(0, 0, 0.147)` in
`gimbal_base_link`, with radius `0.160 m`. A continuous rotational bound over
the current meshes and primitives gives a maximum radius of `0.155567 m`,
leaving at least 4.4 mm model clearance throughout the confirmed travel.
`urdf/gimbal-safety-envelope.urdf` contains the same sphere as a collision-only
proxy. Use it instead of the covered exterior primitives for coarse checks,
while retaining detailed bodies for gimbal self-collision. The sphere excludes
the support pole, cables and unmodelled hardware, and is not wired to motor
control. Revalidate it whenever geometry or kinematics change. The browser
preview has a separate safety-sphere checkbox.

## Entry points

- `urdf/stereo-gimbal.urdf`: standalone description.
- `hardware.yaml`: three motors in the atomic `gimbal` control group.
- `components/bello-hardwared/config/stereo-gimbal.yaml`: CAN0 deployment.
- `components/bello-hardwared/config/stereo-gimbal-vcan.yaml`: vCAN deployment.
- `components/robstride-emulator/config/stereo-gimbal-vcan.yaml`: protocol emulator.
- `components/bello-visualizer/config/stereo-gimbal.example.yaml`: native viewer.

Component paths above are relative to the repository root. These are standalone
commissioning profiles. A combined deployment should add these IDs and the
`gimbal` group to the same daemon that owns the arms' CAN0 bus. The combined
model in `../milly-7dof-gimbal/` places the gimbal between arm mounting plates
450 mm apart and suspends its base 300 mm above their plane, facing the arm
workspace. The standalone model retains its own base frame.
