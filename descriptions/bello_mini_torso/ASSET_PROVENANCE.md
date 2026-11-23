# Bello Mini Torso asset provenance

`urdf/bello_mini_torso.urdf` is derived from
[`bello_mini.urdf`](../bello_mini/urdf/bello_mini.urdf). It describes the Mini
upper body mounted to a fixed base through both active hip-pitch motors. Both
motors drive the same torso-pitch motion and contribute torque in parallel.

## Retained assembly

The description retains the pelvis, waist yaw and roll, both seven-joint arms,
head, camera, and IMU frames. Milly grippers replace the hand proxies. Both complete leg subtrees are
removed, starting at `left_hip_pitch_link` and `right_hip_pitch_link`, along with
their ankle transmissions. The source hip-pitch child-link inertias belong to
the removed leg assemblies; the hip-pitch motors themselves are already
included in the source pelvis inertia and are retained without adding mass.

Before the gripper replacement, the modeled moving mass was **28.180 kg**: the source's 48.580 kg minus
two 10.200 kg leg assemblies. This is a derivation from the full robot's mass
allocation, not a separate measurement of the torso variant. Shared meshes are
referenced directly from the Mini and Milly descriptions. Trimmed wrist visuals
are the exception described below.
Inherited geometry and mass approximations are documented in the
[Mini provenance](../bello_mini/ASSET_PROVENANCE.md).

## Articulated Milly grippers

The gripper subtrees come from Milly's updated URDF on stack `main` at
`de0369a`, including the exterior meshes, rounded jaws, primitive colliders,
crank, rods, pin frames, wrist camera/bracket and jaw mass properties. Assets reference Milly directly.
`scripts/mini_torso_grippers.py` applies this integration using structured XML.
Run it with `uv run --with numpy --with scipy --with trimesh` to regenerate.

Both grippers retain Milly's revolute crank, two prismatic fingers and two
passive revolute rods, with separately prefixed `bello:slider_crank` declarations.
The nonlinear linkage equations determine all four passive positions from each
gripper motor angle, keeping rod tips and slider pins coincident throughout
the motion. The dimensions, phase, joint origins, axes and limits match Milly.
The emulator starts at Milly's operational open angle, **-2.2 rad**; the joints
are no longer baked into fixed open transforms.

`left_gripper` is motor ID **38 on can2**, and `right_gripper` is motor ID
**48 on can3**. Each owns a separate motor group. Motor model, direction,
command limits, health thresholds and damping inherit the Milly gripper
configuration. These are software assignments, not a measurement or programming
of the installed actuators. The 17 torso/arm joints and both hip motors remain
active; the grippers add two independent coordinates and eight passive joints.

The user approved a **provisional direct mount**, with the **camera bracket's
wrist-facing mounting face flush against the wrist exterior**. The generator
uses the complete `camera.stl` bracket/housing mesh in the gripper frame, not
the gripper housing rear face. That mounting face is approximately gripper-local
Z = -0.067249985 m. The housing rear face is 12 mm farther forward; seating the
housing instead embeds the camera bracket in the wrist.

The distal wrist seating planes come from the retained visual meshes:
approximately Z = -0.040959354 m (left) and -0.040959476 m (right). With rotation
`[pi, 0, 0]`, `wrist_roll_link` to `gripper_base` translations are
`[0, 0, -0.108209339]` m (left) and `[0, 0, -0.108209461]` m (right).
This moves the assembly approximately 15.462 mm outward from the original
hand-origin-based mount. The small left/right difference is inherited CAD
precision. The former hand origin at Z = -0.037497804 m is inside the casing
and is not a valid seating plane for this assembly.

Gripper +Z points along the former hand's -Z; the jaw travel axis is wrist +X.
Both sides use the same physical gripper without mirroring its mechanism.
The complete gripper, TCP, camera, inertial and collision subtrees move together.
The 12 mm interval between the wrist face and housing is occupied by the
existing camera bracket. No extra spacer or adapter geometry is added.
Confirm the CAD-derived seating plane against the installed mount before
hardware use.

`left_gripper_tcp` and `right_gripper_tcp` replace the old sphere tool frames.
Each TCP is at the midpoint of the rounded jaw-tip centres, `[0, 0, 0.0795]` in the
gripper base, with axes aligned to its wrist. This is a nominal geometric TCP,
not a measured grasp frame. Payloads attach here. The open jaws' collision
shapes participate in self and optional ground clearance with the existing
rigid-body/neighbor exclusions. Milly's primitives are approximations, not
guaranteed enclosing surfaces; its crank/rods have no collision geometry.

Each 0.470 kg hand proxy is removed. Each new gripper assembly uses 0.468684965731
kg: Milly's 0.356246799525 kg residual SDK wrist aggregate, 15 g crank, two 10 g
rods and two approximately 38.719 g jaws. The residual inertia is rigidly
transformed from Milly's wrist into the gripper base without adding it twice.
**The residual includes Milly wrist hardware and is not a measured standalone
gripper inertia.** It is an explicit provisional mass proxy, pending gripper-only
mass/COM/inertia and adapter measurements. Gravity compensation now includes
this estimate; it is not calibrated for the replacement hardware. The resulting
modeled moving mass is approximately **28.177370 kg**.

Each gripper also carries Milly's complete `camera.stl` bracket/housing mesh
and its three primitive colliders (housing, stalk and mounting foot).
`left_camera_link` / `right_camera_link` and their `*_camera_optical_frame`
children start from Milly's lens position and orientation relative to the
gripper base. At the user's request, the entire left camera/bracket is then
rotated 180 degrees about gripper +Z, putting it on the outer side of the left
wrist. The right camera retains its original mount. The fixed mount transform is `inverse(milly_wrist_from_gripper) *
milly_wrist_from_camera`, composed with the provisional torso gripper mount.
For the left side, this transform is pre-multiplied by `Rz(pi)`.
Camera +X is forward; optical +Z is forward, +X right and +Y down. These
nominal CAD extrinsics require calibration on the installed bracket. The camera
has no separately measured inertia; it remains included in the residual SDK
wrist aggregate proxy, without an added duplicate mass. The existing torso
camera and IMU frames are retained.

The original wrist GLBs contain full hands. The generator retains only named
wrist cases, the outer frame and wrist brackets, excluding the old hands and
temporary wrist adapter. These filtered GLBs and matching STL companions live
in `meshes/`; all other Mini visuals and wrist inertias/colliders are unchanged.

## Mounting and coordinates

`base_link` is the fixed root at the midpoint of the source hip-pitch joint
origins, expressed in the original pelvis frame as approximately
`[-0.010, 0, -0.180]` m. Its axes follow the Mini convention: +X forward,
+Y left, +Z up. Separate fixed `left_hip_pitch_mount_frame` and
`right_hip_pitch_mount_frame` links locate the two original mounting axes.

`torso_pitch_joint` rotates `torso_pitch_link` about base +Y. At zero pitch the
upper body has the source robot's upright orientation. Positive pitch tilts
the upper body forward. A fixed `torso_pitch_to_pelvis_joint` preserves the
original pelvis frame and all downstream joint, mesh, collision, and sensor
transforms. The source pelvis inertia is transferred to `torso_pitch_link`,
with its COM translated into the pivot frame; its mass and COM inertia tensor
are unchanged. This gives the moving pitch link an explicit inertia without
counting the pelvis twice.

The user specified the hip-pitch axis at world zero and an invisible base.
The base and mounting frames have no visual or collision geometry and no
fabricated mass. The torso MuJoCo profile selects `root: fixed`, an initial
root position of `[0, 0, 0]`, and `floor: false`. The floor is omitted because
the retained pelvis and sensor geometry extend below the pivot.

## Parallel hip actuation

Including the grippers, there are **19 independent movable joints and 20 actuator
declarations**, plus eight passive linkage joints.
Two separate `transmission_interface/SimpleTransmission` elements name
`left_hip_pitch_motor` and `right_hip_pitch_motor` and both reference
`torso_pitch_joint`. The remaining 16 transmissions name the existing arm and
waist motors. Each transmission has one actuator and one joint, following the
[SimpleTransmission definition](https://docs.ros.org/en/iron/p/transmission_interface/generated/classtransmission__interface_1_1SimpleTransmission.html).

Both hip reductions are `-1` relative to the source's normalized hip-joint
coordinates: with the original leg-side outputs fixed, pelvis pitch reverses
the old pelvis-to-leg relative angle. For zero-aligned source coordinates,
`q_left = q_right = -q_torso` and
`tau_torso = -(tau_left + tau_right)`. These normalized coordinate mappings do
not replace physical encoder-direction and zero calibration. The original
hardware description's left/right protocol directions are separate from the
transmission reductions.

The stack parses these simple mappings alongside existing differentials.
MuJoCo creates one native tendon/actuator per motor, both wrapping the same
pitch joint, so efforts sum by virtual work.

`hardware.yaml` preserves the Mini motor names, buses, CAN IDs, models, and
protocol directions. Both hip motors belong to `torso_pitch`; the waist axes
belong to `waist`, and both arms belong to `upper_body`. The daemon requires
all actuators on a shared joint to belong to one group, rejects conflicting
joint-equivalent position/velocity targets, and checks encoder disagreement.
The pitch group explicitly allows at most 0.02 rad disagreement and disables
both motors on a safe-position fault. Each hip uses `fault_damping_kd=20`,
half the source single-motor setting, and a 1 rad/s command velocity cap.
These gains, disagreement tolerance, and inherited sensor/transport bindings
remain commissioning settings to verify on the physical assembly.

The physical and vCAN deployments are
`components/bello-hardwared/config/bello-mini-torso.yaml` and
`bello-mini-torso-vcan.yaml`. The native profiles are
`components/bello-mujoco/config/bello_mini_torso.yaml` and
`emulator_mini_torso.yaml`.
See [the bring-up guide](README.md) for commands and review artifacts.

## Travel and effort

The user specified **60 degrees forward and 30 degrees backward**, encoded as
`[-pi/6, pi/3]` in torso-pitch coordinates. Upright is zero. The joint velocity
envelope remains the source's **15 rad/s**, narrowed to 1 rad/s by the hardware
profile. The nominal joint effort
envelope is **240 Nm**, the sum of the source's two 120 Nm hip envelopes at the
assumed unit reductions. The effort envelope is inherited from the source;
it is not a measured continuous motor rating. Validate encoder zeros, effective
ratios, torque sharing, physical stops, and gains during commissioning.

The descriptions and meshes are proprietary. This repository does not grant
permission to redistribute them. External distribution requires a separate
written agreement with the applicable rights holder.
