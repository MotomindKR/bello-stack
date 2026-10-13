# Bello Mini Torso asset provenance

`urdf/bello_mini_torso.urdf` is derived from
[`bello_mini.urdf`](../bello_mini/urdf/bello_mini.urdf). It describes the Mini
upper body mounted to a fixed base through both active hip-pitch motors. Both
motors drive the same torso-pitch motion and contribute torque in parallel.

## Retained assembly

The description retains the pelvis, waist yaw and roll, both seven-joint arms,
head, hand proxies, camera, and IMU frames. Both complete leg subtrees are
removed, starting at `left_hip_pitch_link` and `right_hip_pitch_link`, along with
their ankle transmissions. The source hip-pitch child-link inertias belong to
the removed leg assemblies; the hip-pitch motors themselves are already
included in the source pelvis inertia and are retained without adding mass.

The modeled moving assembly mass is **28.180 kg**: the source's 48.580 kg minus
two 10.200 kg leg assemblies. This is a derivation from the full robot's mass
allocation, not a separate measurement of the torso variant. Shared meshes are
referenced directly from `../bello_mini/meshes/`; no mesh assets are copied.
Inherited geometry and mass approximations are documented in the
[Mini provenance](../bello_mini/ASSET_PROVENANCE.md).

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
the retained pelvis and sensor geometry extend below the pivot. The viewer
also uses a fixed identity root; no foot-contact pose estimator is required.

## Parallel hip actuation

There are **17 independent movable joints and 18 actuator declarations**.
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
pitch joint, so efforts sum by virtual work. The viewer reconstructs the joint
from both encoders and divides requested joint torque equally for the unit
reductions. Its pitch gain defaults are per motor: `kp=50`, `kd=2`, giving
combined joint stiffness 100 and damping 4 in the normalized unit mapping.

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
`emulator_mini_torso.yaml`; the viewer profile is
`components/bello-visualizer/config/bello-mini-torso.example.yaml`.
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
