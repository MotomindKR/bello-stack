# Milly asset provenance

The Milly kinematic origins and collision meshes in this directory were
adapted from `platform/ros2/src/milly_ros/milly_description` at commit
`74d6c46176a91e11564e4cbbf7d183080d7cfb95` of
`MotomindKR/motomind_platform`. The calibrated motor and safety envelopes were
adapted from `configs/robots/motomind_milly/milly_teleop.yaml` at commit
`61292d0b514a09a1623be7cb5f477d0247a70d65` in the same repository.

The profile includes the source description's detailed per-part visual meshes
and lower-detail collision meshes, with package URIs rewritten as repository-
relative paths. The source ROS adapter maps one rotary gripper actuator to two
display-only prismatic joints; this profile instead represents that actuator
with a bounded virtual rotary joint so the model is directly loadable by
`bello-hardwared` and `bello-visualizer`.

The descriptions and meshes are proprietary. This repository does not grant
permission to redistribute them. External distribution requires a separate
written agreement with the applicable rights holder.
