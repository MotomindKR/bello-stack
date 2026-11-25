# Dual 7-DOF Milly with Linker O6 hands and stereo gimbal

This fixture mounts the [rigid O6 palms](../milly-7dof-o6/) on the two arms of the [450 mm stereo workcell](../milly-7dof-gimbal/). Both thumb edges point up in the review pose. Each palm has one collision box, a 360 g hand mass and a fixed end-effector frame. The gimbal carries the stereo pair and provides the only cameras on this fixture.

| Device | Bus and address | Owner |
| --- | --- | --- |
| Left and right arm joints | can0 and can1, IDs 1–7 per arm | hardwared, dual_arm group |
| Gimbal yaw, pitch, roll | can0, IDs 11–13 | hardwared, gimbal group |
| Left and right O6 | can0 ID 0x28, can1 ID 0x27 | bello-hand, Linux SocketCAN |

Use the same [physical deployment](../../components/bello-hardwared/config/milly-7dof-o6-gimbal.yaml) or [vCAN deployment](../../components/bello-hardwared/config/milly-7dof-o6-gimbal-vcan.yaml) for hardwared and the hand CLI. The [hand inventory](hands.yaml) maps each address to its palm frame. The arm settings come from the O6 profile, including the J3 command limits of 0.2 rad/s and 2 N·m. The gimbal uses the stereo fixture’s motor settings. Check the installed hand addresses and mount offsets before commissioning.

## Commission the assembled workcell

1. **Mounts and stopping.** With drive power isolated, check the J7 plates, palm fixtures, gimbal fasteners and cable routing. Support gravity-loaded parts. Verify the installed emergency stop and restart behavior for the arms, gimbal and both independently driven hands in the safeguarded commissioning setup. A hand can retain its target after a driver or bus failure. [Hand driver](../../components/bello-hand/README.md) · [Robot commissioning guidance](https://www.osha.gov/otm/section-4-safety-hazards/chapter-4)
2. **Device mapping and zero.** With motion inhibited, check the installed addresses against the table above and read positions and fault bytes from each hand. Establish each arm and gimbal encoder zero against a physical reference. In reduced-speed manual operation, verify one axis and direction at a time. The profile sets J3 direction to −1 on both arms and limits its commands to 0.2 rad/s and 2 N·m. Verify all six finger channels independently with the hand unloaded. [Hardware inventory and limits](hardware.yaml)
3. **Palm frames and clearance.** Measure each J7-to-palm transform on the assembly. Check thumbs up and palms inward in the documented review pose, then compare each measured palm centre and axis with its URDF EEF. Verify finger and cable clearance separately from the palm collision box. [Mount dimensions and review pose](../milly-7dof-o6/README.md)
4. **Coupled gimbal travel.** CAD shows the camera housing intersects the rear yaw support at pitch +90° combined with roll +90° or −90°. Exclude those combinations before powered travel. Establish and enforce a coupled operating envelope that clears the housing, routed cables, and both arms before enabling automatic gimbal commands. [CAD clearance measurements](../stereo-gimbal/clearance-check.json)
5. **Shared CAN under load.** Record utilization, CAN errors and motor feedback age on each 1 Mbit/s bus with the arms and gimbal active, then repeat with hand traffic. Check the 75% configured utilization budget and the 250 ms group feedback timeout. Verify fault, stop and deliberate restart behavior for each owner under controlled conditions. [Group settings](hardware.yaml) · [Coordinated hand integration](https://github.com/MotomindKR/bello-stack/issues/73)
6. **Stereo and dynamics.** On the physical USB feed, cover each lens in turn to verify the left-first 3200 × 1200 layout. Image a calibration target at multiple gimbal poses and check per-eye reprojection, stereo alignment, mount orientation, frame timing and drops. Record installed wrist-fixture and gimbal mass, centre of mass and inertia before relying on model-based dynamics. [Camera calibration](../../components/bello-sim/README.md#dual-milly-stereo) · [Provisional mass properties](https://github.com/MotomindKR/bello-stack/issues/78)

## Simulation

Run these commands in separate terminals to start the fixed-hand model with one calibrated, left-first 3200 × 1200 stereo stream and the matching loopback arm and gimbal deployment. Finger motion is outside this model.

```sh
cargo run -p bello-sim -- run --config components/bello-sim/config/emulator_milly_7dof_o6_gimbal.yaml
cargo run -p bello-hardwared -- --config components/bello-hardwared/config/milly-7dof-o6-gimbal-sim.yaml
```

Both fixtures use the [same stereo calibration and frame service](../../components/bello-sim/README.md#dual-milly-stereo). The [model profile](../../components/bello-sim/config/milly_7dof_o6_gimbal.yaml) supplies provisional gimbal inertias.
