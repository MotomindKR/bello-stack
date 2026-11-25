# Dual 7-DOF Milly with Linker O6 hands and stereo gimbal

This fixture mounts the [rigid O6 palms](../milly-7dof-o6/) on the two arms of the [450 mm stereo workcell](../milly-7dof-gimbal/). Both thumb edges point up in the review pose. Each palm has one collision box, a 360 g hand mass and a fixed end-effector frame. The gimbal carries the stereo pair and provides the only cameras on this fixture.

| Device | Bus and address | Owner |
| --- | --- | --- |
| Left and right arm joints | can0 and can1, IDs 1–7 per arm | hardwared, dual_arm group |
| Gimbal yaw, pitch, roll | can0, IDs 11–13 | hardwared, gimbal group |
| Left and right O6 | can0 ID 0x28, can1 ID 0x27 | bello-hand, Linux SocketCAN |

Use the same [physical deployment](../../components/bello-hardwared/config/milly-7dof-o6-gimbal.yaml) or [vCAN deployment](../../components/bello-hardwared/config/milly-7dof-o6-gimbal-vcan.yaml) for hardwared and the hand CLI. The [hand inventory](hands.yaml) maps each address to its palm frame. The arm settings come from the O6 profile, including the J3 command limits of 0.2 rad/s and 2 N·m. The gimbal uses the stereo fixture’s motor settings. Check the installed hand addresses and mount offsets before commissioning.

## Simulation

Run these commands in separate terminals to start the fixed-hand model with two calibrated camera streams and the matching loopback arm and gimbal deployment. Finger motion is outside this model.

```sh
cargo run -p bello-sim -- run --config components/bello-sim/config/emulator_milly_7dof_o6_gimbal.yaml
cargo run -p bello-hardwared -- --config components/bello-hardwared/config/milly-7dof-o6-gimbal-sim.yaml
```

Both fixtures use the [same stereo calibration and frame service](../../components/bello-sim/README.md#dual-milly-stereo). The [model profile](../../components/bello-sim/config/milly_7dof_o6_gimbal.yaml) supplies provisional gimbal inertias.
