# Dual 7-DOF Milly with stereo gimbal

The arm mounting plates are 450 mm apart. The gimbal base is centred between them, 426 mm above the mounting plane and 50 mm behind the J1 axes. At zero, the stereo optical centre is 607 mm above the plane and faces −X in the fixture frame. Both wrist cameras remain on the arms. [Mounting geometry](mounting.json)

One hardware daemon owns the 19 motors in two independently leased groups. The left and right arms use CAN IDs 1–8 on can0 and can1, respectively, in the dual_arm group. The gimbal group uses IDs 11–13 on can0. [Hardware inventory](hardware.yaml)

The workcell includes a ground plate and a provisional 60 × 60 mm support post. The standalone gimbal description documents internal clearance and the optional swept-volume proxy. Commission encoder zeros and check coupled gimbal travel on the installed assembly. [Gimbal geometry and clearance](../stereo-gimbal/) · [Stereo simulation](../../components/bello-sim/README.md#dual-milly-stereo)
