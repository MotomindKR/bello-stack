from bello_hardware import CoordinateTarget, Robot

with Robot.connect("http://127.0.0.1:50051") as robot:
    group = robot.group("emulated_group")
    initial = {
        "emulated_joint_1": CoordinateTarget.position(0.0, kp=20.0, kd=1.0),
        "emulated_joint_2": CoordinateTarget.position(0.0, kp=20.0, kd=1.0),
    }
    with group.control("python-example") as control:
        control.start(initial)
        control.send(
            {
                "emulated_joint_1": CoordinateTarget.position(0.1, kp=20.0, kd=1.0),
                "emulated_joint_2": CoordinateTarget.position(-0.1, kp=20.0, kd=1.0),
            }
        )
