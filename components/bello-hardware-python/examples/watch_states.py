from bello_hardware import Robot, StateUnavailableError

with Robot.connect("http://127.0.0.1:50051") as robot, robot.states() as states:
    while states.wait_for_change(timeout=1.0):
        try:
            motor = states.coordinate("emulated_joint_1")
        except StateUnavailableError:
            continue
        print(
            f"position={motor.position:.3f} "
            f"velocity={motor.velocity:.3f} "
            f"temperature={motor.temperature:.1f}"
        )
