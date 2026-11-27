from bello_hardware import Robot

with Robot.connect("http://127.0.0.1:50051") as robot:
    reader = robot.states()
    while True:
        frame = reader.read(timeout=1.0)
        motor = next(motor for motor in frame.motors if motor.coordinate == "emulated_joint_1")
        print(
            f"position={motor.position:.3f} "
            f"velocity={motor.velocity:.3f} "
            f"temperature={motor.temperature:.1f}"
        )
