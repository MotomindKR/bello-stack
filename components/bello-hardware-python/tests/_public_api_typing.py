from typing_extensions import assert_type

from bello_hardware import ControlSession, Motor, MotorGroup, MotorLimits, Robot, TlsCredentials


def check_public_types(robot: Robot, control: ControlSession) -> None:
    assert_type(robot.motors, tuple[Motor, ...])
    assert_type(robot.motors[0].limits, MotorLimits)
    assert_type(robot.group("leg"), MotorGroup)
    assert_type(control.group, MotorGroup)
    assert_type(TlsCredentials.from_files(), TlsCredentials)
