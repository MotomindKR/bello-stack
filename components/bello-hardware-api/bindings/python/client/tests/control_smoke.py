"""Full-stack smoke driver invoked by the Cargo vCAN integration test."""

from __future__ import annotations

import sys

from bello_hardware import CoordinateTarget, Robot


def main(endpoint: str) -> None:
    frame = {
        "e2e_joint_1": CoordinateTarget.position(0.0, kp=20.0, kd=1.0),
        "e2e_joint_2": CoordinateTarget.position(0.0, kp=20.0, kd=1.0),
    }
    with Robot.connect(endpoint) as robot:
        state = robot.states(groups=("e2e_group",)).read(timeout=1.0)
        if {motor.coordinate for motor in state.motors} != {
            "e2e_joint_1",
            "e2e_joint_2",
        }:
            raise RuntimeError("state frame does not contain the complete motor group")
        if {group.group for group in state.groups} != {"e2e_group"}:
            raise RuntimeError("state frame does not contain the selected group")

        control = robot.group("e2e_group").control(
            "bello-hardware-python-e2e",
            lease_duration=5.0,
            command_validity=0.2,
        )
        with control:
            control.start(frame)
            control.send(frame)
            control.try_submit(frame).result()
            control.stop()
            control.stop()
            control.resume(frame)
            control.disable()


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise SystemExit("usage: control_smoke.py ENDPOINT")
    main(sys.argv[1])
