from __future__ import annotations

import threading
import time

import pytest

from bello_hardware import Robot

from .conftest import RunningServer


class _Child:
    def __init__(self) -> None:
        self.closed = threading.Event()

    def close(self) -> None:
        self.closed.set()


class _FailingChild:
    def close(self) -> None:
        raise RuntimeError("child cleanup failed")


def test_robot_close_waits_for_reserved_child_admission(hardware_server: RunningServer) -> None:
    robot = Robot.connect(hardware_server.endpoint)
    child = _Child()
    robot._begin_child()

    closer = threading.Thread(target=robot.close)
    closer.start()
    deadline = time.monotonic() + 1.0
    while robot._state != "closing" and time.monotonic() < deadline:
        time.sleep(0.001)
    assert closer.is_alive()

    robot._finish_child(child)
    closer.join(timeout=1.0)
    assert not closer.is_alive()
    assert child.closed.is_set()


def test_robot_cleanup_does_not_mask_a_body_error(hardware_server: RunningServer) -> None:
    robot = Robot.connect(hardware_server.endpoint)
    robot._begin_child()
    robot._finish_child(_FailingChild())

    with (
        pytest.warns(RuntimeWarning, match="robot cleanup failed"),
        pytest.raises(ValueError, match="body failed"),
        robot,
    ):
        raise ValueError("body failed")
