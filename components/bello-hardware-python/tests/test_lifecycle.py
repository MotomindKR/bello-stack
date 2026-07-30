from __future__ import annotations

import threading
import time
from typing import cast

import grpc  # type: ignore[import-untyped]
import pytest

from bello_hardware._inventory import Inventory
from bello_hardware.robot import Robot


class _Child:
    def __init__(self) -> None:
        self.closed = threading.Event()

    def close(self) -> None:
        self.closed.set()


class _FailingChild:
    def close(self) -> None:
        raise RuntimeError("child cleanup failed")


def test_robot_close_waits_for_reserved_child_admission() -> None:
    channel = grpc.insecure_channel("127.0.0.1:1")
    robot = Robot(channel, cast(Inventory, object()), rpc_timeout=0.2)
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


def test_robot_cleanup_does_not_mask_a_body_error() -> None:
    channel = grpc.insecure_channel("127.0.0.1:1")
    robot = Robot(channel, cast(Inventory, object()), rpc_timeout=0.2)
    robot._begin_child()
    robot._finish_child(_FailingChild())

    with (
        pytest.warns(RuntimeWarning, match="robot cleanup failed"),
        pytest.raises(ValueError, match="body failed"),
        robot,
    ):
        raise ValueError("body failed")
