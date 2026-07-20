from __future__ import annotations

import threading
import time
from typing import Any, cast

import grpc  # type: ignore[import-untyped]
import pytest

from bello_hardware._inventory import Inventory
from bello_hardware.robot import Robot
from bello_hardware.states import RobotStates


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


class _BlockingCall:
    def __init__(self) -> None:
        self.cancelled = threading.Event()

    def __iter__(self) -> _BlockingCall:
        return self

    def __next__(self) -> Any:
        self.cancelled.wait()
        raise StopIteration

    def cancel(self) -> None:
        self.cancelled.set()


class _BlockingStub:
    def __init__(
        self,
        entered: threading.Event,
        release: threading.Event,
        call: _BlockingCall,
    ) -> None:
        self._entered = entered
        self._release = release
        self._call = call

    def WatchTelemetry(self, request: Any) -> _BlockingCall:
        del request
        self._entered.set()
        self._release.wait()
        return self._call


class _EmptyInventory:
    def __init__(self) -> None:
        self.imus: tuple[()] = ()
        self._joints: dict[str, str] = {}
        self._motors: dict[str, Any] = {}


class _BlockingRobot:
    group_names: tuple[str, ...] = ()
    _rpc_timeout = 0.2

    def __init__(self) -> None:
        self.inventory = _EmptyInventory()
        self.entered = threading.Event()
        self.release = threading.Event()
        self.call = _BlockingCall()
        self.unregistered = threading.Event()

    def _begin_child(self) -> None:
        pass

    def _finish_child(self, child: object) -> None:
        del child

    def _unregister(self, child: object) -> None:
        del child
        self.unregistered.set()

    def _new_stub(self) -> _BlockingStub:
        return _BlockingStub(self.entered, self.release, self.call)


def test_telemetry_close_cancels_a_stream_opened_during_shutdown() -> None:
    robot = _BlockingRobot()
    states = RobotStates(cast(Any, robot), groups=(), imus=(), reconnect=True, reconnect_delay=0.1)
    assert robot.entered.wait(1.0)

    closer = threading.Thread(target=states.close)
    closer.start()
    time.sleep(0.02)
    robot.release.set()
    closer.join(timeout=1.0)

    assert not closer.is_alive()
    assert not states._thread.is_alive()
    assert robot.call.cancelled.is_set()
    assert robot.unregistered.is_set()
