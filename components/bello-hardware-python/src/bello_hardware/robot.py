"""Top-level blocking robot connection."""

from __future__ import annotations

import threading
import warnings
from contextlib import suppress
from typing import Any, Literal, Protocol

import grpc  # type: ignore[import-untyped]

from ._channel import create_channel
from ._generated import hardware_pb2 as proto
from ._generated import hardware_pb2_grpc
from ._inventory import Inventory, MotorGroup
from .errors import BelloError, RobotCleanupError, rpc_error
from .frames import RobotStateReader
from .states import RobotStates
from .tls import TlsCredentials
from .types import Motor


class _Closeable(Protocol):
    def close(self) -> None: ...


class Robot:
    """Connected robot with a cached, validated hardware inventory."""

    def __init__(
        self,
        channel: object,
        inventory: Inventory,
        *,
        rpc_timeout: float,
    ) -> None:
        self._channel: Any = channel
        self._stub: Any = hardware_pb2_grpc.HardwareServiceStub(  # type: ignore[no-untyped-call]
            channel
        )
        self.inventory = inventory
        self._rpc_timeout = rpc_timeout
        self._children: set[_Closeable] = set()
        self._lifecycle = threading.Condition()
        self._state = "open"
        self._pending_children = 0
        self._close_error: RobotCleanupError | None = None

    @classmethod
    def connect(
        cls,
        endpoint: str = "http://127.0.0.1:50051",
        *,
        timeout: float = 5.0,
        tls: TlsCredentials | None = None,
    ) -> Robot:
        channel = create_channel(endpoint, timeout=timeout, tls=tls)
        stub: Any = hardware_pb2_grpc.HardwareServiceStub(  # type: ignore[no-untyped-call]
            channel
        )
        try:
            response = stub.ListHardware(proto.ListHardwareRequest(), timeout=timeout)
            inventory = Inventory(response)
        except grpc.RpcError as error:
            channel.close()
            raise rpc_error("list hardware", error) from error
        except BaseException:
            channel.close()
            raise
        return cls(channel, inventory, rpc_timeout=timeout)

    @property
    def group_names(self) -> tuple[str, ...]:
        return self.inventory.group_names

    @property
    def motors(self) -> tuple[Motor, ...]:
        return self.inventory.motors

    def group(self, name: str) -> MotorGroup:
        definition = self.inventory._definitions.get(name)
        if definition is None:
            raise KeyError(f"unknown motor group {name!r}")
        return MotorGroup(self, definition)

    def states(
        self,
        *,
        groups: tuple[str, ...] = (),
        imus: tuple[str, ...] = (),
        reconnect: bool = True,
        reconnect_delay: float = 0.2,
    ) -> RobotStates:
        self._ensure_open()
        return RobotStates(
            self,
            groups=groups,
            imus=imus,
            reconnect=reconnect,
            reconnect_delay=reconnect_delay,
        )

    def state_reader(
        self,
        *,
        groups: tuple[str, ...] = (),
        imus: tuple[str, ...] = (),
    ) -> RobotStateReader:
        self._ensure_open()
        return RobotStateReader(
            self,
            groups=groups,
            imus=imus,
        )

    def close(self) -> None:
        with self._lifecycle:
            if self._state == "closed":
                if self._close_error is not None:
                    raise self._close_error
                return
            if self._state == "closing":
                self._lifecycle.wait_for(lambda: self._state == "closed")
                if self._close_error is not None:
                    raise self._close_error
                return
            self._state = "closing"
            self._lifecycle.wait_for(lambda: self._pending_children == 0)
            children = tuple(self._children)
        errors: list[BaseException] = []
        for child in children:
            try:
                child.close()
            except BaseException as error:
                errors.append(error)
        try:
            self._channel.close()
        except BaseException as error:
            errors.append(error)
        close_error = RobotCleanupError(errors) if errors else None
        with self._lifecycle:
            self._close_error = close_error
            self._state = "closed"
            self._lifecycle.notify_all()
        if close_error is not None:
            raise close_error

    def __enter__(self) -> Robot:
        self._ensure_open()
        return self

    def __exit__(self, exc_type: object, exc: object, traceback: object) -> Literal[False]:
        try:
            self.close()
        except RobotCleanupError as cleanup:
            if exc is None:
                raise
            with suppress(Warning):
                warnings.warn(str(cleanup), RuntimeWarning, stacklevel=2)
        return False

    def _new_stub(self) -> Any:
        with self._lifecycle:
            if self._state == "closed":
                raise BelloError("robot connection is closed")
        return hardware_pb2_grpc.HardwareServiceStub(  # type: ignore[no-untyped-call]
            self._channel
        )

    def _begin_child(self) -> None:
        with self._lifecycle:
            if self._state != "open":
                raise BelloError("robot connection is closing or closed")
            self._pending_children += 1

    def _finish_child(self, child: _Closeable | None) -> None:
        with self._lifecycle:
            if self._pending_children == 0:
                raise RuntimeError("child admission was not reserved")
            if child is not None:
                self._children.add(child)
            self._pending_children -= 1
            self._lifecycle.notify_all()

    def _unregister(self, child: _Closeable) -> None:
        with self._lifecycle:
            self._children.discard(child)

    def _ensure_open(self) -> None:
        with self._lifecycle:
            if self._state != "open":
                raise BelloError("robot connection is closing or closed")
