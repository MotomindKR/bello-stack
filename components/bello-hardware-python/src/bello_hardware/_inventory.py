"""Inventory conversion and the public motor-group view."""

from __future__ import annotations

import math
from collections.abc import Iterable, Mapping
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

from ._generated import hardware_pb2 as proto
from .errors import InvalidTargetError, InventoryError
from .types import (
    Bus,
    CoordinateTarget,
    DeviceState,
    DeviceStatus,
    GroupState,
    Imu,
    Motor,
    SerialDevice,
)

if TYPE_CHECKING:
    from .control import ControlSession
    from .robot import Robot


def _status(message: proto.DeviceStatus) -> DeviceStatus:
    try:
        state = DeviceState(message.state)
    except ValueError as error:
        raise InventoryError(f"unknown device state {message.state}") from error
    return DeviceStatus(state, message.detail, message.transition_count)


def _optional_status(message: object, field: str) -> DeviceStatus | None:
    if not message.HasField(field):  # type: ignore[attr-defined]
        return None
    return _status(getattr(message, field))


def _require_unique_names(messages: Iterable[Any], kind: str) -> None:
    names: set[str] = set()
    for message in messages:
        name = message.name
        if not name.strip() or name in names:
            raise InventoryError(f"invalid or duplicate {kind} {name!r}")
        names.add(name)


@dataclass(frozen=True, slots=True)
class _GroupDefinition:
    name: str
    control_rate_hz: float
    initial_state: GroupState
    motors: tuple[Motor, ...]


class Inventory:
    """Validated hardware metadata returned at connection time."""

    def __init__(self, response: proto.ListHardwareResponse) -> None:
        _require_unique_names(response.buses, "bus")
        _require_unique_names(response.serial_devices, "serial device")
        _require_unique_names(response.imus, "IMU")
        self.buses = tuple(
            Bus(item.name, item.interface, item.host_id, _optional_status(item, "status"))
            for item in response.buses
        )
        self.serial_devices = tuple(
            SerialDevice(
                item.name,
                item.port,
                item.baud_rate,
                _optional_status(item, "status"),
            )
            for item in response.serial_devices
        )
        self.imus = tuple(
            Imu(item.name, item.serial_device, _optional_status(item, "status"))
            for item in response.imus
        )

        group_messages: dict[str, proto.GroupInfo] = {}
        for group in response.groups:
            if not group.name.strip() or group.name in group_messages:
                raise InventoryError(f"invalid or duplicate motor group {group.name!r}")
            if not math.isfinite(group.control_rate_hz) or group.control_rate_hz <= 0.0:
                raise InventoryError(f"motor group {group.name!r} has an invalid control rate")
            group_messages[group.name] = group

        motors: dict[str, Motor] = {}
        coordinates: dict[str, str] = {}
        grouped: dict[str, list[Motor]] = {name: [] for name in group_messages}
        for item in response.motors:
            if not item.name.strip() or not item.coordinate.strip() or not item.group.strip():
                raise InventoryError("motor name, coordinate, and group must be non-empty")
            if item.name in motors:
                raise InventoryError(f"duplicate motor {item.name!r}")
            if item.coordinate in coordinates:
                raise InventoryError(f"coordinate {item.coordinate!r} is assigned more than once")
            if item.group not in grouped:
                raise InventoryError(f"motor {item.name!r} references unknown group {item.group!r}")
            if not item.HasField("address") or not item.address.bus.strip():
                raise InventoryError(f"motor {item.name!r} has no valid bus address")
            motor = Motor(
                item.name,
                item.coordinate,
                item.group,
                item.address.bus,
                item.address.id,
                item.model,
                item.direction,
                _optional_status(item, "status"),
            )
            motors[motor.name] = motor
            coordinates[motor.coordinate] = motor.name
            grouped[motor.group].append(motor)

        definitions: dict[str, _GroupDefinition] = {}
        for name, message in group_messages.items():
            members = tuple(sorted(grouped[name], key=lambda motor: motor.name))
            if not members:
                raise InventoryError(f"motor group {name!r} has no motors")
            reported = {(address.bus, address.id) for address in message.motors}
            resolved = {(motor.bus, motor.id) for motor in members}
            if len(message.motors) != len(members) or reported != resolved:
                raise InventoryError(f"motor group {name!r} membership is inconsistent")
            try:
                state = GroupState(message.state)
            except ValueError as error:
                raise InventoryError(f"unknown state {message.state} for group {name!r}") from error
            definitions[name] = _GroupDefinition(name, message.control_rate_hz, state, members)

        self._definitions = definitions
        self._motors = motors
        self._coordinates = coordinates

    @property
    def motors(self) -> tuple[Motor, ...]:
        return tuple(self._motors[name] for name in sorted(self._motors))

    @property
    def group_names(self) -> tuple[str, ...]:
        return tuple(sorted(self._definitions))


class MotorGroup:
    """A configured motor group resolved from daemon inventory."""

    def __init__(self, robot: Robot, definition: _GroupDefinition) -> None:
        self._robot = robot
        self._definition = definition

    @property
    def name(self) -> str:
        return self._definition.name

    @property
    def control_rate_hz(self) -> float:
        return self._definition.control_rate_hz

    @property
    def initial_state(self) -> GroupState:
        return self._definition.initial_state

    @property
    def motors(self) -> tuple[Motor, ...]:
        return self._definition.motors

    @property
    def coordinates(self) -> tuple[str, ...]:
        return tuple(motor.coordinate for motor in self.motors)

    def control(
        self,
        owner: str,
        *,
        lease_duration: float | None = None,
        command_validity: float | None = None,
        acknowledgement_timeout: float = 1.0,
        rpc_timeout: float = 5.0,
    ) -> ControlSession:
        from .control import ControlSession

        return ControlSession.acquire(
            self,
            owner,
            lease_duration=lease_duration,
            command_validity=command_validity,
            acknowledgement_timeout=acknowledgement_timeout,
            rpc_timeout=rpc_timeout,
        )

    def _commands(self, targets: Mapping[str, CoordinateTarget]) -> list[proto.MotorCommand]:
        expected = set(self.coordinates)
        actual = set(targets)
        unknown = sorted(actual - expected)
        missing = sorted(expected - actual)
        if unknown:
            raise InvalidTargetError(f"unknown coordinates for group {self.name!r}: {unknown}")
        if missing:
            raise InvalidTargetError(f"incomplete frame for group {self.name!r}: missing {missing}")

        by_coordinate = {motor.coordinate: motor for motor in self.motors}
        commands = []
        for coordinate in self.coordinates:
            target = targets[coordinate]
            if not isinstance(target, CoordinateTarget):
                raise InvalidTargetError(
                    f"target for coordinate {coordinate!r} is not a CoordinateTarget"
                )
            commands.append(
                proto.MotorCommand(
                    motor_name=by_coordinate[coordinate].name,
                    position=target.position_radians,
                    velocity=target.velocity_radians_per_second,
                    torque_feedforward=target.torque_feedforward_newton_metres,
                    kp=target.kp,
                    kd=target.kd,
                )
            )
        return commands
