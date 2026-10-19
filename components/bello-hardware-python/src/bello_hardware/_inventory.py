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
    MotorLimits,
    PowerBoard,
    SerialDevice,
)

_MAX_CAN_ID = 0xFF
_MAX_ROBSTRIDE_MOTOR_ID = 127
_SUPPORTED_ROBSTRIDE_MODELS = frozenset(
    {
        "ROBSTRIDE_00",
        "ROBSTRIDE_01",
        "ROBSTRIDE_02",
        "ROBSTRIDE_03",
        "ROBSTRIDE_04",
        "ROBSTRIDE_05",
        "ROBSTRIDE_06",
    }
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


def _index_unique_names(messages: Iterable[Any], kind: str) -> dict[str, Any]:
    indexed: dict[str, Any] = {}
    for message in messages:
        name = message.name
        if not name.strip() or name in indexed:
            raise InventoryError(f"invalid or duplicate {kind} {name!r}")
        indexed[name] = message
    return indexed


@dataclass(frozen=True, slots=True)
class _GroupDefinition:
    name: str
    control_rate_hz: float
    initial_state: GroupState
    motors: tuple[Motor, ...]


class Inventory:
    """Validated hardware metadata returned at connection time."""

    def __init__(self, response: proto.ListHardwareResponse) -> None:
        buses = _index_unique_names(response.buses, "bus")
        serial_devices = _index_unique_names(response.serial_devices, "serial device")
        _index_unique_names(response.imus, "IMU")

        transports: set[str] = set()
        for item in response.buses:
            if not item.transport.strip():
                raise InventoryError(f"bus {item.name!r} has no transport")
            if item.host_id > _MAX_CAN_ID:
                raise InventoryError(f"bus {item.name!r} host ID must be in 0..={_MAX_CAN_ID}")
            if item.transport in transports:
                raise InventoryError(f"CAN transport {item.transport!r} is assigned more than once")
            transports.add(item.transport)

        ports: set[str] = set()
        for item in response.serial_devices:
            if not item.port.strip():
                raise InventoryError(f"serial device {item.name!r} has no port")
            if item.baud_rate <= 0:
                raise InventoryError(f"serial device {item.name!r} has an invalid baud rate")
            if item.port in ports:
                raise InventoryError(f"serial port {item.port!r} is assigned more than once")
            ports.add(item.port)

        claimed_serial_devices: set[str] = set()
        for item in response.imus:
            if not item.source_frame.strip():
                raise InventoryError(f"IMU {item.name!r} has no source frame")
            if not item.output_frame.strip():
                raise InventoryError(f"IMU {item.name!r} has no output frame")
            if item.serial_device not in serial_devices:
                raise InventoryError(
                    f"IMU {item.name!r} references unknown serial device {item.serial_device!r}"
                )
            if item.serial_device in claimed_serial_devices:
                raise InventoryError(
                    f"serial device {item.serial_device!r} is assigned to more than one IMU"
                )
            claimed_serial_devices.add(item.serial_device)
        _index_unique_names(response.power_boards, "power board")
        for board in response.power_boards:
            if (
                board.serial_device not in serial_devices
                or board.serial_device in claimed_serial_devices
            ):
                raise InventoryError("invalid power board serial ownership")
            if board.profile not in ("agx", "nano_nx") or board.stale_timeout_ms <= 0:
                raise InventoryError("invalid power board profile or freshness limit")
            claimed_serial_devices.add(board.serial_device)
        self.power_boards = tuple(
            PowerBoard(
                b.name,
                b.serial_device,
                b.profile,
                b.stale_timeout_ms,
                b.sync_period_ms,
                _optional_status(b, "status"),
            )
            for b in response.power_boards
        )
        unused_serial_devices = sorted(set(serial_devices) - claimed_serial_devices)
        if unused_serial_devices:
            raise InventoryError(
                f"serial devices are not assigned to a sensor: {unused_serial_devices}"
            )

        self.buses = tuple(
            Bus(item.name, item.transport, item.host_id, _optional_status(item, "status"))
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
            Imu(
                item.name,
                item.serial_device,
                item.source_frame,
                item.output_frame,
                _optional_status(item, "status"),
            )
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
        addresses: set[tuple[str, int]] = set()
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
            if item.address.bus not in buses:
                raise InventoryError(
                    f"motor {item.name!r} references unknown bus {item.address.bus!r}"
                )
            if item.address.id > _MAX_ROBSTRIDE_MOTOR_ID:
                raise InventoryError(
                    f"motor {item.name!r} ID must be in 0..={_MAX_ROBSTRIDE_MOTOR_ID}"
                )
            if item.address.id == buses[item.address.bus].host_id:
                raise InventoryError(f"motor {item.name!r} ID conflicts with bus host ID")
            address = (item.address.bus, item.address.id)
            if address in addresses:
                raise InventoryError(
                    f"motor address {item.address.bus}:{item.address.id} is assigned more than once"
                )
            addresses.add(address)
            if not item.model.strip():
                raise InventoryError(f"motor {item.name!r} has no model")
            if item.model not in _SUPPORTED_ROBSTRIDE_MODELS:
                raise InventoryError(
                    f"motor {item.name!r} has unsupported Robstride model {item.model!r}"
                )
            if item.direction not in (-1, 1):
                raise InventoryError(
                    f"motor {item.name!r} direction must be -1 or 1, got {item.direction}"
                )
            if not item.HasField("limits"):
                raise InventoryError(f"motor {item.name!r} has no resolved command limits")
            try:
                limits = MotorLimits(
                    item.limits.position_min,
                    item.limits.position_max,
                    item.limits.velocity_min,
                    item.limits.velocity_max,
                    item.limits.torque_min,
                    item.limits.torque_max,
                    item.limits.kp_min,
                    item.limits.kp_max,
                    item.limits.kd_min,
                    item.limits.kd_max,
                )
            except ValueError as error:
                raise InventoryError(f"motor {item.name!r} has invalid command limits") from error
            motor = Motor(
                item.name,
                item.coordinate,
                item.group,
                item.address.bus,
                item.address.id,
                item.model,
                item.direction,
                limits,
                _optional_status(item, "status"),
            )
            motors[motor.name] = motor
            coordinates[motor.coordinate] = motor.name
            grouped[motor.group].append(motor)

        unused_buses = sorted(set(buses) - {bus for bus, _motor_id in addresses})
        if unused_buses:
            raise InventoryError(f"CAN buses have no motors: {unused_buses}")

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
