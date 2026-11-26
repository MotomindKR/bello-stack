from __future__ import annotations

from collections.abc import Iterable, Mapping
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

from ._generated import hardware_pb2 as proto
from .errors import InvalidTargetError, InventoryError
from .types import (
    Bus,
    CoordinateTarget,
    DeviceStatus,
    Imu,
    Motor,
    MotorLimits,
    PowerBoard,
    SerialDevice,
)

if TYPE_CHECKING:
    from .control import ControlSession
    from .robot import Robot


def _status(message: proto.DeviceStatus) -> DeviceStatus:
    return DeviceStatus(message.state, message.detail, message.transition_count)


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
    initial_state: proto.GroupState
    motors: tuple[Motor, ...]


class Inventory:
    """Hardware metadata indexed at connection time."""

    def __init__(self, response: proto.ListHardwareResponse) -> None:
        self.power_boards = tuple(
            PowerBoard(
                b.name,
                b.serial_device,
                b.profile,
                b.stale_timeout_ms,
                b.sync_period_ms,
                _optional_status(b, "status"),
            )
            for b in _index_unique_names(response.power_boards, "power board").values()
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
            for item in _index_unique_names(response.imus, "IMU").values()
        )

        group_messages = _index_unique_names(response.groups, "motor group")
        grouped: dict[str, list[Motor]] = {name: [] for name in group_messages}
        motors: dict[str, Motor] = {}
        coordinates: set[str] = set()
        for item in response.motors:
            if not item.name.strip() or item.name in motors:
                raise InventoryError(f"invalid or duplicate motor {item.name!r}")
            if not item.coordinate.strip() or item.coordinate in coordinates:
                raise InventoryError(f"invalid or duplicate coordinate {item.coordinate!r}")
            if item.group not in grouped:
                raise InventoryError(f"motor {item.name!r} references unknown group {item.group!r}")
            if not item.HasField("address"):
                raise InventoryError(f"motor {item.name!r} has no bus address")
            if not item.HasField("limits"):
                raise InventoryError(f"motor {item.name!r} has no resolved command limits")
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
                item.firmware_version if item.HasField("firmware_version") else None,
            )
            motors[motor.name] = motor
            coordinates.add(motor.coordinate)
            grouped[motor.group].append(motor)

        definitions: dict[str, _GroupDefinition] = {}
        for name, message in group_messages.items():
            members = tuple(sorted(grouped[name], key=lambda motor: motor.name))
            definitions[name] = _GroupDefinition(
                name, message.control_rate_hz, message.state, members
            )

        self._definitions = definitions
        self._motors = motors

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
    def initial_state(self) -> proto.GroupState:
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
        max_pending: int = 1,
        rpc_timeout: float = 5.0,
    ) -> ControlSession:
        from .control import ControlSession

        return ControlSession._acquire(
            self,
            owner,
            lease_duration=lease_duration,
            command_validity=command_validity,
            max_pending=max_pending,
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

        commands = []
        for motor in self.motors:
            coordinate = motor.coordinate
            target = targets[coordinate]
            if not isinstance(target, CoordinateTarget):
                raise InvalidTargetError(
                    f"target for coordinate {coordinate!r} is not a CoordinateTarget"
                )
            commands.append(
                proto.MotorCommand(
                    motor_name=motor.name,
                    position=target.position_radians,
                    velocity=target.velocity_radians_per_second,
                    torque_feedforward=target.torque_feedforward_newton_metres,
                    kp=target.kp,
                    kd=target.kd,
                )
            )
        return commands
