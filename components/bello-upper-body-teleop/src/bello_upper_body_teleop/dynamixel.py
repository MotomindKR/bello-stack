"""Synchronous Dynamixel Protocol 2.0 acquisition and weak centering feedback."""

from __future__ import annotations

import importlib
from collections.abc import Mapping, Sequence
from contextlib import suppress
from dataclasses import dataclass
from types import ModuleType
from typing import Any, Literal

from .config import (
    UPPER_BODY_JOINTS,
    JointMapping,
    TeachingArmSample,
    map_raw_positions,
)

MasterTorqueMode = Literal["disable", "enable", "unchanged"]

_ADDR_OPERATING_MODE = 11
_ADDR_CURRENT_LIMIT = 38
_ADDR_TORQUE_ENABLE = 64
_ADDR_POSITION_D_GAIN = 80
_ADDR_POSITION_I_GAIN = 82
_ADDR_POSITION_P_GAIN = 84
_ADDR_GOAL_CURRENT = 102
_ADDR_GOAL_POSITION = 116
_ADDR_PRESENT_POSITION = 132
_LEN_CURRENT_LIMIT = 2
_LEN_POSITION_GAIN = 2
_LEN_GOAL_CURRENT = 2
_LEN_GOAL_POSITION = 4
_LEN_PRESENT_POSITION = 4
_ENCODER_COUNTS_PER_TURN = 4_096
_CURRENT_BASED_POSITION_CONTROL_MODE = 5
_TORQUE_ENABLE = 1
_TORQUE_DISABLE = 0
_MASTER_P_GAIN = 10
_LIMIT_P_GAIN = _MASTER_P_GAIN * 10
_MASTER_GOAL_CURRENT = 800
_HOLD_P_GAIN = 250
_WRIST_YAW_ZERO_POSITION = 2_048
_WRIST_YAW_OPPOSITE_POSITION = _WRIST_YAW_ZERO_POSITION + _ENCODER_COUNTS_PER_TURN // 2


@dataclass(frozen=True, slots=True)
class HeldDynamixel:
    """A teaching-rig motor that holds position but does not command the robot."""

    dxl_id: int
    name: str
    goal_position: int
    p_gain: int = _HOLD_P_GAIN

    def __post_init__(self) -> None:
        if isinstance(self.dxl_id, bool) or not isinstance(self.dxl_id, int):
            raise TypeError("held Dynamixel ID must be an integer")
        if not 0 <= self.dxl_id <= 252:
            raise ValueError("held Dynamixel ID must be in the range 0..=252")
        if not self.name.strip():
            raise ValueError("held Dynamixel name must be non-empty")
        if isinstance(self.goal_position, bool) or not isinstance(
            self.goal_position,
            int,
        ):
            raise TypeError("held Dynamixel goal position must be an integer")
        if not -(2**31) <= self.goal_position < 2**31:
            raise ValueError("held Dynamixel goal position must fit a signed 32-bit value")
        if isinstance(self.p_gain, bool) or not isinstance(self.p_gain, int):
            raise TypeError("held Dynamixel P gain must be an integer")
        if not 0 <= self.p_gain <= 16_383:
            raise ValueError("held Dynamixel P gain must be in the range 0..=16383")


AUXILIARY_HELD_DYNAMIXELS: tuple[HeldDynamixel, ...] = (
    HeldDynamixel(9, "waist_yaw", 512),
    HeldDynamixel(27, "left_wrist_yaw", _WRIST_YAW_OPPOSITE_POSITION),
    HeldDynamixel(17, "right_wrist_yaw", _WRIST_YAW_OPPOSITE_POSITION),
)


class DynamixelError(RuntimeError):
    """Base class for teaching-arm acquisition failures."""


class DynamixelDependencyError(DynamixelError):
    """The Dynamixel SDK is not installed."""


class DynamixelCommunicationError(DynamixelError):
    """A port, packet, or servo operation failed."""


def _load_sdk() -> ModuleType:
    try:
        return importlib.import_module("dynamixel_sdk")
    except ModuleNotFoundError as error:
        raise DynamixelDependencyError(
            "dynamixel-sdk is required; install the bello-upper-body-teleop package "
            "into its runtime environment"
        ) from error


def _signed_32(value: int) -> int:
    value &= 0xFFFFFFFF
    return value - 0x100000000 if value & 0x80000000 else value


def _nearest_turn_equivalent(target: int, reference: int) -> int:
    turns = round((reference - target) / _ENCODER_COUNTS_PER_TURN)
    equivalent = target + turns * _ENCODER_COUNTS_PER_TURN
    if not -(2**31) <= equivalent < 2**31:
        raise ValueError("nearest Dynamixel position target does not fit signed 32 bits")
    return equivalent


class DynamixelTeachingArms:
    """Read mapped joints, center them weakly, and hold unactuated rig joints."""

    def __init__(
        self,
        *,
        port: str = "/dev/ttyUSB0",
        baudrate: int = 1_000_000,
        joints: Sequence[JointMapping] = UPPER_BODY_JOINTS,
        held_dynamixels: Sequence[HeldDynamixel] = AUXILIARY_HELD_DYNAMIXELS,
        torque_mode: MasterTorqueMode = "disable",
        sdk: Any | None = None,
    ) -> None:
        if not port.strip():
            raise ValueError("Dynamixel port must be non-empty")
        if isinstance(baudrate, bool) or not isinstance(baudrate, int):
            raise TypeError("Dynamixel baud rate must be an integer")
        if baudrate <= 0:
            raise ValueError("Dynamixel baud rate must be positive")
        if not joints:
            raise ValueError("at least one Dynamixel joint is required")
        mapped_ids = [joint.dxl_id for joint in joints]
        held_ids = [motor.dxl_id for motor in held_dynamixels]
        ids = [*mapped_ids, *held_ids]
        coordinates = [joint.coordinate for joint in joints]
        if len(set(ids)) != len(ids):
            raise ValueError("mapped and held Dynamixel IDs must be unique")
        if len(set(coordinates)) != len(coordinates):
            raise ValueError("Dynamixel joint coordinates must be unique")
        held_names = [motor.name for motor in held_dynamixels]
        if len(set(held_names)) != len(held_names):
            raise ValueError("held Dynamixel names must be unique")
        if torque_mode not in ("disable", "enable", "unchanged"):
            raise ValueError("master torque mode must be 'disable', 'enable', or 'unchanged'")

        self._joints = tuple(joints)
        self._held_dynamixels = tuple(held_dynamixels)
        self._ids = tuple(ids)
        self._sdk: Any = sdk if sdk is not None else _load_sdk()
        self._port_handler: Any = self._sdk.PortHandler(port)
        self._packet_handler: Any = self._sdk.PacketHandler(2.0)
        self._group_read: Any = self._sdk.GroupSyncRead(
            self._port_handler,
            self._packet_handler,
            _ADDR_PRESENT_POSITION,
            _LEN_PRESENT_POSITION,
        )
        self._closed = False
        self._opened = False
        self._master_torque_enabled = False
        self._disable_master_torque_on_close = False
        self._feedback_goal_positions: dict[int, int] | None = None

        try:
            if not self._port_handler.openPort():
                raise DynamixelCommunicationError(f"failed to open Dynamixel port {port!r}")
            self._opened = True
            if not self._port_handler.setBaudRate(baudrate):
                raise DynamixelCommunicationError(
                    f"failed to set Dynamixel port {port!r} to {baudrate} baud"
                )
            for dxl_id in ids:
                if not self._group_read.addParam(dxl_id):
                    raise DynamixelCommunicationError(
                        f"failed to add Dynamixel ID {dxl_id} to synchronous read"
                    )
            if torque_mode == "disable":
                self._set_master_torque(False)
            elif torque_mode == "enable":
                self._enable_master_torque()
        except BaseException:
            with suppress(BaseException):
                self.close()
            raise

    @property
    def ids(self) -> tuple[int, ...]:
        return self._ids

    @property
    def held_ids(self) -> tuple[int, ...]:
        return tuple(motor.dxl_id for motor in self._held_dynamixels)

    @property
    def held_names(self) -> tuple[str, ...]:
        return tuple(motor.name for motor in self._held_dynamixels)

    @property
    def master_torque_enabled(self) -> bool:
        """Whether this reader safely enabled and now owns teaching-arm torque."""

        return self._master_torque_enabled

    def read(self) -> TeachingArmSample:
        """Read and calibrate one complete teaching-arm position sample."""

        if self._closed:
            raise DynamixelCommunicationError("Dynamixel teaching-arm reader is closed")
        raw_positions = self._read_raw_positions()
        sample = map_raw_positions(raw_positions, self._joints)
        if self._master_torque_enabled:
            self._write_feedback(raw_positions)
        return sample

    def _read_raw_positions(self) -> dict[int, int]:
        result = int(self._group_read.txRxPacket())
        if result != int(self._sdk.COMM_SUCCESS):
            raise DynamixelCommunicationError(
                f"Dynamixel synchronous read failed: {self._communication_detail(result)}"
            )

        raw_positions: dict[int, int] = {}
        for dxl_id in self._ids:
            if not self._group_read.isAvailable(
                dxl_id,
                _ADDR_PRESENT_POSITION,
                _LEN_PRESENT_POSITION,
            ):
                raise DynamixelCommunicationError(
                    f"position is unavailable for Dynamixel ID {dxl_id}"
                )
            value = int(
                self._group_read.getData(
                    dxl_id,
                    _ADDR_PRESENT_POSITION,
                    _LEN_PRESENT_POSITION,
                )
            )
            raw_positions[dxl_id] = _signed_32(value)
        return raw_positions

    def close(self) -> None:
        if self._closed:
            return
        cleanup_error: BaseException | None = None
        if self._opened:
            if self._disable_master_torque_on_close:
                try:
                    self._set_master_torque(False)
                except BaseException as error:
                    cleanup_error = error
            self._port_handler.closePort()
            self._opened = False
        self._closed = True
        if cleanup_error is not None:
            raise cleanup_error

    def __enter__(self) -> DynamixelTeachingArms:
        return self

    def __exit__(self, exc_type: object, exc: object, traceback: object) -> Literal[False]:
        self.close()
        return False

    def _enable_master_torque(self) -> None:
        # Configuration registers and the initial goal must be written with
        # torque off. Capturing every live position before torque-on prevents a
        # stale Goal Position register from pulling an arm unexpectedly.
        self._disable_master_torque_on_close = True
        self._set_master_torque(False)
        for dxl_id in self._ids:
            mode = self._read_register(
                dxl_id,
                _ADDR_OPERATING_MODE,
                1,
                "operating mode",
            )
            if mode != _CURRENT_BASED_POSITION_CONTROL_MODE:
                raise DynamixelCommunicationError(
                    f"teaching-arm Dynamixel {dxl_id} uses operating mode "
                    f"{mode}; expected current-based position mode "
                    f"{_CURRENT_BASED_POSITION_CONTROL_MODE}"
                )
            configured_current_limit = self._read_register(
                dxl_id,
                _ADDR_CURRENT_LIMIT,
                _LEN_CURRENT_LIMIT,
                "current limit",
            )
            if configured_current_limit < _MASTER_GOAL_CURRENT:
                raise DynamixelCommunicationError(
                    f"teaching-arm Dynamixel {dxl_id} current limit "
                    f"{configured_current_limit} is below the requested "
                    f"{_MASTER_GOAL_CURRENT}"
                )

        captured_positions = self._read_raw_positions()
        self._sync_write(
            _ADDR_GOAL_POSITION,
            _LEN_GOAL_POSITION,
            captured_positions,
            "captured goal position",
        )
        zeros = dict.fromkeys(self._ids, 0)
        self._sync_write(
            _ADDR_POSITION_D_GAIN,
            _LEN_POSITION_GAIN,
            zeros,
            "position D gain",
        )
        self._sync_write(
            _ADDR_POSITION_I_GAIN,
            _LEN_POSITION_GAIN,
            zeros,
            "position I gain",
        )
        self._sync_write(
            _ADDR_POSITION_P_GAIN,
            _LEN_POSITION_GAIN,
            {
                **{joint.dxl_id: _MASTER_P_GAIN for joint in self._joints},
                **{motor.dxl_id: motor.p_gain for motor in self._held_dynamixels},
            },
            "position P gain",
        )
        self._sync_write(
            _ADDR_GOAL_CURRENT,
            _LEN_GOAL_CURRENT,
            dict.fromkeys(self._ids, _MASTER_GOAL_CURRENT),
            "goal current",
        )
        self._feedback_goal_positions = {
            **{
                joint.dxl_id: _nearest_turn_equivalent(
                    joint.raw_position_for_robot_position(
                        joint.default_position_radians,
                    ),
                    captured_positions[joint.dxl_id],
                )
                for joint in self._joints
            },
            **{
                motor.dxl_id: _nearest_turn_equivalent(
                    motor.goal_position,
                    captured_positions[motor.dxl_id],
                )
                for motor in self._held_dynamixels
            },
        }
        self._set_master_torque(True)
        self._master_torque_enabled = True
        self._write_feedback(captured_positions)

    def _write_feedback(self, raw_positions: Mapping[int, int]) -> None:
        if self._feedback_goal_positions is None:
            raise DynamixelCommunicationError(
                "teaching-arm feedback goals are unavailable while torque is enabled"
            )
        self._sync_write(
            _ADDR_GOAL_POSITION,
            _LEN_GOAL_POSITION,
            self._feedback_goal_positions,
            "feedback goal positions",
        )
        self._sync_write(
            _ADDR_POSITION_P_GAIN,
            _LEN_POSITION_GAIN,
            {
                **{
                    joint.dxl_id: (
                        _LIMIT_P_GAIN
                        if self._joint_limit_active(
                            joint,
                            raw_positions[joint.dxl_id],
                        )
                        else _MASTER_P_GAIN
                    )
                    for joint in self._joints
                },
                **{motor.dxl_id: motor.p_gain for motor in self._held_dynamixels},
            },
            "position feedback P gain",
        )

    @staticmethod
    def _joint_limit_active(joint: JointMapping, raw_position: int) -> bool:
        position = joint.unclamped_robot_position(raw_position)
        return position <= joint.minimum_radians or position >= joint.maximum_radians

    def _set_master_torque(self, enabled: bool) -> None:
        value = _TORQUE_ENABLE if enabled else _TORQUE_DISABLE
        action = "enable" if enabled else "disable"
        self._sync_write(
            _ADDR_TORQUE_ENABLE,
            1,
            dict.fromkeys(self._ids, value),
            f"torque-{action}",
        )

        for dxl_id in self._ids:
            torque_enabled = self._read_register(
                dxl_id,
                _ADDR_TORQUE_ENABLE,
                1,
                f"torque-{action}",
            )
            if torque_enabled != value:
                state = "enabled" if torque_enabled else "disabled"
                raise DynamixelCommunicationError(
                    f"teaching-arm Dynamixel {dxl_id} still reports torque "
                    f"{state}; expected {action}d"
                )
        self._master_torque_enabled = enabled

    def _sync_write(
        self,
        address: int,
        length: int,
        values: Mapping[int, int],
        description: str,
    ) -> None:
        # A Sync Write is a broadcast instruction, so Dynamixels do not emit
        # per-servo status packets regardless of their Status Return Level.
        # This avoids leaving write acknowledgements queued before the explicit
        # readbacks used for safety-critical state.
        group_write = self._sdk.GroupSyncWrite(
            self._port_handler,
            self._packet_handler,
            address,
            length,
        )
        try:
            for dxl_id in self._ids:
                value = values[dxl_id]
                encoded = [
                    (value & ((1 << (length * 8)) - 1)) >> (byte * 8) & 0xFF
                    for byte in range(length)
                ]
                if not group_write.addParam(dxl_id, encoded):
                    raise DynamixelCommunicationError(
                        f"failed to add teaching-arm Dynamixel {dxl_id} "
                        f"to {description} synchronous write"
                    )
            result = group_write.txPacket()
            if int(result) != int(self._sdk.COMM_SUCCESS):
                raise DynamixelCommunicationError(
                    f"failed to send teaching-arm {description} synchronous write: "
                    f"{self._communication_detail(int(result))}"
                )
        finally:
            group_write.clearParam()

    def _read_register(
        self,
        dxl_id: int,
        address: int,
        length: int,
        description: str,
    ) -> int:
        if length == 1:
            value, result, error = self._packet_handler.read1ByteTxRx(
                self._port_handler,
                dxl_id,
                address,
            )
        elif length == 2:
            value, result, error = self._packet_handler.read2ByteTxRx(
                self._port_handler,
                dxl_id,
                address,
            )
        else:
            raise ValueError("only one- and two-byte register reads are supported")
        if int(result) != int(self._sdk.COMM_SUCCESS) or int(error) != 0:
            raise DynamixelCommunicationError(
                f"failed to read {description} on teaching-arm Dynamixel {dxl_id}: "
                f"{self._communication_detail(int(result))}, device_error={int(error)}"
            )
        return int(value)

    def _communication_detail(self, result: int) -> str:
        detail = self._packet_handler.getTxRxResult(result)
        return str(detail)
