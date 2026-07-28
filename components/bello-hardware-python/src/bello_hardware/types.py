"""Small immutable values used by the public client API."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum, IntEnum


class DeviceState(IntEnum):
    UNSPECIFIED = 0
    STARTING = 1
    CONNECTING = 2
    ONLINE = 3
    DEGRADED = 4
    RECONNECTING = 5
    FAULT = 6
    SHUTDOWN = 7


class GroupState(IntEnum):
    UNSPECIFIED = 0
    IDLE = 1
    ENABLED = 2
    RUNNING = 3
    FAULT = 4
    SHUTDOWN = 5


class CommandResult(IntEnum):
    UNSPECIFIED = 0
    ACCEPTED = 1
    UNKNOWN_GROUP = 2
    LEASE_LOST = 3
    INVALID_STATE = 4
    INCOMPLETE_GROUP = 5
    STALE_SEQUENCE = 6
    UNSAFE = 7
    SUPERSEDED = 8
    CONTROLLER_STOPPED = 9
    EXPIRED = 10


class FaultCause(IntEnum):
    UNSPECIFIED = 0
    COMMAND_LIMIT = 1
    COMMAND_POSITION_STEP = 2
    COMMAND_TIMEOUT = 3
    SAFE_POSITION = 4
    STALE_FEEDBACK = 5
    TEMPERATURE = 6
    REPORTED_MOTOR_FAULT = 7
    MODE_STATUS = 8
    COMMUNICATION = 9
    ENABLE = 10
    START = 11
    STOP = 12
    TRANSMISSION = 13
    LEASE_LOSS = 14
    DISABLE = 15
    RECOVER = 16
    EXIT = 17
    INTERNAL = 18


class ControlState(Enum):
    READY = "ready"
    ENABLED = "enabled"
    RUNNING = "running"
    UNCERTAIN = "uncertain"
    CLOSED = "closed"


@dataclass(frozen=True, slots=True)
class DeviceStatus:
    state: DeviceState
    detail: str
    transition_count: int


@dataclass(frozen=True, slots=True)
class MotorLimits:
    position_min: float
    position_max: float
    velocity_min: float
    velocity_max: float
    torque_min: float
    torque_max: float
    kp_min: float
    kp_max: float
    kd_min: float
    kd_max: float

    def __post_init__(self) -> None:
        for name, minimum, maximum in (
            ("position", self.position_min, self.position_max),
            ("velocity", self.velocity_min, self.velocity_max),
            ("torque", self.torque_min, self.torque_max),
            ("kp", self.kp_min, self.kp_max),
            ("kd", self.kd_min, self.kd_max),
        ):
            if not math.isfinite(minimum) or not math.isfinite(maximum) or minimum >= maximum:
                raise ValueError(f"{name} limits must be a finite increasing range")
        if self.kp_min < 0.0 or self.kd_min < 0.0:
            raise ValueError("gain limits must be non-negative")


@dataclass(frozen=True, slots=True)
class Motor:
    name: str
    coordinate: str
    group: str
    bus: str
    id: int
    model: str
    direction: int
    limits: MotorLimits
    status: DeviceStatus | None


@dataclass(frozen=True, slots=True)
class Bus:
    name: str
    interface: str
    host_id: int
    status: DeviceStatus | None


@dataclass(frozen=True, slots=True)
class SerialDevice:
    name: str
    port: str
    baud_rate: int
    status: DeviceStatus | None


@dataclass(frozen=True, slots=True)
class Imu:
    name: str
    serial_device: str
    status: DeviceStatus | None


@dataclass(frozen=True, slots=True)
class CoordinateTarget:
    """One hybrid impedance target, expressed in SI units."""

    position_radians: float = 0.0
    velocity_radians_per_second: float = 0.0
    torque_feedforward_newton_metres: float = 0.0
    kp: float = 0.0
    kd: float = 0.0

    def __post_init__(self) -> None:
        values = {
            "position": self.position_radians,
            "velocity": self.velocity_radians_per_second,
            "torque_feedforward": self.torque_feedforward_newton_metres,
            "kp": self.kp,
            "kd": self.kd,
        }
        for name, value in values.items():
            if not math.isfinite(value):
                raise ValueError(f"{name} must be finite")
        if self.kp < 0.0 or self.kd < 0.0:
            raise ValueError("kp and kd must be non-negative")

    @classmethod
    def position(
        cls,
        position: float,
        *,
        kp: float,
        kd: float,
        velocity: float = 0.0,
        torque_feedforward: float = 0.0,
    ) -> CoordinateTarget:
        return cls(position, velocity, torque_feedforward, kp, kd)

    @classmethod
    def torque(cls, torque: float) -> CoordinateTarget:
        return cls(0.0, 0.0, torque, 0.0, 0.0)

    @classmethod
    def damping(cls, kd: float) -> CoordinateTarget:
        return cls(0.0, 0.0, 0.0, 0.0, kd)

    @classmethod
    def hybrid(
        cls,
        position: float,
        velocity: float,
        torque_feedforward: float,
        kp: float,
        kd: float,
    ) -> CoordinateTarget:
        return cls(position, velocity, torque_feedforward, kp, kd)


@dataclass(frozen=True, slots=True)
class Quaternion:
    x: float
    y: float
    z: float
    w: float


@dataclass(frozen=True, slots=True)
class Vector3:
    x: float
    y: float
    z: float


@dataclass(frozen=True, slots=True)
class MotorState:
    monotonic_ns: int
    group: str
    motor: str
    coordinate: str
    position: float
    velocity: float
    torque: float
    temperature: float
    fault_bits: int
    mode_status: int
    feedback_age_ms: int


@dataclass(frozen=True, slots=True)
class ImuState:
    monotonic_ns: int
    imu: str
    orientation: Quaternion
    angular_velocity: Vector3
    linear_acceleration: Vector3
    device_timestamp_ms: int
    sample_age_ms: int


@dataclass(frozen=True, slots=True)
class GroupStatus:
    monotonic_ns: int
    group: str
    state: GroupState
    fault_detail: str
    fault_cause: FaultCause
    command_count: int
    control_tick: int
    jitter_us: int


@dataclass(frozen=True, slots=True)
class HealthStatus:
    monotonic_ns: int
    component: str
    status: DeviceStatus | None
    dropped_events: int
