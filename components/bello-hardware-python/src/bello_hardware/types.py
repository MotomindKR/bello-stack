from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum

from bello_hardware_api.hardware_pb2 import DeviceState, GroupCause, GroupState


class ControlState(Enum):
    READY = "ready"
    DAMPED = "damped"
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
    firmware_version: str | None = None


@dataclass(frozen=True, slots=True)
class Bus:
    name: str
    transport: str
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
    source_frame: str
    output_frame: str
    status: DeviceStatus | None


@dataclass(frozen=True, slots=True)
class PowerBoard:
    name: str
    serial_device: str
    profile: str
    stale_timeout_ms: int
    sync_period_ms: int
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
class StateFrameMotor:
    sample_monotonic_ns: int
    sample_age_ns: int
    group: str
    motor: str
    coordinate: str
    position: float
    velocity: float
    torque: float
    temperature: float
    fault_bits: int
    mode_status: int

    @property
    def feedback_age_ms(self) -> int:
        return self.sample_age_ns // 1_000_000


@dataclass(frozen=True, slots=True)
class StateFrameImu:
    sample_monotonic_ns: int
    sample_age_ns: int
    imu: str
    orientation: Quaternion
    angular_velocity: Vector3
    linear_acceleration: Vector3
    device_timestamp_ms: int

    @property
    def sample_age_ms(self) -> int:
        return self.sample_age_ns // 1_000_000


@dataclass(frozen=True, slots=True)
class StateFrameGroup:
    sample_monotonic_ns: int
    sample_age_ns: int
    group: str
    state: GroupState
    detail: str
    cause: GroupCause
    command_count: int
    control_tick: int
    jitter_us: int


@dataclass(frozen=True, slots=True)
class HardwareStateFrame:
    captured_monotonic_ns: int
    received_monotonic_ns: int
    round_trip_ns: int
    motors: tuple[StateFrameMotor, ...]
    imus: tuple[StateFrameImu, ...]
    groups: tuple[StateFrameGroup, ...]
    oldest_sample_age_ns: int
    sample_span_ns: int
