from __future__ import annotations

import time
from collections.abc import Set
from typing import TYPE_CHECKING

import grpc  # type: ignore[import-untyped]
from bello_hardware_api import hardware_pb2 as proto

from .errors import ProtocolError, rpc_error
from .types import (
    HardwareStateFrame,
    Quaternion,
    StateFrameGroup,
    StateFrameImu,
    StateFrameMotor,
    Vector3,
)

if TYPE_CHECKING:
    from .robot import Robot

STATE_FRAME_RETRY_SECONDS = 0.01


class RobotStateReader:
    """Synchronously read complete latest motor, group, and IMU frames."""

    def __init__(
        self,
        robot: Robot,
        *,
        groups: tuple[str, ...],
        imus: tuple[str, ...],
    ) -> None:
        inventory = robot.inventory
        self._expected_groups = set(groups or inventory.group_names)
        self._expected_imus = set(imus) if imus else {imu.name for imu in inventory.imus}
        self._motors = {
            motor.name: motor for motor in inventory.motors if motor.group in self._expected_groups
        }
        self._robot = robot
        self._request = proto.GetStateFrameRequest(groups=groups, imus=imus)

    def read(self, timeout: float | None = None) -> HardwareStateFrame:
        """Wait for a complete snapshot and conservatively measure the full delay."""
        if timeout is not None and timeout <= 0.0:
            raise ValueError("state frame timeout must be positive")
        self._robot._ensure_open()
        timeout_seconds = self._robot._rpc_timeout if timeout is None else timeout
        deadline = time.monotonic() + timeout_seconds
        request_started_ns = time.monotonic_ns()
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise TimeoutError("timed out waiting for complete hardware state")
            try:
                response = self._robot._stub.GetStateFrame(
                    self._request,
                    timeout=remaining,
                )
                break
            except grpc.RpcError as error:
                if error.code() == grpc.StatusCode.DEADLINE_EXCEEDED:
                    raise TimeoutError("timed out waiting for complete hardware state") from error
                if error.code() != grpc.StatusCode.FAILED_PRECONDITION:
                    raise rpc_error("get state frame", error) from error
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise TimeoutError("timed out waiting for complete hardware state") from error
                time.sleep(min(STATE_FRAME_RETRY_SECONDS, remaining))
        received_ns = time.monotonic_ns()
        return self._decode(
            response,
            received_monotonic_ns=received_ns,
            round_trip_ns=max(0, received_ns - request_started_ns),
        )

    def _decode(
        self,
        response: proto.GetStateFrameResponse,
        *,
        received_monotonic_ns: int,
        round_trip_ns: int,
    ) -> HardwareStateFrame:
        motor_names = [motor.motor_name for motor in response.motors]
        group_names = [group.group for group in response.groups]
        imu_names = [imu.imu_name for imu in response.imus]
        self._require_complete("motor", motor_names, self._motors.keys())
        self._require_complete("group", group_names, self._expected_groups)
        self._require_complete("IMU", imu_names, self._expected_imus)

        return HardwareStateFrame(
            captured_monotonic_ns=response.captured_monotonic_ns,
            received_monotonic_ns=received_monotonic_ns,
            round_trip_ns=round_trip_ns,
            motors=tuple(self._decode_motor(motor) for motor in response.motors),
            imus=tuple(self._decode_imu(imu) for imu in response.imus),
            groups=tuple(self._decode_group(group) for group in response.groups),
            oldest_sample_age_ns=response.oldest_sample_age_ns,
            sample_span_ns=response.sample_span_ns,
        )

    def _decode_motor(self, motor: proto.StateMotor) -> StateFrameMotor:
        configured = self._motors.get(motor.motor_name)
        if configured is None or configured.group != motor.group:
            raise ProtocolError(f"state frame references invalid motor {motor.motor_name!r}")
        return StateFrameMotor(
            sample_monotonic_ns=motor.sample_monotonic_ns,
            sample_age_ns=motor.sample_age_ns,
            group=motor.group,
            motor=motor.motor_name,
            coordinate=configured.coordinate,
            position=motor.position,
            velocity=motor.velocity,
            torque=motor.torque,
            temperature=motor.temperature,
            fault_bits=motor.fault_bits,
            mode_status=motor.mode_status,
        )

    @staticmethod
    def _decode_group(group: proto.StateGroup) -> StateFrameGroup:
        return StateFrameGroup(
            sample_monotonic_ns=group.sample_monotonic_ns,
            sample_age_ns=group.sample_age_ns,
            group=group.group,
            state=group.state,
            detail=group.detail,
            cause=group.cause,
            command_count=group.command_count,
            control_tick=group.control_tick,
            jitter_us=group.jitter_us,
        )

    @staticmethod
    def _decode_imu(imu: proto.StateImu) -> StateFrameImu:
        if not (
            imu.HasField("orientation")
            and imu.HasField("angular_velocity")
            and imu.HasField("linear_acceleration")
        ):
            raise ProtocolError(f"state frame IMU {imu.imu_name!r} is incomplete")
        return StateFrameImu(
            sample_monotonic_ns=imu.sample_monotonic_ns,
            sample_age_ns=imu.sample_age_ns,
            imu=imu.imu_name,
            orientation=Quaternion(
                imu.orientation.x,
                imu.orientation.y,
                imu.orientation.z,
                imu.orientation.w,
            ),
            angular_velocity=Vector3(
                imu.angular_velocity.x,
                imu.angular_velocity.y,
                imu.angular_velocity.z,
            ),
            linear_acceleration=Vector3(
                imu.linear_acceleration.x,
                imu.linear_acceleration.y,
                imu.linear_acceleration.z,
            ),
            device_timestamp_ms=imu.device_timestamp_ms,
        )

    @staticmethod
    def _require_complete(kind: str, received: list[str], expected: Set[str]) -> None:
        actual = set(received)
        if len(received) != len(actual):
            raise ProtocolError(f"state frame contains duplicate {kind} entries")
        if actual != expected:
            raise ProtocolError(
                f"state frame {kind} set is incomplete: missing={sorted(expected - actual)}, "
                f"unexpected={sorted(actual - expected)}"
            )
