"""Thread-safe latest-value aggregation over the telemetry stream."""

from __future__ import annotations

import threading
import time
from dataclasses import replace
from typing import TYPE_CHECKING, Any, Literal

import grpc  # type: ignore[import-untyped]

from ._generated import hardware_pb2 as proto
from .errors import ProtocolError, StateUnavailableError, WorkerShutdownError, rpc_error
from .types import (
    DeviceState,
    DeviceStatus,
    GroupCause,
    GroupState,
    GroupStatus,
    HealthStatus,
    ImuState,
    MotorState,
    Quaternion,
    Vector3,
)

if TYPE_CHECKING:
    from .robot import Robot

_RETRYABLE = {
    grpc.StatusCode.UNAVAILABLE,
    grpc.StatusCode.CANCELLED,
    grpc.StatusCode.DEADLINE_EXCEEDED,
    grpc.StatusCode.UNKNOWN,
}


class _ConsumerCursor(threading.local):
    def __init__(self) -> None:
        self.seen_update = 0


class RobotStates:
    """Latest motor, group, IMU, and component-health telemetry."""

    def __init__(
        self,
        robot: Robot,
        *,
        groups: tuple[str, ...],
        imus: tuple[str, ...],
        reconnect: bool,
        reconnect_delay: float,
    ) -> None:
        if reconnect and reconnect_delay <= 0.0:
            raise ValueError("telemetry reconnect delay must be positive")
        unknown_groups = sorted(set(groups) - set(robot.group_names))
        known_imus = {imu.name for imu in robot.inventory.imus}
        unknown_imus = sorted(set(imus) - known_imus)
        if unknown_groups:
            raise ValueError(f"unknown telemetry groups: {unknown_groups}")
        if unknown_imus:
            raise ValueError(f"unknown telemetry IMUs: {unknown_imus}")
        self._robot = robot
        self._request = proto.WatchTelemetryRequest(
            groups=groups,
            include_motors=True,
            include_imu=True,
            include_health=True,
            imus=imus,
        )
        self._reconnect = reconnect
        self._reconnect_delay = reconnect_delay
        self._condition = threading.Condition()
        self._call_lock = threading.Lock()
        self._close_lock = threading.Lock()
        self._stop = threading.Event()
        self._call: Any = None
        self._closed = False
        self._generation = 0
        self._update_count = 0
        self._consumer_cursor = _ConsumerCursor()
        self._monotonic_ns = 0
        self._motors: dict[str, MotorState] = {}
        self._motor_received_ns: dict[str, int] = {}
        self._groups: dict[str, GroupStatus] = {}
        self._imus: dict[str, ImuState] = {}
        self._imu_received_ns: dict[str, int] = {}
        self._health: dict[str, HealthStatus] = {}
        self._error: BaseException | None = None
        self._ended = False
        self._thread = threading.Thread(
            target=self._run,
            name="bello-telemetry",
            daemon=True,
        )
        robot._begin_child()
        try:
            self._thread.start()
        except BaseException:
            robot._finish_child(None)
            raise
        robot._finish_child(self)

    @property
    def generation(self) -> int:
        with self._condition:
            return self._generation

    @property
    def monotonic_ns(self) -> int:
        with self._condition:
            return self._monotonic_ns

    def wait_for_change(self, timeout: float | None = None) -> bool:
        """Wait for unobserved telemetry; return false after timeout or normal EOF."""
        if timeout is not None and timeout < 0.0:
            raise ValueError("timeout must be non-negative")
        deadline = None if timeout is None else time.monotonic() + timeout
        with self._condition:
            while self._consumer_cursor.seen_update == self._update_count:
                if self._error is not None:
                    raise self._error
                if self._ended or self._stop.is_set():
                    return False
                remaining = None if deadline is None else deadline - time.monotonic()
                if remaining is not None and remaining <= 0.0:
                    return False
                self._condition.wait(remaining)
            self._consumer_cursor.seen_update = self._update_count
            return True

    def motor(self, name: str) -> MotorState:
        with self._condition:
            try:
                return self._current_motor(name)
            except KeyError as error:
                raise StateUnavailableError(f"no current state for motor {name!r}") from error

    def coordinate(self, name: str) -> MotorState:
        motor = self._robot.inventory._coordinates.get(name)
        if motor is None:
            raise StateUnavailableError(f"unknown coordinate {name!r}")
        return self.motor(motor)

    def group(self, name: str) -> GroupStatus:
        with self._condition:
            try:
                return self._groups[name]
            except KeyError as error:
                raise StateUnavailableError(f"no current state for motor group {name!r}") from error

    def imu(self, name: str) -> ImuState:
        with self._condition:
            try:
                return self._current_imu(name)
            except KeyError as error:
                raise StateUnavailableError(f"no current state for IMU {name!r}") from error

    def health(self, component: str) -> HealthStatus | None:
        with self._condition:
            return self._health.get(component)

    @property
    def motors(self) -> tuple[MotorState, ...]:
        with self._condition:
            return tuple(self._current_motor(name) for name in sorted(self._motors))

    @property
    def groups(self) -> tuple[GroupStatus, ...]:
        with self._condition:
            return tuple(self._groups[name] for name in sorted(self._groups))

    @property
    def imus(self) -> tuple[ImuState, ...]:
        with self._condition:
            return tuple(self._current_imu(name) for name in sorted(self._imus))

    def close(self) -> None:
        with self._close_lock:
            if self._closed:
                return
            self._stop.set()
            with self._call_lock:
                call = self._call
                if call is not None:
                    call.cancel()
            with self._condition:
                self._condition.notify_all()
            if self._thread is not threading.current_thread():
                self._thread.join(timeout=self._robot._rpc_timeout)
                if self._thread.is_alive():
                    raise WorkerShutdownError("telemetry receiver did not stop before its deadline")
            self._closed = True
            self._robot._unregister(self)

    def __enter__(self) -> RobotStates:
        return self

    def __exit__(self, exc_type: object, exc: object, traceback: object) -> Literal[False]:
        self.close()
        return False

    def _run(self) -> None:
        reconnecting = False
        while not self._stop.is_set():
            if reconnecting and self._stop.wait(self._reconnect_delay):
                return
            call: Any = None
            try:
                call = self._robot._new_stub().WatchTelemetry(self._request)
                with self._call_lock:
                    if self._stop.is_set():
                        call.cancel()
                        return
                    self._call = call
                if reconnecting:
                    self._clear_after_reconnect()
                reconnecting = False
                for response in call:
                    if self._stop.is_set():
                        return
                    self._apply(response)
                with self._condition:
                    self._ended = True
                    self._condition.notify_all()
                return
            except grpc.RpcError as error:
                if self._stop.is_set():
                    return
                if self._reconnect and error.code() in _RETRYABLE:
                    reconnecting = True
                    continue
                self._fail(rpc_error("watch telemetry", error))
                return
            except BaseException as error:
                self._fail(error)
                return
            finally:
                with self._call_lock:
                    if self._call is call:
                        self._call = None

    def _clear_after_reconnect(self) -> None:
        with self._condition:
            self._generation += 1
            self._monotonic_ns = 0
            self._motors.clear()
            self._motor_received_ns.clear()
            self._groups.clear()
            self._imus.clear()
            self._imu_received_ns.clear()
            self._health.clear()

    def _apply(self, response: proto.WatchTelemetryResponse) -> None:
        payload = response.WhichOneof("payload")
        if payload is None:
            raise ProtocolError("telemetry response has no payload")
        monotonic_ns = response.monotonic_ns
        with self._condition:
            self._monotonic_ns = monotonic_ns
            if payload == "motor":
                motor = response.motor
                configured = self._robot.inventory._motors.get(motor.motor_name)
                if configured is None or configured.group != motor.group:
                    raise ProtocolError(f"telemetry references invalid motor {motor.motor_name!r}")
                self._motors[motor.motor_name] = MotorState(
                    monotonic_ns,
                    motor.group,
                    motor.motor_name,
                    configured.coordinate,
                    motor.position,
                    motor.velocity,
                    motor.torque,
                    motor.temperature,
                    motor.fault_bits,
                    motor.mode_status,
                    motor.feedback_age_ms,
                )
                self._motor_received_ns[motor.motor_name] = time.monotonic_ns()
            elif payload == "imu":
                imu = response.imu
                known = {item.name for item in self._robot.inventory.imus}
                if imu.imu_name not in known:
                    raise ProtocolError(f"telemetry references unknown IMU {imu.imu_name!r}")
                if not (
                    imu.HasField("orientation")
                    and imu.HasField("angular_velocity")
                    and imu.HasField("linear_acceleration")
                ):
                    raise ProtocolError(f"IMU {imu.imu_name!r} telemetry is incomplete")
                self._imus[imu.imu_name] = ImuState(
                    monotonic_ns,
                    imu.imu_name,
                    Quaternion(
                        imu.orientation.x,
                        imu.orientation.y,
                        imu.orientation.z,
                        imu.orientation.w,
                    ),
                    Vector3(
                        imu.angular_velocity.x,
                        imu.angular_velocity.y,
                        imu.angular_velocity.z,
                    ),
                    Vector3(
                        imu.linear_acceleration.x,
                        imu.linear_acceleration.y,
                        imu.linear_acceleration.z,
                    ),
                    imu.device_timestamp_ms,
                    imu.sample_age_ms,
                )
                self._imu_received_ns[imu.imu_name] = time.monotonic_ns()
            elif payload == "group":
                group = response.group
                if group.group not in self._robot.group_names:
                    raise ProtocolError(f"telemetry references unknown group {group.group!r}")
                try:
                    state = GroupState(group.state)
                    cause = GroupCause(group.cause)
                except ValueError as error:
                    raise ProtocolError("group telemetry contains an unknown enum value") from error
                self._groups[group.group] = GroupStatus(
                    monotonic_ns,
                    group.group,
                    state,
                    group.detail,
                    cause,
                    group.command_count,
                    group.control_tick,
                    group.jitter_us,
                )
            elif payload == "health":
                health = response.health
                status = None
                if health.HasField("device_status"):
                    try:
                        device_state = DeviceState(health.device_status.state)
                    except ValueError as error:
                        raise ProtocolError("health telemetry has an unknown state") from error
                    status = DeviceStatus(
                        device_state,
                        health.device_status.detail,
                        health.device_status.transition_count,
                    )
                self._health[health.component] = HealthStatus(
                    monotonic_ns,
                    health.component,
                    status,
                    health.dropped_events,
                )
                if (
                    status is not None
                    and status.state != DeviceState.ONLINE
                    and health.component.startswith("imu/")
                ):
                    imu_name = health.component.removeprefix("imu/")
                    self._imus.pop(imu_name, None)
                    self._imu_received_ns.pop(imu_name, None)
            self._update_count += 1
            self._condition.notify_all()

    def _current_motor(self, name: str) -> MotorState:
        state = self._motors[name]
        return replace(
            state,
            feedback_age_ms=self._current_age_ms(
                state.feedback_age_ms,
                self._motor_received_ns[name],
            ),
        )

    def _current_imu(self, name: str) -> ImuState:
        state = self._imus[name]
        return replace(
            state,
            sample_age_ms=self._current_age_ms(
                state.sample_age_ms,
                self._imu_received_ns[name],
            ),
        )

    @staticmethod
    def _current_age_ms(reported_age_ms: int, received_ns: int) -> int:
        elapsed_ns = max(0, time.monotonic_ns() - received_ns)
        return reported_age_ms + elapsed_ns // 1_000_000

    def _fail(self, error: BaseException) -> None:
        with self._condition:
            self._error = error
            self._condition.notify_all()
