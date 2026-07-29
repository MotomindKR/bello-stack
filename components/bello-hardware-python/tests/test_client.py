from __future__ import annotations

import time
from threading import Thread

import grpc  # type: ignore[import-untyped]
import pytest

from bello_hardware import (
    AcknowledgementError,
    CleanupError,
    CommandPipeline,
    CommandRejectedError,
    ControlRpcError,
    ControlState,
    CoordinateTarget,
    DeviceState,
    InvalidTargetError,
    LeaseLostError,
    Robot,
    StateUnavailableError,
)
from bello_hardware._generated import hardware_pb2 as proto

from .conftest import RunningServer


def frame(position: float = 0.0) -> dict[str, CoordinateTarget]:
    return {
        "hip": CoordinateTarget.position(position, kp=20.0, kd=1.0),
        "knee": CoordinateTarget.position(-position, kp=20.0, kd=1.0),
    }


def assert_control_state(actual: ControlState, expected: ControlState) -> None:
    assert actual is expected


def test_blocking_control_lifecycle_is_small(hardware_server: RunningServer) -> None:
    with Robot.connect(hardware_server.endpoint) as robot:
        assert robot.group_names == ("leg",)
        group = robot.group("leg")
        assert group.coordinates == ("hip", "knee")
        assert group.motors[0].limits.torque_max == 60.0

        with group.control("pytest") as control:
            control.start(frame())
            assert control.state is ControlState.RUNNING
            control.send(frame(0.2))

    assert hardware_server.service.command_count == 2
    assert hardware_server.service.release_count == 1


def test_close_can_preserve_damping(hardware_server: RunningServer) -> None:
    with Robot.connect(hardware_server.endpoint) as robot:
        control = robot.group("leg").control("pytest")
        control.start(frame())
        control.close(preserve_damping=True)

        assert control.state is ControlState.CLOSED

    assert hardware_server.service.state == proto.GROUP_STATE_DAMPED
    assert hardware_server.service.disable_count == 0
    assert hardware_server.service.release_count == 1


def test_async_commands_pipeline_before_acknowledgements(
    hardware_server: RunningServer,
) -> None:
    with (
        Robot.connect(hardware_server.endpoint) as robot,
        robot.group("leg").control("pytest", acknowledgement_timeout=0.5) as control,
    ):
        control.start(frame())
        hardware_server.service.command_delay = 0.04
        started = time.monotonic()
        receipts = [control.send_async(frame(value)) for value in (0.1, 0.2, 0.3)]
        submit_elapsed = time.monotonic() - started
        assert submit_elapsed < 0.04
        assert receipts[-1].acknowledgement_latency_ms is None
        for receipt in receipts:
            receipt.result()
        assert all(receipt.done() for receipt in receipts)
        assert all(receipt.acknowledgement_latency_ms is not None for receipt in receipts)
        assert receipts[0].acknowledgement_latency_ms is not None
        assert receipts[0].acknowledgement_latency_ms >= 30.0

    assert hardware_server.service.command_count == 4


def test_bounded_command_pipeline_preserves_fixed_rate_submission(
    hardware_server: RunningServer,
) -> None:
    with (
        Robot.connect(hardware_server.endpoint) as robot,
        robot.group("leg").control("pytest", acknowledgement_timeout=0.5) as control,
    ):
        control.start(frame())
        hardware_server.service.command_delay = 0.04
        pipeline = CommandPipeline(control, max_pending=4)

        started = time.monotonic()
        for value in (0.1, 0.2, 0.3):
            pipeline.send(frame(value))
        assert time.monotonic() - started < 0.04
        assert pipeline.pending_count == 3

        pipeline.drain()

        assert pipeline.pending_count == 0
        assert len(pipeline.acknowledgement_latencies_ms) == 3
        assert max(pipeline.acknowledgement_latencies_ms) >= 30.0
        assert pipeline.maximum_submission_latency_ms < 40.0


def test_command_pipeline_rejects_invalid_pending_bound(
    hardware_server: RunningServer,
) -> None:
    with (
        Robot.connect(hardware_server.endpoint) as robot,
        robot.group("leg").control("pytest") as control,
    ):
        with pytest.raises(ValueError, match="positive"):
            CommandPipeline(control, max_pending=0)
        with pytest.raises(TypeError, match="integer"):
            CommandPipeline(control, max_pending=True)


def test_async_rejection_preserves_running_session_state(
    hardware_server: RunningServer,
) -> None:
    with (
        Robot.connect(hardware_server.endpoint) as robot,
        robot.group("leg").control("pytest") as control,
    ):
        control.start(frame())
        hardware_server.service.reject_next = True
        receipt = control.send_async(frame(0.1))
        with pytest.raises(CommandRejectedError):
            receipt.result()
        assert control.state is ControlState.RUNNING


def test_lease_is_renewed_in_the_background(hardware_server: RunningServer) -> None:
    with (
        Robot.connect(hardware_server.endpoint) as robot,
        robot.group("leg").control("pytest", lease_duration=0.05),
    ):
        time.sleep(0.08)
    assert hardware_server.service.renew_count >= 1


def test_rejected_command_preserves_running_session_state(
    hardware_server: RunningServer,
) -> None:
    with (
        Robot.connect(hardware_server.endpoint) as robot,
        robot.group("leg").control("pytest") as control,
    ):
        control.start(frame())
        hardware_server.service.reject_next = True
        with pytest.raises(CommandRejectedError):
            control.send(frame(0.1))
        assert control.state is ControlState.RUNNING


def test_incomplete_frames_fail_before_transmission(hardware_server: RunningServer) -> None:
    with (
        Robot.connect(hardware_server.endpoint) as robot,
        robot.group("leg").control("pytest") as control,
        pytest.raises(InvalidTargetError),
    ):
        control.start({"hip": CoordinateTarget.damping(1.0)})
    assert hardware_server.service.command_count == 0


def test_latest_state_cache(hardware_server: RunningServer) -> None:
    with Robot.connect(hardware_server.endpoint) as robot, robot.states() as states:
        deadline = time.monotonic() + 1.0
        while len(states.groups) == 0 and time.monotonic() < deadline:
            assert states.wait_for_change(timeout=1.0)
        hip = states.coordinate("hip")
        assert hip.position == pytest.approx(0.25)
        assert hip.temperature == pytest.approx(32.0)
        assert states.group("leg").monotonic_ns == 2_000


def test_cached_sample_ages_continue_increasing(
    hardware_server: RunningServer,
) -> None:
    with Robot.connect(hardware_server.endpoint) as robot, robot.states() as states:
        deadline = time.monotonic() + 1.0
        while (not states.motors or not states.imus) and time.monotonic() < deadline:
            assert states.wait_for_change(timeout=1.0)
        motor_age = states.motor("hip_motor").feedback_age_ms
        imu_age = states.imu("body").sample_age_ms

        time.sleep(0.03)

        assert states.motor("hip_motor").feedback_age_ms >= motor_age + 20
        assert states.imu("body").sample_age_ms >= imu_age + 20


def test_unhealthy_imu_invalidates_cached_sample(
    hardware_server: RunningServer,
) -> None:
    with Robot.connect(hardware_server.endpoint) as robot, robot.states() as states:
        deadline = time.monotonic() + 1.0
        while not states.imus and time.monotonic() < deadline:
            assert states.wait_for_change(timeout=1.0)

        hardware_server.service.emit_telemetry(
            proto.WatchTelemetryResponse(
                monotonic_ns=5_000,
                health=proto.HealthTelemetry(
                    component="imu/body",
                    device_status=proto.DeviceStatus(
                        state=proto.DEVICE_STATE_RECONNECTING,
                        detail="no valid IMU sample",
                        transition_count=2,
                    ),
                ),
            )
        )
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            health = states.health("imu/body")
            if (
                health is not None
                and health.status is not None
                and health.status.state == DeviceState.RECONNECTING
            ):
                break
            assert states.wait_for_change(timeout=1.0)

        with pytest.raises(StateUnavailableError):
            states.imu("body")


def test_target_validation() -> None:
    with pytest.raises(ValueError, match="finite"):
        CoordinateTarget.position(float("nan"), kp=1.0, kd=1.0)
    with pytest.raises(ValueError, match="non-negative"):
        CoordinateTarget.damping(-1.0)


def test_acknowledgement_timeout_makes_session_uncertain(
    hardware_server: RunningServer,
) -> None:
    with (
        Robot.connect(hardware_server.endpoint) as robot,
        robot.group("leg").control("pytest", acknowledgement_timeout=0.02) as control,
    ):
        control.start(frame())
        hardware_server.service.command_delay = 0.1
        with pytest.raises(AcknowledgementError):
            control.send(frame(0.1))
        assert control.state is ControlState.UNCERTAIN


def test_control_rpc_error_preserves_status(hardware_server: RunningServer) -> None:
    hardware_server.service.enable_failure = grpc.StatusCode.FAILED_PRECONDITION
    with (
        Robot.connect(hardware_server.endpoint) as robot,
        robot.group("leg").control("pytest") as control,
        pytest.raises(ControlRpcError) as raised,
    ):
        control.start(frame())
    assert raised.value.code is grpc.StatusCode.FAILED_PRECONDITION
    assert raised.value.operation == "enable group"


def test_renewal_failure_is_reported(hardware_server: RunningServer) -> None:
    hardware_server.service.renew_failure = grpc.StatusCode.PERMISSION_DENIED
    with (
        Robot.connect(hardware_server.endpoint) as robot,
        robot.group("leg").control("pytest", lease_duration=0.04) as control,
    ):
        deadline = time.monotonic() + 1.0
        while control._renewal_error is None and time.monotonic() < deadline:
            time.sleep(0.005)
        assert hardware_server.service.renew_count >= 1
        with pytest.raises(LeaseLostError):
            control.recover()


def test_cleanup_preserves_structured_errors(hardware_server: RunningServer) -> None:
    with Robot.connect(hardware_server.endpoint) as robot:
        control = robot.group("leg").control("pytest")
        hardware_server.service.disable_failure = grpc.StatusCode.INTERNAL
        with pytest.raises(CleanupError) as raised:
            control.close()
    assert isinstance(raised.value.errors[0], ControlRpcError)
    assert raised.value.errors[0].code is grpc.StatusCode.INTERNAL


def test_body_error_is_not_masked_by_cleanup_failure(hardware_server: RunningServer) -> None:
    with Robot.connect(hardware_server.endpoint) as robot:
        control = robot.group("leg").control("pytest")
        hardware_server.service.disable_failure = grpc.StatusCode.INTERNAL
        with (
            pytest.warns(RuntimeWarning, match="control cleanup failed"),
            pytest.raises(ValueError, match="body failed"),
            control,
        ):
            raise ValueError("body failed")


def test_session_can_resume_after_stop(hardware_server: RunningServer) -> None:
    with (
        Robot.connect(hardware_server.endpoint) as robot,
        robot.group("leg").control("pytest") as control,
    ):
        control.start(frame())
        control.stop()
        assert_control_state(control.state, ControlState.DAMPED)
        control.resume(frame(0.1))
        assert_control_state(control.state, ControlState.RUNNING)


def test_resume_rolls_an_ambiguous_start_back_to_damped(
    hardware_server: RunningServer,
) -> None:
    with (
        Robot.connect(hardware_server.endpoint) as robot,
        robot.group("leg").control("pytest") as control,
    ):
        control.start(frame())
        control.stop()
        hardware_server.service.start_failure = grpc.StatusCode.UNAVAILABLE
        with pytest.raises(ControlRpcError):
            control.resume(frame(0.1))
        assert control.state is ControlState.DAMPED


def test_telemetry_reconnect_clears_state_and_increments_generation(
    hardware_server: RunningServer,
) -> None:
    hardware_server.service.telemetry_failures = 1
    with Robot.connect(hardware_server.endpoint) as robot, robot.states() as states:
        deadline = time.monotonic() + 1.0
        while not states.motors and time.monotonic() < deadline:
            assert states.wait_for_change(timeout=1.0)
        assert states.generation == 1
        assert hardware_server.service.telemetry_calls >= 2


def test_each_waiting_thread_has_an_independent_state_cursor(
    hardware_server: RunningServer,
) -> None:
    with Robot.connect(hardware_server.endpoint) as robot, robot.states() as states:
        deadline = time.monotonic() + 1.0
        while not states.groups and time.monotonic() < deadline:
            assert states.wait_for_change(timeout=1.0)
        results: list[bool] = []
        waiters = [
            Thread(target=lambda: results.append(states.wait_for_change(0.1))) for _ in range(2)
        ]
        for waiter in waiters:
            waiter.start()
        for waiter in waiters:
            waiter.join()
        assert results == [True, True]
