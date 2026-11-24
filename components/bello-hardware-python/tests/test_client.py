from __future__ import annotations

import threading
import time
from concurrent.futures import ThreadPoolExecutor

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
    InvalidTargetError,
    LeaseLostError,
    Robot,
    RpcError,
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
        assert robot.inventory.imus[0].source_frame == "imu_sensor_link"
        assert robot.inventory.imus[0].output_frame == "imu_link"

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


@pytest.mark.parametrize("late", [False, True], ids=["on-time", "late"])
def test_async_receipt_uses_acknowledgement_deadline(
    hardware_server: RunningServer, late: bool
) -> None:
    acknowledgement_timeout = 0.25
    with (
        Robot.connect(hardware_server.endpoint) as robot,
        robot.group("leg").control(
            "pytest", acknowledgement_timeout=acknowledgement_timeout
        ) as control,
    ):
        control.start(frame())
        acknowledge = threading.Event()
        if not late:
            acknowledge.set()
        hardware_server.service.command_ack_gate = acknowledge
        receipt = control.send_async(frame(0.1))
        deadline = time.monotonic() + acknowledgement_timeout

        if late:
            time.sleep(acknowledgement_timeout)
            acknowledge.set()

        wait_until = time.monotonic() + 2.0
        while not receipt.done() and time.monotonic() < wait_until:
            time.sleep(0.001)
        assert receipt.done()
        latency_ms = receipt.acknowledgement_latency_ms
        assert latency_ms is not None

        if late:
            assert latency_ms > acknowledgement_timeout * 1000
            with pytest.raises(AcknowledgementError):
                receipt.result()
            assert control.state is ControlState.UNCERTAIN
        else:
            assert latency_ms < acknowledgement_timeout * 1000
            time.sleep(max(0.0, deadline - time.monotonic()))
            receipt.result()
            assert control.state is ControlState.RUNNING
            control.send(frame(0.2))


@pytest.mark.parametrize("during_resume", [False, True], ids=["after-resume", "during-resume"])
def test_closed_stream_receipt_preserves_resumed_control(
    hardware_server: RunningServer, during_resume: bool
) -> None:
    with (
        Robot.connect(hardware_server.endpoint) as robot,
        robot.group("leg").control(
            "pytest", acknowledgement_timeout=1.0, rpc_timeout=3.0
        ) as control,
    ):
        control.start(frame())
        acknowledge = threading.Event()
        hardware_server.service.command_ack_gate = acknowledge
        receipt = control.send_async(frame(0.1))
        assert not receipt.done()
        control.stop()
        acknowledge.set()

        if during_resume:
            resume_started = threading.Event()
            allow_resume = threading.Event()
            read_started = threading.Event()

            def pause_resume() -> None:
                resume_started.set()
                assert allow_resume.wait(2.0)

            def read_receipt() -> None:
                read_started.set()
                receipt.result()

            hardware_server.service.before_start_control = pause_resume
            with ThreadPoolExecutor(max_workers=2) as executor:
                resume = executor.submit(control.resume, frame())
                try:
                    assert resume_started.wait(2.0)
                    result = executor.submit(read_receipt)
                    assert read_started.wait(2.0)
                finally:
                    allow_resume.set()
                resume.result(timeout=3.0)
                with pytest.raises(AcknowledgementError):
                    result.result(timeout=3.0)
        else:
            control.resume(frame())
            with pytest.raises(AcknowledgementError):
                receipt.result()

        assert control.state is ControlState.RUNNING
        control.send(frame(0.2))


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
        control.send(frame(0.2))


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
        control.send(frame(0.2))


def test_incomplete_frames_fail_before_transmission(hardware_server: RunningServer) -> None:
    with (
        Robot.connect(hardware_server.endpoint) as robot,
        robot.group("leg").control("pytest") as control,
        pytest.raises(InvalidTargetError),
    ):
        control.start({"hip": CoordinateTarget.damping(1.0)})
    assert hardware_server.service.command_count == 0


def test_complete_state_frame_uses_one_synchronous_rpc(
    hardware_server: RunningServer,
) -> None:
    with Robot.connect(hardware_server.endpoint) as robot:
        reader = robot.states(
            groups=("leg",),
            imus=("body",),
        )
        assert hardware_server.service.state_frame_calls == 0
        state = reader.read(timeout=1.0)

        assert hardware_server.service.state_frame_calls == 1
        positions = {motor.coordinate: motor.position for motor in state.motors}
        assert positions == pytest.approx({"hip": 0.25, "knee": -0.5})
        assert state.imus[0].orientation.w == pytest.approx(1.0)
        assert state.groups[0].state.name == "IDLE"
        assert state.groups[0].sample_age_ns == 500
        assert state.oldest_sample_age_ns == 1_000
        assert state.sample_span_ns == 500


def test_state_frame_reports_complete_read_duration(
    hardware_server: RunningServer,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    timestamps = iter((1_000_000, 7_000_000))
    monkeypatch.setattr(
        "bello_hardware.frames.time.monotonic_ns",
        lambda: next(timestamps),
    )
    with Robot.connect(hardware_server.endpoint) as robot:
        reader = robot.states(
            groups=("leg",),
            imus=("body",),
        )
        state = reader.read(timeout=1.0)

        assert state.round_trip_ns == 6_000_000


def test_state_frame_waits_for_complete_daemon_cache(
    hardware_server: RunningServer,
) -> None:
    hardware_server.service.state_frame_incomplete_count = 2
    with Robot.connect(hardware_server.endpoint) as robot:
        state = robot.states(groups=("leg",), imus=("body",)).read(timeout=1.0)

        assert hardware_server.service.state_frame_calls == 3
        assert {motor.coordinate for motor in state.motors} == {"hip", "knee"}


def test_state_frame_cache_wait_honors_timeout(
    hardware_server: RunningServer,
) -> None:
    hardware_server.service.state_frame_incomplete_count = 100
    with (
        Robot.connect(hardware_server.endpoint) as robot,
        pytest.raises(TimeoutError, match="complete hardware state"),
    ):
        robot.states(groups=("leg",), imus=("body",)).read(timeout=0.001)


def test_state_frame_transport_unavailable_is_not_retried(
    hardware_server: RunningServer,
) -> None:
    hardware_server.service.state_frame_failure = grpc.StatusCode.UNAVAILABLE
    with (
        Robot.connect(hardware_server.endpoint) as robot,
        pytest.raises(RpcError) as failure,
    ):
        robot.states(groups=("leg",), imus=("body",)).read(timeout=1.0)

    assert failure.value.code is grpc.StatusCode.UNAVAILABLE
    assert hardware_server.service.state_frame_calls == 1


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
