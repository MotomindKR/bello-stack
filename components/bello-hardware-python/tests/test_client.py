from __future__ import annotations

import threading
import time
from concurrent.futures import ThreadPoolExecutor
from typing import cast

import grpc  # type: ignore[import-untyped]
import pytest
from bello_hardware_api import hardware_pb2 as proto

from bello_hardware import (
    AcknowledgementError,
    CleanupError,
    CommandQueueFullError,
    CommandRejectedError,
    ControlRequestError,
    ControlState,
    CoordinateTarget,
    InvalidSessionStateError,
    InvalidTargetError,
    InventoryError,
    LeaseLostError,
    ProtocolError,
    RequestError,
    Robot,
)

from .conftest import RunningServer


def frame(position: float = 0.0) -> dict[str, CoordinateTarget]:
    return {
        "hip": CoordinateTarget.position(position, kp=20.0, kd=1.0),
        "knee": CoordinateTarget.position(-position, kp=20.0, kd=1.0),
    }


def assert_control_state(actual: ControlState, expected: ControlState) -> None:
    assert actual is expected


def test_refresh_replaces_inventory_and_retains_existing_handles(
    hardware_server: RunningServer,
) -> None:
    with Robot.connect(hardware_server.endpoint) as robot:
        original = robot.inventory
        group = robot.group("leg")
        reader = robot.states()
        hardware_server.service.inventory.motors[0].coordinate = "updated_hip"

        updated = robot.refresh_inventory()

        assert updated is robot.inventory
        assert "hip" in group.coordinates
        assert "updated_hip" in robot.group("leg").coordinates
        assert "hip" in {motor.coordinate for motor in reader.read().motors}
        assert "updated_hip" in {motor.coordinate for motor in robot.states().read().motors}
        assert "hip" in {motor.coordinate for motor in original.motors}

        hardware_server.service.inventory_failure = grpc.StatusCode.UNAVAILABLE
        with pytest.raises(RequestError) as failed:
            robot.refresh_inventory()
        assert failed.value.code == "UNAVAILABLE"
        assert robot.inventory is updated

        hardware_server.service.inventory_failure = None
        hardware_server.service.inventory.motors[0].ClearField("address")
        with pytest.raises(InventoryError):
            robot.refresh_inventory()
        assert robot.inventory is updated


def test_blocking_control_lifecycle_is_small(hardware_server: RunningServer) -> None:
    with Robot.connect(hardware_server.endpoint) as robot:
        assert robot.inventory.group_names == ("leg",)
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


@pytest.mark.parametrize(("seconds", "milliseconds"), [(0.0005, 1), (0.0015, 2), (2.007, 2007)])
def test_command_validity_rounds_up_on_the_wire(
    hardware_server: RunningServer,
    seconds: float,
    milliseconds: int,
) -> None:
    with (
        Robot.connect(hardware_server.endpoint) as robot,
        robot.group("leg").control("pytest", command_validity=seconds) as control,
    ):
        control.start(frame())
        assert hardware_server.service.start_valid_for_ms == milliseconds


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
        robot.group("leg").control("pytest", max_pending=3) as control,
    ):
        control.start(frame())
        gate = threading.Event()
        hardware_server.service.command_ack_gate = gate
        try:
            receipts = [control.try_submit(frame(value)) for value in (0.1, 0.2, 0.3)]
            assert all(not receipt.done() for receipt in receipts)
        finally:
            gate.set()
        for receipt in receipts:
            receipt.result()
            assert receipt.done()
            assert receipt.acknowledgement_latency_ms is not None

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
        receipt = control.try_submit(frame(0.1))
        deadline = time.monotonic() + acknowledgement_timeout

        if late:
            time.sleep(acknowledgement_timeout)
            acknowledge.set()

        wait_until = time.monotonic() + 2.0
        while not receipt.done() and time.monotonic() < wait_until:
            time.sleep(0.001)
        assert receipt.done()
        latency_ms = receipt.acknowledgement_latency_ms

        if late:
            assert latency_ms is None
            with pytest.raises(AcknowledgementError):
                receipt.result()
            assert control.state is ControlState.UNCERTAIN
        else:
            assert latency_ms is not None and latency_ms < acknowledgement_timeout * 1000
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
        receipt = control.try_submit(frame(0.1))
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


def test_full_queue_rejects_try_submit_and_send_waits_for_capacity(
    hardware_server: RunningServer,
) -> None:
    with (
        Robot.connect(hardware_server.endpoint) as robot,
        robot.group("leg").control("pytest", max_pending=1, acknowledgement_timeout=2) as control,
    ):
        control.start(frame())
        gate = threading.Event()
        hardware_server.service.command_ack_gate = gate
        receipt = control.try_submit(frame(0.1))
        started = threading.Event()

        def send() -> None:
            started.set()
            control.send(frame(0.2))

        with ThreadPoolExecutor(max_workers=1) as executor:
            try:
                waiting = executor.submit(send)
                assert started.wait(1)
                with pytest.raises(CommandQueueFullError):
                    control.try_submit(frame(0.3))
                assert not waiting.done()
            finally:
                gate.set()
            waiting.result(timeout=2)
        receipt.result()
        assert hardware_server.service.command_count == 3


def test_dropped_receipt_timeout_marks_session_uncertain(hardware_server: RunningServer) -> None:
    with (
        Robot.connect(hardware_server.endpoint) as robot,
        robot.group("leg").control("pytest", acknowledgement_timeout=0.025) as control,
    ):
        control.start(frame())
        gate = threading.Event()
        hardware_server.service.command_ack_gate = gate
        try:
            control.try_submit(frame(0.1))
            until = time.monotonic() + 2
            while control.state is ControlState.RUNNING and time.monotonic() < until:
                time.sleep(0.005)
            assert control.state is ControlState.UNCERTAIN
        finally:
            gate.set()


@pytest.mark.parametrize(
    "group_state, result, expected",
    [
        (proto.GROUP_STATE_RUNNING, proto.COMMAND_RESULT_UNSAFE, ControlState.UNCERTAIN),
        (proto.GROUP_STATE_DAMPED, proto.COMMAND_RESULT_UNSAFE, ControlState.DAMPED),
        (proto.GROUP_STATE_DAMPED, proto.COMMAND_RESULT_LEASE_LOST, ControlState.UNCERTAIN),
    ],
)
def test_async_rejection_updates_control_before_receipt_is_observed(
    hardware_server: RunningServer,
    group_state: proto.GroupState,
    result: proto.CommandResult,
    expected: ControlState,
) -> None:
    with (
        Robot.connect(hardware_server.endpoint) as robot,
        robot.group("leg").control("pytest") as control,
    ):
        control.start(frame())
        hardware_server.service.next_command_result = result
        hardware_server.service.state = group_state
        receipt = control.try_submit(frame(0.1))
        until = time.monotonic() + 2
        while control.state is ControlState.RUNNING and time.monotonic() < until:
            time.sleep(0.005)
        assert control.state is expected
        with pytest.raises(CommandRejectedError) as rejected:
            receipt.result()
        assert rejected.value.result == result
        with pytest.raises(InvalidSessionStateError):
            control.try_submit(frame(0.2))


@pytest.mark.parametrize("unknown_field", ["result", "state"])
def test_unknown_acknowledgement_makes_session_uncertain(
    hardware_server: RunningServer, unknown_field: str
) -> None:
    with (
        Robot.connect(hardware_server.endpoint) as robot,
        robot.group("leg").control("pytest") as control,
    ):
        control.start(frame())
        if unknown_field == "result":
            hardware_server.service.next_command_result = cast(proto.CommandResult, 1234)
        else:
            hardware_server.service.state = cast(proto.GroupState, 1234)
        with pytest.raises(ProtocolError, match="1234"):
            control.send(frame())
        assert control.state is ControlState.UNCERTAIN


def test_lease_is_renewed_in_the_background(hardware_server: RunningServer) -> None:
    with (
        Robot.connect(hardware_server.endpoint) as robot,
        robot.group("leg").control("pytest", lease_duration=0.05),
    ):
        time.sleep(0.08)
    assert hardware_server.service.renew_count >= 1


def test_rejected_blocking_command_requires_explicit_recovery(
    hardware_server: RunningServer,
) -> None:
    with (
        Robot.connect(hardware_server.endpoint) as robot,
        robot.group("leg").control("pytest") as control,
    ):
        control.start(frame())
        hardware_server.service.next_command_result = proto.COMMAND_RESULT_UNSAFE
        with pytest.raises(CommandRejectedError):
            control.send(frame(0.1))
        assert control.state is ControlState.UNCERTAIN
        with pytest.raises(InvalidSessionStateError):
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
        assert state.groups[0].state == proto.GROUP_STATE_IDLE
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
        pytest.raises(RequestError) as failure,
    ):
        robot.states(groups=("leg",), imus=("body",)).read(timeout=1.0)

    assert failure.value.code == "UNAVAILABLE"
    assert hardware_server.service.state_frame_calls == 1


def test_target_validation() -> None:
    with pytest.raises(ValueError, match="finite"):
        CoordinateTarget.position(float("nan"), kp=1.0, kd=1.0)
    with pytest.raises(ValueError, match="non-negative"):
        CoordinateTarget.damping(-1.0)


@pytest.mark.parametrize("resuming", [False, True])
def test_start_cannot_mark_a_closed_stream_running(
    hardware_server: RunningServer, resuming: bool
) -> None:
    with (
        Robot.connect(hardware_server.endpoint) as robot,
        robot.group("leg").control("pytest") as control,
    ):
        if resuming:
            control.start(frame())
            control.stop()
        close_stream = threading.Event()
        allow_start_reply = threading.Event()
        hardware_server.service.close_stream_gate = close_stream

        def delay_start_reply() -> None:
            close_stream.set()
            assert allow_start_reply.wait(5)

        hardware_server.service.before_start_control = delay_start_reply
        with ThreadPoolExecutor(max_workers=1) as executor:
            attempt = executor.submit(control.resume if resuming else control.start, frame())
            try:
                assert close_stream.wait(5)
                until = time.monotonic() + 3
                while control.state is ControlState.DAMPED and time.monotonic() < until:
                    time.sleep(0.005)
                assert control.state is ControlState.UNCERTAIN
            finally:
                allow_start_reply.set()
            with pytest.raises(AcknowledgementError):
                attempt.result(timeout=3)
        assert_control_state(control.state, ControlState.DAMPED)


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


def test_unconfirmed_enable_leaves_state_uncertain(hardware_server: RunningServer) -> None:
    hardware_server.service.lose_enable_reply = True
    with (
        Robot.connect(hardware_server.endpoint) as robot,
        robot.group("leg").control("pytest") as control,
    ):
        with pytest.raises(ControlRequestError):
            control.start(frame())
        assert hardware_server.service.state == proto.GROUP_STATE_DAMPED
        assert control.state is ControlState.UNCERTAIN
        control.stop()
        control.resume(frame())


def test_control_rpc_error_preserves_status(hardware_server: RunningServer) -> None:
    hardware_server.service.enable_failure = grpc.StatusCode.FAILED_PRECONDITION
    with (
        Robot.connect(hardware_server.endpoint) as robot,
        robot.group("leg").control("pytest") as control,
        pytest.raises(ControlRequestError) as raised,
    ):
        control.start(frame())
    assert raised.value.code == "FAILED_PRECONDITION"
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


def test_disable_can_retry_after_an_unconfirmed_reply(hardware_server: RunningServer) -> None:
    with (
        Robot.connect(hardware_server.endpoint) as robot,
        robot.group("leg").control("pytest") as control,
    ):
        control.start(frame())
        hardware_server.service.disable_failure = grpc.StatusCode.UNAVAILABLE
        with pytest.raises(ControlRequestError) as failure:
            control.disable()
        assert failure.value.code == "UNAVAILABLE"
        assert_control_state(control.state, ControlState.UNCERTAIN)
        hardware_server.service.disable_failure = None
        control.disable()
        assert_control_state(control.state, ControlState.READY)


def test_cleanup_preserves_structured_errors(hardware_server: RunningServer) -> None:
    with Robot.connect(hardware_server.endpoint) as robot:
        control = robot.group("leg").control("pytest")
        hardware_server.service.disable_failure = grpc.StatusCode.INTERNAL
        with pytest.raises(CleanupError) as raised:
            control.close()
    assert isinstance(raised.value.errors[0], ControlRequestError)
    assert raised.value.errors[0].code == "INTERNAL"


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


@pytest.mark.parametrize("resuming", [False, True])
def test_lost_start_reply_rolls_back_to_damped(
    hardware_server: RunningServer,
    resuming: bool,
) -> None:
    with (
        Robot.connect(hardware_server.endpoint) as robot,
        robot.group("leg").control("pytest") as control,
    ):
        if resuming:
            control.start(frame())
            control.stop()
        hardware_server.service.start_failure = grpc.StatusCode.UNAVAILABLE
        with pytest.raises(ControlRequestError):
            (control.resume if resuming else control.start)(frame(0.1))
        assert control.state is ControlState.DAMPED
        assert hardware_server.service.state == proto.GROUP_STATE_DAMPED
        hardware_server.service.start_failure = None
        control.resume(frame(0.1))
        control.send(frame())
