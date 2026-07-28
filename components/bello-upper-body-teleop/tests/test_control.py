from __future__ import annotations

import threading
from collections.abc import Callable
from itertools import pairwise
from types import SimpleNamespace
from typing import Literal, cast

import pytest
from bello_hardware import CoordinateTarget, Robot

from bello_upper_body_teleop.config import (
    UPPER_BODY_COORDINATES,
    UPPER_BODY_JOINTS,
    TeachingArmSample,
    map_raw_positions,
)
from bello_upper_body_teleop.control import (
    WRIST_YAW_COORDINATES,
    OperatorConfirmationError,
    TeleopSettings,
    build_target_frame,
    build_wrist_yaw_hold_frame,
    interpolate_positions,
    required_transition_duration,
    run_teleoperation,
    slew_positions,
)


def test_target_frame_is_complete_and_scales_only_kp() -> None:
    positions = {coordinate: 0.25 for coordinate in UPPER_BODY_COORDINATES}

    frame = build_target_frame(positions, gain_scale=0.5)

    assert tuple(frame) == UPPER_BODY_COORDINATES
    for joint in UPPER_BODY_JOINTS:
        target = frame[joint.coordinate]
        assert target.position_radians == pytest.approx(0.25)
        assert target.kp == pytest.approx(joint.kp * 0.5)
        assert target.kd == pytest.approx(joint.kd)


def test_target_frame_rejects_partial_input() -> None:
    with pytest.raises(ValueError, match="incomplete target positions"):
        build_target_frame({}, gain_scale=1.0)


def test_extreme_teaching_arm_samples_cannot_build_out_of_range_targets() -> None:
    for raw_position in (-1_000_000, 1_000_000):
        sample = map_raw_positions({joint.dxl_id: raw_position for joint in UPPER_BODY_JOINTS})
        frame = build_target_frame(sample.positions, gain_scale=1.0)

        for joint in UPPER_BODY_JOINTS:
            position = frame[joint.coordinate].position_radians
            assert joint.minimum_radians <= position <= joint.maximum_radians


def test_target_frame_rejects_a_position_outside_the_effective_limits() -> None:
    positions = {joint.coordinate: joint.default_position_radians for joint in UPPER_BODY_JOINTS}
    joint = UPPER_BODY_JOINTS[0]
    positions[joint.coordinate] = joint.maximum_radians + 0.01

    with pytest.raises(ValueError, match="outside its effective"):
        build_target_frame(positions, gain_scale=1.0)


def test_wrist_yaw_hold_frame_is_complete_and_scales_only_kp() -> None:
    positions = {
        "left_wrist_yaw_joint": -0.3,
        "right_wrist_yaw_joint": 0.4,
    }

    frame = build_wrist_yaw_hold_frame(
        positions,
        gain_scale=0.5,
        kp=10.0,
        kd=1.0,
    )

    assert tuple(frame) == WRIST_YAW_COORDINATES
    assert frame["left_wrist_yaw_joint"].position_radians == pytest.approx(-0.3)
    assert frame["right_wrist_yaw_joint"].position_radians == pytest.approx(0.4)
    assert all(target.kp == pytest.approx(5.0) for target in frame.values())
    assert all(target.kd == pytest.approx(1.0) for target in frame.values())


def test_slew_positions_limits_each_joint_independently() -> None:
    previous = {"left": 0.0, "right": 1.0}
    requested = {"left": 0.4, "right": 0.4}

    assert slew_positions(previous, requested, 0.1) == {
        "left": pytest.approx(0.1),
        "right": pytest.approx(0.9),
    }


def test_interpolation_uses_common_progress_and_velocity_safe_duration() -> None:
    start = {"left": 0.0, "right": 1.0}
    target = {"left": 2.0, "right": 0.5}

    assert interpolate_positions(start, target, 0.25) == {
        "left": pytest.approx(0.5),
        "right": pytest.approx(0.875),
    }
    assert required_transition_duration(
        start,
        target,
        minimum_seconds=5.0,
        maximum_velocity_radians_per_second=0.25,
    ) == pytest.approx(8.0)


@pytest.mark.parametrize(
    "factory",
    [
        lambda: TeleopSettings(rate_hz=0.0),
        lambda: TeleopSettings(maximum_velocity_radians_per_second=-1.0),
        lambda: TeleopSettings(gain_ramp_seconds=-0.1),
        lambda: TeleopSettings(transition_duration_seconds=0.0),
        lambda: TeleopSettings(duration_seconds=0.0),
    ],
)
def test_settings_reject_unsafe_timing_values(
    factory: Callable[[], TeleopSettings],
) -> None:
    with pytest.raises(ValueError):
        factory()


class _FakeState:
    position = 0.2
    feedback_age_ms = 1.0


class _FakeStates:
    def __init__(self) -> None:
        self.closed = False

    def coordinate(self, _coordinate: str) -> _FakeState:
        return _FakeState()

    def wait_for_change(self, timeout: float | None = None) -> bool:
        return timeout is not None

    def close(self) -> None:
        self.closed = True

    def __enter__(self) -> _FakeStates:
        return self

    def __exit__(self, exc_type: object, exc: object, traceback: object) -> Literal[False]:
        self.close()
        return False


class _FakeReceipt:
    acknowledgement_latency_ms = 0.1

    def done(self) -> bool:
        return True

    def result(self) -> None:
        return


class _FakeControl:
    def __init__(self) -> None:
        self.started: dict[str, CoordinateTarget] | None = None
        self.sent: list[dict[str, CoordinateTarget]] = []
        self.closed = False
        self.start_wait_for: threading.Event | None = None
        self.start_unblocked_by_event: bool | None = None
        self.send_signal: threading.Event | None = None

    def start(self, targets: dict[str, CoordinateTarget]) -> None:
        self.started = targets
        if self.start_wait_for is not None:
            self.start_unblocked_by_event = self.start_wait_for.wait(timeout=0.5)

    def send_async(self, targets: dict[str, CoordinateTarget]) -> _FakeReceipt:
        self.sent.append(targets)
        if self.send_signal is not None:
            self.send_signal.set()
        return _FakeReceipt()

    def __enter__(self) -> _FakeControl:
        return self

    def __exit__(self, exc_type: object, exc: object, traceback: object) -> Literal[False]:
        self.closed = True
        return False


class _FakeGroup:
    def __init__(
        self,
        name: str = "upper_body",
        coordinates: tuple[str, ...] = UPPER_BODY_COORDINATES,
    ) -> None:
        self.name = name
        self.coordinates = coordinates
        self.motors = tuple(
            SimpleNamespace(
                name=f"{coordinate}_motor",
                coordinate=coordinate,
                limits=SimpleNamespace(position_min=-100.0, position_max=100.0),
            )
            for coordinate in coordinates
        )
        self.session = _FakeControl()
        self.control_calls = 0

    def control(self, _owner: str, **_kwargs: object) -> _FakeControl:
        self.control_calls += 1
        return self.session


class _FakeRobot:
    def __init__(self) -> None:
        self.group_instance = _FakeGroup()
        self.wrist_group_instance = _FakeGroup(
            "wrist_yaw",
            WRIST_YAW_COORDINATES,
        )
        self.states_instance = _FakeStates()
        self.wrist_states_instance = _FakeStates()

    def group(self, name: str) -> _FakeGroup:
        if name == "wrist_yaw":
            return self.wrist_group_instance
        return self.group_instance

    def states(self, **kwargs: object) -> _FakeStates:
        if kwargs.get("groups") == ("wrist_yaw",):
            return self.wrist_states_instance
        return self.states_instance


class _FakeSource:
    def __init__(self) -> None:
        positions = {coordinate: 0.0 for coordinate in UPPER_BODY_COORDINATES}
        self.sample = TeachingArmSample(positions, (), {})
        self.read_count = 0

    def read(self) -> TeachingArmSample:
        self.read_count += 1
        return self.sample


class _FakeConfirmationGate:
    def __init__(
        self,
        robot: _FakeRobot,
        decisions: tuple[bool | None, ...],
    ) -> None:
        self._robot = robot
        self._decisions = list(decisions)
        self._pending_token: str | None = None
        self.requests: list[tuple[str, str]] = []
        self.teleop_request_sent_count: int | None = None
        self.teleop_confirmed_sent_count: int | None = None

    def request(self, *, token: str, message: str) -> None:
        assert self._pending_token is None
        self._pending_token = token
        self.requests.append((token, message))
        if token == "TELEOP":
            self.teleop_request_sent_count = len(self._robot.group_instance.session.sent)

    def poll(self) -> bool | None:
        token = self._pending_token
        assert token is not None
        session = self._robot.group_instance.session
        if token == "HOLD":
            assert self._robot.group_instance.control_calls == 0
            assert self._robot.wrist_group_instance.control_calls == 0
            assert session.started is None
        elif token == "TELEOP":
            assert self._robot.group_instance.control_calls == 1
            assert session.started is not None
            assert session.sent
            assert all(
                target.position_radians == pytest.approx(0.2)
                for target in session.sent[-1].values()
            )
        decision = self._decisions.pop(0)
        if token == "TELEOP" and decision is True:
            self.teleop_confirmed_sent_count = len(session.sent)
        if decision is not None:
            self._pending_token = None
        return decision


def test_run_holds_feedback_then_interpolates_only_after_second_confirmation() -> None:
    robot = _FakeRobot()
    source = _FakeSource()
    confirmation = _FakeConfirmationGate(robot, (True, None, True))
    statuses: list[str] = []
    live_started_at_read: int | None = None

    def should_stop() -> bool:
        return live_started_at_read is not None and source.read_count > live_started_at_read

    def report(message: str) -> None:
        nonlocal live_started_at_read
        statuses.append(message)
        if message.startswith("transition complete"):
            live_started_at_read = source.read_count

    run_teleoperation(
        cast(Robot, robot),
        source,
        TeleopSettings(
            rate_hz=1_000.0,
            maximum_velocity_radians_per_second=1_000.0,
            gain_ramp_seconds=0.002,
            transition_duration_seconds=0.0001,
        ),
        confirmation=confirmation,
        should_stop=should_stop,
        status=report,
    )

    session = robot.group_instance.session
    assert session.started is not None
    assert session.closed
    assert robot.states_instance.closed
    assert source.read_count >= 5
    assert [token for token, _message in confirmation.requests] == ["HOLD", "TELEOP"]
    assert all(target.position_radians == pytest.approx(0.2) for target in session.started.values())
    assert all(target.kp == 0.0 for target in session.started.values())
    assert len(session.sent) >= 4
    assert confirmation.teleop_request_sent_count is not None
    assert confirmation.teleop_confirmed_sent_count is not None
    assert confirmation.teleop_confirmed_sent_count > confirmation.teleop_request_sent_count
    hold_frames = session.sent[: confirmation.teleop_confirmed_sent_count]
    assert all(
        target.position_radians == pytest.approx(0.2)
        for frame in hold_frames
        for target in frame.values()
    )
    assert all(target.kp > 0.0 for target in hold_frames[-1].values())
    left_positions = [
        frame["left_shoulder_pitch_joint"].position_radians
        for frame in session.sent[confirmation.teleop_confirmed_sent_count :]
    ]
    assert left_positions[-1] == pytest.approx(0.0)
    assert all(later <= earlier for earlier, later in pairwise(left_positions))
    assert any(message.startswith("HOLD confirmed") for message in statuses)
    assert any(message.startswith("TELEOP confirmed") for message in statuses)
    assert any(message.startswith("transition complete") for message in statuses)


def test_optional_wrist_yaw_group_holds_captured_positions_for_entire_run() -> None:
    robot = _FakeRobot()
    startup_keepalive = threading.Event()
    robot.group_instance.session.send_signal = startup_keepalive
    robot.wrist_group_instance.session.start_wait_for = startup_keepalive
    source = _FakeSource()
    confirmation = _FakeConfirmationGate(robot, (True, None, True))
    live_started_at_read: int | None = None

    def should_stop() -> bool:
        return live_started_at_read is not None and source.read_count > live_started_at_read

    def report(message: str) -> None:
        nonlocal live_started_at_read
        if message.startswith("transition complete"):
            live_started_at_read = source.read_count

    run_teleoperation(
        cast(Robot, robot),
        source,
        TeleopSettings(
            wrist_yaw_group="wrist_yaw",
            rate_hz=1_000.0,
            maximum_velocity_radians_per_second=1_000.0,
            gain_ramp_seconds=0.002,
            transition_duration_seconds=0.0001,
        ),
        confirmation=confirmation,
        should_stop=should_stop,
        status=report,
    )

    wrist_session = robot.wrist_group_instance.session
    assert wrist_session.started is not None
    assert wrist_session.closed
    assert robot.wrist_states_instance.closed
    assert tuple(wrist_session.started) == WRIST_YAW_COORDINATES
    assert all(
        target.position_radians == pytest.approx(0.2) for target in wrist_session.started.values()
    )
    assert all(target.kp == 0.0 for target in wrist_session.started.values())
    assert wrist_session.start_unblocked_by_event is True
    assert wrist_session.sent
    assert len(robot.group_instance.session.sent) > len(wrist_session.sent)
    assert all(
        target.position_radians == pytest.approx(0.2)
        for frame in wrist_session.sent
        for target in frame.values()
    )
    assert all(target.kp == pytest.approx(10.0) for target in wrist_session.sent[-1].values())
    assert all(target.kd == pytest.approx(1.0) for target in wrist_session.sent[-1].values())
    assert "wrist_yaw" in confirmation.requests[0][1]


def test_optional_wrist_yaw_group_rejects_coordinate_mismatch_before_control() -> None:
    robot = _FakeRobot()
    robot.wrist_group_instance.coordinates = ("unexpected_joint",)
    source = _FakeSource()

    with pytest.raises(ValueError, match="wrist-yaw coordinate mismatch"):
        run_teleoperation(
            cast(Robot, robot),
            source,
            TeleopSettings(wrist_yaw_group="wrist_yaw"),
            confirmation=_FakeConfirmationGate(robot, (True, True)),
        )

    assert source.read_count == 0
    assert robot.group_instance.control_calls == 0
    assert robot.wrist_group_instance.control_calls == 0


@pytest.mark.parametrize(
    ("case", "message"),
    [
        ("missing", "motor mapping mismatch"),
        ("duplicate", "assigns coordinate .* more than once"),
        ("narrow", "does not contain the effective teleop range"),
    ],
)
def test_live_motor_envelope_is_validated_before_input_or_control(
    case: str,
    message: str,
) -> None:
    robot = _FakeRobot()
    source = _FakeSource()
    if case == "missing":
        robot.group_instance.motors = robot.group_instance.motors[:-1]
    elif case == "duplicate":
        robot.group_instance.motors[1].coordinate = robot.group_instance.motors[0].coordinate
    else:
        joint = UPPER_BODY_JOINTS[0]
        robot.group_instance.motors[0].limits.position_max = joint.maximum_radians - 0.01

    with pytest.raises(ValueError, match=message):
        run_teleoperation(
            cast(Robot, robot),
            source,
            TeleopSettings(),
            confirmation=_FakeConfirmationGate(robot, (True, True)),
        )

    assert source.read_count == 0
    assert robot.group_instance.control_calls == 0


def test_declining_hold_never_acquires_control() -> None:
    robot = _FakeRobot()
    source = _FakeSource()
    confirmation = _FakeConfirmationGate(robot, (False,))

    with pytest.raises(OperatorConfirmationError, match="HOLD"):
        run_teleoperation(
            cast(Robot, robot),
            source,
            TeleopSettings(),
            confirmation=confirmation,
        )

    assert robot.group_instance.control_calls == 0
    assert robot.group_instance.session.started is None
    assert source.read_count == 1


def test_declining_teleop_cleans_up_while_robot_is_still_held() -> None:
    robot = _FakeRobot()
    source = _FakeSource()
    confirmation = _FakeConfirmationGate(robot, (True, False))

    with pytest.raises(OperatorConfirmationError, match="TELEOP"):
        run_teleoperation(
            cast(Robot, robot),
            source,
            TeleopSettings(gain_ramp_seconds=0.0),
            confirmation=confirmation,
        )

    session = robot.group_instance.session
    assert session.started is not None
    assert session.closed
    assert session.sent
    assert all(
        target.position_radians == pytest.approx(0.2) for target in session.sent[-1].values()
    )
