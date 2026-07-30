"""Safe handoff from teaching-arm samples to a Bello motor-group session."""

from __future__ import annotations

import math
import time
from collections.abc import Callable, Mapping, Sequence
from concurrent.futures import ThreadPoolExecutor
from contextlib import ExitStack
from dataclasses import dataclass
from typing import Protocol

from bello_hardware import (
    CommandPipeline,
    CoordinateTarget,
    Motor,
    Robot,
)

from .config import (
    UPPER_BODY_JOINTS,
    JointMapping,
    TeachingArmSample,
    validate_group_coordinates,
)

WRIST_YAW_COORDINATES = (
    "left_wrist_yaw_joint",
    "right_wrist_yaw_joint",
)
_POSITION_LIMIT_TOLERANCE_RADIANS = 1e-9


class PositionSource(Protocol):
    def read(self) -> TeachingArmSample: ...


class ConfirmationGate(Protocol):
    """Non-blocking operator confirmation used while motor commands are streamed."""

    def request(self, *, token: str, message: str) -> None: ...

    def poll(self) -> bool | None: ...


class OperatorConfirmationError(RuntimeError):
    """The operator declined or could not provide a required confirmation."""


@dataclass(frozen=True, slots=True)
class TeleopSettings:
    group: str = "upper_body"
    wrist_yaw_group: str | None = None
    owner: str = "dynamixel-upper-body-teleop"
    rate_hz: float = 30.0
    maximum_velocity_radians_per_second: float = 0.5
    gain_ramp_seconds: float = 5.0
    transition_duration_seconds: float = 5.0
    telemetry_timeout_seconds: float = 5.0
    maximum_feedback_age_seconds: float = 0.25
    command_validity_seconds: float = 0.25
    acknowledgement_timeout_seconds: float = 1.0
    rpc_timeout_seconds: float = 5.0
    pipeline_depth: int = 2
    wrist_yaw_kp: float = 10.0
    wrist_yaw_kd: float = 1.0
    duration_seconds: float | None = None

    def __post_init__(self) -> None:
        if not self.group.strip() or not self.owner.strip():
            raise ValueError("group and owner must be non-empty")
        if self.wrist_yaw_group is not None and not self.wrist_yaw_group.strip():
            raise ValueError("wrist-yaw group must be non-empty when enabled")
        positive = {
            "rate": self.rate_hz,
            "maximum velocity": self.maximum_velocity_radians_per_second,
            "transition duration": self.transition_duration_seconds,
            "telemetry timeout": self.telemetry_timeout_seconds,
            "maximum feedback age": self.maximum_feedback_age_seconds,
            "command validity": self.command_validity_seconds,
            "acknowledgement timeout": self.acknowledgement_timeout_seconds,
            "RPC timeout": self.rpc_timeout_seconds,
        }
        for name, value in positive.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be positive and finite")
        if not math.isfinite(self.gain_ramp_seconds) or self.gain_ramp_seconds < 0.0:
            raise ValueError("gain ramp must be non-negative and finite")
        if isinstance(self.pipeline_depth, bool) or not isinstance(self.pipeline_depth, int):
            raise TypeError("pipeline depth must be an integer")
        if self.pipeline_depth <= 0:
            raise ValueError("pipeline depth must be positive")
        if not math.isfinite(self.wrist_yaw_kp) or self.wrist_yaw_kp <= 0.0:
            raise ValueError("wrist-yaw kp must be positive and finite")
        if not math.isfinite(self.wrist_yaw_kd) or self.wrist_yaw_kd < 0.0:
            raise ValueError("wrist-yaw kd must be non-negative and finite")
        if self.duration_seconds is not None and (
            not math.isfinite(self.duration_seconds) or self.duration_seconds <= 0.0
        ):
            raise ValueError("duration must be positive and finite")


def build_target_frame(
    positions: Mapping[str, float],
    gain_scale: float,
    joints: tuple[JointMapping, ...] = UPPER_BODY_JOINTS,
) -> dict[str, CoordinateTarget]:
    """Create the complete impedance frame required by the daemon."""

    if not math.isfinite(gain_scale) or not 0.0 <= gain_scale <= 1.0:
        raise ValueError("gain scale must be finite and in the range 0..=1")
    expected = {joint.coordinate for joint in joints}
    actual = set(positions)
    if actual != expected:
        raise ValueError(
            f"incomplete target positions: missing={sorted(expected - actual)}, "
            f"unexpected={sorted(actual - expected)}"
        )

    frame: dict[str, CoordinateTarget] = {}
    for joint in joints:
        position = float(positions[joint.coordinate])
        if not math.isfinite(position):
            raise ValueError(f"position for {joint.coordinate!r} must be finite")
        if not joint.minimum_radians <= position <= joint.maximum_radians:
            raise ValueError(
                f"position for {joint.coordinate!r} is outside its effective "
                f"[{joint.minimum_radians}, {joint.maximum_radians}] rad limits"
            )
        frame[joint.coordinate] = CoordinateTarget.position(
            position,
            kp=joint.kp * gain_scale,
            kd=joint.kd,
        )
    return frame


def _validate_live_joint_limits(
    motors: Sequence[Motor],
    joints: Sequence[JointMapping] = UPPER_BODY_JOINTS,
) -> None:
    expected = {joint.coordinate for joint in joints}
    live_by_coordinate: dict[str, Motor] = {}
    for motor in motors:
        if motor.coordinate in live_by_coordinate:
            raise ValueError(
                f"live upper-body inventory assigns coordinate {motor.coordinate!r} more than once"
            )
        live_by_coordinate[motor.coordinate] = motor

    missing = sorted(expected - set(live_by_coordinate))
    unexpected = sorted(set(live_by_coordinate) - expected)
    if missing or unexpected:
        raise ValueError(
            f"live upper-body motor mapping mismatch: missing={missing}, unexpected={unexpected}"
        )

    for joint in joints:
        motor = live_by_coordinate[joint.coordinate]
        live_minimum = motor.limits.position_min
        live_maximum = motor.limits.position_max
        if (
            joint.minimum_radians < live_minimum - _POSITION_LIMIT_TOLERANCE_RADIANS
            or joint.maximum_radians > live_maximum + _POSITION_LIMIT_TOLERANCE_RADIANS
        ):
            raise ValueError(
                f"live motor {motor.name!r} position range "
                f"[{live_minimum}, {live_maximum}] does not contain the effective "
                f"teleop range [{joint.minimum_radians}, {joint.maximum_radians}] "
                f"for {joint.coordinate!r}"
            )


def build_wrist_yaw_hold_frame(
    positions: Mapping[str, float],
    gain_scale: float,
    *,
    kp: float = 10.0,
    kd: float = 1.0,
) -> dict[str, CoordinateTarget]:
    """Build a complete captured-position hold for the optional wrist-yaw group."""

    if not math.isfinite(gain_scale) or not 0.0 <= gain_scale <= 1.0:
        raise ValueError("gain scale must be finite and in the range 0..=1")
    if not math.isfinite(kp) or kp <= 0.0:
        raise ValueError("wrist-yaw kp must be positive and finite")
    if not math.isfinite(kd) or kd < 0.0:
        raise ValueError("wrist-yaw kd must be non-negative and finite")
    expected = set(WRIST_YAW_COORDINATES)
    actual = set(positions)
    if actual != expected:
        raise ValueError(
            f"incomplete wrist-yaw hold positions: missing={sorted(expected - actual)}, "
            f"unexpected={sorted(actual - expected)}"
        )
    frame: dict[str, CoordinateTarget] = {}
    for coordinate in WRIST_YAW_COORDINATES:
        position = float(positions[coordinate])
        if not math.isfinite(position):
            raise ValueError(f"position for {coordinate!r} must be finite")
        frame[coordinate] = CoordinateTarget.position(
            position,
            kp=kp * gain_scale,
            kd=kd,
        )
    return frame


def slew_positions(
    previous: Mapping[str, float],
    requested: Mapping[str, float],
    maximum_delta: float,
) -> dict[str, float]:
    """Limit every coordinate to a bounded position change for one command tick."""

    if not math.isfinite(maximum_delta) or maximum_delta <= 0.0:
        raise ValueError("maximum position delta must be positive and finite")
    if set(previous) != set(requested):
        raise ValueError("previous and requested positions must contain the same coordinates")

    limited: dict[str, float] = {}
    for coordinate, old_position in previous.items():
        new_position = requested[coordinate]
        if not math.isfinite(old_position) or not math.isfinite(new_position):
            raise ValueError(f"positions for {coordinate!r} must be finite")
        requested_delta = new_position - old_position
        if abs(requested_delta) <= maximum_delta:
            limited[coordinate] = new_position
        else:
            limited[coordinate] = old_position + math.copysign(
                maximum_delta,
                requested_delta,
            )
    return limited


def interpolate_positions(
    start: Mapping[str, float],
    target: Mapping[str, float],
    progress: float,
) -> dict[str, float]:
    """Linearly interpolate a complete position frame."""

    if not math.isfinite(progress) or not 0.0 <= progress <= 1.0:
        raise ValueError("interpolation progress must be finite and in the range 0..=1")
    if set(start) != set(target):
        raise ValueError("start and target positions must contain the same coordinates")

    interpolated: dict[str, float] = {}
    for coordinate, start_position in start.items():
        target_position = target[coordinate]
        if not math.isfinite(start_position) or not math.isfinite(target_position):
            raise ValueError(f"positions for {coordinate!r} must be finite")
        interpolated[coordinate] = start_position + (target_position - start_position) * progress
    return interpolated


def required_transition_duration(
    start: Mapping[str, float],
    target: Mapping[str, float],
    *,
    minimum_seconds: float,
    maximum_velocity_radians_per_second: float,
) -> float:
    """Choose a transition duration that respects the configured velocity bound."""

    if not math.isfinite(minimum_seconds) or minimum_seconds <= 0.0:
        raise ValueError("minimum transition duration must be positive and finite")
    if (
        not math.isfinite(maximum_velocity_radians_per_second)
        or maximum_velocity_radians_per_second <= 0.0
    ):
        raise ValueError("maximum velocity must be positive and finite")
    if set(start) != set(target):
        raise ValueError("start and target positions must contain the same coordinates")

    maximum_distance = 0.0
    for coordinate, start_position in start.items():
        target_position = target[coordinate]
        if not math.isfinite(start_position) or not math.isfinite(target_position):
            raise ValueError(f"positions for {coordinate!r} must be finite")
        maximum_distance = max(maximum_distance, abs(target_position - start_position))
    if maximum_distance == 0.0:
        return 0.0
    return max(
        minimum_seconds,
        maximum_distance / maximum_velocity_radians_per_second,
    )


def wait_for_initial_positions(
    robot: Robot,
    group_name: str,
    *,
    timeout_seconds: float,
    maximum_feedback_age_seconds: float,
) -> dict[str, float]:
    """Read one complete, fresh position frame for a motor group."""

    group = robot.group(group_name)
    reader = robot.states(groups=(group_name,), imus=())
    deadline = time.monotonic() + timeout_seconds
    maximum_age_ns = int(maximum_feedback_age_seconds * 1_000_000_000)
    last_problem = "no complete state frame received"
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            raise TimeoutError(
                f"timed out waiting for fresh {group_name!r} feedback: {last_problem}"
            )
        try:
            frame = reader.read(timeout=remaining)
        except TimeoutError as error:
            raise TimeoutError(
                f"timed out waiting for fresh {group_name!r} feedback: {last_problem}"
            ) from error

        consumer_delay_ns = max(0, time.monotonic_ns() - frame.received_monotonic_ns)
        transport_delay_ns = frame.round_trip_ns + consumer_delay_ns
        stale = [
            motor
            for motor in frame.motors
            if motor.sample_age_ns + transport_delay_ns > maximum_age_ns
        ]
        if not stale:
            positions = {motor.coordinate: motor.position for motor in frame.motors}
            expected = set(group.coordinates)
            actual = set(positions)
            if actual != expected:
                raise RuntimeError(
                    f"state frame coordinate mismatch for {group_name!r}: "
                    f"missing={sorted(expected - actual)}, "
                    f"unexpected={sorted(actual - expected)}"
                )
            return positions

        worst = max(stale, key=lambda motor: motor.sample_age_ns)
        total_age_ms = (worst.sample_age_ns + transport_delay_ns) / 1_000_000.0
        last_problem = f"stale feedback for {worst.coordinate!r}: {total_age_ms:.1f} ms"
        time.sleep(min(0.01, max(0.0, deadline - time.monotonic())))


def run_teleoperation(
    robot: Robot,
    source: PositionSource,
    settings: TeleopSettings,
    *,
    confirmation: ConfirmationGate,
    should_stop: Callable[[], bool] | None = None,
    status: Callable[[str], None] | None = None,
) -> None:
    """Run until interrupted, stopped, expired, or any hardware operation fails."""

    stop_requested = should_stop if should_stop is not None else lambda: False
    report = status if status is not None else lambda _message: None
    group = robot.group(settings.group)
    validate_group_coordinates(group.coordinates)
    _validate_live_joint_limits(group.motors)
    wrist_yaw_group = (
        robot.group(settings.wrist_yaw_group) if settings.wrist_yaw_group is not None else None
    )
    if wrist_yaw_group is not None:
        expected_wrist_coordinates = set(WRIST_YAW_COORDINATES)
        actual_wrist_coordinates = set(wrist_yaw_group.coordinates)
        if actual_wrist_coordinates != expected_wrist_coordinates or len(
            wrist_yaw_group.coordinates
        ) != len(WRIST_YAW_COORDINATES):
            raise ValueError(
                "wrist-yaw coordinate mismatch: "
                f"missing={sorted(expected_wrist_coordinates - actual_wrist_coordinates)}, "
                f"unexpected={sorted(actual_wrist_coordinates - expected_wrist_coordinates)}"
            )

    # Prove that a complete teaching-arm sample is available before acquiring a
    # motor lease or enabling the upper body.
    sample = source.read()
    period = 1.0 / settings.rate_hz
    maximum_delta = settings.maximum_velocity_radians_per_second * period

    confirmation.request(
        token="HOLD",
        message=(
            f"Teaching-arm input and daemon group {group.name!r}"
            + (f" plus {wrist_yaw_group.name!r}" if wrist_yaw_group is not None else "")
            + " are ready. HOLD will acquire control and stabilize the upper body"
            + (" and wrist-yaw motors" if wrist_yaw_group is not None else "")
            + " at fresh measured robot positions."
        ),
    )
    while True:
        if stop_requested():
            return
        decision = confirmation.poll()
        if decision is True:
            break
        if decision is False:
            raise OperatorConfirmationError("HOLD was not confirmed")
        # Keep validating the input bus while waiting. When master torque is
        # active this also keeps emitting its weak zero-position feedback goal.
        sample = source.read()
        time.sleep(min(period, 0.05))

    # Confirm that the teaching-arm bus is still healthy, then acquire fresh
    # robot feedback as close as possible to the moment control is enabled.
    sample = source.read()
    with ExitStack() as stack:
        initial_positions = wait_for_initial_positions(
            robot,
            group.name,
            timeout_seconds=settings.telemetry_timeout_seconds,
            maximum_feedback_age_seconds=settings.maximum_feedback_age_seconds,
        )
        wrist_yaw_positions: dict[str, float] | None = None
        if wrist_yaw_group is not None:
            wrist_yaw_positions = wait_for_initial_positions(
                robot,
                wrist_yaw_group.name,
                timeout_seconds=settings.telemetry_timeout_seconds,
                maximum_feedback_age_seconds=settings.maximum_feedback_age_seconds,
            )

        control = stack.enter_context(
            group.control(
                settings.owner,
                command_validity=settings.command_validity_seconds,
                acknowledgement_timeout=settings.acknowledgement_timeout_seconds,
                rpc_timeout=settings.rpc_timeout_seconds,
            )
        )
        wrist_yaw_control = (
            stack.enter_context(
                wrist_yaw_group.control(
                    settings.owner,
                    command_validity=settings.command_validity_seconds,
                    acknowledgement_timeout=settings.acknowledgement_timeout_seconds,
                    rpc_timeout=settings.rpc_timeout_seconds,
                )
            )
            if wrist_yaw_group is not None
            else None
        )
        initial_hold_frame = build_target_frame(initial_positions, gain_scale=0.0)
        control.start(initial_hold_frame)
        pipeline = CommandPipeline(control, max_pending=settings.pipeline_depth)
        wrist_yaw_pipeline: CommandPipeline | None = None
        if wrist_yaw_control is not None and wrist_yaw_positions is not None:
            wrist_yaw_hold_frame = build_wrist_yaw_hold_frame(
                wrist_yaw_positions,
                gain_scale=0.0,
                kp=settings.wrist_yaw_kp,
                kd=settings.wrist_yaw_kd,
            )
            # Enabling another group may take longer than upper_body's command
            # watchdog. Start it on a worker while the calling thread keeps the
            # already-running upper body at its captured pose.
            with ThreadPoolExecutor(
                max_workers=1,
                thread_name_prefix="bello-wrist-yaw-start",
            ) as executor:
                start_future = executor.submit(
                    wrist_yaw_control.start,
                    wrist_yaw_hold_frame,
                )
                startup_tick = time.monotonic()
                while not start_future.done():
                    pipeline.send(initial_hold_frame)
                    pipeline.poll()
                    startup_tick += period
                    delay = startup_tick - time.monotonic()
                    if delay > 0.0:
                        time.sleep(delay)
                    else:
                        startup_tick = time.monotonic()
                start_future.result()
                pipeline.poll()
            wrist_yaw_pipeline = CommandPipeline(
                wrist_yaw_control,
                max_pending=settings.pipeline_depth,
            )
        control_started_at = time.monotonic()
        next_tick = control_started_at
        last_clipped: tuple[str, ...] = ()
        completed_normally = False

        def finish_requested(now: float) -> bool:
            return stop_requested() or (
                settings.duration_seconds is not None
                and now - control_started_at >= settings.duration_seconds
            )

        def advance_tick(current_tick: float) -> float:
            following_tick = current_tick + period
            delay = following_tick - time.monotonic()
            if delay > 0.0:
                time.sleep(delay)
                return following_tick
            return time.monotonic()

        def report_clipping(current_sample: TeachingArmSample) -> None:
            nonlocal last_clipped
            if current_sample.clipped_coordinates == last_clipped:
                return
            last_clipped = current_sample.clipped_coordinates
            if last_clipped:
                report("teaching-arm limits active for " + ", ".join(last_clipped))
            else:
                report("teaching-arm positions are within configured limits")

        def send_frames(
            upper_body_frame: Mapping[str, CoordinateTarget],
            wrist_yaw_gain_scale: float,
        ) -> None:
            pipeline.send(upper_body_frame)
            pipeline.poll()
            if wrist_yaw_pipeline is not None and wrist_yaw_positions is not None:
                wrist_yaw_pipeline.send(
                    build_wrist_yaw_hold_frame(
                        wrist_yaw_positions,
                        gain_scale=wrist_yaw_gain_scale,
                        kp=settings.wrist_yaw_kp,
                        kd=settings.wrist_yaw_kd,
                    )
                )
                wrist_yaw_pipeline.poll()

        def drain_pipelines() -> None:
            pipeline.drain()
            if wrist_yaw_pipeline is not None:
                wrist_yaw_pipeline.drain()

        try:
            report(
                f"HOLD confirmed; stabilizing {group.name!r} at its measured position "
                + (
                    f"and {wrist_yaw_group.name!r} at its measured positions "
                    if wrist_yaw_group is not None
                    else ""
                )
                + f"over {settings.gain_ramp_seconds:.1f} seconds"
            )
            hold_started_at = control_started_at
            while True:
                now = time.monotonic()
                if finish_requested(now):
                    completed_normally = True
                    return
                elapsed = now - hold_started_at
                gain_scale = (
                    1.0
                    if settings.gain_ramp_seconds == 0.0
                    else min(elapsed / settings.gain_ramp_seconds, 1.0)
                )
                send_frames(
                    build_target_frame(initial_positions, gain_scale),
                    wrist_yaw_gain_scale=gain_scale,
                )
                sample = source.read()
                report_clipping(sample)
                if gain_scale >= 1.0:
                    drain_pipelines()
                    break
                next_tick = advance_tick(next_tick)

            current_difference = max(
                abs(sample.positions[coordinate] - initial_positions[coordinate])
                for coordinate in initial_positions
            )
            report(
                f"upper body is holding its measured position; current teaching-arm "
                f"offset is at most {current_difference:.3f} rad"
            )
            confirmation.request(
                token="TELEOP",
                message=(
                    "The upper body is actively holding. TELEOP will snapshot the "
                    "current teaching-arm pose and begin the slow transition."
                ),
            )

            transition_target: dict[str, float] | None = None
            while transition_target is None:
                now = time.monotonic()
                if finish_requested(now):
                    completed_normally = True
                    return
                sample = source.read()
                report_clipping(sample)
                send_frames(
                    build_target_frame(initial_positions, gain_scale=1.0),
                    wrist_yaw_gain_scale=1.0,
                )
                decision = confirmation.poll()
                if decision is True:
                    transition_target = dict(sample.positions)
                    drain_pipelines()
                    break
                if decision is False:
                    raise OperatorConfirmationError("TELEOP was not confirmed")
                next_tick = advance_tick(next_tick)

            transition_seconds = required_transition_duration(
                initial_positions,
                transition_target,
                minimum_seconds=settings.transition_duration_seconds,
                maximum_velocity_radians_per_second=(settings.maximum_velocity_radians_per_second),
            )
            maximum_transition = max(
                abs(transition_target[coordinate] - initial_positions[coordinate])
                for coordinate in initial_positions
            )
            report(
                f"TELEOP confirmed; interpolating {maximum_transition:.3f} rad maximum "
                f"offset over {transition_seconds:.1f} seconds"
            )

            transition_started_at = time.monotonic()
            previous = dict(initial_positions)
            while True:
                now = time.monotonic()
                if finish_requested(now):
                    completed_normally = True
                    return
                sample = source.read()
                report_clipping(sample)
                progress = (
                    1.0
                    if transition_seconds == 0.0
                    else min((now - transition_started_at) / transition_seconds, 1.0)
                )
                interpolated = interpolate_positions(
                    initial_positions,
                    transition_target,
                    progress,
                )
                previous = slew_positions(previous, interpolated, maximum_delta)
                send_frames(
                    build_target_frame(previous, gain_scale=1.0),
                    wrist_yaw_gain_scale=1.0,
                )
                if progress >= 1.0 and previous == transition_target:
                    drain_pipelines()
                    break
                next_tick = advance_tick(next_tick)

            report(
                f"transition complete; live teleoperation active at "
                f"{settings.rate_hz:.1f} Hz with maximum velocity "
                f"{settings.maximum_velocity_radians_per_second:.3f} rad/s"
            )
            while True:
                now = time.monotonic()
                if finish_requested(now):
                    break
                sample = source.read()
                report_clipping(sample)
                previous = slew_positions(previous, sample.positions, maximum_delta)
                send_frames(
                    build_target_frame(previous, gain_scale=1.0),
                    wrist_yaw_gain_scale=1.0,
                )
                next_tick = advance_tick(next_tick)
            completed_normally = True
        finally:
            if completed_normally:
                drain_pipelines()
