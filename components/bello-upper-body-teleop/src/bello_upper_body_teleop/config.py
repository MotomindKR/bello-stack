"""Teaching-arm calibration and the Bello upper-body coordinate mapping."""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass

_POSITION_SCALE_RADIANS = math.pi / 2048.0


@dataclass(frozen=True, slots=True)
class JointMapping:
    """Map one Dynamixel encoder to one daemon coordinate."""

    dxl_id: int
    coordinate: str
    position_offset_radians: float
    action_gain: float
    minimum_radians: float
    maximum_radians: float
    kp: float
    kd: float
    default_position_radians: float = 0.0

    def __post_init__(self) -> None:
        if isinstance(self.dxl_id, bool) or not isinstance(self.dxl_id, int):
            raise TypeError("Dynamixel ID must be an integer")
        if not 0 <= self.dxl_id <= 252:
            raise ValueError("Dynamixel ID must be in the range 0..=252")
        if not self.coordinate.strip():
            raise ValueError("coordinate must be non-empty")
        values = (
            self.position_offset_radians,
            self.action_gain,
            self.minimum_radians,
            self.maximum_radians,
            self.kp,
            self.kd,
            self.default_position_radians,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("joint mapping values must be finite")
        if self.action_gain == 0.0:
            raise ValueError("action gain must be nonzero")
        if self.minimum_radians >= self.maximum_radians:
            raise ValueError("joint minimum must be lower than its maximum")
        if not self.minimum_radians <= self.default_position_radians <= self.maximum_radians:
            raise ValueError("joint default position must be within configured limits")
        if self.kp < 0.0 or self.kd < 0.0:
            raise ValueError("joint gains must be non-negative")

    def map_raw_position(self, raw_position: int) -> tuple[float, bool]:
        """Convert a signed Dynamixel position count to a clamped robot position."""

        unclamped = self.unclamped_robot_position(raw_position)
        position = min(max(unclamped, self.minimum_radians), self.maximum_radians)
        return position, position != unclamped

    def unclamped_robot_position(self, raw_position: int) -> float:
        """Convert an encoder count without applying robot joint limits."""

        if isinstance(raw_position, bool) or not isinstance(raw_position, int):
            raise TypeError("raw position must be an integer")
        master_radians = raw_position * _POSITION_SCALE_RADIANS
        return self.action_gain * (master_radians - self.position_offset_radians)

    def raw_position_for_robot_position(self, position_radians: float) -> int:
        """Convert a robot-space position back to a Dynamixel encoder target."""

        if not math.isfinite(position_radians):
            raise ValueError("robot position must be finite")
        if not self.minimum_radians <= position_radians <= self.maximum_radians:
            raise ValueError(
                f"robot position for {self.coordinate!r} must be within configured limits"
            )
        master_radians = position_radians / self.action_gain + self.position_offset_radians
        return round(master_radians / _POSITION_SCALE_RADIANS)


@dataclass(frozen=True, slots=True)
class TeachingArmSample:
    """One complete, calibrated sample from both teaching arms."""

    positions: dict[str, float]
    clipped_coordinates: tuple[str, ...]
    raw_positions: dict[int, int]


def _joint(
    dxl_id: int,
    coordinate: str,
    offset_turns: float,
    gain: float,
    limits_degrees: tuple[float, float],
    *,
    kp: float,
    kd: float,
    default_position_radians: float = 0.0,
) -> JointMapping:
    return JointMapping(
        dxl_id=dxl_id,
        coordinate=coordinate,
        position_offset_radians=offset_turns * math.pi,
        action_gain=gain,
        minimum_radians=math.radians(limits_degrees[0]),
        maximum_radians=math.radians(limits_degrees[1]),
        kp=kp,
        kd=kd,
        default_position_radians=default_position_radians,
    )


# This is the six-joint sphere-hand morphology configured by
# components/bello-hardwared/config/bello.yaml. The original teaching rig also
# has a torso, a seventh wrist joint, and grippers; those devices are deliberately
# excluded from the robot-coordinate mapping because the daemon's upper_body
# group does not contain them. The Dynamixel reader separately locks the torso
# at calibrated zero and the seventh wrist joints at their configured
# half-turn-opposite positions.
#
# Limits are the intersection of the original teaching-arm action limits and the
# physical upper-body actuator limits. The tighter limit wins.
#
# Impedance gains match the deployed upper-body hold configuration in
# ~/motomind_platform/configs/robots/motomind_bello/new_actuator/upper_hold.yaml.
UPPER_BODY_JOINTS: tuple[JointMapping, ...] = (
    _joint(
        21,
        "left_shoulder_pitch_joint",
        1.5,
        1.0,
        (-120.0, 25.0),
        kp=70.0,
        kd=2.0,
    ),
    _joint(
        22,
        "left_shoulder_roll_joint",
        0.5,
        1.0,
        (-10.0, math.degrees(2.0)),
        kp=70.0,
        kd=2.0,
    ),
    _joint(
        23,
        "left_shoulder_yaw_joint",
        1.0,
        -1.0,
        (math.degrees(-1.5), math.degrees(1.5)),
        kp=20.0,
        kd=1.0,
    ),
    _joint(
        24,
        "left_elbow_pitch_joint",
        1.0,
        1.0,
        (0.0, math.degrees(2.4)),
        kp=40.0,
        kd=2.0,
    ),
    _joint(
        25,
        "left_elbow_yaw_joint",
        1.0,
        -1.0,
        (-150.0, 30.0),
        kp=20.0,
        kd=1.0,
        default_position_radians=-math.pi / 2.0,
    ),
    _joint(
        26,
        "left_wrist_pitch_joint",
        1.0,
        -1.0,
        (math.degrees(-1.5), math.degrees(1.5)),
        kp=10.0,
        kd=1.0,
    ),
    _joint(
        11,
        "right_shoulder_pitch_joint",
        0.5,
        1.0,
        (-25.0, 120.0),
        kp=70.0,
        kd=2.0,
    ),
    _joint(
        12,
        "right_shoulder_roll_joint",
        1.5,
        -1.0,
        (-10.0, math.degrees(2.0)),
        kp=70.0,
        kd=2.0,
    ),
    _joint(
        13,
        "right_shoulder_yaw_joint",
        1.0,
        -1.0,
        (math.degrees(-1.5), math.degrees(1.5)),
        kp=20.0,
        kd=1.0,
    ),
    _joint(
        14,
        "right_elbow_pitch_joint",
        1.0,
        -1.0,
        (0.0, math.degrees(2.4)),
        kp=40.0,
        kd=2.0,
    ),
    _joint(
        15,
        "right_elbow_yaw_joint",
        1.0,
        -1.0,
        (-30.0, 150.0),
        kp=20.0,
        kd=1.0,
        default_position_radians=math.pi / 2.0,
    ),
    _joint(
        16,
        "right_wrist_pitch_joint",
        1.0,
        1.0,
        (math.degrees(-1.5), math.degrees(1.5)),
        kp=10.0,
        kd=1.0,
    ),
)

UPPER_BODY_COORDINATES = tuple(joint.coordinate for joint in UPPER_BODY_JOINTS)


def map_raw_positions(
    raw_positions: Mapping[int, int],
    joints: Sequence[JointMapping] = UPPER_BODY_JOINTS,
) -> TeachingArmSample:
    """Map a complete raw Dynamixel sample into daemon coordinates."""

    missing = [joint.dxl_id for joint in joints if joint.dxl_id not in raw_positions]
    if missing:
        raise ValueError(f"missing Dynamixel positions for IDs {missing}")

    positions: dict[str, float] = {}
    clipped: list[str] = []
    copied_raw: dict[int, int] = {}
    for joint in joints:
        raw_position = raw_positions[joint.dxl_id]
        if isinstance(raw_position, bool) or not isinstance(raw_position, int):
            raise TypeError(f"raw position for Dynamixel {joint.dxl_id} must be an integer")
        position, was_clipped = joint.map_raw_position(raw_position)
        positions[joint.coordinate] = position
        copied_raw[joint.dxl_id] = raw_position
        if was_clipped:
            clipped.append(joint.coordinate)
    return TeachingArmSample(positions, tuple(clipped), copied_raw)


def validate_group_coordinates(
    coordinates: Sequence[str],
    joints: Sequence[JointMapping] = UPPER_BODY_JOINTS,
) -> None:
    """Reject a daemon group that does not exactly match the teaching-arm mapping."""

    expected = {joint.coordinate for joint in joints}
    actual = set(coordinates)
    missing = sorted(expected - actual)
    unknown = sorted(actual - expected)
    if missing or unknown or len(coordinates) != len(joints):
        raise ValueError(
            "upper-body coordinate mismatch: "
            f"missing={missing}, unexpected={unknown}, "
            f"expected_count={len(joints)}, actual_count={len(coordinates)}"
        )
