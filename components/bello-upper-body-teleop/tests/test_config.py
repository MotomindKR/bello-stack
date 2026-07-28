import math

import pytest

from bello_upper_body_teleop.config import (
    UPPER_BODY_COORDINATES,
    UPPER_BODY_JOINTS,
    map_raw_positions,
    validate_group_coordinates,
)


def _raw_position(offset_radians: float) -> int:
    return round(offset_radians / math.pi * 2048.0)


def test_neutral_master_positions_map_to_zero() -> None:
    raw = {
        joint.dxl_id: _raw_position(joint.position_offset_radians) for joint in UPPER_BODY_JOINTS
    }

    sample = map_raw_positions(raw)

    assert sample.clipped_coordinates == ()
    assert sample.raw_positions == raw
    for position in sample.positions.values():
        assert position == pytest.approx(0.0, abs=1e-12)


def test_configured_default_pose_sets_opposing_elbow_yaw_positions() -> None:
    expected_defaults = dict.fromkeys(UPPER_BODY_COORDINATES, 0.0)
    expected_defaults["left_elbow_yaw_joint"] = -math.pi / 2.0
    expected_defaults["right_elbow_yaw_joint"] = math.pi / 2.0

    assert {
        joint.coordinate: joint.default_position_radians for joint in UPPER_BODY_JOINTS
    } == expected_defaults

    raw = {
        joint.dxl_id: joint.raw_position_for_robot_position(
            joint.default_position_radians,
        )
        for joint in UPPER_BODY_JOINTS
    }
    assert raw[25] == 3_072
    assert raw[15] == 1_024
    sample = map_raw_positions(raw)

    assert sample.clipped_coordinates == ()
    for coordinate, expected in expected_defaults.items():
        assert sample.positions[coordinate] == pytest.approx(
            expected,
            abs=math.pi / 2048.0,
        )


def test_mapping_clamps_to_tighter_physical_limits() -> None:
    raw = {
        joint.dxl_id: _raw_position(joint.position_offset_radians) for joint in UPPER_BODY_JOINTS
    }
    left_elbow = next(
        joint for joint in UPPER_BODY_JOINTS if joint.coordinate == "left_elbow_pitch_joint"
    )
    raw[left_elbow.dxl_id] = _raw_position(left_elbow.position_offset_radians + math.pi)

    sample = map_raw_positions(raw)

    assert sample.positions[left_elbow.coordinate] == pytest.approx(2.4)
    assert sample.clipped_coordinates == (left_elbow.coordinate,)


def test_robot_position_converts_back_to_master_encoder_target() -> None:
    for joint in UPPER_BODY_JOINTS:
        for position in (joint.minimum_radians, 0.0, joint.maximum_radians):
            raw = joint.raw_position_for_robot_position(position)
            mapped, _clipped = joint.map_raw_position(raw)
            assert mapped == pytest.approx(position, abs=math.pi / 2048.0)


def test_mapping_requires_every_configured_dynamixel() -> None:
    with pytest.raises(ValueError, match="missing Dynamixel positions"):
        map_raw_positions({})


def test_group_validation_accepts_any_order_but_rejects_a_mismatch() -> None:
    validate_group_coordinates(tuple(reversed(UPPER_BODY_COORDINATES)))

    with pytest.raises(ValueError, match="upper-body coordinate mismatch"):
        validate_group_coordinates((*UPPER_BODY_COORDINATES[:-1], "head_yaw_joint"))


def test_impedance_gains_match_deployed_upper_body_hold() -> None:
    expected_per_arm = (
        (70.0, 2.0),
        (70.0, 2.0),
        (20.0, 1.0),
        (40.0, 2.0),
        (20.0, 1.0),
        (10.0, 1.0),
    )

    assert tuple((joint.kp, joint.kd) for joint in UPPER_BODY_JOINTS[:6]) == expected_per_arm
    assert tuple((joint.kp, joint.kd) for joint in UPPER_BODY_JOINTS[6:]) == expected_per_arm
