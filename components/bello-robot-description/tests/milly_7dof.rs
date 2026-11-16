use bello_robot_description::RobotDescription;
use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};
use std::{collections::BTreeMap, path::Path};

fn load(relative: &str) -> RobotDescription {
    RobotDescription::load(
        &Path::new(env!("CARGO_MANIFEST_DIR")).join(format!("../../descriptions/{relative}")),
    )
    .unwrap()
}

fn shifted(name: &str) -> String {
    for kind in ["joint", "link"] {
        for i in 3..=6 {
            if name == format!("{kind}_{i}") {
                return format!("{kind}_{}", i + 1);
            }
        }
    }
    name.to_owned()
}

#[test]
fn zero_added_roll_and_wrist_yaw_preserve_legacy_link_poses_and_gripper_closure() {
    let original = load("milly/urdf/milly.urdf");
    let added = load("milly-7dof/urdf/milly-7dof.urdf");
    assert_eq!(added.motor_coordinates().len(), 8);
    assert_eq!(added.slider_cranks().len(), 1);
    for case in 0..12 {
        let positions: BTreeMap<String, f64> = BTreeMap::from([
            ("joint_1".into(), -0.4 + 0.05 * f64::from(case)),
            ("joint_2".into(), 0.4 + 0.08 * f64::from(case)),
            ("joint_3".into(), 0.9 + 0.04 * f64::from(case)),
            ("joint_4".into(), -0.3),
            ("joint_5".into(), 0.2),
            // The wrist-yaw axis has intentionally been corrected against CAD.
            // Its zero pose, including the original payload reference, is preserved.
            ("joint_6".into(), 0.0),
            ("gripper".into(), -2.2 + 0.15 * f64::from(case)),
        ]);
        let mut remapped: BTreeMap<_, _> = positions
            .iter()
            .map(|(name, value)| (shifted(name), *value))
            .collect();
        remapped.insert("joint_3".into(), 0.0);
        for link in original.links() {
            let expected = original
                .link_transform("base_link", &link.name, &positions)
                .unwrap();
            let actual = added
                .link_transform("base_link", &shifted(&link.name), &remapped)
                .unwrap();
            assert!(
                (expected.to_homogeneous() - actual.to_homogeneous()).norm() < 1e-10,
                "legacy frame changed: {} in pose {case}",
                link.name
            );
        }
    }
}

#[test]
fn wrist_yaw_rotates_payload_about_the_motor_shaft_in_single_and_dual_profiles() {
    for (profile, prefixes) in [
        ("milly-7dof.urdf", vec![""]),
        ("milly-7dof-dual.urdf", vec!["left_", "right_"]),
    ] {
        let robot = load(&format!("milly-7dof/urdf/{profile}"));
        let mut positions = BTreeMap::new();
        for prefix in &prefixes {
            for i in 1..=7 {
                positions.insert(format!("{prefix}joint_{i}"), 0.0);
            }
            positions.insert(format!("{prefix}gripper"), -2.2);
        }
        for prefix in prefixes {
            let parent = format!("{prefix}link_6");
            let coordinate = format!("{prefix}joint_7");
            let joint = robot.joint(&coordinate).unwrap();
            let shaft_direction = joint.origin.rotation * joint.axis.as_ref();
            assert!((shaft_direction - Vector3::x()).norm() < 1e-12);
            // The cylindrical wrist/motor faces in CAD define y=0, z=23.5 mm
            // in link_6. The point chosen along that line does not affect motion.
            assert!(joint.origin.translation.y.abs() < 1e-12);
            assert!((joint.origin.translation.z - 0.0235).abs() < 1e-12);
            for payload in ["gripper_base", "camera_optical_frame"] {
                let payload = format!("{prefix}{payload}");
                positions.insert(coordinate.clone(), 0.0);
                let before = robot.link_transform(&parent, &payload, &positions).unwrap();
                for angle in [-1.6, -0.7, 0.8, 1.6] {
                    positions.insert(coordinate.clone(), angle);
                    let actual = robot.link_transform(&parent, &payload, &positions).unwrap();
                    let expected = Isometry3::translation(0.0, 0.0, 0.0235)
                        * Isometry3::from_parts(
                            Translation3::identity(),
                            UnitQuaternion::from_axis_angle(&Vector3::x_axis(), angle),
                        )
                        * Isometry3::translation(0.0, 0.0, -0.0235)
                        * before;
                    assert!(
                        (actual.to_homogeneous() - expected.to_homogeneous()).norm() < 1e-10,
                        "{profile}: {payload} does not rotate about its motor shaft at {angle}"
                    );
                }
            }
            positions.insert(coordinate, 0.0);
        }
    }
}

#[test]
fn added_roll_rotates_distal_chain_about_upper_arm_without_moving_shoulder() {
    let robot = load("milly-7dof/urdf/milly-7dof.urdf");
    let mut positions: BTreeMap<_, _> = (1..=7).map(|i| (format!("joint_{i}"), 0.0)).collect();
    positions.insert("gripper".into(), -2.2);
    positions.insert("joint_2".into(), 0.6);
    let shoulder = robot
        .link_transform("base_link", "link_2", &positions)
        .unwrap();
    let camera = robot
        .link_transform("base_link", "camera_optical_frame", &positions)
        .unwrap();
    let joint = robot.joint("joint_3").unwrap();
    let axis_frame = shoulder * joint.origin;
    let expected_rotation = axis_frame
        * Isometry3::from_parts(
            Translation3::identity(),
            UnitQuaternion::from_axis_angle(&joint.axis, 0.7),
        )
        * axis_frame.inverse();
    positions.insert("joint_3".into(), 0.7);
    let actual = robot
        .link_transform("base_link", "camera_optical_frame", &positions)
        .unwrap();
    let expected = expected_rotation * camera;
    assert!((expected.to_homogeneous() - actual.to_homogeneous()).norm() < 1e-10);
    assert!((camera.to_homogeneous() - actual.to_homogeneous()).norm() > 0.1);
    let unmoved = robot
        .link_transform("base_link", "link_2", &positions)
        .unwrap();
    assert!((shoulder.to_homogeneous() - unmoved.to_homogeneous()).norm() < 1e-12);
}

#[test]
fn dual_profile_exposes_sixteen_motor_coordinates_with_independent_rolls() {
    let robot = load("milly-7dof/urdf/milly-7dof-dual.urdf");
    assert_eq!(robot.motor_coordinates().len(), 16);
    assert_eq!(robot.slider_cranks().len(), 2);
    let mut positions = BTreeMap::new();
    for side in ["left", "right"] {
        for i in 1..=7 {
            positions.insert(format!("{side}_joint_{i}"), 0.0);
        }
        positions.insert(format!("{side}_gripper"), -2.2);
    }
    let before: Vec<_> = ["left", "right"]
        .into_iter()
        .map(|side| {
            robot
                .link_transform(
                    "dual_milly_base",
                    &format!("{side}_camera_optical_frame"),
                    &positions,
                )
                .unwrap()
        })
        .collect();
    positions.insert("left_joint_3".into(), 0.4);
    for (i, side) in ["left", "right"].into_iter().enumerate() {
        let after = robot
            .link_transform(
                "dual_milly_base",
                &format!("{side}_camera_optical_frame"),
                &positions,
            )
            .unwrap();
        let change = (before[i].to_homogeneous() - after.to_homogeneous()).norm();
        if side == "left" {
            assert!(change > 0.1);
        } else {
            assert!(change < 1e-12);
        }
    }
}
