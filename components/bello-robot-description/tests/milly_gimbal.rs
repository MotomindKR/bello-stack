use bello_robot_description::RobotDescription;
use nalgebra::Vector3;
use std::{collections::BTreeMap, path::Path};

fn load() -> RobotDescription {
    RobotDescription::load(
        &Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../descriptions/milly-7dof-gimbal/urdf/milly-7dof-gimbal.urdf"),
    )
    .unwrap()
}

fn positions() -> BTreeMap<String, f64> {
    let mut q = BTreeMap::new();
    for side in ["left", "right"] {
        for i in 1..=7 {
            q.insert(format!("{side}_joint_{i}"), 0.0);
        }
        q.insert(format!("{side}_gripper"), -2.11);
    }
    for axis in ["yaw", "pitch", "roll"] {
        q.insert(format!("gimbal_{axis}"), 0.0);
    }
    q
}

#[test]
fn level_camera_is_600_mm_above_the_450_mm_arm_mounting_plates() {
    let robot = load();
    let q = positions();
    assert_eq!(robot.motor_coordinates().len(), 19);
    assert_eq!(robot.slider_cranks().len(), 2);
    assert!(robot.link("center_d405_link").is_none());
    let mount_underside_in_arm = Vector3::new(0.0, -0.045, 0.004);
    let mut centres = Vec::new();
    for side in ["left", "right"] {
        let frame = robot
            .link_transform("dual_milly_base", &format!("{side}_base_link"), &q)
            .unwrap();
        centres.push(frame.translation.vector + frame.rotation * mount_underside_in_arm);
    }
    assert!(((centres[1] - centres[0]).norm() - 0.450).abs() < 1e-12);
    let gimbal = robot
        .link_transform("dual_milly_base", "gimbal_base_link", &q)
        .unwrap();
    let expected_mount = (centres[0] + centres[1]) / 2.0 + Vector3::new(0.0, 0.0, 0.419);
    assert!((gimbal.translation.vector - expected_mount).norm() < 1e-12);
    let camera = robot
        .link_transform("dual_milly_base", "stereo_camera_optical_frame", &q)
        .unwrap();
    assert!((camera.rotation * Vector3::z() + Vector3::x()).norm() < 1e-12);
    assert!((camera.translation.z - centres[0].z - 0.600).abs() < 1e-12);
}

#[test]
fn gimbal_and_arm_branches_move_independently() {
    let robot = load();
    let mut q = positions();
    let arms_before: Vec<_> = ["left_gripper_base", "right_gripper_base"]
        .into_iter()
        .map(|link| robot.link_transform("dual_milly_base", link, &q).unwrap())
        .collect();
    q.insert("gimbal_yaw".into(), 0.5);
    q.insert("gimbal_pitch".into(), -0.4);
    q.insert("gimbal_roll".into(), 0.3);
    for (i, link) in ["left_gripper_base", "right_gripper_base"]
        .into_iter()
        .enumerate()
    {
        let actual = robot.link_transform("dual_milly_base", link, &q).unwrap();
        assert!((actual.to_homogeneous() - arms_before[i].to_homogeneous()).norm() < 1e-12);
    }
    let camera_before = robot
        .link_transform("dual_milly_base", "stereo_camera_optical_frame", &q)
        .unwrap();
    q.insert("left_joint_3".into(), 0.6);
    q.insert("right_joint_4".into(), 1.2);
    let camera_after = robot
        .link_transform("dual_milly_base", "stereo_camera_optical_frame", &q)
        .unwrap();
    assert!((camera_before.to_homogeneous() - camera_after.to_homogeneous()).norm() < 1e-12);
}
