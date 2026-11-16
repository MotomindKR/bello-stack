use bello_robot_description::{Geometry, RobotDescription};
use nalgebra::{UnitQuaternion, Vector3};
use std::{collections::BTreeMap, f64::consts::FRAC_PI_2, path::Path};

fn load() -> RobotDescription {
    RobotDescription::load(
        &Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../descriptions/stereo-gimbal/urdf/stereo-gimbal.urdf"),
    )
    .unwrap()
}

fn positions(yaw: f64, pitch: f64, roll: f64) -> BTreeMap<String, f64> {
    BTreeMap::from([
        ("gimbal_yaw".into(), yaw),
        ("gimbal_pitch".into(), pitch),
        ("gimbal_roll".into(), roll),
    ])
}

#[test]
fn cad_zero_pose_has_forward_optics_and_the_measured_lens_spacing() {
    let robot = load();
    let q = positions(0.0, 0.0, 0.0);
    assert_eq!(robot.motor_coordinates().len(), 3);
    let centre = robot
        .link_transform("gimbal_base_link", "stereo_camera_optical_frame", &q)
        .unwrap();
    assert!((centre.translation.vector - Vector3::new(0.112, 0.0, 0.181)).norm() < 1e-12);
    assert!((centre.rotation * Vector3::z() - Vector3::x()).norm() < 1e-12);
    assert!((centre.rotation * Vector3::x() + Vector3::y()).norm() < 1e-12);
    assert!((centre.rotation * Vector3::y() + Vector3::z()).norm() < 1e-12);
    let left = robot
        .link_transform("gimbal_base_link", "stereo_left_camera_optical_frame", &q)
        .unwrap();
    let right = robot
        .link_transform("gimbal_base_link", "stereo_right_camera_optical_frame", &q)
        .unwrap();
    assert!(
        (left.translation.vector - right.translation.vector - Vector3::new(0.0, 0.0592, 0.0))
            .norm()
            < 1e-12
    );
}

#[test]
fn camera_orientation_and_stereo_baseline_follow_yaw_pitch_roll() {
    let robot = load();
    let zero = robot
        .link_transform(
            "gimbal_base_link",
            "stereo_camera_optical_frame",
            &positions(0.0, 0.0, 0.0),
        )
        .unwrap();
    for (yaw, pitch, roll) in [
        (0.4, -0.5, 0.6),
        (-FRAC_PI_2, 0.0, 0.0),
        (0.0, FRAC_PI_2, 0.0),
        (0.0, 0.0, -FRAC_PI_2),
        (FRAC_PI_2, -FRAC_PI_2, FRAC_PI_2),
    ] {
        let q = positions(yaw, pitch, roll);
        let orientation = UnitQuaternion::from_euler_angles(roll, pitch, yaw);
        let camera = robot
            .link_transform("gimbal_base_link", "stereo_camera_optical_frame", &q)
            .unwrap();
        assert!(camera.rotation.angle_to(&(orientation * zero.rotation)) < 1e-12);
        let left = robot
            .link_transform("gimbal_base_link", "stereo_left_camera_optical_frame", &q)
            .unwrap();
        let right = robot
            .link_transform("gimbal_base_link", "stereo_right_camera_optical_frame", &q)
            .unwrap();
        assert!(
            (left.translation.vector
                - right.translation.vector
                - orientation * Vector3::new(0.0, 0.0592, 0.0))
            .norm()
                < 1e-12
        );
    }
}

#[test]
fn roll_keeps_the_lens_midpoint_on_its_shaft_and_preserves_the_parent_stators() {
    let robot = load();
    let before = positions(0.3, -0.4, 0.0);
    let after = positions(0.3, -0.4, FRAC_PI_2);
    let midpoint = |q: &BTreeMap<String, f64>| {
        robot
            .link_transform("gimbal_base_link", "stereo_camera_link", q)
            .unwrap()
            .translation
            .vector
    };
    assert!((midpoint(&before) - midpoint(&after)).norm() < 1e-12);
    for link in ["gimbal_base_link", "gimbal_yaw_link", "gimbal_pitch_link"] {
        let a = robot
            .link_transform("gimbal_base_link", link, &before)
            .unwrap();
        let b = robot
            .link_transform("gimbal_base_link", link, &after)
            .unwrap();
        assert!((a.to_homogeneous() - b.to_homogeneous()).norm() < 1e-12);
    }
    for link in robot.links() {
        for collision in &link.collisions {
            assert!(!matches!(collision.geometry, Geometry::Mesh { .. }));
        }
    }
}
