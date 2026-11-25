use bello_robot_description::{Geometry, RobotDescription};
use nalgebra::{Isometry3, Point3, UnitQuaternion, Vector3};
use std::{collections::BTreeMap, f64::consts::FRAC_PI_2, path::Path};

struct Fixture {
    robot: RobotDescription,
    joints: BTreeMap<String, f64>,
}

impl Fixture {
    fn load(profile: &str) -> Self {
        let path = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../descriptions")
            .join(profile);
        let robot = RobotDescription::load(&path).unwrap();
        let joints = robot
            .motor_coordinates()
            .into_iter()
            .map(|name| (name.to_owned(), 0.0))
            .collect();
        Self { robot, joints }
    }

    fn set(&mut self, joint: &str, radians: f64) {
        *self.joints.get_mut(joint).unwrap() = radians;
    }

    fn pose(&self, link: &str) -> Isometry3<f64> {
        self.robot
            .link_transform(self.robot.root_link(), link, &self.joints)
            .unwrap()
    }
}

fn assert_pose(actual: Isometry3<f64>, expected: Isometry3<f64>) {
    let delta = actual.inv_mul(&expected);
    assert!(
        delta.translation.vector.norm() < 1e-9 && delta.rotation.angle() < 1e-9,
        "pose error: {delta:?}"
    );
}

#[test]
fn zero_roll_preserves_legacy_arm_poses_and_gripper_closure() {
    let mut legacy = Fixture::load("milly/urdf/milly.urdf");
    let mut arm = Fixture::load("milly-7dof/urdf/milly-7dof.urdf");
    for (i, angle) in [-0.4, 0.6, 1.2, -0.3, 0.2, 0.0].into_iter().enumerate() {
        let joint = i + 1;
        let shifted = if joint < 3 { joint } else { joint + 1 };
        legacy.set(&format!("joint_{joint}"), angle);
        arm.set(&format!("joint_{shifted}"), angle);
    }
    for grip in [-2.1, -1.0, 0.0] {
        legacy.set("gripper", grip);
        arm.set("gripper", grip);
        for link in legacy.robot.links() {
            let shifted = match link.name.as_str() {
                "link_3" => "link_4",
                "link_4" => "link_5",
                "link_5" => "link_6",
                "link_6" => "link_7",
                other => other,
            };
            assert_pose(arm.pose(shifted), legacy.pose(&link.name));
        }
    }
}

#[test]
fn payloads_rotate_about_the_cad_shafts() {
    for (profile, sides) in [
        ("milly-7dof/urdf/milly-7dof.urdf", &[""][..]),
        (
            "milly-7dof/urdf/milly-7dof-dual.urdf",
            &["left_", "right_"][..],
        ),
    ] {
        let mut arm = Fixture::load(profile);
        assert_eq!(arm.joints.len(), 8 * sides.len());
        for &side in sides {
            for (joint, parent, shaft_z, angle) in [(3, 2, 0.097892511, -0.7), (7, 6, 0.0235, 0.7)]
            {
                let parent = format!("{side}link_{parent}");
                let joint = format!("{side}joint_{joint}");
                let payloads =
                    ["gripper_base", "camera_optical_frame"].map(|link| format!("{side}{link}"));
                let parent_pose = arm.pose(&parent);
                let before = payloads.each_ref().map(|link| arm.pose(link));
                let rotation = Isometry3::rotation_wrt_point(
                    UnitQuaternion::from_axis_angle(&Vector3::x_axis(), angle),
                    Point3::new(0.0, 0.0, shaft_z),
                );
                arm.set(&joint, 0.7);
                assert_pose(arm.pose(&parent), parent_pose);
                for (link, home) in payloads.iter().zip(before) {
                    assert_pose(
                        arm.pose(link),
                        parent_pose * rotation * parent_pose.inverse() * home,
                    );
                }
                arm.set(&joint, 0.0);
            }
        }
    }
}

#[test]
fn stereo_optics_follow_the_gimbal_axes() {
    let mut rig = Fixture::load("stereo-gimbal/urdf/stereo-gimbal.urdf");
    let optical = "stereo_camera_optical_frame";
    let home = rig.pose(optical);
    assert_pose(
        home,
        Isometry3::translation(0.112, 0.0, 0.181)
            * UnitQuaternion::from_euler_angles(-FRAC_PI_2, 0.0, -FRAC_PI_2),
    );
    for (yaw, pitch, roll) in [
        (0.0, 0.0, 0.0),
        (0.4, -0.5, 0.6),
        (-FRAC_PI_2, FRAC_PI_2, -FRAC_PI_2),
    ] {
        rig.set("gimbal_yaw", yaw);
        rig.set("gimbal_pitch", pitch);
        let midpoint = rig.pose(optical).translation;
        rig.set("gimbal_roll", roll);
        let rotation = UnitQuaternion::from_euler_angles(roll, pitch, yaw);
        assert_pose(
            rig.pose(optical),
            Isometry3::from_parts(midpoint, rotation * home.rotation),
        );
        let [left, right] = [
            "stereo_left_camera_optical_frame",
            "stereo_right_camera_optical_frame",
        ]
        .map(|link| rig.pose(link).translation.vector);
        assert!((left - right - rotation * Vector3::y() * 0.0592).norm() < 1e-9);
    }
    assert!(
        rig.robot
            .links()
            .flat_map(|link| &link.collisions)
            .all(|c| !matches!(c.geometry, Geometry::Mesh { .. }))
    );
}

#[test]
fn mounted_stereo_and_arms_move_independently() {
    let mut rig = Fixture::load("milly-7dof-gimbal/urdf/milly-7dof-gimbal.urdf");
    let [left, right] = ["left_base_link", "right_base_link"]
        .map(|link| rig.pose(link) * Point3::new(0.0, -0.045, 0.004));
    assert!((right - left - Vector3::y() * 0.450).norm() < 1e-9);
    let expected_mount = left + (right - left) / 2.0 + Vector3::new(0.0425, 0.0, 0.426);
    assert!(
        (rig.pose("gimbal_base_link").translation.vector - expected_mount.coords).norm() < 1e-9
    );
    let optical = "stereo_camera_optical_frame";
    let home = rig.pose(optical);
    assert_eq!(rig.joints.len(), 19);
    assert_eq!(rig.robot.slider_cranks().len(), 2);
    assert!((home.translation.z - left.z - 0.607).abs() < 1e-9);
    assert!((home.rotation * Vector3::z() + Vector3::x()).norm() < 1e-9);
    assert!(rig.robot.link("center_d405_link").is_none());

    let arms = ["left_gripper_base", "right_gripper_base"];
    let initial = arms.map(|link| rig.pose(link));
    rig.set("gimbal_pitch", -0.4);
    rig.set("gimbal_yaw", 0.5);
    rig.set("gimbal_roll", 0.3);
    let tilted = rig.pose(optical);
    assert!(tilted.rotation.angle_to(&home.rotation) > 0.1);
    for (link, pose) in arms.into_iter().zip(initial) {
        assert_pose(rig.pose(link), pose);
    }
    rig.set("left_joint_3", 0.6);
    assert_pose(rig.pose(arms[1]), initial[1]);
    rig.set("right_joint_4", 1.2);
    assert_pose(rig.pose(optical), tilted);
}
