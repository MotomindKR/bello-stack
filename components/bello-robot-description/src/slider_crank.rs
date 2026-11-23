//! Geometric closure for the two mirrored sliders of a rotary gripper.
//!
//! URDF remains a tree. The four passive joints are calculated from the motor
//! joint; they are not independent motor coordinates or affine mimic joints.
use std::collections::{BTreeMap, BTreeSet};

use anyhow::{Context, bail};
use roxmltree::Node;

use super::{CoordinateState, Joint, JointKind, JointState, finite_attribute, required_attribute};

pub const XML_NAMESPACE: &str = "https://motomind.kr/bello/urdf";

#[derive(Clone, Debug)]
pub struct SliderCrank {
    pub name: String,
    pub driver: String,
    pub sliders: [String; 2],
    pub rods: [String; 2],
    pub crank_radius: f64,
    pub rod_length: f64,
    pub slider_offset: f64,
    /// Crank angle at driver position zero, in radians.
    pub phase: f64,
    /// Slider coordinate represented by the URDF joint's zero origin.
    pub slider_reference: f64,
}

impl SliderCrank {
    pub fn passive_joints(&self) -> impl Iterator<Item = &str> {
        self.sliders.iter().chain(&self.rods).map(String::as_str)
    }

    /// Positive-root assembly mode: each rod points outwards from the crank.
    /// Both slider axes point outwards; the second rod frame is rotated by pi.
    pub fn resolve(&self, driver: CoordinateState) -> anyhow::Result<[(String, JointState); 4]> {
        if !driver.position.is_finite() || !driver.velocity.is_finite() {
            bail!(
                "slider-crank {:?} received a non-finite driver state",
                self.name
            );
        }
        let theta = driver.position + self.phase;
        let (sin, cos) = theta.sin_cos();
        let vertical = self.slider_offset - self.crank_radius * sin;
        let radicand = self.rod_length * self.rod_length - vertical * vertical;
        if !radicand.is_finite() || radicand <= 0.0 {
            bail!(
                "slider-crank {:?} has no nonsingular closure at {}",
                self.name,
                driver.position
            );
        }
        let horizontal = radicand.sqrt();
        let slider = JointState {
            position: self.crank_radius * cos + horizontal - self.slider_reference,
            velocity: self.crank_radius * (-sin + vertical * cos / horizontal) * driver.velocity,
        };
        let rod = JointState {
            position: vertical.atan2(horizontal) - theta,
            velocity: (-self.crank_radius * cos / horizontal - 1.0) * driver.velocity,
        };
        Ok([
            (self.sliders[0].clone(), slider),
            (self.sliders[1].clone(), slider),
            (self.rods[0].clone(), rod),
            (self.rods[1].clone(), rod),
        ])
    }
}

pub(super) fn parse(
    robot: Node<'_, '_>,
    joints: &BTreeMap<String, Joint>,
    transmitted: &BTreeSet<&str>,
) -> anyhow::Result<Vec<SliderCrank>> {
    let mut result = Vec::new();
    let mut names = BTreeSet::new();
    let mut passive = BTreeSet::new();
    for node in robot
        .children()
        .filter(|node| node.has_tag_name((XML_NAMESPACE, "slider_crank")))
    {
        let pair = |attribute| -> anyhow::Result<[String; 2]> {
            let members: Vec<_> = required_attribute(node, attribute)?
                .split_whitespace()
                .map(str::to_owned)
                .collect();
            members
                .try_into()
                .map_err(|_| anyhow::anyhow!("slider-crank {attribute} needs exactly two joints"))
        };
        let linkage = SliderCrank {
            name: required_attribute(node, "name")?.to_owned(),
            driver: required_attribute(node, "driver")?.to_owned(),
            sliders: pair("sliders")?,
            rods: pair("rods")?,
            crank_radius: finite_attribute(node, "crank_radius")?,
            rod_length: finite_attribute(node, "rod_length")?,
            slider_offset: finite_attribute(node, "slider_offset")?,
            phase: finite_attribute(node, "phase")?,
            slider_reference: finite_attribute(node, "slider_reference")?,
        };
        if !names.insert(linkage.name.clone()) {
            bail!("duplicate slider-crank {:?}", linkage.name);
        }
        // A strict inequality also excludes toggle singularities for every angle.
        if linkage.crank_radius <= 0.0
            || linkage.rod_length <= linkage.crank_radius + linkage.slider_offset.abs()
            || linkage.slider_reference <= 0.0
        {
            bail!(
                "slider-crank {:?} has invalid or singular dimensions",
                linkage.name
            );
        }
        let driver = joints
            .get(&linkage.driver)
            .with_context(|| format!("unknown slider-crank driver {:?}", linkage.driver))?;
        if driver.kind != JointKind::Revolute || transmitted.contains(linkage.driver.as_str()) {
            bail!(
                "slider-crank driver {:?} must be a direct revolute motor joint",
                linkage.driver
            );
        }
        for (members, kind) in [
            (&linkage.sliders, JointKind::Prismatic),
            (&linkage.rods, JointKind::Revolute),
        ] {
            for name in members {
                let joint = joints
                    .get(name)
                    .with_context(|| format!("unknown slider-crank passive joint {name:?}"))?;
                if joint.kind != kind
                    || name == &linkage.driver
                    || transmitted.contains(name.as_str())
                    || !passive.insert(name.clone())
                {
                    bail!("invalid or multiply driven slider-crank passive joint {name:?}");
                }
            }
        }
        result.push(linkage);
    }
    for linkage in &result {
        if passive.contains(&linkage.driver) {
            bail!("slider-crank driver {:?} is itself passive", linkage.driver);
        }
    }
    Ok(result)
}

#[cfg(test)]
mod tests {
    use std::io::Write;
    use std::path::Path;

    use super::*;
    use crate::RobotDescription;

    const LINKAGE_URDF: &str = r#"
<robot name="linkage_fixture" xmlns:bello="https://motomind.kr/bello/urdf">
  <link name="base"/><link name="crank"/><link name="slider_a"/><link name="slider_b"/>
  <link name="rod_a"/><link name="rod_b"/>
  <joint name="driver" type="revolute">
    <parent link="base"/><child link="crank"/>
    <limit lower="-3" upper="3" effort="1" velocity="1"/>
  </joint>
  <joint name="slide_a" type="prismatic">
    <parent link="base"/><child link="slider_a"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
  <joint name="slide_b" type="prismatic">
    <parent link="base"/><child link="slider_b"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
  <joint name="hinge_a" type="revolute">
    <parent link="crank"/><child link="rod_a"/>
    <limit lower="-4" upper="4" effort="1" velocity="1"/>
  </joint>
  <joint name="hinge_b" type="revolute">
    <parent link="crank"/><child link="rod_b"/>
    <limit lower="-4" upper="4" effort="1" velocity="1"/>
  </joint>
  <bello:slider_crank name="linkage" driver="driver" sliders="slide_a slide_b" rods="hinge_a hinge_b"
    crank_radius="0.02" rod_length="0.04" slider_offset="0.01" phase="0.4" slider_reference="0.05"/>
</robot>
"#;

    fn load_fixture(xml: &str) -> anyhow::Result<RobotDescription> {
        let mut file = tempfile::NamedTempFile::new()?;
        file.write_all(xml.as_bytes())?;
        RobotDescription::load(file.path())
    }

    #[test]
    fn both_loops_close_and_velocities_match_position_derivatives() {
        let robot = load_fixture(LINKAGE_URDF).unwrap();
        let linkage = &robot.slider_cranks()[0];
        for i in 0..=360 {
            let q = std::f64::consts::TAU * i as f64 / 360.0;
            let states = linkage
                .resolve(CoordinateState {
                    position: q,
                    velocity: 0.7,
                })
                .unwrap();
            let theta = q + linkage.phase;
            for (side, sign) in [1.0, -1.0].into_iter().enumerate() {
                let beta = theta + states[side + 2].1.position;
                let slider = states[side].1.position + linkage.slider_reference;
                let endpoint_x =
                    sign * (linkage.crank_radius * theta.cos() + linkage.rod_length * beta.cos());
                let endpoint_y =
                    sign * (linkage.crank_radius * theta.sin() + linkage.rod_length * beta.sin());
                assert!((endpoint_x - sign * slider).abs() < 1e-12);
                assert!((endpoint_y - sign * linkage.slider_offset).abs() < 1e-12);
            }
            let eps = 1e-6;
            let next = linkage
                .resolve(CoordinateState {
                    position: q + eps,
                    velocity: 0.0,
                })
                .unwrap();
            let prev = linkage
                .resolve(CoordinateState {
                    position: q - eps,
                    velocity: 0.0,
                })
                .unwrap();
            for j in 0..4 {
                let numerical = (next[j].1.position - prev[j].1.position) / (2.0 * eps) * 0.7;
                assert!((states[j].1.velocity - numerical).abs() < 1e-8);
            }
        }
    }

    #[test]
    fn invalid_feedback_is_rejected() {
        let robot = load_fixture(LINKAGE_URDF).unwrap();
        let linkage = &robot.slider_cranks()[0];
        for position in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert!(
                linkage
                    .resolve(CoordinateState {
                        position,
                        velocity: 0.0
                    })
                    .is_err()
            );
        }
        assert!(
            linkage
                .resolve(CoordinateState {
                    position: 0.0,
                    velocity: f64::NAN
                })
                .is_err()
        );
    }

    #[test]
    fn passive_joints_cannot_be_commanded_as_motor_coordinates() {
        let robot = load_fixture(LINKAGE_URDF).unwrap();
        assert_eq!(robot.motor_coordinates(), BTreeSet::from(["driver"]));
        for joint in ["slide_a", "slide_b", "hinge_a", "hinge_b"] {
            assert!(robot.coordinate_mapping(joint).is_none());
            assert!(robot.coordinate_position_limits(joint).is_none());
        }
        assert!(
            robot
                .resolve_passive_positions(&BTreeMap::new())
                .unwrap()
                .is_empty()
        );
    }

    fn milly() -> RobotDescription {
        let path =
            Path::new(env!("CARGO_MANIFEST_DIR")).join("../../descriptions/milly/urdf/milly.urdf");
        RobotDescription::load(&path).unwrap()
    }

    #[test]
    fn milly_gripper_closes_within_joint_limits_throughout_motor_travel() {
        let robot = milly();
        let linkage = robot
            .slider_cranks()
            .iter()
            .find(|item| item.driver == "gripper")
            .unwrap();
        let driver = robot
            .joint(&linkage.driver)
            .unwrap()
            .limits
            .as_ref()
            .unwrap();
        let mut previous_slider = f64::INFINITY;
        for step in 0..=100 {
            let q = driver.lower + (driver.upper - driver.lower) * step as f64 / 100.0;
            let positions = BTreeMap::from([(linkage.driver.clone(), q)]);
            let resolved = robot.resolve_passive_positions(&positions).unwrap();
            for name in linkage.passive_joints() {
                let limits = robot.joint(name).unwrap().limits.as_ref().unwrap();
                assert!(
                    (limits.lower..=limits.upper).contains(&resolved[name]),
                    "{name} at {q}"
                );
            }
            let opening = resolved[&linkage.sliders[0]];
            assert!(opening < previous_slider);
            previous_slider = opening;
            for side in [1, 2] {
                let tip = robot
                    .link_transform("link_6", &format!("gripper_rod_{side}_tip"), &positions)
                    .unwrap();
                let pin = robot
                    .link_transform("link_6", &format!("finger_{side}_pin"), &positions)
                    .unwrap();
                assert!((tip.translation.vector - pin.translation.vector).norm() < 1e-10);
            }
        }
    }

    #[test]
    fn milly_fixed_camera_matches_cad_and_optical_axes() {
        let robot = milly();
        robot
            .fixed_transform("link_6", "camera_optical_frame")
            .unwrap();
        // REP-103 optical axes expressed in the camera's body frame.
        let optical = robot
            .fixed_transform("camera_link", "camera_optical_frame")
            .unwrap();
        assert!(
            (optical.rotation * nalgebra::Vector3::z() - nalgebra::Vector3::x()).norm() < 1e-10
        );
        assert!(
            (optical.rotation * nalgebra::Vector3::x() + nalgebra::Vector3::y()).norm() < 1e-10
        );
        assert!(
            (optical.rotation * nalgebra::Vector3::y() + nalgebra::Vector3::z()).norm() < 1e-10
        );
        let zero = (1..=6).map(|i| (format!("joint_{i}"), 0.0)).collect();
        let lens = robot
            .link_transform("base_link", "camera_optical_frame", &zero)
            .unwrap();
        // CAD lens centre and cylinder axis, independently registered to the arm
        // base: reverse CAD X/Y and translate Y by -95 mm.
        assert!(
            (lens.translation.vector - nalgebra::Vector3::new(0.0, 0.0717094, 0.3148766)).norm()
                < 1e-7
        );
        assert!(
            (lens.rotation * nalgebra::Vector3::z()
                - nalgebra::Vector3::new(0.0, 3.0_f64.sqrt() / 2.0, -0.5))
            .norm()
                < 1e-7
        );
    }

    #[test]
    fn mini_torso_grippers_close_independently_with_fixed_cameras_and_tcps() {
        let path = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../descriptions/bello_mini_torso/urdf/bello_mini_torso.urdf");
        let robot = RobotDescription::load(&path).unwrap();
        for (side, other) in [("left", "right"), ("right", "left")] {
            let driver = format!("{side}_gripper");
            let base = format!("{side}_wrist_roll_link");
            for frame in ["camera_optical_frame", "gripper_tcp"] {
                robot
                    .fixed_transform(&base, &format!("{side}_{frame}"))
                    .unwrap();
            }
            let limits = robot.joint(&driver).unwrap().limits.as_ref().unwrap();
            for step in 0..=100 {
                let q = limits.lower + (limits.upper - limits.lower) * step as f64 / 100.;
                let positions = BTreeMap::from([(driver.clone(), q)]);
                let resolved = robot.resolve_passive_positions(&positions).unwrap();
                assert!(!resolved.keys().any(|name| name.starts_with(other)));
                for finger in [1, 2] {
                    let tip = robot
                        .link_transform(
                            &base,
                            &format!("{side}_gripper_rod_{finger}_tip"),
                            &positions,
                        )
                        .unwrap();
                    let pin = robot
                        .link_transform(&base, &format!("{side}_finger_{finger}_pin"), &positions)
                        .unwrap();
                    assert!((tip.translation.vector - pin.translation.vector).norm() < 1e-10);
                }
            }
        }
    }

    #[test]
    fn malformed_linkage_metadata_is_rejected() {
        for (valid, invalid, expected_error) in [
            (
                r#"rod_length="0.04""#,
                r#"rod_length="0.02""#,
                "invalid or singular dimensions",
            ),
            (
                r#"crank_radius="0.02""#,
                r#"crank_radius="NaN""#,
                "crank_radius",
            ),
            (
                r#"driver="driver""#,
                r#"driver="missing""#,
                "unknown slider-crank driver",
            ),
            (
                r#"sliders="slide_a slide_b""#,
                r#"sliders="slide_a slide_a""#,
                "invalid or multiply driven slider-crank passive joint",
            ),
            (
                r#"rods="hinge_a hinge_b""#,
                r#"rods="slide_a slide_b""#,
                "invalid or multiply driven slider-crank passive joint",
            ),
        ] {
            let xml = LINKAGE_URDF.replace(valid, invalid);
            let error = load_fixture(&xml).unwrap_err().to_string();
            assert!(error.contains(expected_error), "{invalid}: {error}");
        }
    }
}
