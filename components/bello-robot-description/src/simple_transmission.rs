use std::collections::{BTreeMap, BTreeSet};

use anyhow::{Context, bail};
use roxmltree::Node;

use super::{
    CoordinateState, DifferentialTransmission, Joint, JointKind, JointState, child_number,
    required_attribute, transmission_type,
};

#[derive(Clone, Debug)]
pub struct CoordinateMapping {
    pub coefficients: BTreeMap<String, f64>,
    pub offset: f64,
}

/// One actuator attached to one joint; several mappings may share that joint.
#[derive(Clone, Debug)]
pub struct SimpleTransmission {
    pub name: String,
    pub coordinate: String,
    pub joint: String,
    pub reduction: f64,
    /// Joint position when the actuator coordinate is zero.
    pub offset: f64,
}

impl SimpleTransmission {
    pub fn resolve(&self, coordinate: CoordinateState) -> JointState {
        JointState {
            position: coordinate.position / self.reduction + self.offset,
            velocity: coordinate.velocity / self.reduction,
        }
    }

    pub fn joint_torque(&self, motor_torque: f64) -> f64 {
        self.reduction * motor_torque
    }
}

pub(super) fn parse(
    robot: Node<'_, '_>,
    joints: &BTreeMap<String, Joint>,
    differentials: &[DifferentialTransmission],
) -> anyhow::Result<Vec<SimpleTransmission>> {
    let differential_joints: BTreeSet<_> = differentials
        .iter()
        .flat_map(|t| t.joints.iter().map(String::as_str))
        .collect();
    let mut coordinates: BTreeSet<_> = differentials
        .iter()
        .flat_map(|t| t.coordinates.iter().cloned())
        .collect();
    let mut names = BTreeSet::new();
    let mut result = Vec::new();
    for node in robot
        .children()
        .filter(|node| node.has_tag_name("transmission"))
    {
        let name = required_attribute(node, "name")?;
        if !names.insert(name) {
            bail!("duplicate transmission name {name:?}");
        }
        let kind = transmission_type(node);
        if kind.ends_with("DifferentialTransmission") {
            continue;
        }
        if !kind.ends_with("SimpleTransmission") {
            bail!("transmission {name:?} has unsupported type {kind:?}");
        }
        let member = |tag| -> anyhow::Result<Node<'_, '_>> {
            let nodes: Vec<_> = node.children().filter(|n| n.has_tag_name(tag)).collect();
            if nodes.len() != 1 {
                bail!("simple transmission {name:?} must have exactly one {tag}");
            }
            Ok(nodes[0])
        };
        let actuator = member("actuator")?;
        let joint_node = member("joint")?;
        let coordinate = required_attribute(actuator, "name")?.to_owned();
        let joint = required_attribute(joint_node, "name")?.to_owned();
        let resolved = joints
            .get(&joint)
            .with_context(|| format!("transmission {name:?} references unknown joint {joint:?}"))?;
        if resolved.kind == JointKind::Fixed {
            bail!("transmission {name:?} references fixed joint {joint:?}");
        }
        if differential_joints.contains(joint.as_str()) {
            bail!("joint {joint:?} mixes simple and differential transmissions");
        }
        if !coordinates.insert(coordinate.clone()) {
            bail!("coordinate {coordinate:?} belongs to multiple transmissions");
        }
        if joints.contains_key(&coordinate) && coordinate != joint {
            bail!("coordinate {coordinate:?} shadows a different URDF joint");
        }
        let reduction = child_number(actuator, &["mechanicalReduction", "mechanical_reduction"])?
            .unwrap_or(1.0);
        let offset = child_number(joint_node, &["offset"])?.unwrap_or(0.0);
        if !reduction.is_finite() || reduction.abs() <= 1e-12 || !offset.is_finite() {
            bail!("simple transmission {name:?} has invalid reduction or offset");
        }
        result.push(SimpleTransmission {
            name: name.to_owned(),
            coordinate,
            joint,
            reduction,
            offset,
        });
    }
    Ok(result)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::RobotDescription;

    fn load(transmissions: &str) -> anyhow::Result<RobotDescription> {
        let file = tempfile::NamedTempFile::new().unwrap();
        std::fs::write(file.path(), format!(r#"<robot name="parallel"><link name="base"/><link name="torso"><collision><geometry><sphere radius="0.01"/></geometry></collision></link><joint name="pitch" type="revolute"><parent link="base"/><child link="torso"/><limit lower="-0.5" upper="1" effort="240" velocity="2"/></joint>{transmissions}</robot>"#)).unwrap();
        RobotDescription::load(file.path())
    }

    fn transmission(name: &str, motor: &str, reduction: f64) -> String {
        format!(
            r#"<transmission name="{name}"><type>transmission_interface/SimpleTransmission</type><joint name="pitch"><offset>0.1</offset></joint><actuator name="{motor}"><mechanicalReduction>{reduction}</mechanicalReduction></actuator></transmission>"#
        )
    }

    #[test]
    fn parallel_reductions_offsets_and_efforts_preserve_virtual_work() {
        let robot =
            load(&(transmission("left", "a", -2.) + &transmission("right", "b", 4.))).unwrap();
        assert_eq!(robot.motor_coordinates(), BTreeSet::from(["a", "b"]));
        assert_eq!(robot.coordinate_position_limits("a"), Some((-1.8, 1.2)));
        assert_eq!(robot.coordinate_velocity_limit("a"), Some(4.));
        assert_eq!(robot.coordinate_effort_limit("a"), Some(60.));
        let a = &robot.simple_transmissions()[0];
        let b = &robot.simple_transmissions()[1];
        let state = a.resolve(CoordinateState {
            position: -0.6,
            velocity: -1.,
        });
        assert!((state.position - 0.4).abs() < 1e-12);
        assert_eq!(state.velocity, 0.5);
        assert_eq!(a.joint_torque(-3.) + b.joint_torque(5.), 26.);
        assert_eq!((-3. * -1.) + (5. * 2.), 26. * state.velocity);
    }

    #[test]
    fn rejects_duplicate_motor_and_invalid_reduction() {
        assert!(
            load(&(transmission("left", "a", -1.) + &transmission("right", "a", -1.))).is_err()
        );
        assert!(load(&transmission("left", "a", 0.)).is_err());
        assert!(load(&transmission("left", "other_joint", f64::NAN)).is_err());
    }

    #[test]
    fn transmitted_joint_cannot_bypass_its_declared_motor_coordinates() {
        let robot =
            load(&(transmission("left", "a", -1.) + &transmission("right", "b", -1.))).unwrap();
        assert!(robot.coordinate_mapping("pitch").is_none());
        assert!(robot.coordinate_position_limits("pitch").is_none());
        assert!(robot.coordinate_velocity_limit("pitch").is_none());
        assert!(robot.coordinate_effort_limit("pitch").is_none());
        assert!(robot.coordinate_mapping("a").is_some());
        assert!(robot.coordinate_mapping("b").is_some());
    }

    #[test]
    fn kinematics_uses_the_same_least_squares_reconstruction_as_motor_mappings() {
        let robot =
            load(&(transmission("left", "a", -2.) + &transmission("right", "b", 4.))).unwrap();
        let model = robot.kinematic_model("base", "torso", "torso").unwrap();
        assert_eq!(model.required_coordinates(), &["a", "b"]);
        let state = BTreeMap::from([
            (
                "a".into(),
                CoordinateState {
                    position: -0.6,
                    velocity: -1.0,
                },
            ),
            (
                "b".into(),
                CoordinateState {
                    position: 2.0,
                    velocity: 2.8,
                },
            ),
        ]);
        let feet = model.feet(&state).unwrap();
        let q = 0.56_f64;
        let v = 0.66;
        assert!((feet[0].position.y - 0.01 * q.sin()).abs() < 1e-12);
        assert!((feet[0].position.z + 0.01 * q.cos()).abs() < 1e-12);
        assert!((feet[0].relative_velocity.y - 0.01 * q.cos() * v).abs() < 1e-12);
        assert!((feet[0].relative_velocity.z - 0.01 * q.sin() * v).abs() < 1e-12);
    }
}
