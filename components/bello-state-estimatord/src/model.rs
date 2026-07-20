use std::collections::{BTreeMap, BTreeSet};
use std::fs;
use std::path::Path;

use anyhow::{Context, bail};
use nalgebra::{Isometry3, Translation3, Unit, UnitQuaternion, Vector3};
use roxmltree::{Document, Node};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum JointKind {
    Fixed,
    Revolute,
    Prismatic,
}

#[derive(Clone, Debug)]
struct Joint {
    name: String,
    parent: String,
    child: String,
    kind: JointKind,
    origin: Isometry3<f64>,
    axis: Unit<Vector3<f64>>,
}

#[derive(Clone, Debug)]
struct DifferentialTransmission {
    coordinates: [String; 2],
    coordinate_reduction: [f64; 2],
    joints: [String; 2],
    joint_reduction: [f64; 2],
    joint_offset: [f64; 2],
}

impl DifferentialTransmission {
    fn resolve(
        &self,
        coordinates: &BTreeMap<String, CoordinateState>,
    ) -> anyhow::Result<[(String, JointState); 2]> {
        let a1 = coordinates
            .get(&self.coordinates[0])
            .with_context(|| format!("missing coordinate {:?}", self.coordinates[0]))?;
        let a2 = coordinates
            .get(&self.coordinates[1])
            .with_context(|| format!("missing coordinate {:?}", self.coordinates[1]))?;
        let map = |a1: f64, a2: f64, joint: usize| {
            let sum = if joint == 0 {
                a1 / self.coordinate_reduction[0] + a2 / self.coordinate_reduction[1]
            } else {
                a1 / self.coordinate_reduction[0] - a2 / self.coordinate_reduction[1]
            };
            sum / (2.0 * self.joint_reduction[joint])
        };
        Ok([
            (
                self.joints[0].clone(),
                JointState {
                    position: map(a1.position, a2.position, 0) + self.joint_offset[0],
                    velocity: map(a1.velocity, a2.velocity, 0),
                },
            ),
            (
                self.joints[1].clone(),
                JointState {
                    position: map(a1.position, a2.position, 1) + self.joint_offset[1],
                    velocity: map(a1.velocity, a2.velocity, 1),
                },
            ),
        ])
    }
}

#[derive(Clone, Copy, Debug)]
struct JointState {
    position: f64,
    velocity: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct CoordinateState {
    pub position: f64,
    pub velocity: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct FootKinematics {
    /// Contact point relative to the pelvis, expressed in pelvis coordinates.
    pub position: Vector3<f64>,
    /// Contact-point velocity with the pelvis held fixed, in pelvis coordinates.
    pub relative_velocity: Vector3<f64>,
}

#[derive(Clone, Debug)]
struct FootPath {
    joints: Vec<Joint>,
    support_in_foot: Vector3<f64>,
}

#[derive(Clone, Debug)]
pub struct KinematicModel {
    feet: [FootPath; 2],
    transmissions: Vec<DifferentialTransmission>,
    transmitted_joints: BTreeSet<String>,
    required_coordinates: Vec<String>,
}

impl KinematicModel {
    pub fn load(
        path: &Path,
        base: &str,
        left_foot: &str,
        right_foot: &str,
    ) -> anyhow::Result<Self> {
        let xml =
            fs::read_to_string(path).with_context(|| format!("read URDF {}", path.display()))?;
        let document =
            Document::parse(&xml).with_context(|| format!("parse URDF {}", path.display()))?;
        let robot = document.root_element();
        if robot.tag_name().name() != "robot" {
            bail!("{} is not a URDF robot", path.display());
        }
        let joints = parse_joints(robot)?;
        let transmissions = parse_transmissions(robot)?;
        let transmitted_joints = transmissions
            .iter()
            .flat_map(|transmission| transmission.joints.iter().cloned())
            .collect::<BTreeSet<_>>();
        let left_path = path_to(&joints, base, left_foot)?;
        let right_path = path_to(&joints, base, right_foot)?;
        let feet = [
            FootPath {
                support_in_foot: support_point(robot, left_foot, &left_path)?,
                joints: left_path,
            },
            FootPath {
                support_in_foot: support_point(robot, right_foot, &right_path)?,
                joints: right_path,
            },
        ];

        let used_joints = feet
            .iter()
            .flat_map(|foot| foot.joints.iter())
            .filter(|joint| joint.kind != JointKind::Fixed)
            .map(|joint| joint.name.clone())
            .collect::<BTreeSet<_>>();
        for joint in &transmitted_joints {
            if !used_joints.contains(joint) {
                bail!("differential transmission references unused joint {joint:?}");
            }
        }
        let mut required = used_joints
            .difference(&transmitted_joints)
            .cloned()
            .collect::<BTreeSet<_>>();
        for transmission in &transmissions {
            required.extend(transmission.coordinates.iter().cloned());
        }
        Ok(Self {
            feet,
            transmissions,
            transmitted_joints,
            required_coordinates: required.into_iter().collect(),
        })
    }

    pub fn required_coordinates(&self) -> &[String] {
        &self.required_coordinates
    }

    pub fn feet(
        &self,
        coordinates: &BTreeMap<String, CoordinateState>,
    ) -> anyhow::Result<[FootKinematics; 2]> {
        for (name, value) in coordinates {
            if !value.position.is_finite() || !value.velocity.is_finite() {
                bail!("coordinate {name:?} is non-finite");
            }
        }
        let mut joint_states = BTreeMap::new();
        for transmission in &self.transmissions {
            for (joint, value) in transmission.resolve(coordinates)? {
                if joint_states.insert(joint.clone(), value).is_some() {
                    bail!("joint {joint:?} is provided by more than one transmission");
                }
            }
        }
        for foot in &self.feet {
            for joint in &foot.joints {
                if joint.kind == JointKind::Fixed || self.transmitted_joints.contains(&joint.name) {
                    continue;
                }
                let coordinate = coordinates
                    .get(&joint.name)
                    .with_context(|| format!("missing direct coordinate {:?}", joint.name))?;
                joint_states.insert(
                    joint.name.clone(),
                    JointState {
                        position: coordinate.position,
                        velocity: coordinate.velocity,
                    },
                );
            }
        }
        Ok([
            evaluate_foot(&self.feet[0], &joint_states)?,
            evaluate_foot(&self.feet[1], &joint_states)?,
        ])
    }
}

fn evaluate_foot(
    foot: &FootPath,
    states: &BTreeMap<String, JointState>,
) -> anyhow::Result<FootKinematics> {
    let mut transform = Isometry3::identity();
    let mut columns = Vec::new();
    for joint in &foot.joints {
        transform *= joint.origin;
        let origin = transform.translation.vector;
        let axis = transform.rotation * joint.axis.as_ref();
        match joint.kind {
            JointKind::Fixed => {}
            JointKind::Revolute => {
                let state = states
                    .get(&joint.name)
                    .with_context(|| format!("missing state for joint {:?}", joint.name))?;
                columns.push((joint.kind, origin, axis, state.velocity));
                transform *= Isometry3::from_parts(
                    Translation3::identity(),
                    UnitQuaternion::from_axis_angle(&joint.axis, state.position),
                );
            }
            JointKind::Prismatic => {
                let state = states
                    .get(&joint.name)
                    .with_context(|| format!("missing state for joint {:?}", joint.name))?;
                columns.push((joint.kind, origin, axis, state.velocity));
                transform *= Isometry3::translation(
                    joint.axis.x * state.position,
                    joint.axis.y * state.position,
                    joint.axis.z * state.position,
                );
            }
        }
    }
    let position = transform
        .transform_point(&foot.support_in_foot.into())
        .coords;
    let mut relative_velocity = Vector3::zeros();
    for (kind, origin, axis, velocity) in columns {
        let column = match kind {
            JointKind::Revolute => axis.cross(&(position - origin)),
            JointKind::Prismatic => axis,
            JointKind::Fixed => unreachable!(),
        };
        relative_velocity += column * velocity;
    }
    Ok(FootKinematics {
        position,
        relative_velocity,
    })
}

fn parse_joints(robot: Node<'_, '_>) -> anyhow::Result<BTreeMap<String, Joint>> {
    let mut joints = BTreeMap::new();
    for node in robot.children().filter(|node| node.has_tag_name("joint")) {
        let name = required_attribute(node, "name")?.to_owned();
        let kind = match required_attribute(node, "type")? {
            "fixed" => JointKind::Fixed,
            "revolute" | "continuous" => JointKind::Revolute,
            "prismatic" => JointKind::Prismatic,
            other => bail!("joint {name:?} has unsupported URDF type {other:?}"),
        };
        let parent = required_attribute(required_child(node, "parent")?, "link")?.to_owned();
        let child = required_attribute(required_child(node, "child")?, "link")?.to_owned();
        let origin = node
            .children()
            .find(|child| child.has_tag_name("origin"))
            .map(parse_origin)
            .transpose()?
            .unwrap_or_else(Isometry3::identity);
        let axis = node
            .children()
            .find(|child| child.has_tag_name("axis"))
            .map(|axis| parse_vector(axis.attribute("xyz").unwrap_or("1 0 0")))
            .transpose()?
            .unwrap_or_else(Vector3::x);
        let axis = Unit::try_new(axis, 1.0e-12)
            .with_context(|| format!("joint {name:?} has a zero axis"))?;
        let joint = Joint {
            name: name.clone(),
            parent,
            child,
            kind,
            origin,
            axis,
        };
        if joints.insert(name.clone(), joint).is_some() {
            bail!("duplicate URDF joint {name:?}");
        }
    }
    Ok(joints)
}

fn path_to(
    joints: &BTreeMap<String, Joint>,
    base: &str,
    target: &str,
) -> anyhow::Result<Vec<Joint>> {
    let by_child = joints
        .values()
        .map(|joint| (joint.child.as_str(), joint))
        .collect::<BTreeMap<_, _>>();
    let mut reversed = Vec::new();
    let mut link = target;
    let mut visited = BTreeSet::new();
    while link != base {
        if !visited.insert(link.to_owned()) {
            bail!("cycle while resolving URDF path from {base:?} to {target:?}");
        }
        let joint = by_child
            .get(link)
            .with_context(|| format!("no URDF path from {base:?} to link {target:?}"))?;
        reversed.push((*joint).clone());
        link = &joint.parent;
    }
    reversed.reverse();
    Ok(reversed)
}

fn parse_transmissions(robot: Node<'_, '_>) -> anyhow::Result<Vec<DifferentialTransmission>> {
    let mut transmissions = Vec::new();
    let mut used_coordinates = BTreeSet::new();
    let mut used_joints = BTreeSet::new();
    for node in robot
        .descendants()
        .filter(|node| node.has_tag_name("transmission"))
    {
        let kind = node
            .children()
            .find(|child| child.has_tag_name("type") || child.has_tag_name("plugin"))
            .and_then(|child| child.text())
            .unwrap_or_default();
        if !kind.ends_with("DifferentialTransmission") {
            continue;
        }
        let name = node.attribute("name").unwrap_or("unnamed");
        let actuators = ordered_members(node, "actuator", "actuator", name)?;
        let joints = ordered_members(node, "joint", "joint", name)?;
        let coordinates = [actuators[0].0.clone(), actuators[1].0.clone()];
        let joint_names = [joints[0].0.clone(), joints[1].0.clone()];
        for coordinate in &coordinates {
            if !used_coordinates.insert(coordinate.clone()) {
                bail!("coordinate {coordinate:?} belongs to multiple transmissions");
            }
        }
        for joint in &joint_names {
            if !used_joints.insert(joint.clone()) {
                bail!("joint {joint:?} belongs to multiple transmissions");
            }
        }
        transmissions.push(DifferentialTransmission {
            coordinates,
            coordinate_reduction: [actuators[0].1, actuators[1].1],
            joints: joint_names,
            joint_reduction: [joints[0].1, joints[1].1],
            joint_offset: [joints[0].2, joints[1].2],
        });
    }
    Ok(transmissions)
}

fn ordered_members(
    transmission: Node<'_, '_>,
    tag: &str,
    role_prefix: &str,
    transmission_name: &str,
) -> anyhow::Result<Vec<(String, f64, f64)>> {
    let mut members = transmission
        .children()
        .filter(|child| child.has_tag_name(tag))
        .map(|node| {
            let name = required_attribute(node, "name")?.to_owned();
            let role = node.attribute("role").unwrap_or_default();
            let order = role
                .strip_prefix(role_prefix)
                .and_then(|suffix| suffix.parse::<usize>().ok())
                .unwrap_or(usize::MAX);
            let reduction = child_number(node, &["mechanical_reduction", "mechanicalReduction"])?
                .unwrap_or(1.0);
            let offset = child_number(node, &["offset"])?.unwrap_or(0.0);
            if !reduction.is_finite() || reduction.abs() <= 1.0e-12 {
                bail!("transmission {transmission_name:?} member {name:?} has invalid reduction");
            }
            Ok((order, name, reduction, offset))
        })
        .collect::<anyhow::Result<Vec<_>>>()?;
    if members.len() != 2 {
        bail!("differential transmission {transmission_name:?} must have exactly two {tag}s");
    }
    if members.iter().all(|member| member.0 != usize::MAX) {
        members.sort_by_key(|member| member.0);
        if members[0].0 != 1 || members[1].0 != 2 {
            bail!(
                "transmission {transmission_name:?} {tag} roles must be {role_prefix}1 and {role_prefix}2"
            );
        }
    }
    Ok(members
        .into_iter()
        .map(|(_, name, reduction, offset)| (name, reduction, offset))
        .collect())
}

fn support_point(
    robot: Node<'_, '_>,
    link_name: &str,
    path: &[Joint],
) -> anyhow::Result<Vector3<f64>> {
    let link = robot
        .children()
        .find(|node| node.has_tag_name("link") && node.attribute("name") == Some(link_name))
        .with_context(|| format!("URDF has no link {link_name:?}"))?;
    let mut candidates = Vec::new();
    for collision in link
        .children()
        .filter(|node| node.has_tag_name("collision"))
    {
        let origin = collision
            .children()
            .find(|child| child.has_tag_name("origin"))
            .map(parse_origin)
            .transpose()?
            .unwrap_or_else(Isometry3::identity);
        let Some(geometry) = collision
            .children()
            .find(|child| child.has_tag_name("geometry"))
        else {
            continue;
        };
        if let Some(shape) = geometry.children().find(|child| child.has_tag_name("box")) {
            let size = parse_vector(required_attribute(shape, "size")?)? / 2.0;
            for x in [-size.x, size.x] {
                for y in [-size.y, size.y] {
                    for z in [-size.z, size.z] {
                        candidates.push(
                            origin
                                .transform_point(&nalgebra::Point3::new(x, y, z))
                                .coords,
                        );
                    }
                }
            }
        } else if let Some(shape) = geometry
            .children()
            .find(|child| child.has_tag_name("cylinder"))
        {
            let radius = required_attribute(shape, "radius")?.parse::<f64>()?;
            let half = required_attribute(shape, "length")?.parse::<f64>()? / 2.0;
            for z in [-half, half] {
                for index in 0..16 {
                    let angle = std::f64::consts::TAU * index as f64 / 16.0;
                    candidates.push(
                        origin
                            .transform_point(&nalgebra::Point3::new(
                                radius * angle.cos(),
                                radius * angle.sin(),
                                z,
                            ))
                            .coords,
                    );
                }
            }
        }
    }
    if candidates.is_empty() {
        bail!("foot link {link_name:?} has no box or cylinder collision geometry");
    }
    let zero_transform = path.iter().fold(Isometry3::identity(), |transform, joint| {
        transform * joint.origin
    });
    let minimum = candidates
        .iter()
        .map(|point| zero_transform.transform_point(&(*point).into()).z)
        .fold(f64::INFINITY, f64::min);
    let lowest = candidates
        .iter()
        .filter(|point| {
            (zero_transform.transform_point(&(**point).into()).z - minimum).abs() <= 1.0e-7
        })
        .copied()
        .collect::<Vec<_>>();
    Ok(lowest.iter().copied().sum::<Vector3<f64>>() / lowest.len() as f64)
}

fn parse_origin(node: Node<'_, '_>) -> anyhow::Result<Isometry3<f64>> {
    let xyz = parse_vector(node.attribute("xyz").unwrap_or("0 0 0"))?;
    let rpy = parse_vector(node.attribute("rpy").unwrap_or("0 0 0"))?;
    Ok(Isometry3::from_parts(
        Translation3::from(xyz),
        UnitQuaternion::from_euler_angles(rpy.x, rpy.y, rpy.z),
    ))
}

fn parse_vector(text: &str) -> anyhow::Result<Vector3<f64>> {
    let values = text
        .split_whitespace()
        .map(str::parse::<f64>)
        .collect::<Result<Vec<_>, _>>()?;
    if values.len() != 3 || values.iter().any(|value| !value.is_finite()) {
        bail!("expected three finite numbers, got {text:?}");
    }
    Ok(Vector3::new(values[0], values[1], values[2]))
}

fn child_number(node: Node<'_, '_>, names: &[&str]) -> anyhow::Result<Option<f64>> {
    node.children()
        .find(|child| names.iter().any(|name| child.has_tag_name(*name)))
        .and_then(|child| child.text())
        .map(|text| {
            let value = text.trim().parse::<f64>()?;
            if !value.is_finite() {
                bail!("expected finite transmission number, got {text:?}");
            }
            Ok(value)
        })
        .transpose()
}

fn required_child<'a>(node: Node<'a, 'a>, name: &str) -> anyhow::Result<Node<'a, 'a>> {
    node.children()
        .find(|child| child.has_tag_name(name))
        .with_context(|| format!("{} is missing child {name:?}", node.tag_name().name()))
}

fn required_attribute<'a>(node: Node<'a, '_>, name: &str) -> anyhow::Result<&'a str> {
    node.attribute(name)
        .with_context(|| format!("{} is missing attribute {name:?}", node.tag_name().name()))
}

#[cfg(test)]
mod tests {
    use std::io::Write;

    use super::*;

    const SIMPLE_URDF: &str = r#"
<robot name="test">
  <link name="pelvis"/>
  <link name="left_foot">
    <collision><geometry><box size="0.2 0.2 0.2"/></geometry></collision>
  </link>
  <link name="right_foot">
    <collision><geometry><box size="0.2 0.2 0.2"/></geometry></collision>
  </link>
  <joint name="left_joint" type="revolute">
    <origin xyz="0 0 -1"/>
    <parent link="pelvis"/><child link="left_foot"/><axis xyz="0 1 0"/>
  </joint>
  <joint name="right_fixed" type="fixed">
    <origin xyz="0.2 0 -1"/>
    <parent link="pelvis"/><child link="right_foot"/>
  </joint>
</robot>
"#;

    fn load_text(text: &str) -> anyhow::Result<KinematicModel> {
        let mut file = tempfile::NamedTempFile::new()?;
        file.write_all(text.as_bytes())?;
        KinematicModel::load(file.path(), "pelvis", "left_foot", "right_foot")
    }

    #[test]
    fn differential_matches_bello_ankle_equations() {
        let transmission = DifferentialTransmission {
            coordinates: ["motor_1".into(), "motor_2".into()],
            coordinate_reduction: [1.0, 1.0],
            joints: ["roll".into(), "pitch".into()],
            joint_reduction: [1.0, -1.0],
            joint_offset: [0.0, 0.0],
        };
        let values = BTreeMap::from([
            (
                "motor_1".into(),
                CoordinateState {
                    position: -0.5,
                    velocity: -0.3,
                },
            ),
            (
                "motor_2".into(),
                CoordinateState {
                    position: 0.9,
                    velocity: 0.7,
                },
            ),
        ]);
        let resolved = transmission.resolve(&values).unwrap();
        assert_eq!(resolved[0].0, "roll");
        assert!((resolved[0].1.position - 0.2).abs() < 1.0e-12);
        assert!((resolved[1].1.position - 0.7).abs() < 1.0e-12);
        assert!((resolved[0].1.velocity - 0.2).abs() < 1.0e-12);
        assert!((resolved[1].1.velocity - 0.5).abs() < 1.0e-12);
    }

    #[test]
    fn urdf_hierarchy_and_contact_velocity_are_evaluated() {
        let model = load_text(SIMPLE_URDF).unwrap();
        assert_eq!(model.required_coordinates(), &["left_joint"]);
        let feet = model
            .feet(&BTreeMap::from([(
                "left_joint".into(),
                CoordinateState {
                    position: 0.0,
                    velocity: 2.0,
                },
            )]))
            .unwrap();
        assert!((feet[0].position - Vector3::new(0.0, 0.0, -1.1)).norm() < 1.0e-12);
        assert!((feet[0].relative_velocity - Vector3::new(-0.2, 0.0, 0.0)).norm() < 1.0e-12);
        assert!((feet[1].position - Vector3::new(0.2, 0.0, -1.1)).norm() < 1.0e-12);
        assert!(feet[1].relative_velocity.norm() < 1.0e-12);
    }

    #[test]
    fn incomplete_differential_is_rejected() {
        let text = SIMPLE_URDF.replace(
            "</robot>",
            r#"
  <transmission name="broken">
    <type>transmission_interface/DifferentialTransmission</type>
    <actuator name="motor_1" role="actuator1"/>
    <joint name="left_joint" role="joint1"/>
    <joint name="other_joint" role="joint2"/>
  </transmission>
</robot>"#,
        );
        let error = load_text(&text).unwrap_err().to_string();
        assert!(error.contains("exactly two actuators"), "{error}");
    }

    #[test]
    fn canonical_bello_urdf_loads_with_ankle_coordinates() {
        let path = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../../bello_mujoco/assets/source/bello/full_body/urdf/bello_full_body.urdf");
        if !path.is_file() {
            return;
        }
        let model = KinematicModel::load(
            &path,
            "pelvis_link",
            "left_ankle_roll_link",
            "right_ankle_roll_link",
        )
        .unwrap();
        for coordinate in [
            "left_ankle_motor_1",
            "left_ankle_motor_2",
            "right_ankle_motor_1",
            "right_ankle_motor_2",
        ] {
            assert!(
                model
                    .required_coordinates()
                    .iter()
                    .any(|name| name == coordinate)
            );
        }
        assert!(
            !model
                .required_coordinates()
                .iter()
                .any(|name| name == "left_ankle_pitch_joint")
        );
    }
}
