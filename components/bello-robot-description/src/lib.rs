use std::collections::{BTreeMap, BTreeSet};
use std::fs;
use std::path::{Path, PathBuf};

use anyhow::{Context, bail};
use nalgebra::{Isometry3, Translation3, Unit, UnitQuaternion, Vector3};
use roxmltree::{Document, Node};

mod simple_transmission;
pub use simple_transmission::{CoordinateMapping, SimpleTransmission};
mod slider_crank;
pub use slider_crank::SliderCrank;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JointKind {
    Fixed,
    Revolute,
    Prismatic,
}

#[derive(Clone, Debug)]
pub struct JointLimits {
    pub lower: f64,
    pub upper: f64,
    pub effort: f64,
    pub velocity: f64,
}

#[derive(Clone, Debug)]
pub struct Joint {
    pub name: String,
    pub parent: String,
    pub child: String,
    pub kind: JointKind,
    pub origin: Isometry3<f64>,
    pub axis: Unit<Vector3<f64>>,
    pub limits: Option<JointLimits>,
}

#[derive(Clone, Debug)]
pub enum Geometry {
    Mesh { path: PathBuf, scale: Vector3<f64> },
    Box { size: Vector3<f64> },
    Cylinder { radius: f64, length: f64 },
    Sphere { radius: f64 },
}

#[derive(Clone, Debug)]
pub struct GeometryInstance {
    pub origin: Isometry3<f64>,
    pub geometry: Geometry,
    pub rgba: [f64; 4],
}

#[derive(Clone, Debug)]
pub struct Link {
    pub name: String,
    pub visuals: Vec<GeometryInstance>,
    pub collisions: Vec<GeometryInstance>,
}

#[derive(Clone, Debug)]
pub struct DifferentialTransmission {
    pub name: String,
    pub coordinates: [String; 2],
    pub coordinate_reduction: [f64; 2],
    pub joints: [String; 2],
    pub joint_reduction: [f64; 2],
    pub joint_offset: [f64; 2],
}

impl DifferentialTransmission {
    pub fn resolve(
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

    pub fn resolve_torques(
        &self,
        coordinates: &BTreeMap<String, f64>,
    ) -> anyhow::Result<[(String, f64); 2]> {
        let a1 = coordinates
            .get(&self.coordinates[0])
            .with_context(|| format!("missing coordinate {:?}", self.coordinates[0]))?;
        let a2 = coordinates
            .get(&self.coordinates[1])
            .with_context(|| format!("missing coordinate {:?}", self.coordinates[1]))?;
        Ok([
            (
                self.joints[0].clone(),
                self.joint_reduction[0]
                    * (self.coordinate_reduction[0] * a1 + self.coordinate_reduction[1] * a2),
            ),
            (
                self.joints[1].clone(),
                self.joint_reduction[1]
                    * (self.coordinate_reduction[0] * a1 - self.coordinate_reduction[1] * a2),
            ),
        ])
    }
}

#[derive(Clone, Copy, Debug)]
pub struct JointState {
    pub position: f64,
    pub velocity: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct CoordinateState {
    pub position: f64,
    pub velocity: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct FootKinematics {
    /// Contact point relative to the kinematic base, in base coordinates.
    pub position: Vector3<f64>,
    /// Contact-point velocity with the kinematic base fixed, in base coordinates.
    pub relative_velocity: Vector3<f64>,
}

#[derive(Clone, Debug)]
struct FootPath {
    joints: Vec<Joint>,
    support_in_foot: Vector3<f64>,
}

#[derive(Clone, Debug)]
pub struct RobotDescription {
    name: String,
    root_link: String,
    links: BTreeMap<String, Link>,
    joints: BTreeMap<String, Joint>,
    transmissions: Vec<DifferentialTransmission>,
    simple_transmissions: Vec<SimpleTransmission>,
    slider_cranks: Vec<SliderCrank>,
}

impl RobotDescription {
    pub fn load(path: &Path) -> anyhow::Result<Self> {
        let xml =
            fs::read_to_string(path).with_context(|| format!("read URDF {}", path.display()))?;
        let document =
            Document::parse(&xml).with_context(|| format!("parse URDF {}", path.display()))?;
        let robot = document.root_element();
        if robot.tag_name().name() != "robot" {
            bail!("{} is not a URDF robot", path.display());
        }
        let name = required_attribute(robot, "name")?.to_owned();
        let materials = parse_materials(robot)?;
        let links = parse_links(robot, path, &materials)?;
        let joints = parse_joints(robot)?;
        let transmissions = parse_transmissions(robot)?;
        validate_description(&links, &joints, &transmissions)?;
        let simple_transmissions = simple_transmission::parse(robot, &joints, &transmissions)?;
        let transmitted = transmissions
            .iter()
            .flat_map(|t| t.joints.iter().map(String::as_str))
            .chain(simple_transmissions.iter().map(|t| t.joint.as_str()))
            .collect();
        let slider_cranks = slider_crank::parse(robot, &joints, &transmitted)?;

        let child_links = joints
            .values()
            .map(|joint| joint.child.as_str())
            .collect::<BTreeSet<_>>();
        let roots = links
            .keys()
            .filter(|link| !child_links.contains(link.as_str()))
            .cloned()
            .collect::<Vec<_>>();
        if roots.len() != 1 {
            bail!("URDF must have exactly one root link, found {roots:?}");
        }
        Ok(Self {
            name,
            root_link: roots[0].clone(),
            links,
            joints,
            transmissions,
            simple_transmissions,
            slider_cranks,
        })
    }

    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn root_link(&self) -> &str {
        &self.root_link
    }

    pub fn links(&self) -> impl Iterator<Item = &Link> {
        self.links.values()
    }

    pub fn link(&self, name: &str) -> Option<&Link> {
        self.links.get(name)
    }

    pub fn joints(&self) -> impl Iterator<Item = &Joint> {
        self.joints.values()
    }

    pub fn joint(&self, name: &str) -> Option<&Joint> {
        self.joints.get(name)
    }

    pub fn slider_cranks(&self) -> &[SliderCrank] {
        &self.slider_cranks
    }

    pub fn is_passive_joint(&self, name: &str) -> bool {
        self.slider_cranks
            .iter()
            .any(|linkage| linkage.passive_joints().any(|joint| joint == name))
    }

    /// Expand measured model-joint positions with the closed linkage's passive
    /// joints. Missing drivers are left untouched so callers can hold their pose.
    pub fn resolve_passive_positions(
        &self,
        positions: &BTreeMap<String, f64>,
    ) -> anyhow::Result<BTreeMap<String, f64>> {
        let mut resolved = positions.clone();
        for linkage in &self.slider_cranks {
            if let Some(position) = positions.get(&linkage.driver) {
                for (name, state) in linkage.resolve(CoordinateState {
                    position: *position,
                    velocity: 0.0,
                })? {
                    resolved.insert(name, state.position);
                }
            }
        }
        Ok(resolved)
    }

    /// Forward kinematics using model-joint positions, including passive gripper
    /// closure. Motor transmission reconstruction is performed before this call.
    pub fn link_transform(
        &self,
        base: &str,
        target: &str,
        positions: &BTreeMap<String, f64>,
    ) -> anyhow::Result<Isometry3<f64>> {
        if !self.links.contains_key(base) || !self.links.contains_key(target) {
            bail!("unknown link in transform {base:?} -> {target:?}");
        }
        let positions = self.resolve_passive_positions(positions)?;
        let mut transform = Isometry3::identity();
        for joint in path_to(&self.joints, base, target)? {
            transform *= joint.origin;
            if joint.kind == JointKind::Fixed {
                continue;
            }
            let position = *positions
                .get(&joint.name)
                .with_context(|| format!("missing position for joint {:?}", joint.name))?;
            if !position.is_finite() {
                bail!("non-finite position for joint {:?}", joint.name);
            }
            match joint.kind {
                JointKind::Revolute => {
                    transform *= Isometry3::from_parts(
                        Translation3::identity(),
                        UnitQuaternion::from_axis_angle(&joint.axis, position),
                    )
                }
                JointKind::Prismatic => {
                    transform *= Isometry3::translation(
                        joint.axis.x * position,
                        joint.axis.y * position,
                        joint.axis.z * position,
                    )
                }
                JointKind::Fixed => unreachable!(),
            }
        }
        Ok(transform)
    }

    /// Return `base_from_target` when `target` is rigidly attached below `base`.
    pub fn fixed_transform(&self, base: &str, target: &str) -> anyhow::Result<Isometry3<f64>> {
        if !self.links.contains_key(base) {
            bail!("URDF has no link {base:?}");
        }
        if !self.links.contains_key(target) {
            bail!("URDF has no link {target:?}");
        }
        let path = path_to(&self.joints, base, target)?;
        let mut transform = Isometry3::identity();
        for joint in path {
            if joint.kind != JointKind::Fixed {
                bail!(
                    "link {target:?} is not rigidly attached to {base:?}: joint {:?} is movable",
                    joint.name
                );
            }
            transform *= joint.origin;
        }
        Ok(transform)
    }

    pub fn transmissions(&self) -> &[DifferentialTransmission] {
        &self.transmissions
    }

    pub fn simple_transmissions(&self) -> &[SimpleTransmission] {
        &self.simple_transmissions
    }

    pub fn simple_transmission_for_coordinate(
        &self,
        coordinate: &str,
    ) -> Option<&SimpleTransmission> {
        self.simple_transmissions
            .iter()
            .find(|t| t.coordinate == coordinate)
    }

    /// Motor-space affine mapping `coordinate = sum(coefficients * joints) + offset`.
    pub fn coordinate_mapping(&self, coordinate: &str) -> Option<CoordinateMapping> {
        if let Some(t) = self.simple_transmission_for_coordinate(coordinate) {
            return Some(CoordinateMapping {
                coefficients: BTreeMap::from([(t.joint.clone(), t.reduction)]),
                offset: -t.reduction * t.offset,
            });
        }
        if let Some(t) = self.transmission_for_coordinate(coordinate) {
            let index = t.coordinates.iter().position(|c| c == coordinate)?;
            let sign = if index == 0 { 1.0 } else { -1.0 };
            let coefficients = [
                t.coordinate_reduction[index] * t.joint_reduction[0],
                sign * t.coordinate_reduction[index] * t.joint_reduction[1],
            ];
            return Some(CoordinateMapping {
                coefficients: t.joints.iter().cloned().zip(coefficients).collect(),
                offset: -coefficients[0] * t.joint_offset[0] - coefficients[1] * t.joint_offset[1],
            });
        }
        self.direct_coordinate_joint(coordinate)?;
        Some(CoordinateMapping {
            coefficients: BTreeMap::from([(coordinate.to_owned(), 1.0)]),
            offset: 0.0,
        })
    }

    /// Exact motor ownership declared by the description, including direct joints.
    pub fn motor_coordinates(&self) -> BTreeSet<&str> {
        let mut coordinates: BTreeSet<_> = self
            .joints()
            .filter(|j| j.kind != JointKind::Fixed)
            .filter(|j| !self.is_passive_joint(&j.name))
            .map(|j| j.name.as_str())
            .collect();
        for t in &self.transmissions {
            for joint in &t.joints {
                coordinates.remove(joint.as_str());
            }
        }
        for t in &self.simple_transmissions {
            coordinates.remove(t.joint.as_str());
        }
        coordinates.extend(
            self.transmissions
                .iter()
                .flat_map(|t| t.coordinates.iter().map(String::as_str)),
        );
        coordinates.extend(
            self.simple_transmissions
                .iter()
                .map(|t| t.coordinate.as_str()),
        );
        coordinates
    }

    fn direct_coordinate_joint(&self, coordinate: &str) -> Option<&Joint> {
        let joint = self
            .joint(coordinate)
            .filter(|j| j.kind != JointKind::Fixed && !self.is_passive_joint(&j.name))?;
        if self
            .simple_transmissions
            .iter()
            .any(|t| t.joint == coordinate)
            || self
                .transmissions
                .iter()
                .any(|t| t.joints.iter().any(|name| name == coordinate))
        {
            return None;
        }
        Some(joint)
    }

    pub fn transmission_for_coordinate(
        &self,
        coordinate: &str,
    ) -> Option<&DifferentialTransmission> {
        self.transmissions.iter().find(|transmission| {
            transmission
                .coordinates
                .iter()
                .any(|name| name == coordinate)
        })
    }

    pub fn coordinate_position_limits(&self, coordinate: &str) -> Option<(f64, f64)> {
        if let Some(t) = self.simple_transmission_for_coordinate(coordinate) {
            let limits = self.joint(&t.joint)?.limits.as_ref()?;
            let a = t.reduction * (limits.lower - t.offset);
            let b = t.reduction * (limits.upper - t.offset);
            return Some((a.min(b), a.max(b)));
        }
        if let Some(joint) = self.direct_coordinate_joint(coordinate) {
            let limits = joint.limits.as_ref()?;
            return Some((limits.lower, limits.upper));
        }
        let transmission = self.transmission_for_coordinate(coordinate)?;
        let coordinate_index = transmission
            .coordinates
            .iter()
            .position(|name| name == coordinate)?;
        let sign = if coordinate_index == 0 { 1.0 } else { -1.0 };
        let reduction = transmission.coordinate_reduction[coordinate_index];
        let coefficients = [
            reduction * transmission.joint_reduction[0],
            sign * reduction * transmission.joint_reduction[1],
        ];
        let offset = -coefficients[0] * transmission.joint_offset[0]
            - coefficients[1] * transmission.joint_offset[1];
        let mut minimum = offset;
        let mut maximum = offset;
        for (joint_name, coefficient) in transmission.joints.iter().zip(coefficients) {
            let limits = self.joint(joint_name)?.limits.as_ref()?;
            if coefficient >= 0.0 {
                minimum += coefficient * limits.lower;
                maximum += coefficient * limits.upper;
            } else {
                minimum += coefficient * limits.upper;
                maximum += coefficient * limits.lower;
            }
        }
        Some((minimum, maximum))
    }

    pub fn coordinate_velocity_limit(&self, coordinate: &str) -> Option<f64> {
        if let Some(t) = self.simple_transmission_for_coordinate(coordinate) {
            return Some(t.reduction.abs() * self.joint(&t.joint)?.limits.as_ref()?.velocity);
        }
        if let Some(joint) = self.direct_coordinate_joint(coordinate) {
            return Some(joint.limits.as_ref()?.velocity);
        }
        let transmission = self.transmission_for_coordinate(coordinate)?;
        let coordinate_index = transmission
            .coordinates
            .iter()
            .position(|name| name == coordinate)?;
        let reduction = transmission.coordinate_reduction[coordinate_index].abs();
        transmission
            .joints
            .iter()
            .zip(transmission.joint_reduction)
            .try_fold(0.0, |limit, (joint_name, joint_reduction)| {
                Some(
                    limit
                        + reduction
                            * joint_reduction.abs()
                            * self.joint(joint_name)?.limits.as_ref()?.velocity,
                )
            })
    }

    pub fn coordinate_effort_limit(&self, coordinate: &str) -> Option<f64> {
        if let Some(t) = self.simple_transmission_for_coordinate(coordinate) {
            let count = self
                .simple_transmissions
                .iter()
                .filter(|other| other.joint == t.joint)
                .count();
            return Some(
                self.joint(&t.joint)?.limits.as_ref()?.effort / (count as f64 * t.reduction.abs()),
            );
        }
        if let Some(joint) = self.direct_coordinate_joint(coordinate) {
            return Some(joint.limits.as_ref()?.effort);
        }
        let transmission = self.transmission_for_coordinate(coordinate)?;
        let coordinate_index = transmission
            .coordinates
            .iter()
            .position(|name| name == coordinate)?;
        let coordinate_reduction = transmission.coordinate_reduction[coordinate_index].abs();
        transmission
            .joints
            .iter()
            .zip(transmission.joint_reduction)
            .try_fold(0.0, |limit, (joint_name, joint_reduction)| {
                Some(
                    limit
                        + self.joint(joint_name)?.limits.as_ref()?.effort
                            / (2.0 * joint_reduction.abs() * coordinate_reduction),
                )
            })
    }

    pub fn kinematic_model(
        &self,
        base: &str,
        left_foot: &str,
        right_foot: &str,
    ) -> anyhow::Result<KinematicModel> {
        KinematicModel::from_description(self, base, left_foot, right_foot)
    }
}

#[derive(Clone, Debug)]
pub struct KinematicModel {
    feet: [FootPath; 2],
    transmissions: Vec<DifferentialTransmission>,
    simple_transmissions: Vec<SimpleTransmission>,
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
        RobotDescription::load(path)?.kinematic_model(base, left_foot, right_foot)
    }

    fn from_description(
        description: &RobotDescription,
        base: &str,
        left_foot: &str,
        right_foot: &str,
    ) -> anyhow::Result<Self> {
        let joints = &description.joints;
        let transmissions = description.transmissions.clone();
        let mut transmitted_joints = transmissions
            .iter()
            .flat_map(|transmission| transmission.joints.iter().cloned())
            .collect::<BTreeSet<_>>();
        let left_path = path_to(joints, base, left_foot)?;
        let right_path = path_to(joints, base, right_foot)?;
        let feet = [
            FootPath {
                support_in_foot: support_point(
                    description
                        .link(left_foot)
                        .with_context(|| format!("URDF has no link {left_foot:?}"))?,
                    &left_path,
                )?,
                joints: left_path,
            },
            FootPath {
                support_in_foot: support_point(
                    description
                        .link(right_foot)
                        .with_context(|| format!("URDF has no link {right_foot:?}"))?,
                    &right_path,
                )?,
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
        let simple_transmissions: Vec<_> = description
            .simple_transmissions
            .iter()
            .filter(|t| used_joints.contains(&t.joint))
            .cloned()
            .collect();
        transmitted_joints.extend(simple_transmissions.iter().map(|t| t.joint.clone()));
        let mut required = used_joints
            .difference(&transmitted_joints)
            .cloned()
            .collect::<BTreeSet<_>>();
        for transmission in &transmissions {
            required.extend(transmission.coordinates.iter().cloned());
        }
        required.extend(simple_transmissions.iter().map(|t| t.coordinate.clone()));
        Ok(Self {
            feet,
            transmissions,
            simple_transmissions,
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
        let mut simple_states: BTreeMap<String, (JointState, f64)> = BTreeMap::new();
        for t in &self.simple_transmissions {
            let state = t.resolve(
                *coordinates
                    .get(&t.coordinate)
                    .with_context(|| format!("missing coordinate {:?}", t.coordinate))?,
            );
            let (sum, count) = simple_states.entry(t.joint.clone()).or_insert((
                JointState {
                    position: 0.0,
                    velocity: 0.0,
                },
                0.0,
            ));
            let weight = t.reduction * t.reduction;
            sum.position += weight * state.position;
            sum.velocity += weight * state.velocity;
            *count += weight;
        }
        for (joint, (sum, count)) in simple_states {
            joint_states.insert(
                joint,
                JointState {
                    position: sum.position / count,
                    velocity: sum.velocity / count,
                },
            );
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
        let limits = node
            .children()
            .find(|child| child.has_tag_name("limit"))
            .map(|limit| parse_joint_limits(limit, &name))
            .transpose()?;
        if kind != JointKind::Fixed && limits.is_none() {
            bail!("joint {name:?} has no finite URDF limits");
        }
        if kind == JointKind::Fixed && limits.is_some() {
            bail!("fixed joint {name:?} must not declare limits");
        }
        let joint = Joint {
            name: name.clone(),
            parent,
            child,
            kind,
            origin,
            axis,
            limits,
        };
        if joints.insert(name.clone(), joint).is_some() {
            bail!("duplicate URDF joint {name:?}");
        }
    }
    Ok(joints)
}

fn parse_joint_limits(node: Node<'_, '_>, joint: &str) -> anyhow::Result<JointLimits> {
    let lower = finite_attribute(node, "lower")?;
    let upper = finite_attribute(node, "upper")?;
    let effort = finite_attribute(node, "effort")?;
    let velocity = finite_attribute(node, "velocity")?;
    if lower > upper || effort <= 0.0 || velocity <= 0.0 {
        bail!("joint {joint:?} has invalid URDF limits");
    }
    Ok(JointLimits {
        lower,
        upper,
        effort,
        velocity,
    })
}

fn parse_materials(robot: Node<'_, '_>) -> anyhow::Result<BTreeMap<String, [f64; 4]>> {
    let mut materials = BTreeMap::new();
    for material in robot
        .children()
        .filter(|child| child.has_tag_name("material"))
    {
        let name = required_attribute(material, "name")?.to_owned();
        let color = material
            .children()
            .find(|child| child.has_tag_name("color"))
            .with_context(|| format!("material {name:?} has no color"))?;
        let rgba = parse_rgba(required_attribute(color, "rgba")?)?;
        if materials.insert(name.clone(), rgba).is_some() {
            bail!("duplicate URDF material {name:?}");
        }
    }
    Ok(materials)
}

fn parse_links(
    robot: Node<'_, '_>,
    urdf_path: &Path,
    materials: &BTreeMap<String, [f64; 4]>,
) -> anyhow::Result<BTreeMap<String, Link>> {
    let mut links = BTreeMap::new();
    for node in robot.children().filter(|child| child.has_tag_name("link")) {
        let name = required_attribute(node, "name")?.to_owned();
        let visuals = node
            .children()
            .filter(|child| child.has_tag_name("visual"))
            .map(|visual| parse_geometry_instance(visual, urdf_path, materials, true))
            .collect::<anyhow::Result<Vec<_>>>()?;
        let collisions = node
            .children()
            .filter(|child| child.has_tag_name("collision"))
            .map(|collision| parse_geometry_instance(collision, urdf_path, materials, false))
            .collect::<anyhow::Result<Vec<_>>>()?;
        let link = Link {
            name: name.clone(),
            visuals,
            collisions,
        };
        if links.insert(name.clone(), link).is_some() {
            bail!("duplicate URDF link {name:?}");
        }
    }
    if links.is_empty() {
        bail!("URDF contains no links");
    }
    Ok(links)
}

fn parse_geometry_instance(
    node: Node<'_, '_>,
    urdf_path: &Path,
    materials: &BTreeMap<String, [f64; 4]>,
    visual: bool,
) -> anyhow::Result<GeometryInstance> {
    let origin = node
        .children()
        .find(|child| child.has_tag_name("origin"))
        .map(parse_origin)
        .transpose()?
        .unwrap_or_else(Isometry3::identity);
    let geometry_node = required_child(node, "geometry")?;
    let shapes = geometry_node
        .children()
        .filter(Node::is_element)
        .collect::<Vec<_>>();
    if shapes.len() != 1 {
        bail!(
            "{} geometry must contain exactly one shape",
            node.tag_name().name()
        );
    }
    let shape = shapes[0];
    let geometry = match shape.tag_name().name() {
        "mesh" => {
            let filename = required_attribute(shape, "filename")?;
            if filename.starts_with("package://") {
                bail!("package:// mesh paths are not supported: {filename}");
            }
            let path = Path::new(filename);
            let path = if path.is_absolute() {
                path.to_owned()
            } else {
                urdf_path
                    .parent()
                    .unwrap_or_else(|| Path::new("."))
                    .join(path)
            };
            if !path.is_file() {
                bail!("URDF mesh does not exist: {}", path.display());
            }
            let scale = parse_vector(shape.attribute("scale").unwrap_or("1 1 1"))?;
            if scale.iter().any(|value| *value <= 0.0) {
                bail!("URDF mesh scale must be positive");
            }
            Geometry::Mesh { path, scale }
        }
        "box" => {
            let size = parse_vector(required_attribute(shape, "size")?)?;
            if size.iter().any(|value| *value <= 0.0) {
                bail!("URDF box size must be positive");
            }
            Geometry::Box { size }
        }
        "cylinder" => Geometry::Cylinder {
            radius: positive_attribute(shape, "radius")?,
            length: positive_attribute(shape, "length")?,
        },
        "sphere" => Geometry::Sphere {
            radius: positive_attribute(shape, "radius")?,
        },
        other => bail!("unsupported URDF geometry {other:?}"),
    };
    let rgba = if visual {
        node.children()
            .find(|child| child.has_tag_name("material"))
            .map(|material| {
                if let Some(color) = material
                    .children()
                    .find(|child| child.has_tag_name("color"))
                {
                    parse_rgba(required_attribute(color, "rgba")?)
                } else {
                    let name = required_attribute(material, "name")?;
                    materials
                        .get(name)
                        .copied()
                        .with_context(|| format!("unknown URDF material {name:?}"))
                }
            })
            .transpose()?
            .unwrap_or([0.7, 0.72, 0.76, 1.0])
    } else {
        [0.7, 0.72, 0.76, 1.0]
    };
    Ok(GeometryInstance {
        origin,
        geometry,
        rgba,
    })
}

fn validate_description(
    links: &BTreeMap<String, Link>,
    joints: &BTreeMap<String, Joint>,
    transmissions: &[DifferentialTransmission],
) -> anyhow::Result<()> {
    let mut child_links = BTreeSet::new();
    for joint in joints.values() {
        if !links.contains_key(&joint.parent) {
            bail!(
                "joint {:?} references unknown parent link {:?}",
                joint.name,
                joint.parent
            );
        }
        if !links.contains_key(&joint.child) {
            bail!(
                "joint {:?} references unknown child link {:?}",
                joint.name,
                joint.child
            );
        }
        if !child_links.insert(&joint.child) {
            bail!("link {:?} has more than one parent joint", joint.child);
        }
    }
    for transmission in transmissions {
        for joint in &transmission.joints {
            let resolved = joints.get(joint).with_context(|| {
                format!(
                    "transmission {:?} references unknown joint {joint:?}",
                    transmission.name
                )
            })?;
            if resolved.kind == JointKind::Fixed {
                bail!(
                    "transmission {:?} references fixed joint {joint:?}",
                    transmission.name
                );
            }
        }
    }
    Ok(())
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
            name: name.to_owned(),
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

fn support_point(link: &Link, path: &[Joint]) -> anyhow::Result<Vector3<f64>> {
    let mut candidates = Vec::new();
    for collision in &link.collisions {
        match &collision.geometry {
            Geometry::Box { size } => {
                let size = size / 2.0;
                for x in [-size.x, size.x] {
                    for y in [-size.y, size.y] {
                        for z in [-size.z, size.z] {
                            candidates.push(
                                collision
                                    .origin
                                    .transform_point(&nalgebra::Point3::new(x, y, z))
                                    .coords,
                            );
                        }
                    }
                }
            }
            Geometry::Cylinder { radius, length } => {
                let half = length / 2.0;
                for z in [-half, half] {
                    for index in 0..16 {
                        let angle = std::f64::consts::TAU * index as f64 / 16.0;
                        candidates.push(
                            collision
                                .origin
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
            Geometry::Sphere { radius } => {
                candidates.push(
                    collision
                        .origin
                        .transform_point(&nalgebra::Point3::new(0.0, 0.0, -*radius))
                        .coords,
                );
            }
            Geometry::Mesh { .. } => {}
        }
    }
    if candidates.is_empty() {
        bail!(
            "foot link {:?} has no primitive collision geometry",
            link.name
        );
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

fn parse_rgba(text: &str) -> anyhow::Result<[f64; 4]> {
    let values = text
        .split_whitespace()
        .map(str::parse::<f64>)
        .collect::<Result<Vec<_>, _>>()?;
    if values.len() != 4
        || values
            .iter()
            .any(|value| !value.is_finite() || !(0.0..=1.0).contains(value))
    {
        bail!("expected four finite color components in [0, 1], got {text:?}");
    }
    Ok([values[0], values[1], values[2], values[3]])
}

fn finite_attribute(node: Node<'_, '_>, name: &str) -> anyhow::Result<f64> {
    let text = required_attribute(node, name)?;
    let value = text.parse::<f64>()?;
    if !value.is_finite() {
        bail!(
            "{} attribute {name:?} must be finite",
            node.tag_name().name()
        );
    }
    Ok(value)
}

fn positive_attribute(node: Node<'_, '_>, name: &str) -> anyhow::Result<f64> {
    let value = finite_attribute(node, name)?;
    if value <= 0.0 {
        bail!(
            "{} attribute {name:?} must be positive",
            node.tag_name().name()
        );
    }
    Ok(value)
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
  <link name="imu_mount"/><link name="imu"/>
  <joint name="left_joint" type="revolute">
    <origin xyz="0 0 -1"/>
    <parent link="pelvis"/><child link="left_foot"/><axis xyz="0 1 0"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
  <joint name="right_fixed" type="fixed">
    <origin xyz="0.2 0 -1"/>
    <parent link="pelvis"/><child link="right_foot"/>
  </joint>
  <joint name="imu_mount_fixed" type="fixed">
    <origin xyz="0.1 0 0.3" rpy="0 0 -1.5707963267948966"/>
    <parent link="pelvis"/><child link="imu_mount"/>
  </joint>
  <joint name="imu_fixed" type="fixed">
    <origin xyz="0.2 0 0"/>
    <parent link="imu_mount"/><child link="imu"/>
  </joint>
</robot>
"#;

    const NON_UNIT_DIFFERENTIAL_URDF: &str = r#"
<robot name="non_unit_differential">
  <link name="base"/>
  <link name="first"/>
  <link name="second">
    <collision><geometry><sphere radius="0.1"/></geometry></collision>
  </link>
  <joint name="roll" type="revolute">
    <parent link="base"/><child link="first"/>
    <axis xyz="1 0 0"/>
    <limit lower="-2" upper="3" effort="10" velocity="4"/>
  </joint>
  <joint name="pitch" type="revolute">
    <parent link="first"/><child link="second"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1" upper="1" effort="6" velocity="8"/>
  </joint>
  <transmission name="scaled">
    <type>transmission_interface/DifferentialTransmission</type>
    <actuator name="motor_a" role="actuator1">
      <mechanical_reduction>2</mechanical_reduction>
    </actuator>
    <actuator name="motor_b" role="actuator2">
      <mechanical_reduction>-4</mechanical_reduction>
    </actuator>
    <joint name="roll" role="joint1">
      <mechanical_reduction>-3</mechanical_reduction>
    </joint>
    <joint name="pitch" role="joint2">
      <mechanical_reduction>5</mechanical_reduction>
    </joint>
  </transmission>
</robot>
"#;

    fn load_text(text: &str) -> anyhow::Result<KinematicModel> {
        let mut file = tempfile::NamedTempFile::new()?;
        file.write_all(text.as_bytes())?;
        KinematicModel::load(file.path(), "pelvis", "left_foot", "right_foot")
    }

    fn load_description_text(text: &str) -> anyhow::Result<RobotDescription> {
        let mut file = tempfile::NamedTempFile::new()?;
        file.write_all(text.as_bytes())?;
        RobotDescription::load(file.path())
    }

    #[test]
    fn differential_matches_bello_ankle_equations() {
        let transmission = DifferentialTransmission {
            name: "ankle".into(),
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

        let torques = transmission
            .resolve_torques(&BTreeMap::from([
                ("motor_1".into(), 4.0),
                ("motor_2".into(), 6.0),
            ]))
            .unwrap();
        assert_eq!(torques[0], ("roll".into(), 10.0));
        assert_eq!(torques[1], ("pitch".into(), 2.0));
    }

    #[test]
    fn differential_reductions_preserve_virtual_work_and_project_bounds() {
        let description = load_description_text(NON_UNIT_DIFFERENTIAL_URDF).unwrap();
        let model = description
            .kinematic_model("base", "second", "second")
            .unwrap();
        assert_eq!(model.required_coordinates(), &["motor_a", "motor_b"]);
        let transmission = &description.transmissions()[0];
        let coordinate_velocity = BTreeMap::from([
            (
                "motor_a".into(),
                CoordinateState {
                    position: 0.0,
                    velocity: 0.7,
                },
            ),
            (
                "motor_b".into(),
                CoordinateState {
                    position: 0.0,
                    velocity: -1.1,
                },
            ),
        ]);
        let joint_velocity = transmission.resolve(&coordinate_velocity).unwrap();
        let coordinate_torque = BTreeMap::from([("motor_a".into(), 2.0), ("motor_b".into(), -3.0)]);
        let joint_torque = transmission.resolve_torques(&coordinate_torque).unwrap();

        assert_eq!(joint_torque[0], ("roll".into(), -48.0));
        assert_eq!(joint_torque[1], ("pitch".into(), -40.0));
        let coordinate_power = 2.0 * 0.7 + -3.0 * -1.1;
        let joint_power = joint_torque[0].1 * joint_velocity[0].1.velocity
            + joint_torque[1].1 * joint_velocity[1].1.velocity;
        assert!((coordinate_power - joint_power).abs() < 1.0e-12);

        assert_eq!(
            description.coordinate_position_limits("motor_a"),
            Some((-28.0, 22.0))
        );
        assert_eq!(
            description.coordinate_position_limits("motor_b"),
            Some((-44.0, 56.0))
        );
        assert_eq!(
            description.coordinate_velocity_limit("motor_a"),
            Some(104.0)
        );
        assert_eq!(
            description.coordinate_velocity_limit("motor_b"),
            Some(208.0)
        );
        assert!(
            (description.coordinate_effort_limit("motor_a").unwrap() - 17.0 / 15.0).abs() < 1.0e-12
        );
        assert!(
            (description.coordinate_effort_limit("motor_b").unwrap() - 17.0 / 30.0).abs() < 1.0e-12
        );
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
    fn fixed_link_transform_is_resolved_and_movable_paths_are_rejected() {
        let description = load_description_text(SIMPLE_URDF).unwrap();
        let pelvis_from_imu = description.fixed_transform("pelvis", "imu").unwrap();
        assert!(
            (pelvis_from_imu.translation.vector - Vector3::new(0.1, -0.2, 0.3)).norm() < 1.0e-12
        );
        let expected = UnitQuaternion::from_euler_angles(0.0, 0.0, -std::f64::consts::FRAC_PI_2);
        assert!(pelvis_from_imu.rotation.angle_to(&expected) < 1.0e-12);

        let error = description
            .fixed_transform("pelvis", "left_foot")
            .unwrap_err()
            .to_string();
        assert!(error.contains("is movable"), "{error}");
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
}
