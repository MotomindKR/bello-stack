use std::collections::{BTreeMap, BTreeSet};
use std::fs;
use std::path::{Path, PathBuf};

use roxmltree::Node;
use thiserror::Error;

use crate::transform::{Transform, wxyz_to_xyzw};

#[derive(Debug, Error)]
pub enum ModelError {
    #[error("could not read MJCF {path}: {source}")]
    Read {
        path: PathBuf,
        source: std::io::Error,
    },
    #[error("invalid MJCF XML: {0}")]
    Xml(#[from] roxmltree::Error),
    #[error("unsupported MJCF feature: {0}")]
    Unsupported(String),
    #[error("missing or invalid MJCF attribute: {0}")]
    Attribute(String),
    #[error("duplicate MJCF name: {0}")]
    DuplicateName(String),
    #[error("invalid MJCF transform: {0}")]
    InvalidTransform(String),
    #[error("MJCF mesh asset {name:?} does not exist at {path}")]
    MissingMesh { name: String, path: PathBuf },
    #[error("MJCF reference could not be resolved: {0}")]
    UnresolvedReference(String),
}

#[derive(Clone, Debug, PartialEq)]
pub struct MeshAsset {
    pub name: String,
    pub path: PathBuf,
    pub scale: [f64; 3],
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JointKind {
    Hinge,
    Slide,
    Fixed,
    Free,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Joint {
    pub name: String,
    pub kind: JointKind,
    pub position: [f64; 3],
    pub axis: [f64; 3],
    pub range: Option<[f64; 2]>,
}

#[derive(Clone, Debug, PartialEq)]
pub enum GeometryKind {
    Mesh { asset: String },
    Box { half_extents: [f64; 3] },
    Sphere { radius: f64 },
    Cylinder { radius: f64, half_length: f64 },
}

#[derive(Clone, Debug, PartialEq)]
pub struct Geometry {
    pub name: Option<String>,
    pub transform: Transform,
    pub kind: GeometryKind,
    pub rgba: [f64; 4],
}

#[derive(Clone, Debug, PartialEq)]
pub struct Body {
    pub name: String,
    pub transform: Transform,
    pub joints: Vec<Joint>,
    pub geometries: Vec<Geometry>,
    pub children: Vec<Body>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct FixedTendon {
    pub name: String,
    pub coefficients: BTreeMap<String, f64>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct PositionActuator {
    pub name: String,
    /// The directly commanded joint or fixed-tendon coordinate.
    pub coordinate: String,
    pub control_range: [f64; 2],
    pub default_position: f64,
}

#[derive(Clone, Debug)]
pub struct Model {
    pub name: String,
    pub meshes: BTreeMap<String, MeshAsset>,
    pub world: Body,
    pub joints: BTreeMap<String, Joint>,
    pub fixed_tendons: BTreeMap<String, FixedTendon>,
    /// Position actuators indexed by their joint/tendon coordinate name.
    pub position_actuators: BTreeMap<String, PositionActuator>,
}

impl Model {
    pub fn load(path: impl AsRef<Path>) -> Result<Self, ModelError> {
        let path = path.as_ref();
        let xml = fs::read_to_string(path).map_err(|source| ModelError::Read {
            path: path.to_owned(),
            source,
        })?;
        Self::parse(&xml, path)
    }

    pub fn parse(xml: &str, source_path: impl AsRef<Path>) -> Result<Self, ModelError> {
        let source_path = source_path.as_ref();
        let document = roxmltree::Document::parse(xml)?;
        let root = document.root_element();
        if !root.has_tag_name("mujoco") {
            return Err(ModelError::Unsupported(format!(
                "root element must be <mujoco>, found <{}>",
                root.tag_name().name()
            )));
        }
        for node in root.descendants().filter(Node::is_element) {
            match node.tag_name().name() {
                "include" => {
                    return Err(ModelError::Unsupported(
                        "<include> is not allowed; prepare a flattened MJCF".into(),
                    ));
                }
                "default" => {
                    return Err(ModelError::Unsupported(
                        "<default> is not allowed; generate explicit flattened attributes".into(),
                    ));
                }
                _ => {}
            }
        }
        if root
            .children()
            .find(|node| node.has_tag_name("compiler"))
            .and_then(|node| node.attribute("angle"))
            .is_some_and(|angle| angle != "radian")
        {
            return Err(ModelError::Unsupported(
                "compiler angle must be radian".into(),
            ));
        }

        let name = root.attribute("model").unwrap_or("unnamed").to_owned();
        let base = source_path.parent().unwrap_or_else(|| Path::new("."));
        let mut mesh_names = BTreeSet::new();
        let mut meshes = BTreeMap::new();
        if let Some(asset) = root.children().find(|node| node.has_tag_name("asset")) {
            for node in asset.children().filter(Node::is_element) {
                if !node.has_tag_name("mesh") {
                    continue;
                }
                let mesh_name = required(node, "name")?.to_owned();
                insert_name(&mut mesh_names, &mesh_name, "mesh")?;
                for unsupported in ["refpos", "refquat"] {
                    if node.attribute(unsupported).is_some() {
                        return Err(ModelError::Unsupported(format!(
                            "mesh {mesh_name:?} uses {unsupported}; bake mesh reference transforms during asset preparation"
                        )));
                    }
                }
                let file = required(node, "file")?;
                let mesh_path = base.join(file);
                if !mesh_path.is_file() {
                    return Err(ModelError::MissingMesh {
                        name: mesh_name,
                        path: mesh_path,
                    });
                }
                let scale =
                    parse_array::<3>(node.attribute("scale").unwrap_or("1 1 1"), "mesh scale")?;
                if scale.contains(&0.0) {
                    return Err(ModelError::Attribute(format!(
                        "mesh {:?} scale components must be nonzero",
                        mesh_name
                    )));
                }
                meshes.insert(
                    mesh_name.clone(),
                    MeshAsset {
                        name: mesh_name,
                        path: mesh_path,
                        scale,
                    },
                );
            }
        }

        let worldbody = root
            .children()
            .find(|node| node.has_tag_name("worldbody"))
            .ok_or_else(|| ModelError::Attribute("missing <worldbody>".into()))?;
        let mut body_names = BTreeSet::new();
        let mut joint_names = BTreeSet::new();
        let mut geom_names = BTreeSet::new();
        let mut joints = BTreeMap::new();
        let mut geometries = Vec::new();
        let mut children = Vec::new();
        for node in worldbody.children().filter(Node::is_element) {
            match node.tag_name().name() {
                "body" => children.push(parse_body(
                    node,
                    &meshes,
                    &mut body_names,
                    &mut joint_names,
                    &mut geom_names,
                    &mut joints,
                )?),
                "geom" => {
                    if let Some(geometry) = parse_geometry(node, &meshes, &mut geom_names)? {
                        geometries.push(geometry);
                    }
                }
                "light" | "camera" | "site" => {}
                other => {
                    return Err(ModelError::Unsupported(format!(
                        "<{other}> directly under <worldbody>"
                    )));
                }
            }
        }
        let world = Body {
            name: "__world__".into(),
            transform: Transform::IDENTITY,
            joints: Vec::new(),
            geometries,
            children,
        };

        let fixed_tendons = parse_tendons(root, &joints)?;
        for name in fixed_tendons.keys() {
            if joint_names.contains(name) {
                return Err(ModelError::DuplicateName(format!(
                    "coordinate {name:?} is both a joint and a fixed tendon"
                )));
            }
        }
        let position_actuators = parse_position_actuators(root, &joints, &fixed_tendons)?;
        Ok(Self {
            name,
            meshes,
            world,
            joints,
            fixed_tendons,
            position_actuators,
        })
    }
}

fn parse_position_actuators(
    root: Node<'_, '_>,
    joints: &BTreeMap<String, Joint>,
    tendons: &BTreeMap<String, FixedTendon>,
) -> Result<BTreeMap<String, PositionActuator>, ModelError> {
    let Some(actuator_root) = root.children().find(|node| node.has_tag_name("actuator")) else {
        return Ok(BTreeMap::new());
    };
    let mut actuator_names = BTreeSet::new();
    let mut actuators = BTreeMap::new();
    for node in actuator_root.children().filter(Node::is_element) {
        if !node.has_tag_name("position") {
            return Err(ModelError::Unsupported(format!(
                "<{}> actuator; visualizer control coordinates require position actuators",
                node.tag_name().name()
            )));
        }
        let name = required(node, "name")?.to_owned();
        insert_name(&mut actuator_names, &name, "actuator")?;
        let joint = node.attribute("joint");
        let tendon = node.attribute("tendon");
        let coordinate = match (joint, tendon) {
            (Some(joint), None) if joints.contains_key(joint) => joint.to_owned(),
            (None, Some(tendon)) if tendons.contains_key(tendon) => tendon.to_owned(),
            (Some(joint), None) => {
                return Err(ModelError::UnresolvedReference(format!(
                    "position actuator {name:?} references unknown joint {joint:?}"
                )));
            }
            (None, Some(tendon)) => {
                return Err(ModelError::UnresolvedReference(format!(
                    "position actuator {name:?} references unknown tendon {tendon:?}"
                )));
            }
            _ => {
                return Err(ModelError::Attribute(format!(
                    "position actuator {name:?} must reference exactly one joint or tendon"
                )));
            }
        };
        let control_range = parse_array::<2>(required(node, "ctrlrange")?, "actuator ctrlrange")?;
        if control_range[0] > control_range[1] {
            return Err(ModelError::Attribute(format!(
                "position actuator {name:?} has reversed ctrlrange"
            )));
        }
        let default_position = node
            .attribute("user")
            .map(|value| parse_scalar(value, "actuator user default"))
            .transpose()?
            .unwrap_or_else(|| 0.0_f64.clamp(control_range[0], control_range[1]));
        if !(control_range[0]..=control_range[1]).contains(&default_position) {
            return Err(ModelError::Attribute(format!(
                "position actuator {name:?} user default is outside ctrlrange"
            )));
        }
        let actuator = PositionActuator {
            name,
            coordinate: coordinate.clone(),
            control_range,
            default_position,
        };
        if actuators.insert(coordinate.clone(), actuator).is_some() {
            return Err(ModelError::DuplicateName(format!(
                "multiple position actuators command coordinate {coordinate:?}"
            )));
        }
    }
    Ok(actuators)
}

fn parse_body(
    node: Node<'_, '_>,
    meshes: &BTreeMap<String, MeshAsset>,
    body_names: &mut BTreeSet<String>,
    joint_names: &mut BTreeSet<String>,
    geom_names: &mut BTreeSet<String>,
    all_joints: &mut BTreeMap<String, Joint>,
) -> Result<Body, ModelError> {
    let name = required(node, "name")?.to_owned();
    insert_name(body_names, &name, "body")?;
    let transform = parse_transform(node, &format!("body {name:?}"))?;
    let mut joints = Vec::new();
    let mut geometries = Vec::new();
    let mut children = Vec::new();
    for child in node.children().filter(Node::is_element) {
        match child.tag_name().name() {
            "body" => children.push(parse_body(
                child,
                meshes,
                body_names,
                joint_names,
                geom_names,
                all_joints,
            )?),
            "joint" | "freejoint" => {
                let joint = parse_joint(child)?;
                insert_name(joint_names, &joint.name, "joint")?;
                all_joints.insert(joint.name.clone(), joint.clone());
                joints.push(joint);
            }
            "geom" => {
                if let Some(geometry) = parse_geometry(child, meshes, geom_names)? {
                    geometries.push(geometry);
                }
            }
            "inertial" | "site" | "camera" | "light" => {}
            other => {
                return Err(ModelError::Unsupported(format!(
                    "<{other}> in body {name:?}"
                )));
            }
        }
    }
    Ok(Body {
        name,
        transform,
        joints,
        geometries,
        children,
    })
}

fn parse_joint(node: Node<'_, '_>) -> Result<Joint, ModelError> {
    let name = required(node, "name")?.to_owned();
    let kind = if node.has_tag_name("freejoint") {
        JointKind::Free
    } else {
        match node.attribute("type").unwrap_or("hinge") {
            "hinge" => JointKind::Hinge,
            "slide" => JointKind::Slide,
            "free" => JointKind::Free,
            "fixed" => JointKind::Fixed,
            "ball" => {
                return Err(ModelError::Unsupported(format!(
                    "ball joint {name:?}; use hinge, slide, fixed, or free"
                )));
            }
            other => {
                return Err(ModelError::Unsupported(format!(
                    "joint {name:?} has type {other:?}"
                )));
            }
        }
    };
    let position = parse_array::<3>(node.attribute("pos").unwrap_or("0 0 0"), "joint pos")?;
    let mut axis = parse_array::<3>(node.attribute("axis").unwrap_or("0 0 1"), "joint axis")?;
    if matches!(kind, JointKind::Hinge | JointKind::Slide) {
        let norm = axis.iter().map(|value| value * value).sum::<f64>().sqrt();
        if norm <= 1.0e-12 {
            return Err(ModelError::Attribute(format!(
                "joint {name:?} has a zero-length axis"
            )));
        }
        for value in &mut axis {
            *value /= norm;
        }
    }
    let range = node
        .attribute("range")
        .map(|value| parse_array::<2>(value, "joint range"))
        .transpose()?;
    Ok(Joint {
        name,
        kind,
        position,
        axis,
        range,
    })
}

fn parse_geometry(
    node: Node<'_, '_>,
    meshes: &BTreeMap<String, MeshAsset>,
    geom_names: &mut BTreeSet<String>,
) -> Result<Option<Geometry>, ModelError> {
    let name = node.attribute("name").map(str::to_owned);
    if let Some(name) = &name {
        insert_name(geom_names, name, "geometry")?;
    }
    let rgba = parse_array::<4>(
        node.attribute("rgba").unwrap_or("0.5 0.5 0.5 1"),
        "geom rgba",
    )?;
    if rgba[3] <= 0.0 {
        return Ok(None);
    }
    let context = format!("geometry {:?}", name.as_deref().unwrap_or("<unnamed>"));
    let transform = parse_transform(node, &context)?;
    let kind = match node.attribute("type").unwrap_or("sphere") {
        "mesh" => {
            let asset = required(node, "mesh")?.to_owned();
            if !meshes.contains_key(&asset) {
                return Err(ModelError::UnresolvedReference(format!(
                    "{context} references unknown mesh {asset:?}"
                )));
            }
            GeometryKind::Mesh { asset }
        }
        "box" => GeometryKind::Box {
            half_extents: positive_array::<3>(required(node, "size")?, "box size")?,
        },
        "sphere" => GeometryKind::Sphere {
            radius: positive_array::<1>(required(node, "size")?, "sphere size")?[0],
        },
        "cylinder" => {
            let size = positive_array::<2>(required(node, "size")?, "cylinder size")?;
            GeometryKind::Cylinder {
                radius: size[0],
                half_length: size[1],
            }
        }
        other => {
            return Err(ModelError::Unsupported(format!(
                "{context} has type {other:?}; supported visual geoms are mesh, box, sphere, and cylinder"
            )));
        }
    };
    Ok(Some(Geometry {
        name,
        transform,
        kind,
        rgba,
    }))
}

fn parse_tendons(
    root: Node<'_, '_>,
    joints: &BTreeMap<String, Joint>,
) -> Result<BTreeMap<String, FixedTendon>, ModelError> {
    let Some(tendon_root) = root.children().find(|node| node.has_tag_name("tendon")) else {
        return Ok(BTreeMap::new());
    };
    let mut names = BTreeSet::new();
    let mut tendons = BTreeMap::new();
    for node in tendon_root.children().filter(Node::is_element) {
        if !node.has_tag_name("fixed") {
            return Err(ModelError::Unsupported(format!(
                "<{}> tendon; spatial and pulley tendons are not linear coordinates",
                node.tag_name().name()
            )));
        }
        let name = required(node, "name")?.to_owned();
        insert_name(&mut names, &name, "fixed tendon")?;
        let mut coefficients = BTreeMap::new();
        for term in node.children().filter(Node::is_element) {
            if !term.has_tag_name("joint") {
                return Err(ModelError::Unsupported(format!(
                    "fixed tendon {name:?} contains <{}>; only joint coefficients are supported",
                    term.tag_name().name()
                )));
            }
            let joint = required(term, "joint")?.to_owned();
            let Some(definition) = joints.get(&joint) else {
                return Err(ModelError::UnresolvedReference(format!(
                    "fixed tendon {name:?} references unknown joint {joint:?}"
                )));
            };
            if !matches!(definition.kind, JointKind::Hinge | JointKind::Slide) {
                return Err(ModelError::Unsupported(format!(
                    "fixed tendon {name:?} references non-scalar joint {joint:?}"
                )));
            }
            let coefficient = parse_scalar(required(term, "coef")?, "tendon coefficient")?;
            if coefficients.insert(joint.clone(), coefficient).is_some() {
                return Err(ModelError::DuplicateName(format!(
                    "joint {joint:?} appears twice in fixed tendon {name:?}"
                )));
            }
        }
        if coefficients.is_empty() {
            return Err(ModelError::Attribute(format!(
                "fixed tendon {name:?} has no joint coefficients"
            )));
        }
        tendons.insert(name.clone(), FixedTendon { name, coefficients });
    }
    Ok(tendons)
}

fn parse_transform(node: Node<'_, '_>, context: &str) -> Result<Transform, ModelError> {
    for unsupported in ["euler", "axisangle", "xyaxes", "zaxis", "fromto"] {
        if node.attribute(unsupported).is_some() {
            return Err(ModelError::Unsupported(format!(
                "{context} uses {unsupported}; flattened MJCF must use quat"
            )));
        }
    }
    let translation = parse_array::<3>(node.attribute("pos").unwrap_or("0 0 0"), "position")?;
    let wxyz = parse_array::<4>(node.attribute("quat").unwrap_or("1 0 0 0"), "quaternion")?;
    Transform {
        translation,
        orientation_xyzw: wxyz_to_xyzw(wxyz),
    }
    .checked(context)
}

fn required<'a>(node: Node<'a, '_>, attribute: &str) -> Result<&'a str, ModelError> {
    node.attribute(attribute).ok_or_else(|| {
        ModelError::Attribute(format!(
            "<{}> at line {} requires {attribute:?}",
            node.tag_name().name(),
            node.document().text_pos_at(node.range().start).row
        ))
    })
}

fn insert_name(names: &mut BTreeSet<String>, name: &str, category: &str) -> Result<(), ModelError> {
    if name.trim().is_empty() {
        return Err(ModelError::Attribute(format!("{category} name is empty")));
    }
    if !names.insert(name.to_owned()) {
        return Err(ModelError::DuplicateName(format!(
            "{category} {name:?} appears more than once"
        )));
    }
    Ok(())
}

fn parse_scalar(value: &str, context: &str) -> Result<f64, ModelError> {
    let parsed = value.parse::<f64>().map_err(|_| {
        ModelError::Attribute(format!("{context} contains invalid number {value:?}"))
    })?;
    if !parsed.is_finite() {
        return Err(ModelError::Attribute(format!(
            "{context} must be finite, got {value:?}"
        )));
    }
    Ok(parsed)
}

fn parse_array<const N: usize>(value: &str, context: &str) -> Result<[f64; N], ModelError> {
    let values = value
        .split_ascii_whitespace()
        .map(|part| parse_scalar(part, context))
        .collect::<Result<Vec<_>, _>>()?;
    values.try_into().map_err(|values: Vec<f64>| {
        ModelError::Attribute(format!(
            "{context} needs {N} values, found {} in {value:?}",
            values.len()
        ))
    })
}

fn positive_array<const N: usize>(value: &str, context: &str) -> Result<[f64; N], ModelError> {
    let values = parse_array::<N>(value, context)?;
    if values.iter().any(|value| *value <= 0.0) {
        return Err(ModelError::Attribute(format!(
            "{context} values must be positive"
        )));
    }
    Ok(values)
}
