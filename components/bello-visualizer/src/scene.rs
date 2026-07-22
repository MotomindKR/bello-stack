use std::cell::RefCell;
use std::collections::BTreeMap;
use std::fs::File;
use std::io::BufReader;
use std::path::Path;
use std::rc::Rc;

use anyhow::{Context, bail};
use kiss3d::prelude::*;
use mjcf_rs::body::{Geom, GeomType, JointType};

use crate::{Model, Transform};

struct MovingJoint {
    node: SceneNode3d,
    kind: JointType,
    position: Vec3,
    axis: Vec3,
}

/// Main-thread-only Kiss3d hierarchy corresponding one-for-one with MJCF bodies.
pub struct ModelScene {
    root: SceneNode3d,
    moving_joints: BTreeMap<String, MovingJoint>,
}

impl ModelScene {
    pub fn build(
        scene: &mut SceneNode3d,
        model: &Model,
        source_path: &Path,
    ) -> anyhow::Result<Self> {
        let mut root = scene.add_group();
        let children = body_children(model)?;
        let mut moving_joints = BTreeMap::new();
        let base = source_path.parent().unwrap_or_else(|| Path::new("."));
        add_body_contents(&mut root, 0, model, base, &children, &mut moving_joints)?;
        Ok(Self {
            root,
            moving_joints,
        })
    }

    pub fn set_root_transform(&mut self, transform: Transform) {
        set_transform(&mut self.root, transform);
    }

    pub fn set_joint_positions(&mut self, positions: &BTreeMap<String, f64>) {
        for (name, moving) in &mut self.moving_joints {
            let Some(value) = positions
                .get(name)
                .copied()
                .filter(|value| value.is_finite())
            else {
                continue;
            };
            match moving.kind {
                JointType::Hinge => {
                    let rotation = Quat::from_axis_angle(moving.axis, value as f32);
                    moving
                        .node
                        .set_rotation(rotation)
                        .set_position(moving.position - rotation * moving.position);
                }
                JointType::Slide => {
                    moving
                        .node
                        .set_rotation(Quat::IDENTITY)
                        .set_position(moving.axis * value as f32);
                }
                JointType::Ball | JointType::Free => {}
            }
        }
    }
}

fn body_children(model: &Model) -> anyhow::Result<Vec<Vec<usize>>> {
    if model.bodies.is_empty() {
        bail!("MJCF has no world body");
    }
    let mut children = vec![Vec::new(); model.bodies.len()];
    for id in 1..model.bodies.len() {
        let parent = model.bodies[id]
            .parent
            .with_context(|| format!("MJCF body index {id} has no parent"))?;
        children[parent].push(id);
    }
    Ok(children)
}

fn add_body_contents(
    parent: &mut SceneNode3d,
    body_id: usize,
    model: &Model,
    base: &Path,
    children: &[Vec<usize>],
    moving_joints: &mut BTreeMap<String, MovingJoint>,
) -> anyhow::Result<()> {
    let body = &model.bodies[body_id].body;
    let mut body_node = parent.add_group();
    set_pose(&mut body_node, body.pose);
    let mut content = body_node;
    for joint in &body.joints {
        if matches!(joint.type_, JointType::Hinge | JointType::Slide) {
            let name = joint
                .name
                .as_ref()
                .context("validated MJCF joint lost its name")?;
            let node = content.add_group();
            let axis = vector(joint.axis);
            if !axis.is_finite() || axis.length_squared() <= f32::EPSILON {
                bail!("MJCF joint {name:?} has an invalid axis");
            }
            moving_joints.insert(
                name.clone(),
                MovingJoint {
                    node: node.clone(),
                    kind: joint.type_,
                    position: vector(joint.pos),
                    axis: axis.normalize(),
                },
            );
            content = node;
        }
    }
    for geometry in &body.geoms {
        add_geometry(&mut content, geometry, model, base)?;
    }
    for &child in &children[body_id] {
        add_body_contents(&mut content, child, model, base, children, moving_joints)?;
    }
    Ok(())
}

fn add_geometry(
    parent: &mut SceneNode3d,
    geometry: &Geom,
    model: &Model,
    base: &Path,
) -> anyhow::Result<()> {
    let rgba = geom_rgba(model, geometry)?;
    if rgba[3] <= 0.0 {
        return Ok(());
    }
    let mut frame = parent.add_group();
    set_pose(&mut frame, geometry.pose);
    let mut rendered = match geometry.type_ {
        GeomType::Mesh => {
            let mesh_name = geometry
                .mesh
                .as_deref()
                .context("validated mesh geometry lost its asset reference")?;
            let asset = model
                .assets
                .mesh(mesh_name)
                .with_context(|| format!("unknown MJCF mesh {mesh_name:?}"))?;
            let path = model
                .resolve_mesh_file(asset, base)
                .context("MJCF mesh has neither a file nor supported inline data")?;
            let file =
                File::open(&path).with_context(|| format!("open STL mesh {}", path.display()))?;
            let mesh = stl_io::read_stl(&mut BufReader::new(file))
                .with_context(|| format!("read STL mesh {}", path.display()))?;
            let vertices = mesh
                .vertices
                .iter()
                .map(|vertex| Vec3::new(vertex[0], vertex[1], vertex[2]))
                .collect();
            let indices = mesh
                .faces
                .iter()
                .map(|face| {
                    face.vertices.map(|index| {
                        u32::try_from(index).expect("prepared visual mesh index fits in u32")
                    })
                })
                .collect();
            let gpu = Rc::new(RefCell::new(GpuMesh3d::new(
                vertices, indices, None, None, false,
            )));
            frame.add_mesh(gpu, vector(asset.scale))
        }
        GeomType::Box => frame.add_cube(
            (2.0 * geometry.size[0]) as f32,
            (2.0 * geometry.size[1]) as f32,
            (2.0 * geometry.size[2]) as f32,
        ),
        GeomType::Sphere => frame.add_sphere(geometry.size[0] as f32),
        GeomType::Cylinder => {
            let mut cylinder =
                frame.add_cylinder(geometry.size[0] as f32, (2.0 * geometry.size[1]) as f32);
            // Kiss3d cylinders are Y-aligned; MuJoCo cylinders are Z-aligned.
            cylinder.set_rotation(Quat::from_rotation_x(std::f32::consts::FRAC_PI_2));
            cylinder
        }
        other => bail!("unsupported validated visual geometry {other:?}"),
    };
    rendered
        .set_color(Color::new(
            rgba[0] as f32,
            rgba[1] as f32,
            rgba[2] as f32,
            rgba[3] as f32,
        ))
        .enable_backface_culling(false);
    Ok(())
}

fn geom_rgba(model: &Model, geom: &Geom) -> anyhow::Result<[f64; 4]> {
    if let Some(rgba) = geom.rgba {
        return Ok(rgba);
    }
    let Some(material_name) = geom.material.as_deref() else {
        return Ok([0.5, 0.5, 0.5, 1.0]);
    };
    let material = model
        .assets
        .material(material_name)
        .with_context(|| format!("unknown MJCF material {material_name:?}"))?;
    Ok(material.rgba.unwrap_or([0.5, 0.5, 0.5, 1.0]))
}

fn set_pose(node: &mut SceneNode3d, pose: mjcf_rs::Pose) {
    node.set_position(vector(pose.translation.to_array()))
        .set_rotation(Quat::from_xyzw(
            pose.rotation.x as f32,
            pose.rotation.y as f32,
            pose.rotation.z as f32,
            pose.rotation.w as f32,
        ));
}

fn set_transform(node: &mut SceneNode3d, transform: Transform) {
    node.set_position(vector(transform.translation))
        .set_rotation(Quat::from_xyzw(
            transform.orientation_xyzw[0] as f32,
            transform.orientation_xyzw[1] as f32,
            transform.orientation_xyzw[2] as f32,
            transform.orientation_xyzw[3] as f32,
        ));
}

fn vector(values: [f64; 3]) -> Vec3 {
    Vec3::new(values[0] as f32, values[1] as f32, values[2] as f32)
}
