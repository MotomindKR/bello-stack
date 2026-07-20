use std::cell::RefCell;
use std::collections::BTreeMap;
use std::fs::File;
use std::io::BufReader;
use std::rc::Rc;

use anyhow::Context;
use kiss3d::prelude::*;

use crate::Transform;
use crate::mjcf::{Body, Geometry, GeometryKind, JointKind, Model};

struct MovingJoint {
    node: SceneNode3d,
    kind: JointKind,
    position: Vec3,
    axis: Vec3,
}

/// Main-thread-only Kiss3d hierarchy corresponding one-for-one with MJCF bodies.
pub struct ModelScene {
    root: SceneNode3d,
    moving_joints: BTreeMap<String, MovingJoint>,
}

impl ModelScene {
    pub fn build(scene: &mut SceneNode3d, model: &Model) -> anyhow::Result<Self> {
        let mut root = scene.add_group();
        let mut moving_joints = BTreeMap::new();
        add_body_contents(&mut root, &model.world, model, &mut moving_joints)?;
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
                JointKind::Hinge => {
                    let rotation = Quat::from_axis_angle(moving.axis, value as f32);
                    moving
                        .node
                        .set_rotation(rotation)
                        .set_position(moving.position - rotation * moving.position);
                }
                JointKind::Slide => {
                    moving
                        .node
                        .set_rotation(Quat::IDENTITY)
                        .set_position(moving.axis * value as f32);
                }
                JointKind::Fixed | JointKind::Free => {}
            }
        }
    }
}

fn add_body_contents(
    parent: &mut SceneNode3d,
    body: &Body,
    model: &Model,
    moving_joints: &mut BTreeMap<String, MovingJoint>,
) -> anyhow::Result<()> {
    let mut body_node = parent.add_group();
    set_transform(&mut body_node, body.transform);
    let mut content = body_node;
    for joint in &body.joints {
        if matches!(joint.kind, JointKind::Hinge | JointKind::Slide) {
            let node = content.add_group();
            moving_joints.insert(
                joint.name.clone(),
                MovingJoint {
                    node: node.clone(),
                    kind: joint.kind,
                    position: vector(joint.position),
                    axis: vector(joint.axis),
                },
            );
            content = node;
        }
    }
    for geometry in &body.geometries {
        add_geometry(&mut content, geometry, model)?;
    }
    for child in &body.children {
        add_body_contents(&mut content, child, model, moving_joints)?;
    }
    Ok(())
}

fn add_geometry(
    parent: &mut SceneNode3d,
    geometry: &Geometry,
    model: &Model,
) -> anyhow::Result<()> {
    let mut frame = parent.add_group();
    set_transform(&mut frame, geometry.transform);
    let mut rendered = match &geometry.kind {
        GeometryKind::Mesh { asset } => {
            let asset = &model.meshes[asset];
            let file = File::open(&asset.path)
                .with_context(|| format!("open STL mesh {}", asset.path.display()))?;
            let mesh = stl_io::read_stl(&mut BufReader::new(file))
                .with_context(|| format!("read STL mesh {}", asset.path.display()))?;
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
        GeometryKind::Box { half_extents } => frame.add_cube(
            (2.0 * half_extents[0]) as f32,
            (2.0 * half_extents[1]) as f32,
            (2.0 * half_extents[2]) as f32,
        ),
        GeometryKind::Sphere { radius } => frame.add_sphere(*radius as f32),
        GeometryKind::Cylinder {
            radius,
            half_length,
        } => {
            let mut cylinder = frame.add_cylinder(*radius as f32, (2.0 * half_length) as f32);
            // Kiss3d cylinders are Y-aligned; MuJoCo cylinders are Z-aligned.
            cylinder.set_rotation(Quat::from_rotation_x(std::f32::consts::FRAC_PI_2));
            cylinder
        }
    };
    rendered
        .set_color(Color::new(
            geometry.rgba[0] as f32,
            geometry.rgba[1] as f32,
            geometry.rgba[2] as f32,
            geometry.rgba[3] as f32,
        ))
        .enable_backface_culling(false);
    Ok(())
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
