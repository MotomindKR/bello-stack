use std::cell::RefCell;
use std::collections::BTreeMap;
use std::fs::File;
use std::io::BufReader;
use std::rc::Rc;

use anyhow::{Context, bail};
use bello_robot_description::{Geometry, GeometryInstance, JointKind, RobotDescription};
use kiss3d::prelude::*;
use nalgebra::Isometry3;

use crate::Transform;

struct MovingJoint {
    node: SceneNode3d,
    kind: JointKind,
    axis: Vec3,
}

/// Main-thread-only Kiss3d hierarchy corresponding one-for-one with URDF links.
pub struct ModelScene {
    root: SceneNode3d,
    moving_joints: BTreeMap<String, MovingJoint>,
}

impl ModelScene {
    pub fn build(scene: &mut SceneNode3d, description: &RobotDescription) -> anyhow::Result<Self> {
        let mut root = scene.add_group();
        let mut moving_joints = BTreeMap::new();
        add_link_contents(
            &mut root,
            description,
            description.root_link(),
            &mut moving_joints,
        )?;
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
                JointKind::Revolute => {
                    moving
                        .node
                        .set_rotation(Quat::from_axis_angle(moving.axis, value as f32))
                        .set_position(Vec3::ZERO);
                }
                JointKind::Prismatic => {
                    moving
                        .node
                        .set_rotation(Quat::IDENTITY)
                        .set_position(moving.axis * value as f32);
                }
                JointKind::Fixed => {}
            }
        }
    }
}

fn add_link_contents(
    parent: &mut SceneNode3d,
    description: &RobotDescription,
    link_name: &str,
    moving_joints: &mut BTreeMap<String, MovingJoint>,
) -> anyhow::Result<()> {
    let link = description
        .link(link_name)
        .with_context(|| format!("validated URDF lost link {link_name:?}"))?;
    let mut link_node = parent.add_group();
    for visual in &link.visuals {
        add_geometry(&mut link_node, visual)?;
    }
    for joint in description.child_joints(link_name) {
        let mut origin = link_node.add_group();
        set_pose(&mut origin, &joint.origin);
        let mut motion = origin.add_group();
        if joint.kind != JointKind::Fixed {
            let axis = vector3(&joint.axis.into_inner());
            if !axis.is_finite() || axis.length_squared() <= f32::EPSILON {
                bail!("URDF joint {:?} has an invalid axis", joint.name);
            }
            moving_joints.insert(
                joint.name.clone(),
                MovingJoint {
                    node: motion.clone(),
                    kind: joint.kind,
                    axis: axis.normalize(),
                },
            );
        }
        add_link_contents(&mut motion, description, &joint.child, moving_joints)?;
    }
    Ok(())
}

fn add_geometry(parent: &mut SceneNode3d, instance: &GeometryInstance) -> anyhow::Result<()> {
    if instance.rgba[3] <= 0.0 {
        return Ok(());
    }
    let mut frame = parent.add_group();
    set_pose(&mut frame, &instance.origin);
    if let Geometry::Mesh { path, scale } = &instance.geometry
        && path
            .extension()
            .is_some_and(|extension| extension.eq_ignore_ascii_case("glb"))
    {
        let mut model = kiss3d::loader::gltf::load(path)
            .with_context(|| format!("read GLB mesh {}", path.display()))?;
        model
            .root
            .set_local_scale(scale[0] as f32, scale[1] as f32, scale[2] as f32);
        frame.add_child(model.root);
        return Ok(());
    }
    let mut rendered = match &instance.geometry {
        Geometry::Mesh { path, scale } => {
            let file =
                File::open(path).with_context(|| format!("open STL mesh {}", path.display()))?;
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
                        u32::try_from(index).expect("URDF visual mesh index fits in u32")
                    })
                })
                .collect();
            let gpu = Rc::new(RefCell::new(GpuMesh3d::new(
                vertices, indices, None, None, false,
            )));
            frame.add_mesh(gpu, vector3(scale))
        }
        Geometry::Box { size } => frame.add_cube(size.x as f32, size.y as f32, size.z as f32),
        Geometry::Sphere { radius } => frame.add_sphere(*radius as f32),
        Geometry::Cylinder { radius, length } => {
            let mut cylinder = frame.add_cylinder(*radius as f32, *length as f32);
            // Kiss3d cylinders are Y-aligned; URDF cylinders are Z-aligned.
            cylinder.set_rotation(Quat::from_rotation_x(std::f32::consts::FRAC_PI_2));
            cylinder
        }
    };
    rendered
        .set_color(Color::new(
            instance.rgba[0] as f32,
            instance.rgba[1] as f32,
            instance.rgba[2] as f32,
            instance.rgba[3] as f32,
        ))
        .enable_backface_culling(false);
    Ok(())
}

fn set_pose(node: &mut SceneNode3d, pose: &Isometry3<f64>) {
    let translation = pose.translation.vector;
    let rotation = pose.rotation.quaternion();
    node.set_position(vector3(&translation))
        .set_rotation(Quat::from_xyzw(
            rotation.i as f32,
            rotation.j as f32,
            rotation.k as f32,
            rotation.w as f32,
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

fn vector3(values: &nalgebra::Vector3<f64>) -> Vec3 {
    Vec3::new(values.x as f32, values.y as f32, values.z as f32)
}
