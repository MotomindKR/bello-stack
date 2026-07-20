use std::fs;

use bello_visualizer::mjcf::{GeometryKind, JointKind, Model, ModelError};
use tempfile::tempdir;

#[test]
fn parses_hierarchy_mesh_primitive_and_wxyz_transforms() {
    let directory = tempdir().unwrap();
    fs::write(
        directory.path().join("visual.stl"),
        b"solid empty\nendsolid empty\n",
    )
    .unwrap();
    let path = directory.path().join("model.xml");
    let xml = r#"
      <mujoco model="fixture">
        <compiler angle="radian"/>
        <asset><mesh name="visual" file="visual.stl" scale="2 3 4"/></asset>
        <worldbody>
          <body name="root" pos="1 2 3" quat="0.5 0.5 0.5 0.5">
            <freejoint name="floating"/>
            <geom name="mesh" type="mesh" mesh="visual" pos="0 1 0" quat="1 0 0 0"/>
            <body name="child" pos="0 0 1">
              <joint name="hinge" type="hinge" pos="0 0.2 0" axis="0 2 0"/>
              <geom name="box" type="box" size="1 2 3"/>
              <geom name="sphere" type="sphere" size="0.4"/>
              <geom name="cylinder" type="cylinder" size="0.2 0.8"/>
            </body>
          </body>
        </worldbody>
      </mujoco>
    "#;
    let model = Model::parse(xml, &path).unwrap();
    assert_eq!(model.meshes["visual"].scale, [2.0, 3.0, 4.0]);
    let root = &model.world.children[0];
    assert_eq!(root.transform.translation, [1.0, 2.0, 3.0]);
    assert_eq!(root.transform.orientation_xyzw, [0.5, 0.5, 0.5, 0.5]);
    assert_eq!(root.joints[0].kind, JointKind::Free);
    assert!(matches!(root.geometries[0].kind, GeometryKind::Mesh { .. }));
    let child = &root.children[0];
    assert_eq!(child.joints[0].axis, [0.0, 1.0, 0.0]);
    assert!(matches!(
        child.geometries[0].kind,
        GeometryKind::Box {
            half_extents: [1.0, 2.0, 3.0]
        }
    ));
    assert!(matches!(
        child.geometries[2].kind,
        GeometryKind::Cylinder {
            radius: 0.2,
            half_length: 0.8
        }
    ));
}

#[test]
fn ignores_transparent_collision_geometries() {
    let model = Model::parse(
        r#"<mujoco><worldbody><geom name="collision" type="capsule" rgba="1 1 1 0"/></worldbody></mujoco>"#,
        "/tmp/model.xml",
    )
    .unwrap();
    assert!(model.world.geometries.is_empty());
}

#[test]
fn parses_direct_and_fixed_tendon_position_actuator_limits() {
    let model = Model::parse(
        r#"<mujoco><worldbody><body name="body">
          <joint name="direct" range="-1 1"/>
          <joint name="pitch" range="-2 2"/>
          <joint name="roll" range="-0.5 0.5"/>
        </body></worldbody><tendon>
          <fixed name="coupled"><joint joint="pitch" coef="-1"/><joint joint="roll" coef="1"/></fixed>
        </tendon><actuator>
          <position name="direct_position" joint="direct" ctrlrange="-1 1" user="0.25"/>
          <position name="coupled_position" tendon="coupled" ctrlrange="-2.5 2.5" user="-0.5"/>
        </actuator></mujoco>"#,
        "/tmp/model.xml",
    )
    .unwrap();
    assert_eq!(
        model.position_actuators["direct"].control_range,
        [-1.0, 1.0]
    );
    assert_eq!(model.position_actuators["direct"].default_position, 0.25);
    assert_eq!(
        model.position_actuators["coupled"].control_range,
        [-2.5, 2.5]
    );
    assert_eq!(model.position_actuators["coupled"].default_position, -0.5);
}

#[test]
fn rejects_unflattened_and_unsupported_features() {
    for (xml, expected) in [
        (
            r#"<mujoco><include file="part.xml"/><worldbody/></mujoco>"#,
            "include",
        ),
        (r#"<mujoco><default/><worldbody/></mujoco>"#, "default"),
        (
            r#"<mujoco><worldbody><body name="b"><joint name="j" type="ball"/></body></worldbody></mujoco>"#,
            "ball joint",
        ),
        (
            r#"<mujoco><worldbody/><tendon><spatial name="rope"/></tendon></mujoco>"#,
            "spatial",
        ),
        (
            r#"<mujoco><worldbody><geom type="plane" size="1 1 1"/></worldbody></mujoco>"#,
            "supported visual geoms",
        ),
        (
            r#"<mujoco><worldbody><geom type="cylinder" size="1 1" fromto="0 0 0 0 0 1"/></worldbody></mujoco>"#,
            "fromto",
        ),
    ] {
        let error = Model::parse(xml, "/tmp/model.xml").unwrap_err();
        assert!(error.to_string().contains(expected), "{error}");
    }
}

#[test]
fn rejects_missing_mesh_duplicate_names_and_invalid_quaternions() {
    let missing = Model::parse(
        r#"<mujoco><asset><mesh name="m" file="missing.stl"/></asset><worldbody/></mujoco>"#,
        "/tmp/model.xml",
    )
    .unwrap_err();
    assert!(matches!(missing, ModelError::MissingMesh { .. }));

    let duplicate = Model::parse(
        r#"<mujoco><worldbody><body name="same"/><body name="same"/></worldbody></mujoco>"#,
        "/tmp/model.xml",
    )
    .unwrap_err();
    assert!(matches!(duplicate, ModelError::DuplicateName(_)));

    let invalid = Model::parse(
        r#"<mujoco><worldbody><body name="root" quat="0 0 0 0"/></worldbody></mujoco>"#,
        "/tmp/model.xml",
    )
    .unwrap_err();
    assert!(matches!(invalid, ModelError::InvalidTransform(_)));
}
