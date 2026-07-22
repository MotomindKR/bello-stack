use std::fs;

use bello_visualizer::Model;
use mjcf_rs::body::{GeomType, JointType};
use mjcf_rs::extras::ActuatorKind;
use tempfile::tempdir;

#[test]
fn visualizer_uses_the_mjcf_rs_ast_directly() {
    let directory = tempdir().unwrap();
    fs::write(
        directory.path().join("visual.stl"),
        b"solid empty\nendsolid empty\n",
    )
    .unwrap();
    let path = directory.path().join("model.xml");
    fs::write(
        &path,
        r#"
          <mujoco model="fixture">
            <compiler angle="radian"/>
            <asset><mesh name="visual" file="visual.stl" scale="2 3 4"/></asset>
            <worldbody>
              <body name="root" pos="1 2 3" quat="0.5 0.5 0.5 0.5">
                <freejoint name="floating"/>
                <geom name="mesh" type="mesh" mesh="visual"/>
                <body name="child">
                  <joint name="hinge" axis="0 2 0" range="-1 1"/>
                  <geom name="box" type="box" size="1 2 3"/>
                </body>
              </body>
            </worldbody>
            <actuator>
              <position name="hinge_position" joint="hinge" ctrlrange="-1 1"/>
            </actuator>
          </mujoco>
        "#,
    )
    .unwrap();

    let model = Model::from_file(path).unwrap();
    assert_eq!(model.assets.mesh("visual").unwrap().scale, [2.0, 3.0, 4.0]);
    let root = &model.bodies[model.body_id_by_name("root").unwrap()].body;
    assert_eq!(root.pose.translation.to_array(), [1.0, 2.0, 3.0]);
    assert_eq!(root.pose.rotation.to_array(), [0.5, 0.5, 0.5, 0.5]);
    assert_eq!(root.joints[0].type_, JointType::Free);
    assert_eq!(root.geoms[0].type_, GeomType::Mesh);
    let child = &model.bodies[model.body_id_by_name("child").unwrap()].body;
    assert_eq!(child.joints[0].axis, [0.0, 2.0, 0.0]);
    assert_eq!(child.geoms[0].size, [1.0, 2.0, 3.0]);
    assert_eq!(model.actuators[0].kind, ActuatorKind::Position);
}

#[test]
fn mjcf_rs_resolves_defaults_without_a_bello_parser() {
    let model = Model::from_str(
        r#"
          <mujoco>
            <compiler angle="radian"/>
            <default><joint axis="0 1 0" damping="2"/></default>
            <worldbody><body name="body"><joint name="joint"/></body></worldbody>
          </mujoco>
        "#,
        "/tmp",
    )
    .unwrap();
    let (body, joint) = model.joint_by_name("joint").unwrap();
    let joint = &model.bodies[body].body.joints[joint];
    assert_eq!(joint.axis, [0.0, 1.0, 0.0]);
    assert_eq!(joint.damping, 2.0);
}
