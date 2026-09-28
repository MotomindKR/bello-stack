//! Offline visual inspection: URDF OUTPUT.png [joint=position_radians ...].
use bello_visualizer::{RobotDescription, scene::ModelScene};
use kiss3d::prelude::*;
use std::collections::BTreeMap;

#[kiss3d::main]
async fn main() {
    let mut args = std::env::args().skip(1);
    let urdf = args.next().expect("URDF path");
    let output = args.next().expect("output PNG path");
    let positions: BTreeMap<String, f64> = args
        .map(|argument| {
            let (name, position) = argument.split_once('=').expect("joint=position");
            (
                name.to_owned(),
                position.parse().expect("position in radians"),
            )
        })
        .collect();
    let description = RobotDescription::load(std::path::Path::new(&urdf)).unwrap();
    let mut surface = OffscreenSurface::new(900, 900).await;
    surface.set_shadow_resolution(512);
    surface.set_background_color(Color::new(0.94, 0.95, 0.96, 1.0));
    surface.set_ambient(0.5);
    let mut scene = SceneNode3d::empty();
    scene
        .add_light(Light::point(20.0).with_intensity(3.0))
        .set_position(Vec3::new(2.0, -3.0, 3.0));
    let mut model = ModelScene::build(&mut scene, &description).unwrap();
    model.set_joint_positions(&positions);
    let mut camera = OrbitCamera3d::new(Vec3::new(1.6, -2.0, 0.8), Vec3::new(0.0, 0.0, 0.0));
    camera.set_up_axis_dir(Vec3::Z);
    surface.render_3d(&mut scene, &mut camera).await;
    surface.snap_image().save(output).unwrap();
}
