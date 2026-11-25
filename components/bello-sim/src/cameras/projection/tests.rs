use super::*;
use bello_camera_api::{HEIGHT, WIDTH};
#[cfg(target_os = "linux")]
#[test]
#[ignore = "requires offscreen software Mesa; no GPU or CAN devices"]
fn rendered_markers_match_calibrated_pixels_through_pitch_yaw_and_roll() {
    use mujoco_rs::prelude::*;
    use nalgebra::{UnitQuaternion, Vector3};

    // Verify actual rendered pixels, not a second implementation of the
    // assumed frustum. This catches changes in mjr_render's field semantics.
    let _gl = crate::cameras::context::Context::new().unwrap();
    let model = MjModel::from_xml_string(
        r#"<mujoco><visual>
          <headlight ambient="1 1 1" diffuse="0 0 0" specular="0 0 0"/>
          <quality offsamples="0"/></visual><worldbody>
          <geom type="sphere" pos="-.45 -.3 2" size=".03" rgba="1 0 0 1"/>
          <geom type="sphere" pos=".4 -.1 2.5" size=".03" rgba="0 1 0 1"/>
          <geom type="sphere" pos=".1 .5 2.3" size=".03" rgba="0 0 1 1"/>
        </worldbody></mujoco>"#,
    )
    .unwrap();
    let mut data = MjData::new(&model);
    data.forward();
    let mut context = unsafe { MjrContext::new(&model) };
    context.offscreen();
    let mut scene = MjvScene::new(&model, 20);
    let mut camera = MjvCamera::new_free(&model);
    let k = [320., 280., 303., 217.];
    let d = [0.3098908837, 0.1999563911, -0.3083850604, 0.1971900323];
    let markers: [Vector3<f64>; 3] = [
        Vector3::new(-0.45, -0.3, 2.),
        Vector3::new(0.4, -0.1, 2.5),
        Vector3::new(0.1, 0.5, 2.3),
    ];
    for fisheye in [false, true] {
        let p = if fisheye {
            Projection::for_fisheye(640, 480, k, d)
        } else {
            Projection::for_format(640, 480, k)
        }
        .unwrap();
        context.resize_offscreen(p.render_width, p.render_height);
        let viewport = MjrRectangle::new(0, 0, p.render_width as i32, p.render_height as i32);
        let mut pixels = vec![0; p.render_width as usize * p.render_height as usize * 3];
        for (name, angles) in [
            ("home", [0., 0., 0.]),
            ("pitch", [0.22, 0., 0.]),
            ("yaw", [0., 0.3, 0.]),
            ("roll", [0., 0., 0.6]),
            ("combined", [-0.2, -0.3, -0.4]),
        ] {
            let rotation = UnitQuaternion::from_euler_angles(angles[0], angles[1], angles[2]);
            scene.update(
                &mut data,
                &MjvOption::default(),
                &MjvPerturb::default(),
                &mut camera,
            );
            for view in &mut unsafe { scene.ffi_mut() }.camera {
                view.pos = [0.; 3];
                view.forward = (rotation * Vector3::z()).map(|x| x as f32).into();
                view.up = (rotation * -Vector3::y()).map(|x| x as f32).into();
                p.apply_frustum(view);
            }
            scene.render(&viewport, &context);
            context
                .read_pixels(Some(&mut pixels), None, &viewport)
                .unwrap();
            let image = p.sensor_image(&pixels);
            for (channel, marker) in markers.iter().enumerate() {
                let ray = rotation.inverse() * marker;
                let normalized = [ray.x / ray.z, ray.y / ray.z];
                let scale = if fisheye {
                    let radius = normalized[0].hypot(normalized[1]);
                    let theta = radius.atan();
                    let t2 = theta * theta;
                    theta * (1. + t2 * (d[0] + t2 * (d[1] + t2 * (d[2] + t2 * d[3])))) / radius
                } else {
                    1.
                };
                let expected = [
                    k[0] * normalized[0] * scale + k[2],
                    k[1] * normalized[1] * scale + k[3],
                ];
                let mut sum = [0.; 2];
                let mut count = 0;
                for (x, y, color) in image.enumerate_pixels() {
                    if color[channel] > 100 && (0..3).all(|c| c == channel || color[c] < 20) {
                        sum[0] += f64::from(x);
                        sum[1] += f64::from(y);
                        count += 1;
                    }
                }
                assert!(
                    count > 8,
                    "missing marker {channel}, {name}, fisheye={fisheye}"
                );
                let error = (sum[0] / f64::from(count) - expected[0])
                    .hypot(sum[1] / f64::from(count) - expected[1]);
                assert!(
                    error < 1.,
                    "marker {channel}, {name}, fisheye={fisheye}: {error:.3}px error"
                );
            }
        }
    }
}
#[test]
fn raw_fisheye_pixels_round_trip_through_calibrated_rays() {
    let k = [732.978829, 733.716986, 817.852151, 629.197187];
    let d = [0.3098908837, 0.1999563911, -0.3083850604, 0.1971900323];
    for [u, v] in [
        [0., 0.],
        [1599., 1199.],
        [817.852151, 629.197187],
        [1200., 300.],
    ] {
        let [x, y] = fisheye_ray(u, v, k, d).unwrap();
        let r = x.hypot(y);
        let theta = r.atan();
        let t2 = theta * theta;
        let td = theta * (1. + t2 * (d[0] + t2 * (d[1] + t2 * (d[2] + t2 * d[3]))));
        let scale = if r < 1e-12 { 1. } else { td / r };
        assert!((k[0] * x * scale + k[2] - u).abs() < 1e-7);
        assert!((k[1] * y * scale + k[3] - v).abs() < 1e-7);
    }
}
#[test]
fn fisheye_remap_keeps_rgb_order_and_sensor_dimensions() {
    let p = Projection::for_fisheye(160, 120, [73.3, 73.4, 81.8, 62.9], [0.31, 0.2, -0.31, 0.2])
        .unwrap();
    let pixels = [12, 34, 56].repeat(p.render_width as usize * p.render_height as usize);
    let output = p.sensor_image(&pixels);
    assert_eq!(output.dimensions(), (160, 120));
    assert!(output.pixels().all(|p| p.0 == [12, 34, 56]));
}
#[test]
fn asymmetric_focals_and_off_center_principal_point_preserve_pixel_rays() {
    let [fx, fy, cx, cy] = [420., 350., 300., 170.];
    let p = Projection::for_format(WIDTH, HEIGHT, [fx, fy, cx, cy]).unwrap();
    let mut view: crate::ffi::mjvGLCamera = unsafe { std::mem::zeroed() };
    p.apply_frustum(&mut view);
    let near = f64::from(view.frustum_near);
    let top = f64::from(view.frustum_top);
    let bottom = f64::from(view.frustum_bottom);
    // Use the explicit half-width consumed by mjr_render, rather than the
    // viewport-aspect fallback (which only applies when this field is zero).
    let width = 2. * f64::from(view.frustum_width);
    for [x, y, z] in [[0., 0., 1.], [0.2, -0.1, 1.4], [-0.1, 0.2, 0.8]] {
        let rendered_x = (x / z * near - f64::from(view.frustum_center) + width / 2.) / width
            * f64::from(p.render_width)
            - 0.5;
        let rendered_y = (top + y / z * near) / (top - bottom) * f64::from(p.render_height) - 0.5;
        let pixel_x = (rendered_x + 0.5) / p.sx - 0.5;
        let pixel_y = (rendered_y + 0.5) / p.sy - 0.5;
        assert!((pixel_x - (fx * x / z + cx)).abs() < 1e-3);
        assert!((pixel_y - (fy * y / z + cy)).abs() < 1e-3);
    }
}
#[test]
fn configurable_stereo_dimensions_flip_rows_without_changing_intrinsics() {
    let p = Projection::for_format(640, 480, [285., 285., 320., 240.]).unwrap();
    let mut pixels = vec![0; 640 * 480 * 3];
    pixels[..3].copy_from_slice(&[12, 34, 56]);
    let output = p.sensor_image(&pixels);
    assert_eq!(output.dimensions(), (640, 480));
    assert_eq!(output.get_pixel(0, 479).0, [12, 34, 56]);
}
#[test]
fn usb_native_stereo_dimensions_are_supported() {
    let p = Projection::for_format(1600, 1200, [711.5, 711.5, 772.7, 611.4]).unwrap();
    assert_eq!((p.render_width, p.render_height), (1600, 1200));
}
#[test]
fn output_flips_opengl_rows_and_has_exact_capture_dimensions() {
    let p = Projection::for_format(WIDTH, HEIGHT, [320., 320., 320., 180.]).unwrap();
    let mut pixels = vec![0; WIDTH as usize * HEIGHT as usize * 3];
    pixels[..3].copy_from_slice(&[10, 20, 30]);
    let output = p.sensor_image(&pixels);
    assert_eq!(output.dimensions(), (WIDTH, HEIGHT));
    assert_eq!(output.get_pixel(0, HEIGHT - 1).0, [10, 20, 30]);
    assert_eq!(output.get_pixel(0, 0).0, [0, 0, 0]);
}
