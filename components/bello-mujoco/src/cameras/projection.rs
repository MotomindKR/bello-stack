//! Calibrated sensor output over MuJoCo's square-pixel OpenGL frustum.
use anyhow::{Result, ensure};
#[cfg(test)]
use bello_camera_api::{HEIGHT, WIDTH};
use image::RgbImage;
use rayon::prelude::*;
use std::sync::OnceLock;
static REMAP_POOL: OnceLock<rayon::ThreadPool> = OnceLock::new();
pub struct Projection {
    pub width: u32,
    pub height: u32,
    output_width: u32,
    output_height: u32,
    sx: f64,
    sy: f64,
    focal: f64,
    cx: f64,
    cy: f64,
    map: Option<Vec<Sample>>,
}
struct Sample {
    index: usize,
    dx: f32,
    dy: f32,
}

fn fisheye_ray(u: f64, v: f64, [fx, fy, cx, cy]: [f64; 4], d: [f64; 4]) -> Result<[f64; 2]> {
    let x = (u - cx) / fx;
    let y = (v - cy) / fy;
    let rd = x.hypot(y);
    if rd < 1e-12 {
        return Ok([0., 0.]);
    }
    let mut theta = rd.min(1.5);
    for _ in 0..16 {
        let t2 = theta * theta;
        let p = 1. + t2 * (d[0] + t2 * (d[1] + t2 * (d[2] + t2 * d[3])));
        let derivative =
            1. + t2 * (3. * d[0] + t2 * (5. * d[1] + t2 * (7. * d[2] + t2 * 9. * d[3])));
        ensure!(
            derivative.is_finite() && derivative > 1e-8,
            "fisheye calibration is not invertible"
        );
        theta -= (theta * p - rd) / derivative;
    }
    ensure!(
        (0. ..std::f64::consts::FRAC_PI_2 - 0.01).contains(&theta),
        "fisheye ray crosses pinhole horizon"
    );
    let scale = theta.tan() / rd;
    Ok([x * scale, y * scale])
}

impl Projection {
    #[cfg(test)]
    pub fn new(intrinsics: [f64; 4]) -> Result<Self> {
        Self::for_format(WIDTH, HEIGHT, intrinsics)
    }
    pub fn for_format(
        output_width: u32,
        output_height: u32,
        [fx, fy, cx, cy]: [f64; 4],
    ) -> Result<Self> {
        let focal = fx.max(fy);
        let sx = focal / fx;
        let sy = focal / fy;
        let width = (f64::from(output_width) * sx).ceil() as u32;
        let height = (f64::from(output_height) * sy).ceil() as u32;
        ensure!(
            width <= 3840 && height <= 2160,
            "camera focal aspect requires offscreen rendering above 3840x2160"
        );
        Ok(Self {
            width,
            height,
            output_width,
            output_height,
            sx,
            sy,
            focal,
            cx: (cx + 0.5) * sx,
            cy: (cy + 0.5) * sy,
            map: None,
        })
    }
    /// Render a perspective overscan, then map each original sensor pixel to
    /// its calibrated fisheye ray. The stream remains in raw sensor geometry.
    pub fn for_fisheye(
        output_width: u32,
        output_height: u32,
        k: [f64; 4],
        d: [f64; 4],
    ) -> Result<Self> {
        let focal = k[0].max(k[1]);
        let mut rays = Vec::with_capacity(output_width as usize * output_height as usize);
        let (mut low, mut high) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
        for v in 0..output_height {
            for u in 0..output_width {
                let ray = fisheye_ray(f64::from(u), f64::from(v), k, d)?;
                for a in 0..2 {
                    low[a] = low[a].min(ray[a]);
                    high[a] = high[a].max(ray[a]);
                }
                rays.push(ray);
            }
        }
        let width = ((high[0] - low[0]) * focal).ceil() as u32 + 3;
        let height = ((high[1] - low[1]) * focal).ceil() as u32 + 3;
        ensure!(
            width <= 3840 && height <= 2160,
            "fisheye overscan exceeds 3840x2160"
        );
        let map = rays
            .into_iter()
            .map(|ray| {
                let x = (ray[0] - low[0]) * focal + 1.;
                let y = f64::from(height - 1) - ((ray[1] - low[1]) * focal + 1.);
                let x0 = x.floor() as usize;
                let y0 = y.floor() as usize;
                Sample {
                    index: (y0 * width as usize + x0) * 3,
                    dx: (x - x0 as f64) as f32,
                    dy: (y - y0 as f64) as f32,
                }
            })
            .collect();
        Ok(Self {
            width,
            height,
            output_width,
            output_height,
            sx: 1.,
            sy: 1.,
            focal,
            cx: 1.5 - low[0] * focal,
            cy: 1.5 - low[1] * focal,
            map: Some(map),
        })
    }
    pub fn apply(&self, view: &mut crate::ffi::mjvGLCamera) {
        let near = 0.005;
        view.frustum_near = near as f32;
        view.frustum_far = 20.;
        view.orthographic = 0;
        // MuJoCo 3.12's classic renderer uses frustum_width as a HALF-width:
        // glFrustum(center - width, center + width, ...). Supplying the full
        // width halves the horizontal focal length and breaks lens correction.
        // Overscan/resampling preserves independent fx/fy at sensor resolution.
        view.frustum_width = (0.5 * f64::from(self.width) * near / self.focal) as f32;
        view.frustum_center = ((f64::from(self.width) / 2. - self.cx) * near / self.focal) as f32;
        view.frustum_top = (self.cy * near / self.focal) as f32;
        view.frustum_bottom = (-(f64::from(self.height) - self.cy) * near / self.focal) as f32;
    }
    pub fn image(&self, pixels: &[u8]) -> RgbImage {
        let mut output = RgbImage::new(self.output_width, self.output_height);
        if let Some(map) = &self.map {
            let stride = self.width as usize * 3;
            let pool = REMAP_POOL.get_or_init(|| {
                rayon::ThreadPoolBuilder::new()
                    .num_threads(4)
                    .thread_name(|n| format!("fisheye-map-{n}"))
                    .build()
                    .expect("create bounded fisheye remap workers")
            });
            let tile = self.output_width as usize * 32;
            pool.install(|| {
                output
                    .as_mut()
                    .par_chunks_mut(tile * 3)
                    .zip(map.par_chunks(tile))
                    .for_each(|(output, map)| {
                        for (pixel, sample) in output.chunks_exact_mut(3).zip(map) {
                            for (c, value) in pixel.iter_mut().enumerate() {
                                let i = sample.index + c;
                                let top = f32::from(pixels[i]) * (1. - sample.dx)
                                    + f32::from(pixels[i + 3]) * sample.dx;
                                let bottom = f32::from(pixels[i + stride]) * (1. - sample.dx)
                                    + f32::from(pixels[i + stride + 3]) * sample.dx;
                                *value =
                                    (top * (1. - sample.dy) + bottom * sample.dy).round() as u8;
                            }
                        }
                    })
            });
            return output;
        }
        if self.sx == 1. && self.sy == 1. {
            let row = self.output_width as usize * 3;
            for y in 0..self.output_height as usize {
                output.as_mut()[y * row..(y + 1) * row].copy_from_slice(
                    &pixels[(self.output_height as usize - 1 - y) * row
                        ..(self.output_height as usize - y) * row],
                );
            }
            return output;
        }
        for y in 0..self.output_height {
            let py = (f64::from(self.height) - 0.5 - (f64::from(y) + 0.5) * self.sy)
                .clamp(0., f64::from(self.height - 1));
            let y0 = py.floor() as u32;
            let y1 = (y0 + 1).min(self.height - 1);
            let dy = py - f64::from(y0);
            for x in 0..self.output_width {
                let px =
                    ((f64::from(x) + 0.5) * self.sx - 0.5).clamp(0., f64::from(self.width - 1));
                let x0 = px.floor() as u32;
                let x1 = (x0 + 1).min(self.width - 1);
                let dx = px - f64::from(x0);
                for c in 0..3 {
                    let at = |x: u32, y: u32| {
                        f64::from(pixels[(y as usize * self.width as usize + x as usize) * 3 + c])
                    };
                    let top = at(x0, y0) * (1. - dx) + at(x1, y0) * dx;
                    let bottom = at(x0, y1) * (1. - dx) + at(x1, y1) * dx;
                    output.get_pixel_mut(x, y)[c] = (top * (1. - dy) + bottom * dy).round() as u8;
                }
            }
        }
        output
    }
}
#[cfg(test)]
mod tests {
    use super::*;
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
            context.resize_offscreen(p.width, p.height);
            let viewport = MjrRectangle::new(0, 0, p.width as i32, p.height as i32);
            let mut pixels = vec![0; p.width as usize * p.height as usize * 3];
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
                    p.apply(view);
                }
                scene.render(&viewport, &context);
                context
                    .read_pixels(Some(&mut pixels), None, &viewport)
                    .unwrap();
                let image = p.image(&pixels);
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
        let p =
            Projection::for_fisheye(160, 120, [73.3, 73.4, 81.8, 62.9], [0.31, 0.2, -0.31, 0.2])
                .unwrap();
        let pixels = [12, 34, 56].repeat(p.width as usize * p.height as usize);
        let output = p.image(&pixels);
        assert_eq!(output.dimensions(), (160, 120));
        assert!(output.pixels().all(|p| p.0 == [12, 34, 56]));
    }
    #[test]
    fn asymmetric_focals_and_off_center_principal_point_preserve_pixel_rays() {
        let [fx, fy, cx, cy] = [420., 350., 300., 170.];
        let p = Projection::new([fx, fy, cx, cy]).unwrap();
        let mut view: crate::ffi::mjvGLCamera = unsafe { std::mem::zeroed() };
        p.apply(&mut view);
        let near = f64::from(view.frustum_near);
        let top = f64::from(view.frustum_top);
        let bottom = f64::from(view.frustum_bottom);
        // Use the explicit half-width consumed by mjr_render, rather than the
        // viewport-aspect fallback (which only applies when this field is zero).
        let width = 2. * f64::from(view.frustum_width);
        for [x, y, z] in [[0., 0., 1.], [0.2, -0.1, 1.4], [-0.1, 0.2, 0.8]] {
            let rendered_x = (x / z * near - f64::from(view.frustum_center) + width / 2.) / width
                * f64::from(p.width)
                - 0.5;
            let rendered_y = (top + y / z * near) / (top - bottom) * f64::from(p.height) - 0.5;
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
        let output = p.image(&pixels);
        assert_eq!(output.dimensions(), (640, 480));
        assert_eq!(output.get_pixel(0, 479).0, [12, 34, 56]);
    }
    #[test]
    fn usb_native_stereo_dimensions_are_supported() {
        let p = Projection::for_format(1600, 1200, [711.5, 711.5, 772.7, 611.4]).unwrap();
        assert_eq!((p.width, p.height), (1600, 1200));
    }
    #[test]
    fn output_flips_opengl_rows_and_has_exact_capture_dimensions() {
        let p = Projection::new([320., 320., 320., 180.]).unwrap();
        let mut pixels = vec![0; WIDTH as usize * HEIGHT as usize * 3];
        pixels[..3].copy_from_slice(&[10, 20, 30]);
        let output = p.image(&pixels);
        assert_eq!(output.dimensions(), (WIDTH, HEIGHT));
        assert_eq!(output.get_pixel(0, HEIGHT - 1).0, [10, 20, 30]);
        assert_eq!(output.get_pixel(0, 0).0, [0, 0, 0]);
    }
}
