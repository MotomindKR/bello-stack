//! Calibrated sensor output over MuJoCo's square-pixel OpenGL frustum.
use anyhow::{Result, ensure};
use image::RgbImage;
use rayon::prelude::*;
use std::sync::OnceLock;
static REMAP_POOL: OnceLock<rayon::ThreadPool> = OnceLock::new();
pub struct Projection {
    /// Offscreen render dimensions, including any perspective overscan.
    pub render_width: u32,
    pub render_height: u32,
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
            render_width: width,
            render_height: height,
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
            render_width: width,
            render_height: height,
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
    pub fn apply_frustum(&self, view: &mut crate::ffi::mjvGLCamera) {
        let near = 0.005;
        view.frustum_near = near as f32;
        view.frustum_far = 20.;
        view.orthographic = 0;
        // MuJoCo 3.12's classic renderer uses frustum_width as a HALF-width:
        // glFrustum(center - width, center + width, ...). Supplying the full
        // width halves the horizontal focal length and breaks lens correction.
        // Overscan/resampling preserves independent fx/fy at sensor resolution.
        view.frustum_width = (0.5 * f64::from(self.render_width) * near / self.focal) as f32;
        view.frustum_center =
            ((f64::from(self.render_width) / 2. - self.cx) * near / self.focal) as f32;
        view.frustum_top = (self.cy * near / self.focal) as f32;
        view.frustum_bottom =
            (-(f64::from(self.render_height) - self.cy) * near / self.focal) as f32;
    }
    pub fn sensor_image(&self, pixels: &[u8]) -> RgbImage {
        let mut output = RgbImage::new(self.output_width, self.output_height);
        if let Some(map) = &self.map {
            let stride = self.render_width as usize * 3;
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
                        let (output, _) = output.as_chunks_mut::<3>();
                        for (pixel, sample) in output.iter_mut().zip(map) {
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
            let py = (f64::from(self.render_height) - 0.5 - (f64::from(y) + 0.5) * self.sy)
                .clamp(0., f64::from(self.render_height - 1));
            let y0 = py.floor() as u32;
            let y1 = (y0 + 1).min(self.render_height - 1);
            let dy = py - f64::from(y0);
            for x in 0..self.output_width {
                let px = ((f64::from(x) + 0.5) * self.sx - 0.5)
                    .clamp(0., f64::from(self.render_width - 1));
                let x0 = px.floor() as u32;
                let x1 = (x0 + 1).min(self.render_width - 1);
                let dx = px - f64::from(x0);
                for c in 0..3 {
                    let at = |x: u32, y: u32| {
                        f64::from(
                            pixels[(y as usize * self.render_width as usize + x as usize) * 3 + c],
                        )
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
mod tests;
