//! Calibrated pinhole output over MuJoCo's square-pixel OpenGL frustum.
use anyhow::{Result, ensure};
#[cfg(test)]
use bello_camera_api::{HEIGHT, WIDTH};
use image::RgbImage;
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
            width <= 1920 && height <= 1080,
            "camera focal aspect requires offscreen rendering above 1920x1080"
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
        })
    }
    pub fn apply(&self, view: &mut crate::ffi::mjvGLCamera) {
        let near = 0.005;
        view.frustum_near = near as f32;
        view.frustum_far = 20.;
        view.orthographic = 0;
        // frustum_width is not used by the classic renderer. Overscan/resampling
        // preserves independent fx/fy without reducing the requested resolution.
        view.frustum_width = (f64::from(self.width) * near / self.focal) as f32;
        view.frustum_center = ((f64::from(self.width) / 2. - self.cx) * near / self.focal) as f32;
        view.frustum_top = (self.cy * near / self.focal) as f32;
        view.frustum_bottom = (-(f64::from(self.height) - self.cy) * near / self.focal) as f32;
    }
    pub fn image(&self, pixels: &[u8]) -> RgbImage {
        let mut output = RgbImage::new(self.output_width, self.output_height);
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
    #[test]
    fn asymmetric_focals_and_off_center_principal_point_preserve_pixel_rays() {
        let [fx, fy, cx, cy] = [420., 350., 300., 170.];
        let p = Projection::new([fx, fy, cx, cy]).unwrap();
        let mut view: crate::ffi::mjvGLCamera = unsafe { std::mem::zeroed() };
        p.apply(&mut view);
        let near = f64::from(view.frustum_near);
        let top = f64::from(view.frustum_top);
        let bottom = f64::from(view.frustum_bottom);
        let width = (top - bottom) * f64::from(p.width) / f64::from(p.height);
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
