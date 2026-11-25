use anyhow::{Context, Result};
use gst::prelude::*;
use image::RgbImage;

pub struct Encoder {
    pipeline: gst::Pipeline,
    input: gst_app::AppSrc,
    output: gst_app::AppSink,
}

impl Encoder {
    pub fn new() -> Result<Self> {
        gst::init()?;
        // Full-range BT.601 matches the YCbCr convention used by JPEG decoders.
        let pipeline = gst::parse::launch(
            "appsrc name=rgb format=time ! videoconvert ! video/x-raw,format=I420,colorimetry=1:4:0:0 ! jpegenc quality=90 ! appsink name=jpeg sync=false",
        )?.downcast::<gst::Pipeline>().unwrap();
        let input = pipeline.by_name("rgb").unwrap().downcast().unwrap();
        let output = pipeline.by_name("jpeg").unwrap().downcast().unwrap();
        let encoder = Self {
            pipeline,
            input,
            output,
        };
        encoder.pipeline.set_state(gst::State::Playing)?;
        Ok(encoder)
    }

    pub fn encode(&self, image: RgbImage) -> Result<Vec<u8>> {
        let (width, height) = image.dimensions();
        let caps = gst::Caps::builder("video/x-raw")
            .field("format", "RGB")
            .field("width", width as i32)
            .field("height", height as i32)
            .field(
                "framerate",
                gst::Fraction::new(bello_camera_api::FPS as i32, 1),
            )
            .build();
        self.input.set_caps(Some(&caps));
        let row = width as usize * 3;
        let stride = row.next_multiple_of(4);
        let raw = image.into_raw();
        let bytes = if stride == row {
            raw
        } else {
            let mut aligned = vec![0; stride * height as usize];
            for (src, dst) in raw.chunks_exact(row).zip(aligned.chunks_exact_mut(stride)) {
                dst[..row].copy_from_slice(src);
            }
            aligned
        };
        self.input.push_buffer(gst::Buffer::from_mut_slice(bytes))?;
        let sample = self
            .output
            .try_pull_sample(gst::ClockTime::from_seconds(3))
            .context("GStreamer JPEG encoder produced no frame")?;
        Ok(sample
            .buffer()
            .context("missing JPEG buffer")?
            .map_readable()?
            .as_slice()
            .to_vec())
    }
}

impl Drop for Encoder {
    fn drop(&mut self) {
        let _ = self.pipeline.set_state(gst::State::Null);
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn jpeg_preserves_dimensions_and_rgb_order_with_unaligned_rows() {
        let image = RgbImage::from_pixel(1601, 1200, image::Rgb([180, 60, 25]));
        let bytes = Encoder::new().unwrap().encode(image).unwrap();
        let decoded = image::load_from_memory(&bytes).unwrap().to_rgb8();
        assert_eq!(decoded.dimensions(), (1601, 1200));
        for (actual, expected) in decoded
            .get_pixel(800, 600)
            .0
            .into_iter()
            .zip([180i16, 60, 25])
        {
            assert!(
                (i16::from(actual) - expected).abs() < 5,
                "{actual} vs {expected}"
            );
        }
    }
}
