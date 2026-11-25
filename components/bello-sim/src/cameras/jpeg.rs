//! Optional libjpeg-turbo acceleration; no native build dependency or model change.
use anyhow::{Context, Result, ensure};
use image::{RgbImage, codecs::jpeg::JpegEncoder};
use libloading::Library;
use std::ffi::{CStr, c_char, c_int, c_uchar, c_ulong, c_void};

type Init = unsafe extern "C" fn() -> *mut c_void;
type Compress = unsafe extern "C" fn(
    *mut c_void,
    *const c_uchar,
    c_int,
    c_int,
    c_int,
    c_int,
    *mut *mut c_uchar,
    *mut c_ulong,
    c_int,
    c_int,
    c_int,
) -> c_int;
type Free = unsafe extern "C" fn(*mut c_uchar);
type Destroy = unsafe extern "C" fn(*mut c_void) -> c_int;
type Error = unsafe extern "C" fn(*mut c_void) -> *const c_char;

struct Native {
    _library: Library,
    handle: *mut c_void,
    compress: Compress,
    free: Free,
    destroy: Destroy,
    error: Error,
}
impl Native {
    fn new() -> Result<Self> {
        #[cfg(target_os = "linux")]
        let name = "libturbojpeg.so.0";
        #[cfg(target_os = "macos")]
        let name = "libturbojpeg.0.dylib";
        // The stable TurboJPEG 1.2/2.0 ABI is loaded explicitly. This library
        // remains alive until after its handle has been destroyed in Drop.
        let library = unsafe { Library::new(name) }?;
        let init = unsafe { *library.get::<Init>(b"tjInitCompress\0")? };
        let compress = unsafe { *library.get::<Compress>(b"tjCompress2\0")? };
        let free = unsafe { *library.get::<Free>(b"tjFree\0")? };
        let destroy = unsafe { *library.get::<Destroy>(b"tjDestroy\0")? };
        let error = unsafe { *library.get::<Error>(b"tjGetErrorStr2\0")? };
        let handle = unsafe { init() };
        ensure!(!handle.is_null(), "TurboJPEG compressor allocation failed");
        Ok(Self {
            _library: library,
            handle,
            compress,
            free,
            destroy,
            error,
        })
    }
    fn encode(&mut self, image: &RgbImage) -> Result<Vec<u8>> {
        let mut output = std::ptr::null_mut();
        let mut size: c_ulong = 0;
        // RGB packed pixels (0), automatic row stride, 4:2:0 (2), quality 90,
        // fast DCT (2048). TurboJPEG owns and reallocates the output buffer.
        let result = unsafe {
            (self.compress)(
                self.handle,
                image.as_raw().as_ptr(),
                image.width() as c_int,
                0,
                image.height() as c_int,
                0,
                &mut output,
                &mut size,
                2,
                90,
                2048,
            )
        };
        if result != 0 {
            let message = unsafe { (self.error)(self.handle) };
            let message = if message.is_null() {
                "unknown JPEG error".into()
            } else {
                unsafe { CStr::from_ptr(message) }
                    .to_string_lossy()
                    .into_owned()
            };
            if !output.is_null() {
                unsafe { (self.free)(output) };
            }
            anyhow::bail!("TurboJPEG: {message}");
        }
        ensure!(!output.is_null() && size > 0, "empty native JPEG");
        let jpeg = unsafe { std::slice::from_raw_parts(output, size as usize) }.to_vec();
        unsafe { (self.free)(output) };
        Ok(jpeg)
    }
}
impl Drop for Native {
    fn drop(&mut self) {
        unsafe { (self.destroy)(self.handle) };
    }
}

pub struct Encoder(Option<Native>);
impl Encoder {
    pub fn new() -> Self {
        let native = Native::new();
        match &native {
            Ok(_) => eprintln!("Camera JPEG encoder: libjpeg-turbo"),
            Err(e) => eprintln!("Camera JPEG encoder: Rust fallback ({e})"),
        }
        Self(native.ok())
    }
    pub fn encode(&mut self, image: &RgbImage) -> Result<Vec<u8>> {
        ensure!(image.width() > 0 && image.height() > 0, "empty RGB image");
        if let Some(native) = &mut self.0 {
            return native.encode(image);
        }
        let mut jpeg = Vec::with_capacity(65536);
        JpegEncoder::new_with_quality(&mut jpeg, 90)
            .encode_image(image)
            .context("encode JPEG")?;
        Ok(jpeg)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn native_size_jpeg_preserves_dimensions_and_rgb_order() {
        let image = RgbImage::from_pixel(1600, 1200, image::Rgb([180, 60, 25]));
        let bytes = Encoder::new().encode(&image).unwrap();
        let decoded = image::load_from_memory(&bytes).unwrap().to_rgb8();
        assert_eq!(decoded.dimensions(), (1600, 1200));
        for (actual, expected) in decoded
            .get_pixel(800, 600)
            .0
            .into_iter()
            .zip([180i16, 60, 25])
        {
            assert!((i16::from(actual) - expected).abs() < 5);
        }
    }
}
