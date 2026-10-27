//! A private, offscreen OpenGL context. Called only in the renderer child process.
use anyhow::{Result, ensure};

#[cfg(target_os = "macos")]
mod platform {
    use super::*;
    use std::ffi::c_void;
    #[link(name = "OpenGL", kind = "framework")]
    unsafe extern "C" {
        fn CGLChoosePixelFormat(
            attributes: *const u32,
            format: *mut *mut c_void,
            count: *mut i32,
        ) -> i32;
        fn CGLCreateContext(
            format: *mut c_void,
            share: *mut c_void,
            context: *mut *mut c_void,
        ) -> i32;
        fn CGLSetCurrentContext(context: *mut c_void) -> i32;
        fn CGLDestroyPixelFormat(format: *mut c_void) -> i32;
        fn CGLDestroyContext(context: *mut c_void) -> i32;
    }
    pub struct Context(*mut c_void);
    impl Context {
        pub fn new() -> Result<Self> {
            let mut format = std::ptr::null_mut();
            let mut count = 0;
            // Accelerated legacy OpenGL, RGB/depth 24 bits; MuJoCo owns its FBO.
            ensure!(
                unsafe {
                    CGLChoosePixelFormat([73, 8, 24, 12, 24, 0].as_ptr(), &mut format, &mut count)
                } == 0
                    && !format.is_null(),
                "no offscreen CGL pixel format"
            );
            let mut context = std::ptr::null_mut();
            let result = unsafe { CGLCreateContext(format, std::ptr::null_mut(), &mut context) };
            unsafe { CGLDestroyPixelFormat(format) };
            ensure!(
                result == 0 && !context.is_null(),
                "CGL context creation failed"
            );
            let owner = Self(context);
            ensure!(
                unsafe { CGLSetCurrentContext(context) } == 0,
                "CGL make-current failed"
            );
            Ok(owner)
        }
    }
    impl Drop for Context {
        fn drop(&mut self) {
            unsafe {
                CGLSetCurrentContext(std::ptr::null_mut());
                CGLDestroyContext(self.0);
            }
        }
    }
}
#[cfg(target_os = "linux")]
mod platform {
    use super::*;
    use anyhow::Context as _;
    use khronos_egl as egl;
    pub struct Context {
        egl: egl::DynamicInstance<egl::EGL1_5>,
        display: egl::Display,
        context: Option<egl::Context>,
    }
    impl Context {
        pub fn new() -> Result<Self> {
            let egl = unsafe {
                egl::DynamicInstance::<egl::EGL1_5>::load_required_from_filename("libEGL.so.1")?
            };
            let address = egl
                .get_proc_address("eglQueryDevicesEXT")
                .context("EGL device enumeration unavailable")?;
            let query: unsafe extern "system" fn(i32, *mut *mut std::ffi::c_void, *mut i32) -> u32 =
                unsafe { std::mem::transmute(address) };
            let mut devices = [std::ptr::null_mut(); 16];
            let mut count = 0;
            ensure!(
                unsafe { query(16, devices.as_mut_ptr(), &mut count) } != 0 && count > 0,
                "no EGL rendering device"
            );
            let display =
                unsafe { egl.get_platform_display(0x313f, devices[0], &[egl::ATTRIB_NONE])? };
            egl.initialize(display)?;
            let mut owner = Self {
                egl,
                display,
                context: None,
            };
            owner.egl.bind_api(egl::OPENGL_API)?;
            let config = owner
                .egl
                .choose_first_config(
                    display,
                    &[
                        egl::SURFACE_TYPE,
                        egl::PBUFFER_BIT,
                        egl::RENDERABLE_TYPE,
                        egl::OPENGL_BIT,
                        egl::RED_SIZE,
                        8,
                        egl::GREEN_SIZE,
                        8,
                        egl::BLUE_SIZE,
                        8,
                        egl::DEPTH_SIZE,
                        24,
                        egl::NONE,
                    ],
                )?
                .context("EGL offscreen configuration unavailable")?;
            let context = owner
                .egl
                .create_context(display, config, None, &[egl::NONE])?;
            owner.context = Some(context);
            owner.egl.make_current(display, None, None, Some(context))?;
            Ok(owner)
        }
    }
    impl Drop for Context {
        fn drop(&mut self) {
            let _ = self.egl.make_current(self.display, None, None, None);
            if let Some(context) = self.context {
                let _ = self.egl.destroy_context(self.display, context);
            }
            let _ = self.egl.terminate(self.display);
        }
    }
}
pub use platform::Context;
