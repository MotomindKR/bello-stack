use super::Config;
use anyhow::{Context, Result, ensure};
use khronos_egl as egl;
use mujoco_rs::prelude::*;
use std::ffi::CStr;
use std::io::{BufReader, Read, Write};
use std::path::Path;
use std::process::{Command, Stdio};

struct SoftwareContext {
    egl: egl::DynamicInstance<egl::EGL1_5>,
    display: egl::Display,
    context: Option<egl::Context>,
}

impl SoftwareContext {
    fn new() -> Result<Self> {
        ensure!(
            std::env::var("LIBGL_ALWAYS_SOFTWARE").as_deref() == Ok("true")
                && std::env::var("GALLIUM_DRIVER").as_deref() == Ok("llvmpipe"),
            "renderer must start with the software-only environment"
        );
        // Mesa surfaceless requires a null native display. The library stays alive
        // until all GL resources and the current context have been destroyed.
        let egl = unsafe {
            egl::DynamicInstance::<egl::EGL1_5>::load_required_from_filename("libEGL.so.1")?
        };
        let display =
            unsafe { egl.get_platform_display(0x31DD, std::ptr::null_mut(), &[egl::ATTRIB_NONE])? };
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
            .context("Mesa has no offscreen OpenGL configuration")?;
        let context = owner
            .egl
            .create_context(display, config, None, &[egl::NONE])?;
        owner.context = Some(context);
        owner.egl.make_current(display, None, None, Some(context))?;
        let address = owner
            .egl
            .get_proc_address("glGetString")
            .context("missing glGetString")?;
        // EGL provides the standard OpenGL ABI for the named entrypoint.
        let get_string: unsafe extern "system" fn(u32) -> *const u8 =
            unsafe { std::mem::transmute(address) };
        let renderer = unsafe { get_string(0x1F01) };
        ensure!(!renderer.is_null(), "OpenGL renderer name is missing");
        let renderer = unsafe { CStr::from_ptr(renderer.cast()) }.to_str()?;
        ensure!(
            renderer.to_ascii_lowercase().starts_with("llvmpipe"),
            "software llvmpipe required, got {renderer}"
        );
        eprintln!("Video rasterizer: {renderer}");
        Ok(owner)
    }
}

impl Drop for SoftwareContext {
    fn drop(&mut self) {
        let _ = self.egl.make_current(self.display, None, None, None);
        if let Some(context) = self.context {
            let _ = self.egl.destroy_context(self.display, context);
        }
        let _ = self.egl.terminate(self.display);
    }
}

pub(super) fn render(directory: &Path) -> Result<()> {
    crate::Simulation::check_version()?;
    let config: Config =
        serde_json::from_reader(std::fs::File::open(directory.join("video.json"))?)?;
    let mut model = MjModel::from_buffer(&std::fs::read(directory.join("model.mjb"))?)?;
    let nq = model.nq() as usize;
    let max_frames = config.validate(nq)?;
    let file = std::fs::File::open(directory.join("qpos.bin"))?;
    let bytes_per_frame = (nq * 8) as u64;
    let size = file.metadata()?.len();
    ensure!(
        size > 0
            && size.is_multiple_of(bytes_per_frame)
            && size / bytes_per_frame <= max_frames as u64,
        "invalid or oversized recording"
    );
    ensure!(!config.output.exists(), "video output already exists");
    let body = model
        .name_to_id(MjtObj::mjOBJ_BODY, &config.camera.body)
        .context("video camera body missing")?;
    model.vis_mut().headlight.ambient = [0.35; 3];
    model.vis_mut().headlight.diffuse = [0.8; 3];
    // URDF visuals are non-colliding. Hide collision proxies only on bodies
    // with their own visual geometry; retain the floor and primitive-only links.
    let visual_bodies: std::collections::BTreeSet<_> = (0..model.ngeom() as usize)
        .filter(|&i| model.geom_contype()[i] == 0 && model.geom_conaffinity()[i] == 0)
        .map(|i| model.geom_bodyid()[i])
        .collect();
    for i in 0..model.ngeom() as usize {
        let collision = model.geom_contype()[i] != 0 || model.geom_conaffinity()[i] != 0;
        let hidden = collision && visual_bodies.contains(&model.geom_bodyid()[i]);
        model.geom_group_mut()[i] = if hidden { 0 } else { 1 };
    }
    let _software = SoftwareContext::new()?;
    // This local context is dropped before _software, on the same thread.
    let mut context = unsafe { MjrContext::new(&model) };
    context.resize_offscreen(config.width, config.height);
    context.offscreen();
    let mut scene = MjvScene::new(&model, model.ngeom() as usize + 1000);
    let mut camera = MjvCamera::new_free(&model);
    camera.distance = config.camera.distance_m;
    camera.azimuth = config.camera.azimuth_degrees;
    camera.elevation = config.camera.elevation_degrees;
    let mut options = MjvOption::default();
    options.geomgroup[0] = 0;
    options.geomgroup[1] = 1;
    options.sitegroup.fill(0);
    let perturb = MjvPerturb::default();
    let mut data = model.try_make_data()?;
    let viewport = MjrRectangle::new(0, 0, config.width as i32, config.height as i32);
    let mut pixels = vec![0; config.width as usize * config.height as usize * 3];
    let mut poses = BufReader::new(file);
    let output_parent = config
        .output
        .parent()
        .filter(|p| !p.as_os_str().is_empty())
        .unwrap_or(Path::new("."));
    std::fs::create_dir_all(output_parent)?;
    let temporary = tempfile::NamedTempFile::new_in(output_parent)?;
    let mut encoder = Command::new("ffmpeg")
        .args([
            "-nostdin",
            "-hide_banner",
            "-loglevel",
            "error",
            "-y",
            "-f",
            "rawvideo",
            "-pixel_format",
            "rgb24",
            "-video_size",
        ])
        .arg(format!("{}x{}", config.width, config.height))
        .arg("-framerate")
        .arg(config.fps.to_string())
        .args([
            "-i",
            "pipe:0",
            "-vf",
            "vflip",
            "-an",
            "-c:v",
            "libx264",
            "-threads",
            "2",
            "-pix_fmt",
            "yuv420p",
            "-movflags",
            "+faststart",
            "-f",
            "mp4",
        ])
        .arg(temporary.path())
        .stdin(Stdio::piped())
        .stdout(Stdio::null())
        .spawn()?;
    let result = (|| -> Result<()> {
        let mut input = encoder.stdin.take().context("ffmpeg stdin missing")?;
        for _ in 0..size / bytes_per_frame {
            for q in data.qpos_mut() {
                let mut bytes = [0; 8];
                poses.read_exact(&mut bytes)?;
                *q = f64::from_le_bytes(bytes);
                ensure!(q.is_finite(), "nonfinite recorded pose");
            }
            data.forward();
            for (i, offset) in config.camera.lookat_offset_m.iter().enumerate() {
                camera.lookat[i] = data.xpos()[body][i] + offset;
            }
            scene.update(&mut data, &options, &perturb, &mut camera);
            scene.render(&viewport, &context);
            context.read_pixels(Some(&mut pixels), None, &viewport)?;
            input.write_all(&pixels)?;
        }
        Ok(())
    })();
    if result.is_err() {
        let _ = encoder.kill();
    }
    let status = encoder.wait()?;
    result?;
    ensure!(status.success(), "ffmpeg failed: {status}");
    temporary.as_file().sync_all()?;
    temporary.persist_noclobber(&config.output)?;
    Ok(())
}
