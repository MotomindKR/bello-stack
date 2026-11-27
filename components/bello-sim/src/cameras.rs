mod config;
pub use config::{Camera, Config};
mod context;
mod jpeg;
mod projection;
use crate::{Simulation, model};
use anyhow::{Context, Result, ensure};
use bello_camera_worker::{FPS, monotonic_ns, proto, sim_pipe};
use mujoco_rs::prelude::*;
use nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion, Vector3};
use std::{
    io,
    os::{
        fd::{FromRawFd, OwnedFd},
        unix::net::UnixDatagram,
    },
    path::Path,
    process::{Child, Command, Stdio},
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
    },
    time::{Duration, Instant},
};

const HEADER: usize = 40;
const MAGIC: &[u8; 8] = b"BELLOCAM";
pub struct Capture {
    socket: UnixDatagram,
    child: Child,
    _files: tempfile::TempDir,
    packet: Vec<u8>,
    next: Instant,
    sequence: u64,
    dropped: u64,
}
impl Capture {
    pub fn start(config: Config, model: &model::ModelConfig, source: &Simulation) -> Result<Self> {
        Self::start_data(config, model, &source.data)
    }
    pub fn start_data(
        config: Config,
        model: &model::ModelConfig,
        source: &MjData<Box<MjModel>>,
    ) -> Result<Self> {
        config.validate()?;
        let visual = model::build_visual(model)?.simulation;
        ensure!(
            source.model().nq() == visual.model().nq
                && source.model().nmocap() == visual.model().nmocap
                && source.model().jnt_type() == visual.data.model().jnt_type()
                && source.model().jnt_qposadr() == visual.data.model().jnt_qposadr()
                && (0..source.model().njnt() as usize)
                    .all(|i| source.model().id_to_name(MjtObj::mjOBJ_JOINT, i)
                        == visual.data.model().id_to_name(MjtObj::mjOBJ_JOINT, i)),
            "camera visual model state layout differs from physics"
        );
        for lens in config
            .cameras
            .iter()
            .flat_map(|camera| camera.views.lenses())
        {
            visual.id(MjtObj::mjOBJ_BODY, &lens.link)?;
        }
        let files = tempfile::tempdir()?;
        let model_path = files.path().join("model.mjb");
        visual.data.model().save_to_file(&model_path)?;
        let config_path = files.path().join("cameras.json");
        std::fs::write(&config_path, serde_json::to_vec(&config)?)?;
        let (socket, input) = UnixDatagram::pair()?;
        socket.set_nonblocking(true)?;
        let count = source.model().nq() as usize + source.model().nmocap() as usize * 7;
        ensure!(HEADER + count * 8 <= 8192, "camera snapshot exceeds 8 KiB");
        let mut command = Command::new(std::env::current_exe()?);
        command
            .args(["cameras", "--model"])
            .arg(model_path)
            .arg("--config")
            .arg(config_path)
            .stdin(Stdio::from(OwnedFd::from(input)));
        #[cfg(target_os = "linux")]
        {
            use std::os::unix::process::CommandExt;
            let mut cpus = nix::sched::CpuSet::new();
            for cpu in 0..4 {
                cpus.set(cpu)?;
            }
            // Only async-signal-safe syscalls run after fork. Media must not inherit
            // the physics CPU affinity or a real-time scheduler policy.
            unsafe {
                command.pre_exec(move || {
                    nix::sched::sched_setaffinity(nix::unistd::Pid::from_raw(0), &cpus)
                        .map_err(io::Error::from)?;
                    if nix::libc::sched_setscheduler(
                        0,
                        nix::libc::SCHED_OTHER,
                        &nix::libc::sched_param { sched_priority: 0 },
                    ) != 0
                    {
                        return Err(io::Error::last_os_error());
                    }
                    Ok(())
                });
            }
        }
        let child = command.spawn()?;
        let mut packet = vec![0; HEADER + count * 8];
        packet[..8].copy_from_slice(MAGIC);
        packet[32..36].copy_from_slice(&(source.model().nq() as u32).to_le_bytes());
        packet[36..40].copy_from_slice(&(source.model().nmocap() as u32).to_le_bytes());
        Ok(Self {
            socket,
            child,
            _files: files,
            packet,
            next: Instant::now(),
            sequence: 0,
            dropped: 0,
        })
    }
    /// Poll only from the preview process, outside the physics sampling path.
    pub fn check_running(&mut self) -> Result<()> {
        if let Some(status) = self.child.try_wait()? {
            anyhow::bail!("camera renderer exited early: {status}");
        }
        Ok(())
    }

    /// Queue a shutdown after the final snapshot so startup and render errors reach the caller.
    pub fn finish_preview(mut self) -> Result<()> {
        self.check_running()?;
        self.socket.set_nonblocking(false)?;
        self.socket
            .set_write_timeout(Some(Duration::from_secs(5)))?;
        self.socket
            .send(&[])
            .context("request camera renderer shutdown")?;
        let deadline = Instant::now() + Duration::from_secs(5);
        loop {
            if let Some(status) = self.child.try_wait()? {
                ensure!(status.success(), "camera renderer failed: {status}");
                return Ok(());
            }
            ensure!(Instant::now() < deadline, "camera renderer did not finish");
            std::thread::sleep(Duration::from_millis(10));
        }
    }

    /// No rendering, waiting, serialization allocation or filesystem access in physics.
    pub fn sample(&mut self, source: &Simulation) {
        self.sample_data(&source.data);
    }
    pub fn sample_data(&mut self, source: &MjData<Box<MjModel>>) {
        let now = Instant::now();
        if now < self.next {
            return;
        }
        let period = Duration::from_secs_f64(1. / f64::from(FPS));
        self.next += period;
        if self.next < now {
            self.next = now + period;
        }
        self.sequence += 1;
        self.packet[8..16].copy_from_slice(&self.sequence.to_le_bytes());
        self.packet[16..24].copy_from_slice(&monotonic_ns().to_le_bytes());
        self.packet[24..32].copy_from_slice(&source.ffi().time.to_le_bytes());
        let state = source.ffi();
        let model = source.model().ffi();
        // Array lengths are fixed by the validated model; the snapshot owns every byte.
        let positions = if model.nmocap == 0 {
            &[][..]
        } else {
            unsafe { std::slice::from_raw_parts(state.mocap_pos, model.nmocap as usize * 3) }
        };
        let rotations = if model.nmocap == 0 {
            &[][..]
        } else {
            unsafe { std::slice::from_raw_parts(state.mocap_quat, model.nmocap as usize * 4) }
        };
        let values = unsafe { std::slice::from_raw_parts(state.qpos, model.nq as usize) }
            .iter()
            .chain(positions)
            .chain(rotations);
        let (chunks, _) = self.packet[HEADER..].as_chunks_mut::<8>();
        for (bytes, value) in chunks.iter_mut().zip(values) {
            *bytes = value.to_le_bytes();
        }
        if self.socket.send(&self.packet).is_err() {
            self.dropped += 1;
        }
    }
}
impl Drop for Capture {
    fn drop(&mut self) {
        eprintln!(
            "Camera snapshot attempts: {}, dropped: {}",
            self.sequence, self.dropped
        );
        if self.child.try_wait().ok().flatten().is_some() {
            return;
        }
        let _ = nix::sys::signal::kill(
            nix::unistd::Pid::from_raw(self.child.id() as i32),
            nix::sys::signal::Signal::SIGTERM,
        );
        let deadline = Instant::now() + Duration::from_secs(2);
        while Instant::now() < deadline {
            if self.child.try_wait().ok().flatten().is_some() {
                return;
            }
            std::thread::sleep(Duration::from_millis(10));
        }
        let _ = self.child.kill();
        let _ = self.child.wait();
        eprintln!("Camera snapshot drops: {}", self.dropped);
    }
}

struct Eye {
    id: usize,
    mount_from_optical: Isometry3<f64>,
    projection: projection::Projection,
}
impl Eye {
    fn new(model: &MjModel, lens: &config::Lens, width: u32, height: u32) -> Result<Self> {
        let id = model
            .name_to_id(MjtObj::mjOBJ_BODY, &lens.link)
            .context("camera mount missing")?;
        let projection = match lens.fisheye {
            Some(d) => projection::Projection::for_fisheye(width, height, lens.intrinsics, d),
            None => projection::Projection::for_format(width, height, lens.intrinsics),
        }?;
        let t = lens.camera_to_link;
        let mount_from_optical = Isometry3::from_parts(
            Translation3::new(t[0], t[1], t[2]),
            UnitQuaternion::new_normalize(Quaternion::new(t[3], t[4], t[5], t[6])),
        );
        Ok(Self {
            id,
            mount_from_optical,
            projection,
        })
    }

    fn render(
        &self,
        data: &MjData<&MjModel>,
        scene: &mut MjvScene,
        context: &MjrContext,
        pixels: &mut Vec<u8>,
    ) -> Result<image::RgbImage> {
        let p = data.xpos()[self.id];
        let q = data.xquat()[self.id];
        let mount = Isometry3::from_parts(
            Translation3::from(p),
            UnitQuaternion::new_normalize(Quaternion::new(q[0], q[1], q[2], q[3])),
        );
        let optical = mount * self.mount_from_optical;
        let forward = optical.rotation * Vector3::z();
        let up = optical.rotation * (-Vector3::y());
        let viewport = MjrRectangle::new(
            0,
            0,
            self.projection.render_width as i32,
            self.projection.render_height as i32,
        );
        // Only the two fixed-size camera records change; scene allocation invariants remain intact.
        for view in &mut unsafe { scene.ffi_mut() }.camera {
            view.pos = optical.translation.vector.map(|x| x as f32).into();
            view.forward = forward.map(|x| x as f32).into();
            view.up = up.map(|x| x as f32).into();
            self.projection.apply_frustum(view);
        }
        scene.render(&viewport, context);
        pixels.resize(
            self.projection.render_width as usize * self.projection.render_height as usize * 3,
            0,
        );
        context.read_pixels(Some(pixels), None, &viewport)?;
        Ok(self.projection.sensor_image(pixels))
    }
}

pub fn render(model_path: &Path, config_path: &Path) -> Result<()> {
    let config: Config = serde_json::from_slice(&std::fs::read(config_path)?)?;
    config.validate()?;
    Simulation::check_version()?;
    let mut model = MjModel::from_buffer(&std::fs::read(model_path)?)?;
    model.vis_mut().quality.offsamples = config.offscreen_samples;
    let nq = model.nq() as usize;
    let nmocap = model.nmocap() as usize;
    model.vis_mut().headlight.ambient = [0.35; 3];
    model.vis_mut().headlight.diffuse = [0.8; 3];
    for i in 0..model.ngeom() as usize {
        let collision = model.geom_contype()[i] != 0 || model.geom_conaffinity()[i] != 0;
        // Robot collision proxies are never scenery. Explicit scene fixtures
        // and the generated floor remain visible even when contact-enabled.
        let name = model.id_to_name(MjtObj::mjOBJ_GEOM, i).unwrap_or("");
        let scenery = name == "floor" || name.starts_with("scene/");
        model.geom_group_mut()[i] = i32::from(!collision || scenery);
    }
    let _gl = context::Context::new()?;
    let views = config
        .cameras
        .iter()
        .map(|camera| {
            camera
                .views
                .lenses()
                .map(|lens| Eye::new(&model, lens, config.width, config.height))
                .collect::<Result<Vec<_>>>()
        })
        .collect::<Result<Vec<_>>>()?;
    let mut context = unsafe { MjrContext::new(&model) };
    context.resize_offscreen(
        views
            .iter()
            .flatten()
            .map(|eye| eye.projection.render_width)
            .max()
            .unwrap(),
        views
            .iter()
            .flatten()
            .map(|eye| eye.projection.render_height)
            .max()
            .unwrap(),
    );
    context.offscreen();
    let mut scene = MjvScene::new(&model, model.ngeom() as usize + 1000);
    let mut camera = MjvCamera::new_free(&model);
    let mut options = MjvOption::default();
    options.geomgroup[0] = 0;
    options.geomgroup[1] = 1;
    options.sitegroup.fill(0);
    let perturb = MjvPerturb::default();
    let cameras = config
        .cameras
        .iter()
        .map(|c| sim_pipe::Stream {
            camera: proto::Camera {
                name: c.name.clone(),
                width: config.width * c.views.lenses().count() as u32,
                height: config.height,
                fps: FPS,
                source: "mujoco".into(),
                metadata_json: c.metadata().to_string(),
            },
            layout: c.views.layout() as i32,
        })
        .collect();
    let runtime = tokio::runtime::Builder::new_multi_thread()
        .worker_threads(2)
        .enable_all()
        .build()?;
    let _entered = runtime.enter();
    let publisher = runtime.block_on(sim_pipe::Output::open(&config.socket, cameras))?;
    let stop = Arc::new(AtomicBool::new(false));
    let stopping = stop.clone();
    runtime.spawn(async move {
        let mut signal =
            tokio::signal::unix::signal(tokio::signal::unix::SignalKind::terminate()).unwrap();
        tokio::select! {_=signal.recv()=>{},_=tokio::signal::ctrl_c()=>{}}
        stopping.store(true, Ordering::Release);
    });
    // Stdin is the private datagram socket passed by Capture::start, never a user terminal.
    let input = unsafe { UnixDatagram::from_raw_fd(0) };
    input.set_read_timeout(Some(Duration::from_millis(100)))?;
    let mut packet = [0; 8192];
    let mut latest = [0; 8192];
    let mut data = model.try_make_data()?;
    let mut pixels = Vec::with_capacity(
        views
            .iter()
            .flatten()
            .map(|eye| {
                eye.projection.render_width as usize * eye.projection.render_height as usize * 3
            })
            .max()
            .unwrap(),
    );
    let encoder = jpeg::Encoder::new()?;
    let mut last = Instant::now();
    while !stop.load(Ordering::Acquire) {
        let size = match input.recv(&mut latest) {
            Ok(size) => size,
            Err(e)
                if matches!(
                    e.kind(),
                    io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut
                ) =>
            {
                ensure!(
                    last.elapsed() < Duration::from_secs(3),
                    "simulation snapshots stopped"
                );
                continue;
            }
            Err(e) => return Err(e.into()),
        };
        if size == 0 {
            break;
        }
        input.set_nonblocking(true)?;
        while let Ok(n) = input.recv(&mut packet) {
            if n == 0 {
                stop.store(true, Ordering::Release);
                break;
            }
            ensure!(n == size, "snapshot shape changed");
            latest[..n].copy_from_slice(&packet[..n]);
        }
        input.set_nonblocking(false)?;
        last = Instant::now();
        ensure!(
            size == HEADER + (nq + 7 * nmocap) * 8
                && &latest[..8] == MAGIC
                && u32::from_le_bytes(latest[32..36].try_into()?) as usize == nq
                && u32::from_le_bytes(latest[36..40].try_into()?) as usize == nmocap,
            "invalid simulation snapshot"
        );
        let sequence = u64::from_le_bytes(latest[8..16].try_into()?);
        let capture_ns = u64::from_le_bytes(latest[16..24].try_into()?);
        let simtime = f64::from_le_bytes(latest[24..32].try_into()?);
        ensure!(
            simtime.is_finite() && simtime >= 0.,
            "invalid simulation camera time"
        );
        if monotonic_ns().saturating_sub(capture_ns) > 500_000_000 {
            continue;
        }
        let (chunks, _) = latest[HEADER..size].as_chunks::<8>();
        let values = chunks
            .iter()
            .copied()
            .map(f64::from_le_bytes)
            .collect::<Vec<_>>();
        ensure!(
            values.iter().all(|v| v.is_finite()),
            "invalid snapshot state"
        );
        data.qpos_mut().copy_from_slice(&values[..nq]);
        let (positions, _) = values[nq..nq + 3 * nmocap].as_chunks::<3>();
        for (target, chunk) in data.mocap_pos_mut().iter_mut().zip(positions) {
            target.copy_from_slice(chunk);
        }
        let (rotations, _) = values[nq + 3 * nmocap..].as_chunks::<4>();
        for (target, chunk) in data.mocap_quat_mut().iter_mut().zip(rotations) {
            target.copy_from_slice(chunk);
        }
        // Only the simulation clock scalar changes; allocation/layout remain intact.
        unsafe { data.ffi_mut() }.time = simtime;
        data.forward();
        scene.update(&mut data, &options, &perturb, &mut camera);
        for (c, eyes) in config.cameras.iter().zip(&views) {
            let mut rendered = eyes
                .iter()
                .map(|eye| eye.render(&data, &mut scene, &context, &mut pixels));
            let first = rendered.next().unwrap()?;
            let image = match rendered.next().transpose()? {
                Some(right) => side_by_side(first, right),
                None => first,
            };
            let jpeg = encoder.encode(image)?;
            publisher.publish(proto::Frame {
                name: c.name.clone(),
                sequence,
                capture_ns,
                received_ns: monotonic_ns(),
                jpeg,
                flags: 0,
                simulation_time_ns: Some((simtime * 1e9).round() as u64),
            })?;
        }
    }
    Ok(())
}

fn side_by_side(left: image::RgbImage, right: image::RgbImage) -> image::RgbImage {
    let mut packed = image::RgbImage::new(left.width() * 2, left.height());
    image::imageops::replace(&mut packed, &left, 0, 0);
    image::imageops::replace(&mut packed, &right, left.width().into(), 0);
    packed
}

#[cfg(test)]
mod stream_tests;
