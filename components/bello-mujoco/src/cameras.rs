//! Real-time offscreen cameras. Physics sends bounded snapshots to an isolated child.
mod context;
mod jpeg;
mod projection;
use crate::{Simulation, model};
use anyhow::{Context, Result, ensure};
use bello_camera_api::{FPS, Frames, HEIGHT, WIDTH, monotonic_ns, proto};
use mujoco_rs::prelude::*;
use nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion, Vector3};
use serde::{Deserialize, Serialize};
use std::{
    io,
    os::{
        fd::{FromRawFd, OwnedFd},
        unix::net::UnixDatagram,
    },
    path::{Path, PathBuf},
    process::{Child, Command, Stdio},
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
    },
    time::{Duration, Instant},
};

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Config {
    pub socket: PathBuf,
    #[serde(default)]
    pub listen: Option<std::net::SocketAddr>,
    pub cameras: Vec<Camera>,
    #[serde(default = "default_samples")]
    pub offscreen_samples: i32,
    #[serde(default = "default_width")]
    pub width: u32,
    #[serde(default = "default_height")]
    pub height: u32,
}
fn default_samples() -> i32 {
    4
}
fn default_width() -> u32 {
    WIDTH
}
fn default_height() -> u32 {
    HEIGHT
}
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Camera {
    pub name: String,
    pub link: String,
    /// Optical frame: +X right, +Y down, +Z forward, expressed in the mount link.
    pub camera_to_link: [f64; 7],
    /// fx, fy, cx, cy in native sensor pixels. `fisheye` selects lens distortion.
    pub intrinsics: [f64; 4],
    /// Optional OpenCV fisheye k1..k4. Output is the original distorted sensor image.
    #[serde(default)]
    pub fisheye: Option<[f64; 4]>,
}
impl Config {
    pub fn load(path: &Path) -> Result<Self> {
        let config: Self = serde_yaml::from_str(&std::fs::read_to_string(path)?)?;
        config.validate()?;
        Ok(config)
    }
    fn validate(&self) -> Result<()> {
        ensure!(self.socket.is_absolute(), "camera socket must be absolute");
        ensure!(
            !self.cameras.is_empty() && self.cameras.len() <= 16,
            "invalid camera count"
        );
        ensure!(
            self.cameras
                .iter()
                .map(|c| &c.name)
                .collect::<std::collections::BTreeSet<_>>()
                .len()
                == self.cameras.len(),
            "duplicate cameras"
        );
        ensure!(
            self.width > 0 && self.height > 0 && self.width <= 3840 && self.height <= 2160,
            "invalid camera dimensions"
        );
        ensure!(
            [0, 2, 4, 8].contains(&self.offscreen_samples),
            "invalid MSAA samples"
        );
        for c in &self.cameras {
            ensure!(
                !c.link.is_empty()
                    && c.camera_to_link.iter().all(|v| v.is_finite())
                    && (c.camera_to_link[3..].iter().map(|v| v * v).sum::<f64>() - 1.).abs() < 1e-5,
                "invalid camera mount"
            );
            let [fx, fy, cx, cy] = c.intrinsics;
            ensure!(
                [fx, fy, cx, cy].iter().all(|v| v.is_finite())
                    && fx > 0.
                    && fy > 0.
                    && (0. ..=f64::from(self.width)).contains(&cx)
                    && (0. ..=f64::from(self.height)).contains(&cy),
                "invalid camera intrinsics"
            );
            if let Some(d) = c.fisheye {
                ensure!(
                    d.iter().all(|v| v.is_finite()),
                    "invalid fisheye coefficients"
                );
            } else {
                projection::Projection::for_format(self.width, self.height, c.intrinsics)?;
            }
        }
        Ok(())
    }
}
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
    /// Attach the same isolated renderer to an independently owned native plant.
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
        for camera in &config.cameras {
            visual.id(MjtObj::mjOBJ_BODY, &camera.link)?;
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
        for (bytes, value) in self.packet[HEADER..].chunks_exact_mut(8).zip(values) {
            bytes.copy_from_slice(&value.to_le_bytes());
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

pub fn render(model_path: &Path, config_path: &Path) -> Result<()> {
    let config: Config = serde_json::from_slice(&std::fs::read(config_path)?)?;
    config.validate()?;
    Simulation::check_version()?;
    let mut model = MjModel::from_buffer(&std::fs::read(model_path)?)?;
    model.vis_mut().quality.offsamples = config.offscreen_samples;
    let nq = model.nq() as usize;
    let nmocap = model.nmocap() as usize;
    let ids = config
        .cameras
        .iter()
        .map(|c| {
            model
                .name_to_id(MjtObj::mjOBJ_BODY, &c.link)
                .context("camera mount missing")
        })
        .collect::<Result<Vec<_>>>()?;
    model.vis_mut().headlight.ambient = [0.35; 3];
    model.vis_mut().headlight.diffuse = [0.8; 3];
    let visual_bodies: std::collections::BTreeSet<_> = (0..model.ngeom() as usize)
        .filter(|&i| model.geom_contype()[i] == 0 && model.geom_conaffinity()[i] == 0)
        .map(|i| model.geom_bodyid()[i])
        .collect();
    for i in 0..model.ngeom() as usize {
        let collision = model.geom_contype()[i] != 0 || model.geom_conaffinity()[i] != 0;
        model.geom_group_mut()[i] =
            i32::from(!(collision && visual_bodies.contains(&model.geom_bodyid()[i])));
    }
    let _gl = context::Context::new()?;
    let projections = config
        .cameras
        .iter()
        .map(|c| match c.fisheye {
            Some(d) => {
                projection::Projection::for_fisheye(config.width, config.height, c.intrinsics, d)
            }
            None => projection::Projection::for_format(config.width, config.height, c.intrinsics),
        })
        .collect::<Result<Vec<_>>>()?;
    let mut context = unsafe { MjrContext::new(&model) };
    context.resize_offscreen(
        projections.iter().map(|p| p.width).max().unwrap(),
        projections.iter().map(|p| p.height).max().unwrap(),
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
        .map(|c| proto::Camera {
            name: c.name.clone(),
            width: config.width,
            height: config.height,
            fps: FPS,
            source: "mujoco".into(),
            metadata_json: serde_json::to_string(c).unwrap(),
        })
        .collect();
    let frames = Frames::configured(cameras)?;
    let runtime = tokio::runtime::Builder::new_multi_thread()
        .worker_threads(2)
        .enable_all()
        .build()?;
    let _entered = runtime.enter();
    let (listener, _socket) = runtime.block_on(bello_camera_api::socket::bind(&config.socket))?;
    let server = runtime.spawn(
        tonic::transport::Server::builder()
            .add_service(
                proto::camera_source_service_server::CameraSourceServiceServer::new(frames.clone()),
            )
            .serve_with_incoming(tokio_stream::wrappers::UnixListenerStream::new(listener)),
    );
    let tcp_server = config.listen.map(|address| {
        runtime.spawn(
            tonic::transport::Server::builder()
                .add_service(
                    proto::camera_source_service_server::CameraSourceServiceServer::new(
                        frames.clone(),
                    ),
                )
                .serve(address),
        )
    });
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
        projections
            .iter()
            .map(|p| p.width as usize * p.height as usize * 3)
            .max()
            .unwrap(),
    );
    let mut encoder = jpeg::Encoder::new();
    let mut last = Instant::now();
    while !stop.load(Ordering::Acquire) {
        ensure!(
            !server.is_finished() && tcp_server.as_ref().is_none_or(|s| !s.is_finished()),
            "camera frame service stopped"
        );
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
        input.set_nonblocking(true)?;
        while let Ok(n) = input.recv(&mut packet) {
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
        let values = latest[HEADER..size]
            .chunks_exact(8)
            .map(|b| f64::from_le_bytes(b.try_into().unwrap()))
            .collect::<Vec<_>>();
        ensure!(
            values.iter().all(|v| v.is_finite()),
            "invalid snapshot state"
        );
        data.qpos_mut().copy_from_slice(&values[..nq]);
        for (target, chunk) in data
            .mocap_pos_mut()
            .iter_mut()
            .zip(values[nq..nq + 3 * nmocap].chunks_exact(3))
        {
            target.copy_from_slice(chunk);
        }
        for (target, chunk) in data
            .mocap_quat_mut()
            .iter_mut()
            .zip(values[nq + 3 * nmocap..].chunks_exact(4))
        {
            target.copy_from_slice(chunk);
        }
        // Only the simulation clock scalar changes; allocation/layout remain intact.
        unsafe { data.ffi_mut() }.time = simtime;
        data.forward();
        for (i, c) in config.cameras.iter().enumerate() {
            let render_started = Instant::now();
            scene.update(&mut data, &options, &perturb, &mut camera);
            let p = data.xpos()[ids[i]];
            let q = data.xquat()[ids[i]];
            let mount = Isometry3::from_parts(
                Translation3::from(p),
                UnitQuaternion::new_normalize(Quaternion::new(q[0], q[1], q[2], q[3])),
            );
            let t = c.camera_to_link;
            let optical = mount
                * Isometry3::from_parts(
                    Translation3::new(t[0], t[1], t[2]),
                    UnitQuaternion::new_normalize(Quaternion::new(t[3], t[4], t[5], t[6])),
                );
            let forward = optical.rotation * Vector3::z();
            let up = optical.rotation * (-Vector3::y());
            let projection = &projections[i];
            let viewport =
                MjrRectangle::new(0, 0, projection.width as i32, projection.height as i32);
            // Only the two fixed-size camera records change; scene allocation invariants remain intact.
            for view in &mut unsafe { scene.ffi_mut() }.camera {
                view.pos = optical.translation.vector.map(|x| x as f32).into();
                view.forward = forward.map(|x| x as f32).into();
                view.up = up.map(|x| x as f32).into();
                projection.apply(view);
            }
            scene.render(&viewport, &context);
            pixels.resize(
                projection.width as usize * projection.height as usize * 3,
                0,
            );
            context.read_pixels(Some(&mut pixels), None, &viewport)?;
            let image = projection.image(&pixels);
            let render_ms = render_started.elapsed().as_secs_f64() * 1000.;
            let encode_started = Instant::now();
            let jpeg = encoder.encode(&image)?;
            frames.publish(proto::Frame {
                name: c.name.clone(),
                sequence,
                capture_ns,
                received_ns: monotonic_ns(),
                jpeg,
                flags: 0,
                simulation_time_ns: Some((simtime * 1e9).round() as u64),
                metadata_json: serde_json::json!({
                    "image_model": if c.fisheye.is_some() { "fisheye" } else { "pinhole" },
                    "render_ms": render_ms,
                    "jpeg_ms": encode_started.elapsed().as_secs_f64() * 1000., "world_from_optical": {
                        "position": optical.translation.vector.as_slice(),
                        "quaternion_xyzw": optical.rotation.coords.as_slice()
                    },
                    "joint_positions": (0..data.model().njnt() as usize)
                        .filter_map(|j| data.model().id_to_name(MjtObj::mjOBJ_JOINT, j)
                            .map(|name| (name.to_owned(), data.qpos()[data.model().jnt_qposadr()[j] as usize])))
                        .collect::<std::collections::BTreeMap<_, _>>()
                }).to_string(),
            })?;
        }
    }
    server.abort();
    if let Some(server) = tcp_server {
        server.abort();
    }
    Ok(())
}
