//! Bounded pose capture; software rasterization runs in a separate process after shutdown.
use crate::{Simulation, model};
use anyhow::{Context, Result, ensure};
use bello_config::finite;
use mujoco_rs::prelude::*;
use serde::{Deserialize, Serialize};
use std::io::{BufWriter, Write};
use std::os::unix::process::CommandExt;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::time::Duration;

#[cfg(target_os = "linux")]
mod software;

#[derive(Debug, Deserialize, Serialize, garde::Validate)]
#[garde(allow_unvalidated)]
#[serde(deny_unknown_fields)]
pub struct Config {
    pub output: PathBuf,
    #[garde(range(min = 2, max = 1920))]
    pub width: u32,
    #[garde(range(min = 2, max = 1080))]
    pub height: u32,
    #[garde(range(min = 1, max = 120))]
    pub fps: u32,
    #[garde(custom(finite), range(min = 0.0_f64.next_up()))]
    pub max_seconds: f64,
    #[garde(custom(finite), range(min = 0.0_f64.next_up()))]
    pub render_timeout_seconds: f64,
    #[garde(dive)]
    pub camera: Camera,
}

#[derive(Debug, Deserialize, Serialize, garde::Validate)]
#[serde(deny_unknown_fields)]
pub struct Camera {
    #[garde(length(min = 1))]
    pub body: String,
    #[garde(custom(finite), range(min = 0.0_f64.next_up()))]
    pub distance_m: f64,
    #[garde(custom(finite))]
    pub azimuth_degrees: f64,
    #[garde(custom(finite))]
    pub elevation_degrees: f64,
    #[garde(inner(custom(finite)))]
    pub lookat_offset_m: [f64; 3],
}

impl Config {
    fn validate(&self, nq: usize) -> Result<usize> {
        garde::Validate::validate(self)?;
        ensure!(
            self.width.is_multiple_of(2) && self.height.is_multiple_of(2),
            "video dimensions must be even"
        );
        Duration::try_from_secs_f64(self.render_timeout_seconds)?;
        let frames = (self.max_seconds * f64::from(self.fps)).ceil();
        ensure!(
            nq > 0
                && frames.is_finite()
                && frames >= 1.
                && frames * nq as f64 * 8. <= 64. * 1024. * 1024.,
            "video pose buffer exceeds 64 MiB"
        );
        ensure!(
            self.output.extension().is_some_and(|x| x == "mp4"),
            "video output must be .mp4"
        );
        Ok(frames as usize)
    }
}

pub struct Capture {
    config: Config,
    qpos: Vec<f64>,
    nq: usize,
    capacity: usize,
    next_time: Option<f64>,
}

impl Capture {
    pub fn new(mut config: Config, simulation: &Simulation) -> Result<Self> {
        ensure!(
            cfg!(target_os = "linux"),
            "video requires Linux/Lima software Mesa"
        );
        let nq = simulation.data.qpos().len();
        let capacity = config.validate(nq)?;
        config.output = std::env::current_dir()?.join(&config.output);
        simulation.id(MjtObj::mjOBJ_BODY, &config.camera.body)?;
        ensure!(
            1. / f64::from(config.fps) >= simulation.model().opt.timestep,
            "video frame rate exceeds simulation rate"
        );
        ensure!(!config.output.exists(), "video output already exists");
        ensure!(
            Command::new("ffmpeg")
                .arg("-version")
                .stdout(Stdio::null())
                .status()?
                .success(),
            "ffmpeg is unavailable"
        );
        Ok(Self {
            config,
            qpos: Vec::with_capacity(nq * capacity),
            nq,
            capacity,
            next_time: None,
        })
    }

    pub fn sample(&mut self, simulation: &Simulation, active: bool) -> Result<()> {
        self.sample_pose(simulation.data().time, simulation.data.qpos(), active)
    }

    fn sample_pose(&mut self, time: f64, qpos: &[f64], active: bool) -> Result<()> {
        if !active {
            self.next_time = None;
            return Ok(());
        }
        ensure!(
            time.is_finite() && qpos.len() == self.nq && qpos.iter().all(|x| x.is_finite()),
            "invalid video pose"
        );
        let next = self.next_time.get_or_insert(time);
        if time + 1e-10 >= *next {
            ensure!(
                self.qpos.len() / self.nq < self.capacity,
                "video max_seconds reached; increase the explicit recording bound"
            );
            self.qpos.extend_from_slice(qpos);
            *next += 1. / f64::from(self.config.fps);
        }
        Ok(())
    }

    pub fn finish(self, config: &model::ModelConfig, source: &Simulation) -> Result<()> {
        if self.qpos.is_empty() {
            eprintln!("Video: no released-torso frames captured; no video written.");
            return Ok(());
        }
        let visual = model::build_visual(config)?.simulation;
        let a = source.data.model();
        let b = visual.data.model();
        ensure!(
            a.nq() == b.nq()
                && a.jnt_type() == b.jnt_type()
                && a.jnt_qposadr() == b.jnt_qposadr()
                && (0..a.njnt() as usize).all(|i| a.id_to_name(MjtObj::mjOBJ_JOINT, i)
                    == b.id_to_name(MjtObj::mjOBJ_JOINT, i)),
            "video model joint layout changed"
        );
        let directory = self.config.output.with_extension("recording");
        if let Some(parent) = directory.parent() {
            std::fs::create_dir_all(parent)?;
        }
        std::fs::create_dir(&directory)
            .context("recording directory already exists or is unwritable")?;
        b.save_to_file(directory.join("model.mjb"))?;
        serde_json::to_writer(
            std::fs::File::create(directory.join("video.json"))?,
            &self.config,
        )?;
        let mut poses = BufWriter::new(std::fs::File::create(directory.join("qpos.bin"))?);
        for q in &self.qpos {
            poses.write_all(&q.to_le_bytes())?;
        }
        poses.flush()?;
        poses.get_ref().sync_all()?;
        drop(poses);
        eprintln!(
            "Rendering {} frames with software Mesa; recording: {}",
            self.qpos.len() / self.nq,
            directory.display()
        );
        let runtime = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()?;
        let mut child = Command::new(std::env::current_exe()?)
            .args(["render", "--recording"])
            .arg(&directory)
            .env("LIBGL_ALWAYS_SOFTWARE", "true")
            .env("GALLIUM_DRIVER", "llvmpipe")
            .env("LP_NUM_THREADS", "2")
            .stdin(Stdio::null())
            .process_group(0)
            .spawn()?;
        let result = runtime.block_on(async {
            let mut interrupt =
                tokio::signal::unix::signal(tokio::signal::unix::SignalKind::interrupt())?;
            let mut terminate =
                tokio::signal::unix::signal(tokio::signal::unix::SignalKind::terminate())?;
            let mut poll = tokio::time::interval(Duration::from_millis(50));
            let timeout =
                tokio::time::sleep(Duration::from_secs_f64(self.config.render_timeout_seconds));
            tokio::pin!(timeout);
            loop {
                tokio::select! {
                    _ = interrupt.recv() => anyhow::bail!("video rendering interrupted"),
                    _ = terminate.recv() => anyhow::bail!("video rendering terminated"),
                    _ = &mut timeout => anyhow::bail!("video rendering timed out"),
                    _ = poll.tick() => if let Some(status) = child.try_wait()? {
                        ensure!(status.success(), "software video worker failed: {status}");
                        return Ok(());
                    }
                }
            }
        });
        if result.is_err() {
            // The process group includes ffmpeg, even if a driver stalls.
            let _ = nix::sys::signal::killpg(
                nix::unistd::Pid::from_raw(child.id() as i32),
                nix::sys::signal::Signal::SIGKILL,
            );
            child.wait()?;
        }
        result.with_context(|| format!("recording retained at {}", directory.display()))?;
        eprintln!("Video written to {}", self.config.output.display());
        Ok(())
    }
}

pub fn render(recording: &Path) -> Result<()> {
    #[cfg(target_os = "linux")]
    {
        software::render(recording)
    }
    #[cfg(not(target_os = "linux"))]
    {
        let _ = recording;
        anyhow::bail!("video requires Linux/Lima software Mesa")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn config() -> Config {
        Config {
            output: "test.mp4".into(),
            width: 640,
            height: 480,
            fps: 10,
            max_seconds: 1.,
            render_timeout_seconds: 30.,
            camera: Camera {
                body: "body".into(),
                distance_m: 3.,
                azimuth_degrees: 135.,
                elevation_degrees: -14.,
                lookat_offset_m: [0.; 3],
            },
        }
    }

    #[test]
    fn sampling_pauses_resumes_after_reset_and_bounds_memory() {
        let mut capture = Capture {
            config: config(),
            qpos: vec![],
            nq: 2,
            capacity: 3,
            next_time: None,
        };
        capture.sample_pose(1., &[1., 2.], true).unwrap();
        capture.sample_pose(1.05, &[3., 4.], true).unwrap();
        capture.sample_pose(1.1, &[5., 6.], true).unwrap();
        capture.sample_pose(0., &[7., 8.], false).unwrap();
        capture.sample_pose(0.001, &[9., 10.], true).unwrap();
        assert_eq!(capture.qpos, [1., 2., 5., 6., 9., 10.]);
        assert!(capture.sample_pose(0.11, &[0., 0.], true).is_err());
        assert!(capture.sample_pose(0.2, &[f64::NAN, 0.], true).is_err());
        assert!(capture.sample_pose(0.2, &[0.], true).is_err());
    }

    #[test]
    fn invalid_dimensions_rates_and_unbounded_recordings_fail() {
        let mut c = config();
        assert_eq!(c.validate(2).unwrap(), 10);
        c.max_seconds = f64::INFINITY;
        assert!(c.validate(2).is_err());
        c.max_seconds = 1e9;
        assert!(c.validate(2).is_err());
        c.max_seconds = 1.;
        c.width = 639;
        assert!(c.validate(2).is_err());
        c.width = 640;
        c.fps = 0;
        assert!(c.validate(2).is_err());
    }
}
