use anyhow::{Context, Result, ensure};
use bello_camera_worker::proto::config::{Camera, Declaration, inventory};
use serde::{Deserialize, Serialize};
use std::path::{Path, PathBuf};

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Profile {
    socket: PathBuf,
    cameras: Vec<String>,
    #[serde(default = "default_samples")]
    offscreen_samples: i32,
}
pub(crate) fn default_samples() -> i32 {
    4
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Config {
    pub socket: PathBuf,
    pub cameras: Vec<Camera>,
    pub offscreen_samples: i32,
}

impl Config {
    pub(super) fn capture_fps(&self) -> Result<u32> {
        ensure!(
            self.cameras.iter().all(|c| (1..=120).contains(&c.fps)),
            "invalid camera FPS"
        );
        self.cameras
            .iter()
            .map(|c| c.fps)
            .max()
            .context("at least one camera is required")
    }
    pub fn load(path: &Path, hardware_path: &Path) -> Result<Self> {
        let profile: Profile = serde_yaml::from_reader(std::fs::File::open(path)?)?;
        Self::from_streams(
            profile.socket,
            profile.cameras,
            profile.offscreen_samples,
            hardware_path,
        )
    }

    pub(crate) fn from_streams(
        socket: PathBuf,
        names: Vec<String>,
        offscreen_samples: i32,
        hardware_path: &Path,
    ) -> Result<Self> {
        #[derive(Deserialize)]
        struct Hardware {
            #[serde(default)]
            cameras: Vec<Declaration>,
        }
        let hardware: Hardware = serde_yaml::from_str(&std::fs::read_to_string(hardware_path)?)?;
        let mut inventory = inventory(hardware.cameras, hardware_path)?;
        ensure!(socket.is_absolute(), "camera socket must be absolute");
        ensure!(
            [0, 2, 4, 8].contains(&offscreen_samples),
            "invalid MSAA samples"
        );
        ensure!((1..=16).contains(&names.len()), "invalid camera count");
        let cameras = names
            .into_iter()
            .map(|name| {
                inventory
                    .remove(&name)
                    .with_context(|| format!("unknown or repeated simulator camera {name}"))
            })
            .collect::<Result<_>>()?;
        Ok(Self {
            socket,
            cameras,
            offscreen_samples,
        })
    }
}
