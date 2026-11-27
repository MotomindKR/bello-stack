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
fn default_samples() -> i32 {
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
        #[derive(Deserialize)]
        struct Hardware {
            #[serde(default)]
            cameras: Vec<Declaration>,
        }
        let profile: Profile = serde_yaml::from_str(&std::fs::read_to_string(path)?)?;
        let hardware: Hardware = serde_yaml::from_str(&std::fs::read_to_string(hardware_path)?)?;
        let mut inventory = inventory(hardware.cameras, hardware_path)?;
        ensure!(
            profile.socket.is_absolute(),
            "camera socket must be absolute"
        );
        ensure!(
            [0, 2, 4, 8].contains(&profile.offscreen_samples),
            "invalid MSAA samples"
        );
        ensure!(
            (1..=16).contains(&profile.cameras.len()),
            "invalid camera count"
        );
        let cameras = profile
            .cameras
            .into_iter()
            .map(|name| {
                inventory
                    .remove(&name)
                    .with_context(|| format!("unknown or repeated simulator camera {name}"))
            })
            .collect::<Result<_>>()?;
        Ok(Self {
            socket: profile.socket,
            cameras,
            offscreen_samples: profile.offscreen_samples,
        })
    }
}
