use serde::{Deserialize, Serialize};
use std::collections::HashSet;
use std::path::PathBuf;

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "lowercase", deny_unknown_fields)]
pub enum Source {
    Mujoco { socket: PathBuf },
    V4l2 { devices: Vec<Device> },
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Device {
    pub name: String,
    pub device: PathBuf,
    /// Encoded dimensions. For side-by-side stereo, width includes both eyes.
    pub width: u32,
    pub height: u32,
    pub fps: u32,
    #[serde(default)]
    pub stereo: bool,
}

impl Source {
    pub fn validate(&self) -> Result<(), &'static str> {
        match self {
            Self::Mujoco { socket } if !socket.is_absolute() => {
                Err("camera input socket must be absolute")
            }
            Self::V4l2 { devices } => {
                let mut names = HashSet::new();
                let mut paths = HashSet::new();
                if devices.is_empty()
                    || devices.len() > 16
                    || devices.iter().any(|d| {
                        d.name.is_empty()
                            || !names.insert(&d.name)
                            || !d.device.is_absolute()
                            || !paths.insert(&d.device)
                            || !(1..=4096).contains(&d.width)
                            || !(1..=2160).contains(&d.height)
                            || !(1..=120).contains(&d.fps)
                            || (d.stereo && !d.width.is_multiple_of(2))
                    })
                {
                    return Err("invalid camera devices, names, dimensions or frame rate");
                }
                Ok(())
            }
            _ => Ok(()),
        }
    }
}
