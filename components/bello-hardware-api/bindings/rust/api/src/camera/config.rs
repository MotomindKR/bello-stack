use super::{Camera as CameraInfo, CameraLayout};
use anyhow::{Context, Result, ensure};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};
use std::path::{Path, PathBuf};

/// A logical camera in hardware.yaml. Calibration paths are relative to that file.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Declaration {
    pub name: String,
    pub link: String,
    pub layout: Layout,
    pub calibration: PathBuf,
    pub fps: u32,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Layout {
    Mono,
    Stereo,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Lens {
    pub camera_to_link: [f64; 7],
    pub intrinsics: [f64; 4],
    #[serde(default)]
    pub fisheye: Option<[f64; 4]>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Calibration {
    /// Pixel dimensions of one eye.
    pub width: u32,
    pub height: u32,
    pub left: Lens,
    pub right: Option<Lens>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Camera {
    pub name: String,
    pub link: String,
    pub fps: u32,
    pub layout: Layout,
    pub calibration: Calibration,
}

impl Camera {
    pub fn lenses(&self) -> impl Iterator<Item = &Lens> {
        std::iter::once(&self.calibration.left).chain(
            self.calibration
                .right
                .iter()
                .filter(|_| matches!(self.layout, Layout::Stereo)),
        )
    }
    pub fn layout(&self) -> CameraLayout {
        match self.layout {
            Layout::Mono => CameraLayout::Mono,
            Layout::Stereo => CameraLayout::StereoSideBySide,
        }
    }
    pub fn packed_width(&self) -> u32 {
        self.calibration.width * self.lenses().count() as u32
    }
    pub fn info(&self, source: &str) -> CameraInfo {
        let lens_metadata = |lens: &Lens| {
            let mut value = serde_json::json!(lens);
            value["link"] = serde_json::json!(self.link);
            value
        };
        let left = lens_metadata(&self.calibration.left);
        let metadata = match self.lenses().nth(1) {
            Some(right) => {
                serde_json::json!({"name":self.name,"stereo":{"left":left,"right":lens_metadata(right)}})
            }
            None => {
                let mut mono = left;
                mono["name"] = serde_json::json!(self.name);
                mono
            }
        };
        CameraInfo {
            name: self.name.clone(),
            width: self.packed_width(),
            height: self.calibration.height,
            fps: self.fps,
            source: source.into(),
            metadata_json: metadata.to_string(),
        }
    }
}

pub fn inventory(
    declarations: Vec<Declaration>,
    hardware_path: &Path,
) -> Result<BTreeMap<String, Camera>> {
    let mut cameras = BTreeMap::new();
    for decl in declarations {
        ensure!(
            !decl.name.is_empty() && !decl.link.is_empty(),
            "camera name and mount are required"
        );
        let path = hardware_path.with_file_name(&decl.calibration);
        let calibration: Calibration = serde_yaml::from_str(
            &std::fs::read_to_string(&path)
                .with_context(|| format!("reading {}", path.display()))?,
        )
        .with_context(|| format!("invalid calibration {}", path.display()))?;
        let camera = Camera {
            name: decl.name,
            link: decl.link,
            fps: decl.fps,
            layout: decl.layout,
            calibration,
        };
        let c = &camera.calibration;
        ensure!(
            (1..=120).contains(&camera.fps)
                && (1..=4096).contains(&c.width)
                && (1..=2160).contains(&c.height),
            "invalid camera format for {}",
            camera.name
        );
        ensure!(
            !matches!(camera.layout, Layout::Stereo) || (c.right.is_some() && c.width <= 2048),
            "invalid stereo calibration for {}",
            camera.name
        );
        for lens in camera.lenses() {
            let [fx, fy, cx, cy] = lens.intrinsics;
            ensure!(
                lens.camera_to_link.iter().all(|v| v.is_finite())
                    && (lens.camera_to_link[3..].iter().map(|v| v * v).sum::<f64>() - 1.).abs()
                        < 1e-5
                    && lens.intrinsics.iter().all(|v| v.is_finite())
                    && fx > 0.
                    && fy > 0.
                    && (0. ..=f64::from(c.width)).contains(&cx)
                    && (0. ..=f64::from(c.height)).contains(&cy)
                    && lens.fisheye.iter().flatten().all(|v| v.is_finite()),
                "invalid optics for {}",
                camera.name
            );
        }
        ensure!(
            cameras.insert(camera.name.clone(), camera).is_none(),
            "duplicate camera declaration"
        );
    }
    Ok(cameras)
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "lowercase", deny_unknown_fields)]
pub enum Transport {
    Mujoco { socket: PathBuf },
    V4l2 { device: PathBuf },
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Binding {
    pub camera: Camera,
    pub transport: Transport,
}

impl Binding {
    pub fn stream_info(&self) -> (CameraInfo, CameraLayout) {
        let source = match self.transport {
            Transport::Mujoco { .. } => "mujoco",
            Transport::V4l2 { .. } => "v4l2",
        };
        (self.camera.info(source), self.camera.layout())
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Source(pub Vec<Binding>);
impl Source {
    pub fn bind(
        mut inventory: BTreeMap<String, Camera>,
        transports: BTreeMap<String, Transport>,
    ) -> Result<Option<Self>> {
        let mut bindings = Vec::new();
        let mut devices = BTreeSet::new();
        ensure!(
            transports.len() <= 16,
            "at most 16 camera streams can be enabled"
        );
        for (name, transport) in transports {
            let camera = inventory
                .remove(&name)
                .with_context(|| format!("deployment maps unknown camera {name}"))?;
            match &transport {
                Transport::Mujoco { socket } => ensure!(
                    socket.is_absolute(),
                    "camera {name} socket must be absolute"
                ),
                Transport::V4l2 { device } => ensure!(
                    device.is_absolute() && devices.insert(device.clone()),
                    "camera {name} requires a distinct absolute device path"
                ),
            }
            bindings.push(Binding { camera, transport });
        }
        Ok((!bindings.is_empty()).then_some(Self(bindings)))
    }
}
