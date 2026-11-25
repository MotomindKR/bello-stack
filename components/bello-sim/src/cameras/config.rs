use super::projection;
use anyhow::{Result, ensure};
use bello_camera_api::{HEIGHT, WIDTH, proto};
use serde::{Deserialize, Serialize};
use std::path::{Path, PathBuf};

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
pub struct Camera {
    pub name: String,
    #[serde(flatten)]
    pub views: Views,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(untagged, deny_unknown_fields)]
pub enum Views {
    Mono { mono: Lens },
    Stereo { stereo: Stereo },
    // Accept mount and intrinsics directly on a named mono camera.
    Inline(Lens),
}
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Stereo {
    left: Lens,
    right: Lens,
}
impl Camera {
    pub(super) fn metadata(&self) -> serde_json::Value {
        match &self.views {
            Views::Mono { mono } | Views::Inline(mono) => {
                let mut value = serde_json::json!(mono);
                value["name"] = serde_json::json!(self.name);
                value
            }
            Views::Stereo { .. } => serde_json::json!(self),
        }
    }
}
impl Views {
    pub fn lenses(&self) -> impl Iterator<Item = &Lens> {
        let (first, second) = match self {
            Self::Mono { mono } | Self::Inline(mono) => (mono, None),
            Self::Stereo { stereo } => (&stereo.left, Some(&stereo.right)),
        };
        std::iter::once(first).chain(second)
    }
    pub fn layout(&self) -> proto::CameraLayout {
        match self {
            Self::Stereo { .. } => proto::CameraLayout::StereoSideBySide,
            _ => proto::CameraLayout::Mono,
        }
    }
}
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Lens {
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
    pub(super) fn validate(&self) -> Result<()> {
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
        for camera in &self.cameras {
            ensure!(!camera.name.is_empty(), "missing camera name");
            ensure!(
                camera.views.layout() != proto::CameraLayout::StereoSideBySide
                    || self.width <= 2048,
                "packed stereo width exceeds 4096 pixels"
            );
        }
        for c in self.cameras.iter().flat_map(|camera| camera.views.lenses()) {
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mono_accepts_inline_optics_and_rejects_mixed_layouts() {
        let lens = serde_json::json!({"link":"base","camera_to_link":[0,0,0,1,0,0,0],"intrinsics":[100,100,32,32]});
        let mut inline = lens.clone();
        inline["name"] = "front".into();
        let mono: Camera =
            serde_json::from_value(serde_json::json!({"name":"front","mono":lens})).unwrap();
        assert_eq!(
            mono.metadata(),
            serde_json::from_value::<Camera>(inline).unwrap().metadata()
        );
        let mixed =
            serde_json::json!({"name":"ambiguous","mono":lens,"stereo":{"left":lens,"right":lens}});
        assert!(serde_json::from_value::<Camera>(mixed).is_err());
    }
}
