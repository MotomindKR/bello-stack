use std::fs;
use std::net::SocketAddr;
use std::path::{Path, PathBuf};
use std::time::Duration;

use anyhow::{Context, bail};
use serde::Deserialize;

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct EstimatorConfig {
    pub urdf: PathBuf,
    pub hardware_endpoint: String,
    pub listen: SocketAddr,
    #[serde(default = "default_imu")]
    pub imu: String,
    #[serde(default = "default_reference_frame")]
    pub reference_frame: String,
    #[serde(default = "default_base")]
    pub tracked_frame: String,
    #[serde(default = "default_left_foot")]
    pub left_foot: String,
    #[serde(default = "default_right_foot")]
    pub right_foot: String,
    #[serde(default = "default_imu_stale_ms")]
    pub imu_stale_ms: u64,
    #[serde(default = "default_motor_stale_ms")]
    pub motor_stale_ms: u64,
    #[serde(default = "default_contact_entry_speed")]
    pub contact_entry_speed_mps: f64,
    #[serde(default = "default_contact_exit_speed")]
    pub contact_exit_speed_mps: f64,
    #[serde(default = "default_contact_entry_height")]
    pub contact_entry_height_m: f64,
    #[serde(default = "default_contact_exit_height")]
    pub contact_exit_height_m: f64,
    #[serde(default = "default_contact_dwell_ms")]
    pub contact_dwell_ms: u64,
}

impl EstimatorConfig {
    pub fn load(path: &Path) -> anyhow::Result<Self> {
        let text = fs::read_to_string(path)
            .with_context(|| format!("read estimator config {}", path.display()))?;
        let mut value: Self = serde_yaml::from_str(&text)
            .with_context(|| format!("parse estimator config {}", path.display()))?;
        let parent = path.parent().unwrap_or_else(|| Path::new("."));
        if value.urdf.is_relative() {
            value.urdf = parent.join(&value.urdf);
        }
        if !value.urdf.is_file() {
            bail!("URDF does not exist: {}", value.urdf.display());
        }
        for (name, text) in [
            ("hardware_endpoint", value.hardware_endpoint.as_str()),
            ("imu", value.imu.as_str()),
            ("reference_frame", value.reference_frame.as_str()),
            ("tracked_frame", value.tracked_frame.as_str()),
            ("left_foot", value.left_foot.as_str()),
            ("right_foot", value.right_foot.as_str()),
        ] {
            if text.trim().is_empty() || text.trim() != text {
                bail!("{name} must be non-empty without surrounding whitespace");
            }
        }
        if value.imu_stale_ms == 0 || value.motor_stale_ms == 0 || value.contact_dwell_ms == 0 {
            bail!("freshness limits and contact_dwell_ms must be positive");
        }
        for (name, number) in [
            ("contact_entry_speed_mps", value.contact_entry_speed_mps),
            ("contact_exit_speed_mps", value.contact_exit_speed_mps),
            ("contact_entry_height_m", value.contact_entry_height_m),
            ("contact_exit_height_m", value.contact_exit_height_m),
        ] {
            if !number.is_finite() || number <= 0.0 {
                bail!("{name} must be positive and finite");
            }
        }
        if value.contact_entry_speed_mps >= value.contact_exit_speed_mps
            || value.contact_entry_height_m >= value.contact_exit_height_m
        {
            bail!("contact entry thresholds must be smaller than exit thresholds");
        }
        Ok(value)
    }

    pub fn imu_stale(&self) -> Duration {
        Duration::from_millis(self.imu_stale_ms)
    }

    pub fn motor_stale(&self) -> Duration {
        Duration::from_millis(self.motor_stale_ms)
    }

    pub fn contact_dwell(&self) -> Duration {
        Duration::from_millis(self.contact_dwell_ms)
    }
}

fn default_imu() -> String {
    "imu".into()
}
fn default_reference_frame() -> String {
    "odom".into()
}
fn default_base() -> String {
    "pelvis_link".into()
}
fn default_left_foot() -> String {
    "left_ankle_roll_link".into()
}
fn default_right_foot() -> String {
    "right_ankle_roll_link".into()
}
const fn default_imu_stale_ms() -> u64 {
    20
}
const fn default_motor_stale_ms() -> u64 {
    50
}
const fn default_contact_entry_speed() -> f64 {
    0.15
}
const fn default_contact_exit_speed() -> f64 {
    0.30
}
const fn default_contact_entry_height() -> f64 {
    0.03
}
const fn default_contact_exit_height() -> f64 {
    0.06
}
const fn default_contact_dwell_ms() -> u64 {
    40
}
