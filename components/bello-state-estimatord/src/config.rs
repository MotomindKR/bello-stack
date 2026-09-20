use std::fs;
use std::net::SocketAddr;
use std::path::{Path, PathBuf};
use std::time::Duration;

use anyhow::{Context, bail};
use serde::Deserialize;

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct EstimatorConfig {
    /// Explicit opt-in; missing torque never falls back to kinematic evidence.
    pub load_observer: Option<crate::load::LoadConfig>,
    pub urdf: PathBuf,
    pub hardware_endpoint: String,
    pub listen: SocketAddr,
    #[serde(default = "default_imu")]
    pub imu: String,
    pub imu_frame: String,
    #[serde(default = "default_reference_frame")]
    pub reference_frame: String,
    #[serde(default = "default_base")]
    pub tracked_frame: String,
    pub planar: bool,
    pub ground_height_m: f64,
    #[serde(default = "default_left_foot")]
    pub left_foot: String,
    #[serde(default = "default_right_foot")]
    pub right_foot: String,
    #[serde(default = "default_imu_stale_ms")]
    pub imu_stale_ms: u64,
    #[serde(default = "default_motor_stale_ms")]
    pub motor_stale_ms: u64,
    #[serde(default = "default_orientation_lpf_cutoff_hz")]
    pub orientation_lpf_cutoff_hz: f64,
    #[serde(default = "default_gyro_lpf_cutoff_hz")]
    pub gyro_lpf_cutoff_hz: f64,
    #[serde(default = "default_acceleration_lpf_cutoff_hz")]
    pub acceleration_lpf_cutoff_hz: f64,
    #[serde(default = "default_contact_height_full")]
    pub contact_height_full_m: f64,
    #[serde(default = "default_contact_height_zero")]
    pub contact_height_zero_m: f64,
    #[serde(default = "default_contact_motion_full")]
    pub contact_motion_full_mps: f64,
    #[serde(default = "default_contact_motion_zero")]
    pub contact_motion_zero_mps: f64,
    #[serde(default = "default_contact_world_speed_full")]
    pub contact_world_speed_full_mps: f64,
    #[serde(default = "default_contact_world_speed_zero")]
    pub contact_world_speed_zero_mps: f64,
    #[serde(default = "default_contact_probability_rise_ms")]
    pub contact_probability_rise_ms: u64,
    #[serde(default = "default_contact_probability_fall_ms")]
    pub contact_probability_fall_ms: u64,
}

impl EstimatorConfig {
    pub fn load(path: &Path) -> anyhow::Result<Self> {
        let text = fs::read_to_string(path)
            .with_context(|| format!("read estimator config {}", path.display()))?;
        let mut value: Self = serde_yaml::from_str(&text)
            .with_context(|| format!("parse estimator config {}", path.display()))?;
        let parent = path.parent().unwrap_or_else(|| Path::new("."));
        if let Some(observer) = &mut value.load_observer {
            observer.model = parent.join(&observer.model);
            observer.validate()?;
            if !value.planar {
                bail!("load-aware contact requires the flat-ground model");
            }
        }
        if value.urdf.is_relative() {
            value.urdf = parent.join(&value.urdf);
        }
        if !value.urdf.is_file() {
            bail!("URDF does not exist: {}", value.urdf.display());
        }
        for (name, text) in [
            ("hardware_endpoint", value.hardware_endpoint.as_str()),
            ("imu", value.imu.as_str()),
            ("imu_frame", value.imu_frame.as_str()),
            ("reference_frame", value.reference_frame.as_str()),
            ("tracked_frame", value.tracked_frame.as_str()),
            ("left_foot", value.left_foot.as_str()),
            ("right_foot", value.right_foot.as_str()),
        ] {
            if text.trim().is_empty() || text.trim() != text {
                bail!("{name} must be non-empty without surrounding whitespace");
            }
        }
        if value.imu_stale_ms == 0
            || value.motor_stale_ms == 0
            || value.contact_probability_rise_ms == 0
            || value.contact_probability_fall_ms == 0
        {
            bail!("freshness limits and contact probability time constants must be positive");
        }
        for (name, number) in [
            ("contact_height_full_m", value.contact_height_full_m),
            ("contact_height_zero_m", value.contact_height_zero_m),
            ("contact_motion_full_mps", value.contact_motion_full_mps),
            ("contact_motion_zero_mps", value.contact_motion_zero_mps),
            (
                "contact_world_speed_full_mps",
                value.contact_world_speed_full_mps,
            ),
            (
                "contact_world_speed_zero_mps",
                value.contact_world_speed_zero_mps,
            ),
        ] {
            if !number.is_finite() || number < 0.0 {
                bail!("{name} must be nonnegative and finite");
            }
        }
        if !value.ground_height_m.is_finite() {
            bail!("ground_height_m must be finite");
        }
        for (name, cutoff) in [
            ("orientation_lpf_cutoff_hz", value.orientation_lpf_cutoff_hz),
            ("gyro_lpf_cutoff_hz", value.gyro_lpf_cutoff_hz),
            (
                "acceleration_lpf_cutoff_hz",
                value.acceleration_lpf_cutoff_hz,
            ),
        ] {
            if !cutoff.is_finite() || cutoff <= 0.0 {
                bail!("{name} must be positive and finite");
            }
        }
        if value.contact_height_full_m >= value.contact_height_zero_m
            || value.contact_motion_full_mps >= value.contact_motion_zero_mps
            || value.contact_world_speed_full_mps >= value.contact_world_speed_zero_mps
        {
            bail!(
                "full-confidence contact thresholds must be smaller than zero-confidence thresholds"
            );
        }
        Ok(value)
    }

    pub fn imu_stale(&self) -> Duration {
        Duration::from_millis(self.imu_stale_ms)
    }

    pub fn motor_stale(&self) -> Duration {
        Duration::from_millis(self.motor_stale_ms)
    }

    pub fn contact_probability_rise(&self) -> Duration {
        Duration::from_millis(self.contact_probability_rise_ms)
    }

    pub fn contact_probability_fall(&self) -> Duration {
        Duration::from_millis(self.contact_probability_fall_ms)
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
const fn default_orientation_lpf_cutoff_hz() -> f64 {
    20.0
}
const fn default_gyro_lpf_cutoff_hz() -> f64 {
    15.0
}
const fn default_acceleration_lpf_cutoff_hz() -> f64 {
    25.0
}
const fn default_contact_height_full() -> f64 {
    0.015
}
const fn default_contact_height_zero() -> f64 {
    0.06
}
const fn default_contact_motion_full() -> f64 {
    0.10
}
const fn default_contact_motion_zero() -> f64 {
    0.60
}
const fn default_contact_world_speed_full() -> f64 {
    0.10
}
const fn default_contact_world_speed_zero() -> f64 {
    0.50
}
const fn default_contact_probability_rise_ms() -> u64 {
    80
}
const fn default_contact_probability_fall_ms() -> u64 {
    20
}
