use std::fs;
use std::net::SocketAddr;
use std::path::{Path, PathBuf};
use std::time::Duration;

use anyhow::{Context, bail};
use bello_config::{finite, nonnegative, positive, trimmed_name};
use garde::Validate;
use serde::Deserialize;

#[derive(Clone, Debug, Deserialize, Validate)]
#[garde(allow_unvalidated)]
#[serde(deny_unknown_fields)]
pub struct EstimatorConfig {
    pub urdf: PathBuf,
    #[garde(custom(trimmed_name))]
    pub hardware_endpoint: String,
    pub listen: SocketAddr,
    #[serde(default = "default_imu")]
    #[garde(custom(trimmed_name))]
    pub imu: String,
    #[garde(custom(trimmed_name))]
    pub imu_frame: String,
    #[serde(default = "default_reference_frame")]
    #[garde(custom(trimmed_name))]
    pub reference_frame: String,
    #[serde(default = "default_base")]
    #[garde(custom(trimmed_name))]
    pub tracked_frame: String,
    pub planar: bool,
    #[garde(custom(finite))]
    pub ground_height_m: f64,
    #[serde(default = "default_left_foot")]
    #[garde(custom(trimmed_name))]
    pub left_foot: String,
    #[serde(default = "default_right_foot")]
    #[garde(custom(trimmed_name))]
    pub right_foot: String,
    #[serde(default = "default_imu_stale_ms")]
    #[garde(range(min = 1))]
    pub imu_stale_ms: u64,
    #[serde(default = "default_motor_stale_ms")]
    #[garde(range(min = 1))]
    pub motor_stale_ms: u64,
    #[serde(default = "default_orientation_lpf_cutoff_hz")]
    #[garde(custom(positive))]
    pub orientation_lpf_cutoff_hz: f64,
    #[serde(default = "default_gyro_lpf_cutoff_hz")]
    #[garde(custom(positive))]
    pub gyro_lpf_cutoff_hz: f64,
    #[serde(default = "default_acceleration_lpf_cutoff_hz")]
    #[garde(custom(positive))]
    pub acceleration_lpf_cutoff_hz: f64,
    #[serde(default = "default_contact_height_full")]
    #[garde(custom(nonnegative))]
    pub contact_height_full_m: f64,
    #[serde(default = "default_contact_height_zero")]
    #[garde(custom(nonnegative))]
    pub contact_height_zero_m: f64,
    #[serde(default = "default_contact_motion_full")]
    #[garde(custom(nonnegative))]
    pub contact_motion_full_mps: f64,
    #[serde(default = "default_contact_motion_zero")]
    #[garde(custom(nonnegative))]
    pub contact_motion_zero_mps: f64,
    #[serde(default = "default_contact_world_speed_full")]
    #[garde(custom(nonnegative))]
    pub contact_world_speed_full_mps: f64,
    #[serde(default = "default_contact_world_speed_zero")]
    #[garde(custom(nonnegative))]
    pub contact_world_speed_zero_mps: f64,
    #[serde(default = "default_contact_probability_rise_ms")]
    #[garde(range(min = 1))]
    pub contact_probability_rise_ms: u64,
    #[serde(default = "default_contact_probability_fall_ms")]
    #[garde(range(min = 1))]
    pub contact_probability_fall_ms: u64,
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
        value.validate()?;
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
