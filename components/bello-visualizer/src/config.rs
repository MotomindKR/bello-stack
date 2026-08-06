use std::collections::BTreeMap;
use std::fs;
use std::net::SocketAddr;
use std::path::{Path, PathBuf};

use bello_hardware_client::Motor;
use bello_robot_description::RobotDescription;
use serde::Deserialize;
use thiserror::Error;

use crate::Transform;

const fn default_refresh_hz() -> f64 {
    60.0
}
const fn default_command_rate_hz() -> f64 {
    50.0
}
const fn default_validity_ms() -> u64 {
    100
}
const fn default_staleness_ms() -> u64 {
    200
}
const fn default_position_slew_rate() -> f64 {
    0.5
}
const fn default_velocity_bounds() -> BoundedDefault {
    BoundedDefault {
        min: -2.0,
        max: 2.0,
        default: 0.0,
    }
}
const fn default_torque_bounds() -> BoundedDefault {
    BoundedDefault {
        min: -5.0,
        max: 5.0,
        default: 0.0,
    }
}
const fn default_kp_bounds() -> BoundedDefault {
    BoundedDefault {
        min: 0.0,
        max: 100.0,
        default: 20.0,
    }
}
const fn default_kd_bounds() -> BoundedDefault {
    BoundedDefault {
        min: 0.0,
        max: 5.0,
        default: 1.0,
    }
}

#[derive(Debug, Error)]
pub enum ConfigError {
    #[error("could not read visualizer configuration {path}: {source}")]
    Read {
        path: PathBuf,
        source: std::io::Error,
    },
    #[error("invalid visualizer YAML: {0}")]
    Yaml(#[from] serde_yaml::Error),
    #[error("invalid visualizer configuration: {0}")]
    Invalid(String),
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct VisualizerConfig {
    pub urdf: PathBuf,
    pub hardware: EndpointConfig,
    pub remote: RemoteConfig,
    #[serde(default)]
    pub root: RootConfig,
    #[serde(default)]
    pub control_profiles: BTreeMap<String, GroupControlProfile>,
    #[serde(default = "default_refresh_hz")]
    pub refresh_hz: f64,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RemoteConfig {
    pub listen: SocketAddr,
    pub controller_index: usize,
    pub poll_hz: f64,
    pub deadzone: f64,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct EndpointConfig {
    pub endpoint: String,
    pub tls: Option<TlsConfig>,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TlsConfig {
    pub ca_certificate: Option<PathBuf>,
    pub client_certificate: Option<PathBuf>,
    pub client_private_key: Option<PathBuf>,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(tag = "source", rename_all = "snake_case", deny_unknown_fields)]
pub enum RootConfig {
    Fixed {
        #[serde(default)]
        transform: Transform,
    },
    ExternalPose {
        endpoint: String,
        tls: Option<TlsConfig>,
        reference_frame: String,
        tracked_frame: String,
        #[serde(default)]
        tracked_from_model_root: Transform,
        #[serde(default = "default_staleness_ms")]
        staleness_ms: u64,
    },
}

impl Default for RootConfig {
    fn default() -> Self {
        Self::Fixed {
            transform: Transform::IDENTITY,
        }
    }
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GroupControlProfile {
    #[serde(default = "default_command_rate_hz")]
    pub command_rate_hz: f64,
    #[serde(default = "default_validity_ms")]
    pub validity_ms: u64,
    #[serde(default = "default_staleness_ms")]
    pub telemetry_staleness_ms: u64,
    #[serde(default = "default_position_slew_rate")]
    pub position_slew_rate: f64,
    #[serde(default = "default_velocity_bounds")]
    pub velocity: BoundedDefault,
    #[serde(default = "default_torque_bounds")]
    pub torque_feedforward: BoundedDefault,
    #[serde(default = "default_kp_bounds")]
    pub kp: BoundedDefault,
    #[serde(default = "default_kd_bounds")]
    pub kd: BoundedDefault,
}

#[derive(Clone, Debug)]
pub struct ResolvedGroupControlProfile {
    pub command_rate_hz: f64,
    pub validity_ms: u64,
    pub telemetry_staleness_ms: u64,
    pub position_slew_rate: f64,
    pub coordinates: BTreeMap<String, CoordinateControlProfile>,
}

#[derive(Clone, Debug)]
pub struct CoordinateControlProfile {
    pub position: BoundedDefault,
    pub velocity: BoundedDefault,
    pub torque_feedforward: BoundedDefault,
    pub kp: BoundedDefault,
    pub kd: BoundedDefault,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CoordinateLimits {
    pub position_min: f64,
    pub position_max: f64,
    pub velocity_min: f64,
    pub velocity_max: f64,
    pub torque_min: f64,
    pub torque_max: f64,
    pub kp_min: f64,
    pub kp_max: f64,
    pub kd_min: f64,
    pub kd_max: f64,
}

impl From<&Motor> for CoordinateLimits {
    fn from(motor: &Motor) -> Self {
        let limits = motor.limits();
        Self {
            position_min: limits.position_min(),
            position_max: limits.position_max(),
            velocity_min: limits.velocity_min(),
            velocity_max: limits.velocity_max(),
            torque_min: limits.torque_min(),
            torque_max: limits.torque_max(),
            kp_min: limits.kp_min(),
            kp_max: limits.kp_max(),
            kd_min: limits.kd_min(),
            kd_max: limits.kd_max(),
        }
    }
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct BoundedDefault {
    pub min: f64,
    pub max: f64,
    pub default: f64,
}

impl VisualizerConfig {
    pub fn load(path: impl AsRef<Path>) -> Result<Self, ConfigError> {
        let path = path.as_ref();
        let yaml = fs::read_to_string(path).map_err(|source| ConfigError::Read {
            path: path.to_owned(),
            source,
        })?;
        let mut config: Self = serde_yaml::from_str(&yaml)?;
        let base = path.parent().unwrap_or_else(|| Path::new("."));
        make_absolute(&mut config.urdf, base);
        resolve_tls(&mut config.hardware.tls, base);
        if let RootConfig::ExternalPose { tls, .. } = &mut config.root {
            resolve_tls(tls, base);
        }
        config.normalize_transforms()?;
        config.validate()?;
        Ok(config)
    }

    fn normalize_transforms(&mut self) -> Result<(), ConfigError> {
        let (transform, context) = match &mut self.root {
            RootConfig::Fixed { transform } => (transform, "fixed root transform"),
            RootConfig::ExternalPose {
                tracked_from_model_root,
                ..
            } => (tracked_from_model_root, "tracked_from_model_root"),
        };
        *transform = transform
            .checked(context)
            .map_err(|error| ConfigError::Invalid(error.to_string()))?;
        Ok(())
    }

    pub fn validate(&self) -> Result<(), ConfigError> {
        if !self.refresh_hz.is_finite() || self.refresh_hz <= 0.0 {
            return Err(ConfigError::Invalid("refresh_hz must be positive".into()));
        }
        if self.hardware.endpoint.trim().is_empty() {
            return Err(ConfigError::Invalid(
                "hardware.endpoint must be non-empty".into(),
            ));
        }
        if !self.remote.poll_hz.is_finite() || self.remote.poll_hz <= 0.0 {
            return Err(ConfigError::Invalid(
                "remote.poll_hz must be positive and finite".into(),
            ));
        }
        if std::time::Duration::from_secs_f64(1.0 / self.remote.poll_hz).is_zero() {
            return Err(ConfigError::Invalid(
                "remote.poll_hz is too large for the system timer".into(),
            ));
        }
        if !self.remote.deadzone.is_finite()
            || self.remote.deadzone < 0.0
            || self.remote.deadzone >= 1.0
        {
            return Err(ConfigError::Invalid(
                "remote.deadzone must be finite and in [0, 1)".into(),
            ));
        }
        validate_tls(self.hardware.tls.as_ref(), "hardware.tls")?;
        match &self.root {
            RootConfig::Fixed { transform } => {
                transform
                    .checked("fixed root transform")
                    .map_err(|error| ConfigError::Invalid(error.to_string()))?;
            }
            RootConfig::ExternalPose {
                endpoint,
                tls,
                reference_frame,
                tracked_frame,
                tracked_from_model_root,
                staleness_ms,
            } => {
                if endpoint.trim().is_empty()
                    || reference_frame.trim().is_empty()
                    || tracked_frame.trim().is_empty()
                {
                    return Err(ConfigError::Invalid(
                        "external pose endpoint and frame names must be non-empty".into(),
                    ));
                }
                if *staleness_ms == 0 {
                    return Err(ConfigError::Invalid(
                        "external pose staleness_ms must be positive".into(),
                    ));
                }
                validate_tls(tls.as_ref(), "root.tls")?;
                tracked_from_model_root
                    .checked("tracked_from_model_root")
                    .map_err(|error| ConfigError::Invalid(error.to_string()))?;
            }
        }
        for (group, profile) in &self.control_profiles {
            if group.trim().is_empty() {
                return Err(ConfigError::Invalid(
                    "control profile names must be non-empty".into(),
                ));
            }
            if !profile.command_rate_hz.is_finite() || profile.command_rate_hz <= 0.0 {
                return Err(ConfigError::Invalid(format!(
                    "control profile {group:?} command_rate_hz must be positive"
                )));
            }
            if profile.validity_ms == 0 || profile.telemetry_staleness_ms == 0 {
                return Err(ConfigError::Invalid(format!(
                    "control profile {group:?} validity and telemetry staleness must be positive"
                )));
            }
            if !profile.position_slew_rate.is_finite() || profile.position_slew_rate <= 0.0 {
                return Err(ConfigError::Invalid(format!(
                    "control profile {group:?} position_slew_rate must be positive"
                )));
            }
            for (field, range) in [
                ("velocity", profile.velocity),
                ("torque_feedforward", profile.torque_feedforward),
                ("kp", profile.kp),
                ("kd", profile.kd),
            ] {
                range.validate(group, "all coordinates", field)?;
            }
            if profile.kp.min < 0.0 || profile.kd.min < 0.0 {
                return Err(ConfigError::Invalid(format!(
                    "control profile {group:?} gain bounds must be non-negative"
                )));
            }
        }
        Ok(())
    }
}

impl GroupControlProfile {
    pub fn resolve<'a>(
        &self,
        description: &RobotDescription,
        motors: impl IntoIterator<Item = &'a Motor>,
    ) -> Result<ResolvedGroupControlProfile, ConfigError> {
        self.resolve_limits(
            description,
            motors
                .into_iter()
                .map(|motor| (motor.coordinate().to_owned(), CoordinateLimits::from(motor))),
        )
    }

    pub fn resolve_limits(
        &self,
        description: &RobotDescription,
        coordinates: impl IntoIterator<Item = (String, CoordinateLimits)>,
    ) -> Result<ResolvedGroupControlProfile, ConfigError> {
        let mut resolved = BTreeMap::new();
        for (coordinate, limits) in coordinates {
            let hardware = [
                limits.position_min,
                limits.position_max,
                limits.velocity_min,
                limits.velocity_max,
                limits.torque_min,
                limits.torque_max,
                limits.kp_min,
                limits.kp_max,
                limits.kd_min,
                limits.kd_max,
            ];
            if hardware.iter().any(|value| !value.is_finite())
                || limits.position_min > limits.position_max
                || limits.velocity_min > limits.velocity_max
                || limits.torque_min > limits.torque_max
                || limits.kp_min < 0.0
                || limits.kp_min > limits.kp_max
                || limits.kd_min < 0.0
                || limits.kd_min > limits.kd_max
            {
                return Err(ConfigError::Invalid(format!(
                    "hardware coordinate {coordinate:?} has invalid live command limits"
                )));
            }
            let (model_position_min, model_position_max) = description
                .coordinate_position_limits(&coordinate)
                .ok_or_else(|| {
                    ConfigError::Invalid(format!(
                        "hardware coordinate {coordinate:?} has no URDF joint or differential actuator with finite limits"
                    ))
                })?;
            let position_min = limits.position_min.max(model_position_min);
            let position_max = limits.position_max.min(model_position_max);
            if position_min > position_max {
                return Err(ConfigError::Invalid(format!(
                    "hardware and URDF position limits for {coordinate:?} do not overlap"
                )));
            }
            resolved.insert(
                coordinate.clone(),
                CoordinateControlProfile {
                    position: BoundedDefault {
                        min: position_min,
                        max: position_max,
                        default: 0.0_f64.clamp(position_min, position_max),
                    },
                    velocity: self.velocity.intersect(
                        limits.velocity_min,
                        limits.velocity_max,
                        &coordinate,
                        "velocity",
                    )?,
                    torque_feedforward: self.torque_feedforward.intersect(
                        limits.torque_min,
                        limits.torque_max,
                        &coordinate,
                        "torque_feedforward",
                    )?,
                    kp: self
                        .kp
                        .intersect(limits.kp_min, limits.kp_max, &coordinate, "kp")?,
                    kd: self
                        .kd
                        .intersect(limits.kd_min, limits.kd_max, &coordinate, "kd")?,
                },
            );
        }
        if resolved.is_empty() {
            return Err(ConfigError::Invalid(
                "cannot resolve a control profile without hardware coordinates".into(),
            ));
        }
        Ok(ResolvedGroupControlProfile {
            command_rate_hz: self.command_rate_hz,
            validity_ms: self.validity_ms,
            telemetry_staleness_ms: self.telemetry_staleness_ms,
            position_slew_rate: self.position_slew_rate,
            coordinates: resolved,
        })
    }
}

impl BoundedDefault {
    pub(crate) fn clamp_feedback_position(self, value: f64) -> Option<f64> {
        if !value.is_finite() {
            return None;
        }
        Some(value.clamp(self.min, self.max))
    }

    fn validate(self, group: &str, coordinate: &str, field: &str) -> Result<(), ConfigError> {
        if ![self.min, self.max, self.default]
            .iter()
            .all(|value| value.is_finite())
            || self.min > self.max
            || self.default < self.min
            || self.default > self.max
        {
            return Err(ConfigError::Invalid(format!(
                "control profile {group:?} coordinate {coordinate:?} has invalid {field} bounds/default"
            )));
        }
        Ok(())
    }

    fn intersect(
        self,
        hardware_min: f64,
        hardware_max: f64,
        coordinate: &str,
        field: &str,
    ) -> Result<Self, ConfigError> {
        let min = self.min.max(hardware_min);
        let max = self.max.min(hardware_max);
        if min > max || self.default < min || self.default > max {
            return Err(ConfigError::Invalid(format!(
                "control profile default/range for {coordinate:?} {field} is incompatible with live hardware [{hardware_min}, {hardware_max}]"
            )));
        }
        Ok(Self {
            min,
            max,
            default: self.default,
        })
    }
}

fn validate_tls(tls: Option<&TlsConfig>, context: &str) -> Result<(), ConfigError> {
    let Some(tls) = tls else {
        return Ok(());
    };
    if tls.client_certificate.is_some() != tls.client_private_key.is_some() {
        return Err(ConfigError::Invalid(format!(
            "{context} client_certificate and client_private_key must be supplied together"
        )));
    }
    Ok(())
}

fn resolve_tls(tls: &mut Option<TlsConfig>, base: &Path) {
    let Some(tls) = tls else {
        return;
    };
    for path in [
        &mut tls.ca_certificate,
        &mut tls.client_certificate,
        &mut tls.client_private_key,
    ]
    .into_iter()
    .flatten()
    {
        make_absolute(path, base);
    }
}

fn make_absolute(path: &mut PathBuf, base: &Path) {
    if path.is_relative() {
        *path = base.join(&*path);
    }
}
