use anyhow::{Result, bail};

use crate::config::ImuConfig;
use crate::domain::{ImuName, SerialDeviceName};

use std::time::Duration;

/// Validated EBIMU configuration. Construction performs no device I/O.
#[derive(Clone, Debug)]
pub(crate) struct ImuSpec {
    pub(crate) name: ImuName,
    pub(crate) enabled: bool,
    pub(crate) serial_device: SerialDeviceName,
    pub(crate) model: String,
    pub(crate) stale_timeout: Duration,
    pub(crate) output_rate_hz: u32,
    pub(crate) quaternion: bool,
    pub(crate) gyro: bool,
    pub(crate) acceleration: bool,
    pub(crate) device_timestamp: bool,
    pub(crate) configure_on_start: bool,
}

impl TryFrom<&ImuConfig> for ImuSpec {
    type Error = anyhow::Error;

    fn try_from(config: &ImuConfig) -> Result<Self> {
        if config.name.is_empty() {
            bail!("imus[].name must be non-empty");
        }
        if config.serial_device.is_empty() {
            bail!("imus.{}.serial_device must be non-empty", config.name);
        }
        if config.model != "EBIMU" {
            bail!("imus.{}.model must currently be EBIMU", config.name);
        }
        if config.enabled {
            if config.stale_timeout_ms == 0 {
                bail!("imus.{}.stale_timeout_ms must be positive", config.name);
            }
            if !(1..=1000).contains(&config.output_rate_hz) {
                bail!(
                    "imus.{}.output_rate_hz must be in 1..=1000 when enabled",
                    config.name
                );
            }
        }
        Ok(Self {
            name: ImuName::from(config.name.as_str()),
            enabled: config.enabled,
            serial_device: SerialDeviceName::from(config.serial_device.as_str()),
            model: config.model.clone(),
            stale_timeout: Duration::from_millis(config.stale_timeout_ms),
            output_rate_hz: config.output_rate_hz,
            quaternion: config.quaternion,
            gyro: config.gyro,
            acceleration: config.acceleration,
            device_timestamp: config.device_timestamp,
            configure_on_start: config.configure_on_start,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rejects_output_rate_above_device_limit() {
        let config = ImuConfig {
            output_rate_hz: 1001,
            ..Default::default()
        };
        assert!(ImuSpec::try_from(&config).is_err());
    }
}
