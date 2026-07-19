use anyhow::{Result, bail};

use crate::config::ImuConfig;
use crate::domain::{ImuName, SerialDeviceName};

use std::time::Duration;

pub(super) const POLL_PERIOD: Duration = Duration::from_millis(20);

/// Validated EBIMU configuration. Construction performs no device I/O.
#[derive(Clone, Debug)]
pub(crate) struct ImuSpec {
    pub(crate) name: ImuName,
    pub(crate) serial_device: SerialDeviceName,
    pub(crate) stale_timeout: Duration,
    pub(crate) output_interval: Duration,
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
        if !(1..=60_000).contains(&config.stale_timeout_ms) {
            bail!("imus.{}.stale_timeout_ms must be in 1..=60000", config.name);
        }
        if !(1..=1000).contains(&config.output_rate_hz) {
            bail!("imus.{}.output_rate_hz must be in 1..=1000", config.name);
        }
        if 1000 % config.output_rate_hz != 0 {
            bail!(
                "imus.{}.output_rate_hz must divide 1000 exactly because EBIMU configures an integer millisecond interval",
                config.name
            );
        }
        let output_interval = Duration::from_millis(u64::from(1000 / config.output_rate_hz));
        let minimum_stale_timeout = (output_interval * 2).max(POLL_PERIOD);
        let stale_timeout = Duration::from_millis(config.stale_timeout_ms);
        if stale_timeout < minimum_stale_timeout {
            bail!(
                "imus.{}.stale_timeout_ms must be at least {} for output_rate_hz {}",
                config.name,
                minimum_stale_timeout.as_millis(),
                config.output_rate_hz
            );
        }
        Ok(Self {
            name: ImuName::from(config.name.as_str()),
            serial_device: SerialDeviceName::from(config.serial_device.as_str()),
            stale_timeout,
            output_interval,
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

    #[test]
    fn rejects_output_rate_that_cannot_be_encoded_exactly() {
        let config = ImuConfig {
            output_rate_hz: 333,
            ..Default::default()
        };
        assert!(ImuSpec::try_from(&config).is_err());
    }

    #[test]
    fn stale_timeout_covers_at_least_two_sample_periods() {
        let too_short = ImuConfig {
            output_rate_hz: 10,
            stale_timeout_ms: 199,
            ..Default::default()
        };
        assert!(ImuSpec::try_from(&too_short).is_err());

        let valid = ImuConfig {
            stale_timeout_ms: 200,
            ..too_short
        };
        assert!(ImuSpec::try_from(&valid).is_ok());
    }
}
