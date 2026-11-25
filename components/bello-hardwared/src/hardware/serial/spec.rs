use std::time::Duration;

use anyhow::{Context, Result};
use garde::Validate;

use crate::config::SerialDeviceConfig;
use crate::domain::SerialDeviceName;
use crate::hardware::lifecycle::ReconnectPolicy;

#[derive(Clone, Debug)]
pub(crate) struct SerialSpec {
    pub(crate) name: SerialDeviceName,
    pub(crate) port: String,
    pub(crate) baud_rate: u32,
    pub(crate) timeout: Duration,
    pub(super) simulation: bool,
    pub(super) reconnect: ReconnectPolicy,
}

impl SerialSpec {
    pub(crate) fn resolve(config: &SerialDeviceConfig, simulation: bool) -> Result<Self> {
        config
            .validate()
            .with_context(|| format!("serial_devices.{}", config.name))?;
        Ok(Self {
            simulation,
            name: SerialDeviceName::from(config.name.as_str()),
            port: config.port.clone(),
            baud_rate: config.baud_rate,
            timeout: Duration::from_millis(config.timeout_ms),
            reconnect: ReconnectPolicy::try_from(&config.reconnect)
                .with_context(|| format!("serial_devices.{}.reconnect", config.name))?,
        })
    }
}
