use std::time::Duration;

use anyhow::{Result, bail};

use crate::config::SerialDeviceConfig;
use crate::domain::SerialDeviceName;
use crate::hardware::lifecycle::ReconnectPolicy;

#[derive(Clone, Debug)]
pub(crate) struct SerialSpec {
    pub(crate) name: SerialDeviceName,
    pub(crate) port: String,
    pub(crate) baud_rate: u32,
    pub(crate) timeout: Duration,
    pub(super) reconnect: ReconnectPolicy,
}

impl TryFrom<&SerialDeviceConfig> for SerialSpec {
    type Error = anyhow::Error;

    fn try_from(config: &SerialDeviceConfig) -> Result<Self> {
        if config.name.is_empty() {
            bail!("serial_devices[].name must be non-empty");
        }
        if config.port.is_empty() {
            bail!("serial_devices.{}.port must be non-empty", config.name);
        }
        if config.baud_rate == 0 {
            bail!("serial_devices.{}.baud_rate must be positive", config.name);
        }
        if config.timeout_ms == 0 {
            bail!("serial_devices.{}.timeout_ms must be positive", config.name);
        }
        Ok(Self {
            name: SerialDeviceName::from(config.name.as_str()),
            port: config.port.clone(),
            baud_rate: config.baud_rate,
            timeout: Duration::from_millis(config.timeout_ms),
            reconnect: ReconnectPolicy::try_from((
                &config.reconnect,
                "serial_devices[].reconnect",
            ))?,
        })
    }
}
