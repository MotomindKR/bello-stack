use anyhow::{Result, bail};

use crate::config::BusConfig;
use crate::domain::BusName;
use crate::hardware::lifecycle::ReconnectPolicy;

/// Side-effect-free, validated definition of one CAN bus.
#[derive(Clone, Debug)]
pub(crate) struct BusSpec {
    pub(crate) name: BusName,
    pub(crate) interface: String,
    pub(crate) host_id: u8,
    pub(crate) bitrate: u32,
    pub(crate) maximum_utilization: f64,
    pub(crate) reconnect: ReconnectPolicy,
}

impl TryFrom<&BusConfig> for BusSpec {
    type Error = anyhow::Error;

    fn try_from(config: &BusConfig) -> Result<Self> {
        if config.name.is_empty() {
            bail!("buses[].name must be non-empty");
        }
        if config.interface.is_empty() {
            bail!("buses.{}.interface must be non-empty", config.name);
        }
        if config.bitrate == 0 {
            bail!("buses.{}.bitrate must be positive", config.name);
        }
        if !config.maximum_utilization.is_finite()
            || !(0.0..=1.0).contains(&config.maximum_utilization)
            || config.maximum_utilization == 0.0
        {
            bail!(
                "buses.{}.maximum_utilization must be in (0, 1]",
                config.name
            );
        }
        Ok(Self {
            name: BusName::from(config.name.as_str()),
            interface: config.interface.clone(),
            host_id: config.host_id,
            bitrate: config.bitrate,
            maximum_utilization: config.maximum_utilization,
            reconnect: ReconnectPolicy::try_from((&config.reconnect, "buses[].reconnect"))?,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rejects_zero_bitrate() {
        let config: BusConfig = serde_yaml::from_str(
            "name: can\ninterface: vcan0\nbitrate: 0\nmaximum_utilization: 0.5",
        )
        .unwrap();
        assert!(BusSpec::try_from(&config).is_err());
    }
}
