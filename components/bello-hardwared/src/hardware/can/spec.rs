use std::time::Duration;

use anyhow::{Context, Result, bail};
use garde::Validate;

use crate::config::{BusConfig, BusTransportConfig};
use crate::domain::BusName;
use crate::hardware::lifecycle::ReconnectPolicy;

const ESTIMATED_EXTENDED_FRAME_BITS: f64 = 150.0;
const TRANSMISSION_MARGIN: u32 = 2;
const TRANSPORT_SCHEDULING_MARGIN: Duration = Duration::from_millis(20);
const PRIORITY_QUEUE_MARGIN: Duration = Duration::from_millis(50);

#[derive(Clone, Debug)]
pub(crate) struct BusSpec {
    pub(crate) name: BusName,
    pub(crate) transport: BusTransportConfig,
    pub(crate) host_id: u8,
    pub(crate) bitrate: u32,
    pub(crate) maximum_utilization: f64,
    pub(crate) reconnect: ReconnectPolicy,
}

impl BusSpec {
    pub(crate) fn estimated_utilization(&self, frames_per_second: f64) -> f64 {
        frames_per_second * ESTIMATED_EXTENDED_FRAME_BITS / f64::from(self.bitrate)
    }

    pub(crate) fn control_batch_budget(&self, frame_count: usize) -> Duration {
        let wire_time = Duration::from_secs_f64(
            frame_count as f64 * ESTIMATED_EXTENDED_FRAME_BITS / f64::from(self.bitrate),
        );
        wire_time.saturating_mul(TRANSMISSION_MARGIN)
    }

    pub(crate) fn priority_batch_budget(&self, frame_count: usize) -> Duration {
        self.control_batch_budget(frame_count) + TRANSPORT_SCHEDULING_MARGIN
    }

    pub(crate) fn priority_batch_lifetime(&self, frame_count: usize) -> Duration {
        PRIORITY_QUEUE_MARGIN + self.priority_batch_budget(frame_count)
    }
}

impl TryFrom<&BusConfig> for BusSpec {
    type Error = anyhow::Error;

    fn try_from(config: &BusConfig) -> Result<Self> {
        config
            .validate()
            .with_context(|| format!("buses.{}", config.name))?;
        match &config.transport {
            BusTransportConfig::Socketcan { interface } if interface.trim().is_empty() => {
                bail!(
                    "buses.{}.transport.interface must be non-empty",
                    config.name
                );
            }
            BusTransportConfig::Gsusb { serial, .. } if serial.trim().is_empty() => {
                bail!("buses.{}.transport.serial must be non-empty", config.name);
            }
            BusTransportConfig::Sim(bus) => bus.validate()?,
            _ => {}
        }
        Ok(Self {
            name: BusName::from(config.name.as_str()),
            transport: config.transport.clone(),
            host_id: config.host_id,
            bitrate: config.bitrate,
            maximum_utilization: config.maximum_utilization,
            reconnect: ReconnectPolicy::try_from(&config.reconnect)
                .with_context(|| format!("buses.{}.reconnect", config.name))?,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn batch_budget_uses_bitrate_and_full_frame_count() {
        let slow: BusConfig = serde_yaml::from_str(
            "name: can\ntransport: { type: socketcan, interface: vcan0 }\nbitrate: 10000\nmaximum_utilization: 1.0",
        )
        .unwrap();
        let fast: BusConfig = serde_yaml::from_str(
            "name: can\ntransport: { type: socketcan, interface: vcan0 }\nbitrate: 1000000\nmaximum_utilization: 1.0",
        )
        .unwrap();
        let slow = BusSpec::try_from(&slow).unwrap();
        let fast = BusSpec::try_from(&fast).unwrap();

        assert_eq!(slow.control_batch_budget(25), Duration::from_millis(750));
        assert!(slow.control_batch_budget(25) > fast.control_batch_budget(25));
        assert!(slow.control_batch_budget(300) > slow.control_batch_budget(225));
    }
}
