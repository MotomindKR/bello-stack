use anyhow::{Result, bail};

use crate::config::RuntimeConfig;

#[derive(Clone, Debug)]
pub struct RuntimeSpec {
    pub(crate) telemetry_capacity: usize,
}

impl TryFrom<&RuntimeConfig> for RuntimeSpec {
    type Error = anyhow::Error;

    fn try_from(config: &RuntimeConfig) -> Result<Self> {
        if config.telemetry_capacity == 0 {
            bail!("runtime.telemetry_capacity must be positive");
        }
        Ok(Self {
            telemetry_capacity: config.telemetry_capacity,
        })
    }
}
