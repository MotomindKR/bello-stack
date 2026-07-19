use std::net::SocketAddr;

use anyhow::{Result, bail};

use crate::config::GrpcConfig;

#[derive(Clone, Debug)]
pub struct GrpcSpec {
    pub(crate) listen: SocketAddr,
    pub(crate) default_lease_ms: u64,
    pub(crate) maximum_lease_ms: u64,
}

impl GrpcSpec {
    pub fn listen(&self) -> SocketAddr {
        self.listen
    }
}

impl TryFrom<&GrpcConfig> for GrpcSpec {
    type Error = anyhow::Error;

    fn try_from(config: &GrpcConfig) -> Result<Self> {
        if !(20..=60_000).contains(&config.default_lease_ms)
            || config.default_lease_ms > config.maximum_lease_ms
            || config.maximum_lease_ms > 300_000
        {
            bail!("grpc lease durations must satisfy 20 <= default <= maximum <= 300000 ms");
        }
        Ok(Self {
            listen: config.listen,
            default_lease_ms: config.default_lease_ms,
            maximum_lease_ms: config.maximum_lease_ms,
        })
    }
}
