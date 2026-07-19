use std::fs;
use std::path::Path;
use std::time::Duration;

use anyhow::{Context, Result, bail};
use serde::Deserialize;
use tonic::transport::{ClientTlsConfig, Endpoint};

const MAXIMUM_RECONNECT_DELAY_MS: u64 = 60_000;
const CONNECT_TIMEOUT: Duration = Duration::from_secs(5);
const KEEPALIVE_INTERVAL: Duration = Duration::from_secs(10);
const KEEPALIVE_TIMEOUT: Duration = Duration::from_secs(3);

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Config {
    pub hardware_endpoint: String,
    #[serde(default)]
    pub reconnect: ReconnectConfig,
    pub output: OutputConfig,
}

impl Config {
    pub fn load(path: &Path) -> Result<Self> {
        let source = fs::read_to_string(path)
            .with_context(|| format!("reading Rerun configuration {}", path.display()))?;
        let config: Self = serde_yaml::from_str(&source)
            .with_context(|| format!("parsing Rerun configuration {}", path.display()))?;
        config.validate()?;
        Ok(config)
    }

    pub(crate) fn validate(&self) -> Result<()> {
        if self.hardware_endpoint.trim().is_empty() {
            bail!("hardware_endpoint must be non-empty");
        }
        if self.hardware_endpoint.trim() != self.hardware_endpoint {
            bail!("hardware_endpoint must not have leading or trailing whitespace");
        }
        if !self.hardware_endpoint.starts_with("http://")
            && !self.hardware_endpoint.starts_with("https://")
        {
            bail!("hardware_endpoint must start with http:// or https://");
        }
        self.endpoint()?;
        self.reconnect.validate()?;
        self.output.validate()
    }

    pub(crate) fn endpoint(&self) -> Result<Endpoint> {
        let mut endpoint = Endpoint::from_shared(self.hardware_endpoint.clone())
            .context("hardware_endpoint must be a valid HTTP or HTTPS endpoint")?;
        if self.hardware_endpoint.starts_with("https://") {
            endpoint = endpoint
                .tls_config(
                    ClientTlsConfig::new()
                        .with_native_roots()
                        .assume_http2(true),
                )
                .context("configuring hardware_endpoint TLS")?;
        }
        Ok(endpoint
            .connect_timeout(CONNECT_TIMEOUT)
            .http2_keep_alive_interval(KEEPALIVE_INTERVAL)
            .keep_alive_timeout(KEEPALIVE_TIMEOUT)
            .keep_alive_while_idle(true))
    }
}

#[derive(Clone, Debug, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct ReconnectConfig {
    pub initial_delay_ms: u64,
    pub maximum_delay_ms: u64,
}

impl ReconnectConfig {
    fn validate(&self) -> Result<()> {
        if self.initial_delay_ms == 0 {
            bail!("reconnect.initial_delay_ms must be positive");
        }
        if self.maximum_delay_ms == 0 {
            bail!("reconnect.maximum_delay_ms must be positive");
        }
        if self.maximum_delay_ms < self.initial_delay_ms {
            bail!("reconnect.maximum_delay_ms must be at least reconnect.initial_delay_ms");
        }
        if self.maximum_delay_ms > MAXIMUM_RECONNECT_DELAY_MS {
            bail!("reconnect.maximum_delay_ms must not exceed {MAXIMUM_RECONNECT_DELAY_MS}");
        }
        Ok(())
    }
}

impl Default for ReconnectConfig {
    fn default() -> Self {
        Self {
            initial_delay_ms: 100,
            maximum_delay_ms: 5_000,
        }
    }
}

#[derive(Clone, Debug, Deserialize)]
#[serde(tag = "mode", rename_all = "snake_case", deny_unknown_fields)]
pub enum OutputConfig {
    Save { path: String },
    Connect { address: String },
}

impl OutputConfig {
    fn validate(&self) -> Result<()> {
        let (field, value) = match self {
            Self::Save { path } => ("output.path", path),
            Self::Connect { address } => ("output.address", address),
        };
        if value.trim().is_empty() {
            bail!("{field} must be non-empty");
        }
        if value.trim() != value {
            bail!("{field} must not have leading or trailing whitespace");
        }
        if matches!(self, Self::Connect { .. }) {
            validate_host_port(value)?;
        }
        Ok(())
    }
}

fn validate_host_port(value: &str) -> Result<()> {
    if value.chars().any(char::is_whitespace)
        || value.contains('/')
        || value.contains('?')
        || value.contains('#')
        || value.contains('@')
    {
        bail!("output.address must contain only a host and port, without a scheme or path");
    }
    let uri: tonic::codegen::http::Uri = format!("http://{value}")
        .parse()
        .context("output.address must be a valid host:port")?;
    let authority = uri
        .authority()
        .context("output.address must include a host and port")?;
    if authority.host().is_empty() || authority.port_u16().is_none_or(|port| port == 0) {
        bail!("output.address must include a host and a port in 1..=65535");
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn valid_yaml() -> &'static str {
        r#"
hardware_endpoint: http://127.0.0.1:50051
reconnect:
  initial_delay_ms: 100
  maximum_delay_ms: 5000
output:
  mode: save
  path: telemetry.rrd
"#
    }

    #[test]
    fn valid_configuration_is_accepted() {
        let config: Config = serde_yaml::from_str(valid_yaml()).unwrap();
        config.validate().unwrap();
    }

    #[test]
    fn unknown_fields_are_rejected() {
        let source = format!("{}unexpected: true\n", valid_yaml());
        assert!(serde_yaml::from_str::<Config>(&source).is_err());
    }

    #[test]
    fn endpoint_must_be_an_absolute_uri() {
        let mut config: Config = serde_yaml::from_str(valid_yaml()).unwrap();
        config.hardware_endpoint = "127.0.0.1:50051".into();
        assert!(config.validate().is_err());
    }

    #[test]
    fn reconnect_range_is_bounded_and_ordered() {
        let mut config: Config = serde_yaml::from_str(valid_yaml()).unwrap();
        config.reconnect.initial_delay_ms = 0;
        assert!(config.validate().is_err());

        config.reconnect.initial_delay_ms = 200;
        config.reconnect.maximum_delay_ms = 100;
        assert!(config.validate().is_err());

        config.reconnect.maximum_delay_ms = MAXIMUM_RECONNECT_DELAY_MS + 1;
        assert!(config.validate().is_err());
    }

    #[test]
    fn output_value_must_be_nonempty() {
        let mut config: Config = serde_yaml::from_str(valid_yaml()).unwrap();
        config.output = OutputConfig::Save { path: " ".into() };
        assert!(config.validate().is_err());
    }

    #[test]
    fn connect_output_requires_a_socket_address() {
        let mut config: Config = serde_yaml::from_str(valid_yaml()).unwrap();
        config.output = OutputConfig::Connect {
            address: "not-an-address".into(),
        };
        assert!(config.validate().is_err());

        config.output = OutputConfig::Connect {
            address: "127.0.0.1:9876".into(),
        };
        config.validate().unwrap();

        config.output = OutputConfig::Connect {
            address: "localhost:9876".into(),
        };
        config.validate().unwrap();

        config.output = OutputConfig::Connect {
            address: "localhost:0".into(),
        };
        assert!(config.validate().is_err());

        config.output = OutputConfig::Connect {
            address: "http://localhost:9876/proxy".into(),
        };
        assert!(config.validate().is_err());
    }
}
