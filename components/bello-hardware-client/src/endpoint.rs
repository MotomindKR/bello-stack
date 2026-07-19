use std::time::Duration;

use tonic::transport::{ClientTlsConfig, Endpoint};

use crate::{Error, HardwareClient};

const DEFAULT_CONNECT_TIMEOUT: Duration = Duration::from_secs(5);
const DEFAULT_KEEPALIVE_INTERVAL: Duration = Duration::from_secs(10);
const DEFAULT_KEEPALIVE_TIMEOUT: Duration = Duration::from_secs(3);

/// Connection settings for the hardware gRPC endpoint.
#[derive(Clone, Debug)]
pub struct ConnectOptions {
    endpoint: String,
    connect_timeout: Duration,
    keepalive: Option<Keepalive>,
}

#[derive(Clone, Copy, Debug)]
struct Keepalive {
    interval: Duration,
    timeout: Duration,
    while_idle: bool,
}

impl ConnectOptions {
    /// Creates settings with a five-second connect timeout and HTTP/2
    /// keepalive every ten seconds with a three-second acknowledgement timeout.
    pub fn new(endpoint: impl Into<String>) -> Self {
        Self {
            endpoint: endpoint.into(),
            connect_timeout: DEFAULT_CONNECT_TIMEOUT,
            keepalive: Some(Keepalive {
                interval: DEFAULT_KEEPALIVE_INTERVAL,
                timeout: DEFAULT_KEEPALIVE_TIMEOUT,
                while_idle: true,
            }),
        }
    }

    pub fn connect_timeout(mut self, timeout: Duration) -> Self {
        self.connect_timeout = timeout;
        self
    }

    pub fn http2_keepalive(mut self, interval: Duration, timeout: Duration) -> Self {
        self.keepalive = Some(Keepalive {
            interval,
            timeout,
            while_idle: true,
        });
        self
    }

    pub fn keepalive_while_idle(mut self, enabled: bool) -> Self {
        if let Some(keepalive) = &mut self.keepalive {
            keepalive.while_idle = enabled;
        }
        self
    }

    pub fn disable_http2_keepalive(mut self) -> Self {
        self.keepalive = None;
        self
    }

    pub async fn connect(self) -> Result<HardwareClient, Error> {
        HardwareClient::connect_with(self).await
    }

    pub(crate) fn into_endpoint(self) -> Result<Endpoint, Error> {
        if self.endpoint.trim() != self.endpoint {
            return Err(Error::InvalidArgument(
                "endpoint must not have leading or trailing whitespace".into(),
            ));
        }
        let https = self.endpoint.starts_with("https://");
        if !https && !self.endpoint.starts_with("http://") {
            return Err(Error::InvalidArgument(
                "endpoint must start with http:// or https://".into(),
            ));
        }
        if self.connect_timeout.is_zero() {
            return Err(Error::InvalidArgument(
                "connect timeout must be positive".into(),
            ));
        }
        let mut endpoint = Endpoint::from_shared(self.endpoint)?;
        if https {
            endpoint = endpoint.tls_config(
                ClientTlsConfig::new()
                    .with_native_roots()
                    .assume_http2(true),
            )?;
        }
        endpoint = endpoint.connect_timeout(self.connect_timeout);
        if let Some(keepalive) = self.keepalive {
            if keepalive.interval.is_zero() || keepalive.timeout.is_zero() {
                return Err(Error::InvalidArgument(
                    "HTTP/2 keepalive interval and timeout must be positive".into(),
                ));
            }
            endpoint = endpoint
                .http2_keep_alive_interval(keepalive.interval)
                .keep_alive_timeout(keepalive.timeout)
                .keep_alive_while_idle(keepalive.while_idle);
        }
        Ok(endpoint)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn endpoint_requires_an_explicit_http_scheme() {
        assert!(
            ConnectOptions::new("127.0.0.1:50051")
                .into_endpoint()
                .is_err()
        );
        assert!(
            ConnectOptions::new("http://127.0.0.1:50051")
                .into_endpoint()
                .is_ok()
        );
    }

    #[test]
    fn endpoint_rejects_zero_timeouts() {
        assert!(
            ConnectOptions::new("http://127.0.0.1:50051")
                .connect_timeout(Duration::ZERO)
                .into_endpoint()
                .is_err()
        );
        assert!(
            ConnectOptions::new("http://127.0.0.1:50051")
                .http2_keepalive(Duration::ZERO, Duration::from_secs(1))
                .into_endpoint()
                .is_err()
        );
    }
}
