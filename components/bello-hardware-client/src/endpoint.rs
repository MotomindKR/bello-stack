use std::fmt;
use std::path::Path;
use std::time::Duration;

use tonic::transport::{Certificate, ClientTlsConfig, Endpoint, Identity};

use crate::{Error, HardwareClient};

const DEFAULT_CONNECT_TIMEOUT: Duration = Duration::from_secs(5);
const DEFAULT_KEEPALIVE_INTERVAL: Duration = Duration::from_secs(10);
const DEFAULT_KEEPALIVE_TIMEOUT: Duration = Duration::from_secs(3);

#[derive(Clone, Debug)]
pub struct ConnectOptions {
    endpoint: String,
    connect_timeout: Duration,
    keepalive: Option<Keepalive>,
    tls: Option<TlsOptions>,
}

/// Optional private CA and client identity for an HTTPS connection.
#[derive(Clone, Default)]
pub struct TlsOptions {
    ca_certificate: Option<Vec<u8>>,
    identity: Option<(Vec<u8>, Vec<u8>)>,
}

impl fmt::Debug for TlsOptions {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter
            .debug_struct("TlsOptions")
            .field("custom_ca", &self.ca_certificate.is_some())
            .field("client_identity", &self.identity.is_some())
            .finish()
    }
}

impl TlsOptions {
    pub fn new() -> Self {
        Self::default()
    }

    /// Adds a PEM-encoded CA certificate in addition to native trusted roots.
    pub fn ca_certificate(mut self, pem: impl Into<Vec<u8>>) -> Self {
        self.ca_certificate = Some(pem.into());
        self
    }

    /// Sets the PEM-encoded client certificate chain and private key for mTLS.
    pub fn identity(
        mut self,
        certificate_chain: impl Into<Vec<u8>>,
        private_key: impl Into<Vec<u8>>,
    ) -> Self {
        self.identity = Some((certificate_chain.into(), private_key.into()));
        self
    }
}

#[derive(Clone, Copy, Debug)]
struct Keepalive {
    interval: Duration,
    timeout: Duration,
    while_idle: bool,
}

impl ConnectOptions {
    pub fn new(endpoint: impl Into<String>) -> Self {
        Self {
            endpoint: endpoint.into(),
            connect_timeout: DEFAULT_CONNECT_TIMEOUT,
            keepalive: Some(Keepalive {
                interval: DEFAULT_KEEPALIVE_INTERVAL,
                timeout: DEFAULT_KEEPALIVE_TIMEOUT,
                while_idle: true,
            }),
            tls: None,
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

    /// Uses custom trust roots and, optionally, a client identity for mTLS.
    pub fn tls(mut self, tls: TlsOptions) -> Self {
        self.tls = Some(tls);
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
        let http = self.endpoint.starts_with("http://");
        let unix = self.endpoint.starts_with("unix:");
        if !https && !http && !unix {
            return Err(Error::InvalidArgument(
                "endpoint must start with http://, https://, or unix://".into(),
            ));
        }
        if unix {
            if !cfg!(unix) {
                return Err(Error::InvalidArgument(
                    "Unix endpoints are unavailable on this platform; use http:// or https://"
                        .into(),
                ));
            }
            if !self.endpoint.starts_with("unix:///") {
                return Err(Error::InvalidArgument(
                    "Unix endpoint must use unix:///absolute/path".into(),
                ));
            }
            let path = self
                .endpoint
                .strip_prefix("unix://")
                .expect("validated Unix endpoint prefix");
            if !Path::new(path).is_absolute() {
                return Err(Error::InvalidArgument(
                    "Unix endpoint path must be absolute".into(),
                ));
            }
        }
        if !https && self.tls.is_some() {
            return Err(Error::InvalidArgument(
                "TLS options require an https:// endpoint".into(),
            ));
        }
        if self.connect_timeout.is_zero() {
            return Err(Error::InvalidArgument(
                "connect timeout must be positive".into(),
            ));
        }
        let mut endpoint = Endpoint::from_shared(self.endpoint)?;
        if https {
            let mut config = ClientTlsConfig::new()
                .with_native_roots()
                .assume_http2(true);
            if let Some(tls) = self.tls {
                if let Some(ca_certificate) = tls.ca_certificate {
                    config = config.ca_certificate(Certificate::from_pem(ca_certificate));
                }
                if let Some((certificate_chain, private_key)) = tls.identity {
                    config = config.identity(Identity::from_pem(certificate_chain, private_key));
                }
            }
            endpoint = endpoint.tls_config(config)?;
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
        assert_eq!(
            ConnectOptions::new("unix:///run/bello/hardwared.sock")
                .into_endpoint()
                .is_ok(),
            cfg!(unix)
        );
    }

    #[test]
    fn unix_endpoint_requires_an_absolute_path() {
        assert!(
            ConnectOptions::new("unix://relative.sock")
                .into_endpoint()
                .is_err()
        );
    }

    #[test]
    fn tls_options_require_https() {
        assert!(
            ConnectOptions::new("http://127.0.0.1:50051")
                .tls(TlsOptions::new().ca_certificate(b"test".to_vec()))
                .into_endpoint()
                .is_err()
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
