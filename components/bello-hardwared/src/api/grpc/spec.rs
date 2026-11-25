use std::fmt;
use std::fs;
use std::net::SocketAddr;
use std::path::{Path, PathBuf};

use anyhow::{Context, Result, bail};
use garde::Validate;
use tonic::transport::{Certificate, Identity, Server, ServerTlsConfig};

use crate::config::{GrpcConfig, GrpcListenConfig, GrpcTlsConfig};

#[derive(Clone, Debug)]
pub enum GrpcListenSpec {
    Tcp(SocketAddr),
    Unix { path: PathBuf, mode: u32 },
}

#[derive(Clone)]
struct GrpcTlsSpec {
    certificate: Vec<u8>,
    private_key: Vec<u8>,
    client_ca: Option<Vec<u8>>,
}

impl fmt::Debug for GrpcTlsSpec {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter
            .debug_struct("GrpcTlsSpec")
            .field("client_authentication", &self.client_ca.is_some())
            .finish_non_exhaustive()
    }
}

#[derive(Clone, Debug)]
pub struct GrpcSpec {
    listen: Vec<GrpcListenSpec>,
    tls: Option<GrpcTlsSpec>,
    pub(crate) default_lease_ms: u64,
    pub(crate) maximum_lease_ms: u64,
}

impl GrpcSpec {
    pub fn listen(&self) -> &[GrpcListenSpec] {
        &self.listen
    }

    pub fn uses_tls(&self) -> bool {
        self.tls.is_some()
    }

    pub fn requires_client_certificate(&self) -> bool {
        self.tls.as_ref().is_some_and(|tls| tls.client_ca.is_some())
    }

    pub(crate) fn server(&self, listen: &GrpcListenSpec) -> Result<Server> {
        let server = Server::builder();
        let Some(tls) = self
            .tls
            .as_ref()
            .filter(|_| matches!(listen, GrpcListenSpec::Tcp(_)))
        else {
            return Ok(server);
        };
        let mut config = ServerTlsConfig::new().identity(Identity::from_pem(
            tls.certificate.clone(),
            tls.private_key.clone(),
        ));
        if let Some(client_ca) = &tls.client_ca {
            config = config.client_ca_root(Certificate::from_pem(client_ca.clone()));
        }
        server
            .tls_config(config)
            .context("configuring gRPC server TLS")
    }
}

impl TryFrom<&GrpcConfig> for GrpcSpec {
    type Error = anyhow::Error;

    fn try_from(config: &GrpcConfig) -> Result<Self> {
        config.validate().context("grpc")?;
        if config.default_lease_ms > config.maximum_lease_ms {
            bail!("grpc.default_lease_ms must not exceed maximum_lease_ms");
        }
        let mut listen = Vec::with_capacity(config.listen.len());
        for configured in &config.listen {
            let listener = match configured {
                GrpcListenConfig::Tcp { address } => GrpcListenSpec::Tcp(*address),
                GrpcListenConfig::Unix { path, mode } => {
                    if !path.is_absolute() {
                        bail!("grpc Unix socket path must be absolute");
                    }
                    if path.file_name().is_none() {
                        bail!("grpc Unix socket path must name a socket file");
                    }
                    GrpcListenSpec::Unix {
                        path: path.clone(),
                        mode: parse_socket_mode(mode)?,
                    }
                }
            };
            if listen.iter().any(|existing| match (existing, &listener) {
                (GrpcListenSpec::Tcp(a), GrpcListenSpec::Tcp(b)) => a == b,
                (GrpcListenSpec::Unix { path: a, .. }, GrpcListenSpec::Unix { path: b, .. }) => {
                    a == b
                }
                _ => false,
            }) {
                bail!("grpc.listen contains a duplicate listener");
            }
            listen.push(listener);
        }
        if config.tls.is_some()
            && !listen
                .iter()
                .any(|item| matches!(item, GrpcListenSpec::Tcp(_)))
        {
            bail!("grpc TLS requires a TCP listener");
        }
        let tls = config.tls.as_ref().map(load_tls).transpose()?;
        let spec = Self {
            listen,
            tls,
            default_lease_ms: config.default_lease_ms,
            maximum_lease_ms: config.maximum_lease_ms,
        };
        for listener in spec.listen() {
            spec.server(listener)?;
        }
        Ok(spec)
    }
}

fn parse_socket_mode(mode: &str) -> Result<u32> {
    if !(3..=4).contains(&mode.len()) || !mode.bytes().all(|byte| (b'0'..=b'7').contains(&byte)) {
        bail!("grpc Unix socket mode must contain three or four octal digits");
    }
    let parsed = u32::from_str_radix(mode, 8).context("parsing grpc Unix socket mode")?;
    if parsed > 0o777 {
        bail!("grpc Unix socket mode must not set special permission bits");
    }
    Ok(parsed)
}

fn load_tls(config: &GrpcTlsConfig) -> Result<GrpcTlsSpec> {
    Ok(GrpcTlsSpec {
        certificate: read_pem(&config.certificate, "certificate")?,
        private_key: read_pem(&config.private_key, "private key")?,
        client_ca: config
            .client_ca
            .as_ref()
            .map(|path| read_pem(path, "client CA"))
            .transpose()?,
    })
}

fn read_pem(path: &Path, description: &str) -> Result<Vec<u8>> {
    if !path.is_absolute() {
        bail!("gRPC TLS {description} path must be absolute");
    }
    let bytes = fs::read(path)
        .with_context(|| format!("reading gRPC TLS {description} {}", path.display()))?;
    if bytes.is_empty() {
        bail!("gRPC TLS {description} {} is empty", path.display());
    }
    Ok(bytes)
}
