use std::path::Path;
#[cfg(unix)]
use std::{fs, io::ErrorKind, path::PathBuf};

#[cfg(unix)]
use anyhow::Context;
use anyhow::{Result, anyhow, bail};
use tonic::transport::server::TcpIncoming;

use crate::api::grpc::GrpcListenSpec;

pub(crate) enum GrpcIncoming {
    Tcp(TcpIncoming),
    #[cfg(unix)]
    Unix {
        incoming: tokio_stream::wrappers::UnixListenerStream,
        socket: UnixSocketGuard,
    },
}

impl GrpcIncoming {
    pub(crate) fn bind(spec: &GrpcListenSpec) -> Result<Self> {
        match spec {
            GrpcListenSpec::Tcp(address) => TcpIncoming::bind(*address)
                .map(|incoming| incoming.with_nodelay(Some(true)))
                .map(Self::Tcp)
                .map_err(|error| anyhow!("binding gRPC TCP listener {address}: {error}")),
            GrpcListenSpec::Unix { path, mode } => bind_unix(path, *mode),
        }
    }
}

#[cfg(unix)]
fn bind_unix(path: &Path, mode: u32) -> Result<GrpcIncoming> {
    use std::os::unix::fs::{FileTypeExt, MetadataExt, PermissionsExt};

    let parent = path.parent().ok_or_else(|| {
        anyhow!(
            "gRPC Unix socket {} has no parent directory",
            path.display()
        )
    })?;
    if !parent.is_dir() {
        bail!(
            "gRPC Unix socket parent directory {} does not exist",
            parent.display()
        );
    }
    match fs::symlink_metadata(path) {
        Ok(before) => {
            if !before.file_type().is_socket() {
                bail!(
                    "refusing to replace non-socket gRPC Unix path {}",
                    path.display()
                );
            }
            match std::os::unix::net::UnixStream::connect(path) {
                Ok(_) => bail!("gRPC Unix socket {} is already in use", path.display()),
                Err(error) if error.kind() == ErrorKind::ConnectionRefused => {
                    let after = fs::symlink_metadata(path).with_context(|| {
                        format!("rechecking stale gRPC Unix socket {}", path.display())
                    })?;
                    if before.dev() != after.dev() || before.ino() != after.ino() {
                        bail!(
                            "gRPC Unix socket {} changed while checking whether it was stale",
                            path.display()
                        );
                    }
                    fs::remove_file(path).with_context(|| {
                        format!("removing stale gRPC Unix socket {}", path.display())
                    })?;
                }
                Err(error) => {
                    return Err(error).with_context(|| {
                        format!("checking existing gRPC Unix socket {}", path.display())
                    });
                }
            }
        }
        Err(error) if error.kind() == ErrorKind::NotFound => {}
        Err(error) => {
            return Err(error)
                .with_context(|| format!("inspecting gRPC Unix path {}", path.display()));
        }
    }

    let listener = tokio::net::UnixListener::bind(path)
        .with_context(|| format!("binding gRPC Unix socket {}", path.display()))?;
    let metadata = fs::symlink_metadata(path)
        .with_context(|| format!("inspecting gRPC Unix socket {}", path.display()))?;
    let socket = UnixSocketGuard {
        path: path.to_owned(),
        device: metadata.dev(),
        inode: metadata.ino(),
    };
    fs::set_permissions(path, fs::Permissions::from_mode(mode)).with_context(|| {
        format!(
            "setting gRPC Unix socket {} permissions to {mode:04o}",
            path.display()
        )
    })?;
    Ok(GrpcIncoming::Unix {
        incoming: tokio_stream::wrappers::UnixListenerStream::new(listener),
        socket,
    })
}

#[cfg(not(unix))]
fn bind_unix(path: &Path, _mode: u32) -> Result<GrpcIncoming> {
    bail!(
        "gRPC Unix socket {} is not supported on this platform",
        path.display()
    )
}

#[cfg(unix)]
pub(crate) struct UnixSocketGuard {
    path: PathBuf,
    device: u64,
    inode: u64,
}

#[cfg(unix)]
impl Drop for UnixSocketGuard {
    fn drop(&mut self) {
        use std::os::unix::fs::MetadataExt;

        let owned = fs::symlink_metadata(&self.path)
            .is_ok_and(|metadata| metadata.dev() == self.device && metadata.ino() == self.inode);
        if owned && let Err(error) = fs::remove_file(&self.path) {
            tracing::warn!(
                path = %self.path.display(),
                %error,
                "failed to remove gRPC Unix socket during shutdown"
            );
        }
    }
}

#[cfg(test)]
mod tests {
    use tokio_stream::StreamExt;

    use super::*;

    #[tokio::test]
    async fn tcp_connections_disable_nagle() {
        let spec = GrpcListenSpec::Tcp("127.0.0.1:0".parse().expect("valid test address"));
        let mut incoming = match GrpcIncoming::bind(&spec).expect("bind test listener") {
            GrpcIncoming::Tcp(incoming) => incoming,
            #[cfg(unix)]
            GrpcIncoming::Unix { .. } => panic!("TCP listen spec produced a Unix listener"),
        };
        let address = incoming.local_addr().expect("test listener address");
        let client = tokio::spawn(tokio::net::TcpStream::connect(address));
        let accepted = incoming
            .next()
            .await
            .expect("accepted stream")
            .expect("accept connection");
        let _client = client.await.expect("client task").expect("connect client");

        assert!(accepted.nodelay().expect("read TCP_NODELAY"));
    }
}
