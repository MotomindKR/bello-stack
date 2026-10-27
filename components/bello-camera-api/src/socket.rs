//! Local service sockets recover abandoned inodes without replacing a live owner.
use std::{
    io,
    os::unix::fs::{FileTypeExt, MetadataExt, PermissionsExt},
    path::{Path, PathBuf},
    time::Duration,
};
pub struct Socket {
    path: PathBuf,
    device: u64,
    inode: u64,
}
impl Drop for Socket {
    fn drop(&mut self) {
        if std::fs::symlink_metadata(&self.path)
            .is_ok_and(|m| m.dev() == self.device && m.ino() == self.inode)
        {
            let _ = std::fs::remove_file(&self.path);
        }
    }
}
pub async fn bind(path: &Path) -> io::Result<(tokio::net::UnixListener, Socket)> {
    match std::fs::symlink_metadata(path) {
        Ok(old) => {
            if !old.file_type().is_socket() || old.uid() != unsafe { libc::geteuid() } {
                return Err(io::Error::new(
                    io::ErrorKind::AlreadyExists,
                    "socket path has a different owner or file type",
                ));
            }
            match tokio::time::timeout(
                Duration::from_millis(250),
                tokio::net::UnixStream::connect(path),
            )
            .await
            {
                Ok(Err(e)) if e.kind() == io::ErrorKind::ConnectionRefused => {
                    let current = std::fs::symlink_metadata(path)?;
                    if current.dev() != old.dev() || current.ino() != old.ino() {
                        return Err(io::Error::new(
                            io::ErrorKind::AddrInUse,
                            "socket owner changed during startup",
                        ));
                    }
                    std::fs::remove_file(path)?;
                }
                Ok(Err(e)) if e.kind() == io::ErrorKind::NotFound => {}
                _ => {
                    return Err(io::Error::new(
                        io::ErrorKind::AddrInUse,
                        "service socket already has a live owner",
                    ));
                }
            }
        }
        Err(e) if e.kind() == io::ErrorKind::NotFound => {}
        Err(e) => return Err(e),
    }
    let listener = tokio::net::UnixListener::bind(path)?;
    let metadata = std::fs::symlink_metadata(path)?;
    let owner = Socket {
        path: path.into(),
        device: metadata.dev(),
        inode: metadata.ino(),
    };
    std::fs::set_permissions(path, std::fs::Permissions::from_mode(0o600))?;
    Ok((listener, owner))
}
#[cfg(test)]
mod tests {
    use super::*;
    #[tokio::test]
    async fn restart_recovers_an_abandoned_socket_and_preserves_live_owners_and_files() {
        let directory = tempfile::tempdir().unwrap();
        let path = directory.path().join("camera.sock");
        drop(std::os::unix::net::UnixListener::bind(&path).unwrap());
        let (listener, owner) = bind(&path).await.unwrap();
        assert!(bind(&path).await.is_err());
        drop(listener);
        drop(owner);
        assert!(!path.exists());
        std::fs::write(&path, "keep").unwrap();
        assert!(bind(&path).await.is_err());
        assert_eq!(std::fs::read_to_string(path).unwrap(), "keep");
    }
}
