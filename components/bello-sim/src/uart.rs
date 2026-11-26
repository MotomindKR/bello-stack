use anyhow::{Context, Result, ensure};
use nix::fcntl::{FcntlArg, OFlag, fcntl};
use nix::sys::termios::{SetArg, cfmakeraw, tcgetattr, tcsetattr};
use serde::Deserialize;
use std::collections::VecDeque;
use std::ffi::OsString;
use std::fs::{self, File, OpenOptions};
use std::io::{self, ErrorKind, Read, Write};
use std::os::unix::fs::{DirBuilderExt, MetadataExt, OpenOptionsExt, symlink};
use std::path::{Path, PathBuf};

#[derive(Clone, Debug, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Endpoint {
    Device(PathBuf),
    PtyLink(PathBuf),
}

impl Endpoint {
    pub(crate) fn path(&self) -> &Path {
        match self {
            Self::Device(path) | Self::PtyLink(path) => path,
        }
    }
}

fn set_raw(file: &File) -> io::Result<()> {
    let mut settings = tcgetattr(file)?;
    cfmakeraw(&mut settings);
    tcsetattr(file, SetArg::TCSANOW, &settings)?;
    Ok(())
}

struct PtyLink {
    path: PathBuf,
    published: Option<((u64, u64), PathBuf)>,
    _lock: File,
}

impl PtyLink {
    fn claim(path: &Path) -> Result<Self> {
        let name = path.file_name().context("PTY link needs a filename")?;
        let parent = path.parent().context("PTY link needs a parent directory")?;
        if parent == Path::new("/tmp/bello-sim") {
            match fs::DirBuilder::new().mode(0o700).create(parent) {
                Err(e) if e.kind() == ErrorKind::AlreadyExists => {}
                result => result?,
            }
            let dir = fs::symlink_metadata(parent)?;
            ensure!(
                dir.is_dir()
                    && dir.uid() == nix::unistd::geteuid().as_raw()
                    && dir.mode() & 0o777 == 0o700,
                "{} must be a private directory owned by the current user",
                parent.display()
            );
        }
        let mut lock_name = OsString::from(".");
        lock_name.push(name);
        lock_name.push(".lock");
        let lock = OpenOptions::new()
            .write(true)
            .create(true)
            .truncate(false)
            .mode(0o600)
            .custom_flags(nix::libc::O_NOFOLLOW | nix::libc::O_NONBLOCK)
            .open(parent.join(lock_name))?;
        ensure!(
            lock.metadata()?.is_file(),
            "invalid PTY lock for {}",
            path.display()
        );
        lock.try_lock()
            .with_context(|| format!("locking PTY link {}", path.display()))?;
        Ok(Self {
            path: path.into(),
            published: None,
            _lock: lock,
        })
    }

    fn owns(&self, metadata: &fs::Metadata) -> bool {
        self.published.as_ref().is_some_and(|(identity, target)| {
            *identity == (metadata.dev(), metadata.ino())
                && fs::read_link(&self.path).is_ok_and(|path| path == *target)
        })
    }

    fn publish(&mut self) -> Result<File> {
        match fs::symlink_metadata(&self.path) {
            Ok(m) => ensure!(
                m.is_symlink() && (self.published.is_none() || self.owns(&m)),
                "refusing to replace {}",
                self.path.display()
            ),
            Err(e) if e.kind() == ErrorKind::NotFound => {}
            Err(e) => return Err(e.into()),
        }
        let pty = nix::pty::openpty(None, None)?;
        let slave = File::from(pty.slave);
        let master = File::from(pty.master);
        set_raw(&slave)?;
        for file in [&master, &slave] {
            fcntl(file, FcntlArg::F_SETFD(nix::fcntl::FdFlag::FD_CLOEXEC))?;
        }
        fcntl(&master, FcntlArg::F_SETFL(OFlag::O_NONBLOCK))?;
        let target = nix::unistd::ttyname(&slave)?;
        let staging = tempfile::tempdir_in(self.path.with_file_name("."))?;
        let staged = staging.path().join("tty");
        symlink(&target, &staged)?;
        let identity = fs::symlink_metadata(&staged)?;
        drop(slave);
        fs::rename(&staged, &self.path)?;
        self.published = Some(((identity.dev(), identity.ino()), target));
        Ok(master)
    }
}

impl Drop for PtyLink {
    fn drop(&mut self) {
        if let Ok(metadata) = fs::symlink_metadata(&self.path)
            && self.owns(&metadata)
        {
            let _ = fs::remove_file(&self.path);
        }
    }
}

pub(crate) struct Uart<E> {
    // Drop the published link before the master closes and its tty name can be reused.
    port: Port<E>,
    file: File,
    encoder: E,
}

enum Port<E> {
    Device {
        pending: VecDeque<u8>,
    },
    Pty {
        link: PtyLink,
        initial: E,
        connected: bool,
    },
}

impl<E: Clone> Uart<E> {
    pub(crate) fn open(endpoint: &Endpoint, encoder: E) -> Result<Self> {
        let (file, port) = match endpoint {
            Endpoint::PtyLink(path) => {
                let mut link = PtyLink::claim(path)?;
                let master = link.publish()?;
                (
                    master,
                    Port::Pty {
                        link,
                        initial: encoder.clone(),
                        connected: false,
                    },
                )
            }
            Endpoint::Device(path) => {
                let file = OpenOptions::new()
                    .read(true)
                    .write(true)
                    .custom_flags(nix::libc::O_NOCTTY | nix::libc::O_NONBLOCK)
                    .open(path)?;
                set_raw(&file)?;
                (
                    file,
                    Port::Device {
                        pending: VecDeque::new(),
                    },
                )
            }
        };
        Ok(Self {
            port,
            file,
            encoder,
        })
    }

    pub(crate) fn update(
        &mut self,
        encode: impl FnOnce(&mut E, &[u8]) -> Result<Vec<u8>>,
    ) -> Result<()> {
        let mut input = [0; 4096];
        let read = self.file.read(&mut input);
        if let Port::Pty {
            link,
            connected,
            initial,
        } = &mut self.port
        {
            if matches!(read, Ok(0))
                || read
                    .as_ref()
                    .is_err_and(|e| e.raw_os_error() == Some(nix::libc::EIO))
            {
                if *connected {
                    self.file = link.publish()?;
                    *connected = false;
                }
                return Ok(());
            }
            if !*connected {
                if let Err(e) = set_raw(&self.file) {
                    if e.raw_os_error() == Some(nix::libc::EIO) {
                        return Ok(());
                    }
                    return Err(e.into());
                }
                *connected = true;
                self.encoder.clone_from(initial);
            }
        }
        let count = match read {
            Ok(n) => n,
            Err(e) if e.kind() == ErrorKind::WouldBlock => 0,
            Err(e) if e.kind() == ErrorKind::Interrupted => return Ok(()),
            Err(e) => return Err(e.into()),
        };
        let bytes = encode(&mut self.encoder, &input[..count])?;
        match &mut self.port {
            Port::Pty {
                link, connected, ..
            } => {
                if let Err(e) = self.file.write_all(&bytes) {
                    if e.kind() != ErrorKind::WouldBlock && e.raw_os_error() != Some(nix::libc::EIO)
                    {
                        return Err(e.into());
                    }
                    self.file = link.publish()?;
                    *connected = false;
                }
            }
            Port::Device { pending } => {
                ensure!(
                    pending.len() + bytes.len() <= 16384,
                    "emulated UART peer is not consuming output"
                );
                pending.extend(bytes);
                while !pending.is_empty() {
                    let (bytes, _) = pending.as_slices();
                    match self.file.write(bytes) {
                        Ok(0) => return Err(io::Error::from(ErrorKind::WriteZero).into()),
                        Ok(n) => {
                            pending.drain(..n);
                        }
                        Err(e) if e.kind() == ErrorKind::WouldBlock => break,
                        Err(e) if e.kind() == ErrorKind::Interrupted => continue,
                        Err(e) => return Err(e.into()),
                    }
                }
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::os::unix::fs::FileTypeExt;

    #[test]
    fn stale_link_is_replaced_and_live_writer_keeps_ownership() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("imu");
        symlink("missing", &path).unwrap();
        let endpoint = Endpoint::PtyLink(path.clone());
        let port = Uart::open(&endpoint, ()).unwrap();
        assert!(fs::metadata(&path).unwrap().file_type().is_char_device());
        assert!(Uart::open(&endpoint, ()).is_err());
        drop(port);
        assert_eq!(
            fs::symlink_metadata(&path).unwrap_err().kind(),
            ErrorKind::NotFound
        );
        let port = Uart::open(&endpoint, ()).unwrap();
        fs::remove_file(&path).unwrap();
        fs::write(&path, b"keep").unwrap();
        drop(port);
        assert!(Uart::open(&endpoint, ()).is_err());
        assert_eq!(fs::read(path).unwrap(), b"keep");
    }
}
