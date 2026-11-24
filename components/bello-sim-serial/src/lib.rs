#![cfg(unix)]

use std::fs::{self, DirBuilder, File};
use std::io::{self, ErrorKind, IoSlice, IoSliceMut};
use std::mem::MaybeUninit;
use std::os::fd::{AsFd, OwnedFd};
use std::os::unix::fs::{DirBuilderExt, FileTypeExt, MetadataExt};
use std::os::unix::net::{UnixListener, UnixStream};
use std::path::PathBuf;
use std::time::Duration;

use rustix::fs::{OFlags, fcntl_getfl, fcntl_setfl};
use rustix::io::{FdFlags, fcntl_setfd};
use rustix::net::{
    AddressFamily, RecvAncillaryBuffer, RecvAncillaryMessage, RecvFlags, ReturnFlags,
    SendAncillaryBuffer, SendAncillaryMessage, SendFlags, SocketAddrUnix, SocketType, recvmsg,
    sendmsg, socket,
};
use rustix::termios::{OptionalActions, tcgetattr, tcsetattr};

pub fn validate_name(name: &str) -> io::Result<()> {
    if name.is_empty()
        || name.len() > 48
        || !name
            .bytes()
            .all(|c| c.is_ascii_alphanumeric() || matches!(c, b'_' | b'-'))
    {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "PTY endpoint name must be 1–48 letters, digits, hyphens or underscores",
        ));
    }
    Ok(())
}

fn socket_path(name: &str) -> io::Result<PathBuf> {
    validate_name(name)?;
    let uid = rustix::process::geteuid().as_raw();
    let dir = PathBuf::from(format!("/tmp/bello-sim-{uid}"));
    match DirBuilder::new().mode(0o700).create(&dir) {
        Err(e) if e.kind() == ErrorKind::AlreadyExists => {}
        result => result?,
    }
    let meta = dir.symlink_metadata()?;
    if !meta.is_dir() || meta.uid() != uid || meta.mode() & 0o077 != 0 {
        return Err(io::Error::new(
            ErrorKind::PermissionDenied,
            format!("expected a private directory at {}", dir.display()),
        ));
    }
    Ok(dir.join(format!("{name}.sock")))
}

pub struct Server {
    listener: UnixListener,
    path: PathBuf,
    _lock: File,
}

impl Server {
    pub fn bind(name: &str) -> io::Result<Self> {
        let path = socket_path(name)?;
        // Keep the lock inode across restarts so two binders always contend on the same file.
        let lock = File::options()
            .read(true)
            .write(true)
            .create(true)
            .truncate(false)
            .open(path.with_extension("lock"))?;
        lock.try_lock().map_err(|e| match e {
            std::fs::TryLockError::WouldBlock => io::Error::new(
                ErrorKind::AddrInUse,
                format!("simulated serial port {name} is already running"),
            ),
            std::fs::TryLockError::Error(e) => e,
        })?;
        match path.symlink_metadata() {
            Ok(meta) if meta.file_type().is_socket() => fs::remove_file(&path)?,
            Ok(_) => {
                return Err(io::Error::new(
                    ErrorKind::AlreadyExists,
                    format!("{} is not a socket", path.display()),
                ));
            }
            Err(e) if e.kind() == ErrorKind::NotFound => {}
            Err(e) => return Err(e),
        }
        let listener = UnixListener::bind(&path)?;
        listener.set_nonblocking(true)?;
        Ok(Self {
            listener,
            path,
            _lock: lock,
        })
    }

    pub fn accept(&self) -> io::Result<Option<(File, UnixStream)>> {
        let (peer, _) = match self.listener.accept() {
            Ok(peer) => peer,
            Err(e) if matches!(e.kind(), ErrorKind::WouldBlock | ErrorKind::Interrupted) => {
                return Ok(None);
            }
            Err(e) => return Err(e),
        };
        peer.set_nonblocking(true)?;
        let pty = nix::pty::openpty(None, None)?;
        for fd in [&pty.master, &pty.slave] {
            fcntl_setfd(fd, FdFlags::CLOEXEC)?;
        }
        let mut termios = tcgetattr(&pty.slave)?;
        termios.make_raw();
        tcsetattr(&pty.slave, OptionalActions::Now, &termios)?;
        fcntl_setfl(&pty.master, fcntl_getfl(&pty.master)? | OFlags::NONBLOCK)?;

        let mut space = [MaybeUninit::uninit(); rustix::cmsg_space!(ScmRights(1))];
        let mut control = SendAncillaryBuffer::new(&mut space);
        let rights = [pty.slave.as_fd()];
        control.push(SendAncillaryMessage::ScmRights(&rights));
        match sendmsg(
            &peer,
            &[IoSlice::new(b"P")],
            &mut control,
            SendFlags::empty(),
        ) {
            Ok(1) => Ok(Some((pty.master.into(), peer))),
            // The client can time out and disconnect before the next physics tick.
            Ok(_)
            | Err(
                rustix::io::Errno::PIPE
                | rustix::io::Errno::CONNRESET
                | rustix::io::Errno::AGAIN
                | rustix::io::Errno::NOTCONN,
            ) => Ok(None),
            Err(e) => Err(e.into()),
        }
    }
}

impl Drop for Server {
    fn drop(&mut self) {
        let _ = fs::remove_file(&self.path);
    }
}

pub fn connect(name: &str, timeout: Duration) -> io::Result<(OwnedFd, UnixStream)> {
    let address = SocketAddrUnix::new(socket_path(name)?)?;
    let peer = UnixStream::from(socket(AddressFamily::UNIX, SocketType::STREAM, None)?);
    fcntl_setfd(&peer, FdFlags::CLOEXEC)?;
    // A paused server with a full accept queue must not block the daemon's reconnect worker.
    peer.set_nonblocking(true)?;
    rustix::net::connect(&peer, &address)?;
    peer.set_nonblocking(false)?;
    peer.set_read_timeout(Some(timeout))?;
    let mut byte = [0];
    let mut space = [MaybeUninit::uninit(); rustix::cmsg_space!(ScmRights(1))];
    let mut control = RecvAncillaryBuffer::new(&mut space);
    #[cfg(target_os = "linux")]
    let flags = RecvFlags::CMSG_CLOEXEC;
    #[cfg(not(target_os = "linux"))]
    let flags = RecvFlags::empty();
    let msg = recvmsg(
        &peer,
        &mut [IoSliceMut::new(&mut byte)],
        &mut control,
        flags,
    )?;
    if msg.bytes == 1
        && byte == *b"P"
        && !msg.flags.contains(ReturnFlags::CTRUNC)
        && let Some(RecvAncillaryMessage::ScmRights(mut rights)) = control.drain().next()
        && let Some(slave) = rights.next()
        && rights.next().is_none()
    {
        fcntl_setfd(&slave, FdFlags::CLOEXEC)?;
        tcgetattr(&slave)?;
        return Ok((slave, peer));
    }
    Err(io::Error::new(
        ErrorKind::InvalidData,
        "expected one PTY descriptor from the simulator",
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn restart_reclaims_a_dead_socket_without_displacing_a_live_server() {
        let name = format!("restart-test-{}", std::process::id());
        let path = socket_path(&name).unwrap();
        // A killed simulator leaves its socket inode behind.
        drop(UnixListener::bind(&path).unwrap());
        assert!(connect(&name, Duration::from_secs(1)).is_err());
        let server = Server::bind(&name).unwrap();
        assert!(matches!(Server::bind(&name), Err(e) if e.kind() == ErrorKind::AddrInUse));
        let client = UnixStream::connect(&path).unwrap();
        assert!(server.accept().unwrap().is_some());
        drop((client, server));
        assert!(connect(&name, Duration::from_secs(1)).is_err());
    }
}
