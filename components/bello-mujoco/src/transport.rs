use anyhow::Result;
use bello_sim_can::SimSocket;
use robstride_emulator::config::CanInterface;
use robstride_protocol::Frame;

pub(crate) struct Uart {
    file: std::fs::File,
    pending: std::collections::VecDeque<u8>,
}

impl Uart {
    pub(crate) fn open(path: &std::path::Path) -> Result<Self> {
        use nix::sys::termios::{SetArg, cfmakeraw, tcgetattr, tcsetattr};
        use std::os::unix::fs::OpenOptionsExt;
        let file = std::fs::OpenOptions::new()
            .read(true)
            .write(true)
            .custom_flags(nix::libc::O_NOCTTY | nix::libc::O_NONBLOCK)
            .open(path)?;
        let mut settings = tcgetattr(&file)?;
        cfmakeraw(&mut settings);
        tcsetattr(&file, SetArg::TCSANOW, &settings)?;
        Ok(Self {
            file,
            pending: Default::default(),
        })
    }
    pub(crate) fn read(&mut self, buffer: &mut [u8]) -> Result<usize> {
        use std::io::Read;
        match self.file.read(buffer) {
            Ok(n) => Ok(n),
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => Ok(0),
            Err(e) => Err(e.into()),
        }
    }
    pub(crate) fn write(&mut self, bytes: &[u8]) -> Result<()> {
        use std::io::Write;
        anyhow::ensure!(
            self.pending.len() + bytes.len() <= 16384,
            "emulated UART peer is not consuming output"
        );
        self.pending.extend(bytes);
        while !self.pending.is_empty() {
            let (bytes, _) = self.pending.as_slices();
            match self.file.write(bytes) {
                Ok(0) => anyhow::bail!("emulated UART write returned zero"),
                Ok(n) => {
                    self.pending.drain(..n);
                }
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => break,
                Err(e) => return Err(e.into()),
            }
        }
        Ok(())
    }
}

pub(crate) enum CanEndpoint {
    Sim(SimSocket),
    #[cfg(target_os = "linux")]
    Socketcan(socketcan::CanSocket),
}

impl CanEndpoint {
    pub fn open(interface: &CanInterface) -> Result<Self> {
        Ok(match interface {
            CanInterface::Sim(bus) => {
                let socket = bus.open_simulator()?;
                socket.set_nonblocking(true)?;
                Self::Sim(socket)
            }
            #[cfg(target_os = "linux")]
            CanInterface::Socketcan(name) => {
                use socketcan::Socket;
                anyhow::ensure!(
                    !name.contains('/') && !name.contains(".."),
                    "invalid CAN interface name"
                );
                anyhow::ensure!(
                    std::path::Path::new("/sys/devices/virtual/net")
                        .join(name)
                        .is_dir(),
                    "native emulator validation requires a virtual CAN interface"
                );
                let socket = socketcan::CanSocket::open(name)?;
                socket.set_nonblocking(true)?;
                Self::Socketcan(socket)
            }
            #[cfg(not(target_os = "linux"))]
            CanInterface::Socketcan(_) => anyhow::bail!("SocketCAN requires Linux"),
        })
    }

    pub fn read(&self) -> Result<Option<Frame>> {
        let result = match self {
            Self::Sim(socket) => socket.recv(),
            #[cfg(target_os = "linux")]
            Self::Socketcan(socket) => {
                use socketcan::{EmbeddedFrame, Frame as _, Socket};
                match socket.read_frame() {
                    Ok(frame) => {
                        anyhow::ensure!(
                            frame.is_extended()
                                && !frame.is_error_frame()
                                && !frame.is_remote_frame(),
                            "invalid CAN frame type"
                        );
                        Ok(Frame {
                            id: frame.raw_id(),
                            data: frame.data().try_into()?,
                        })
                    }
                    Err(e) => Err(e),
                }
            }
        };
        match result {
            Ok(frame) => Ok(Some(frame)),
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => Ok(None),
            Err(e) => Err(e.into()),
        }
    }

    pub fn write(&self, frame: Frame) -> Result<()> {
        match self {
            Self::Sim(socket) => Ok(socket.send(frame)?),
            #[cfg(target_os = "linux")]
            Self::Socketcan(socket) => {
                use socketcan::{CanFrame, EmbeddedFrame, ExtendedId, Socket};
                let id = ExtendedId::new(frame.id)
                    .ok_or_else(|| anyhow::anyhow!("invalid extended CAN id"))?;
                let frame = CanFrame::new(id, &frame.data)
                    .ok_or_else(|| anyhow::anyhow!("invalid CAN payload"))?;
                Ok(socket.write_frame(&frame)?)
            }
        }
    }
}
