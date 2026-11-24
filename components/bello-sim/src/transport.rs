use anyhow::Result;
use bello_sim_can::SimSocket;
use robstride_emulator::config::CanInterface;
use robstride_protocol::Frame;

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
