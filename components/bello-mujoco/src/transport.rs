use anyhow::Result;
use robstride_protocol::Frame;

#[cfg(target_os = "linux")]
pub struct CanEndpoint(socketcan::CanSocket);

#[cfg(target_os = "linux")]
impl CanEndpoint {
    pub fn open(interface: &str) -> Result<Self> {
        use socketcan::Socket;
        anyhow::ensure!(
            !interface.contains('/') && !interface.contains(".."),
            "invalid CAN interface name"
        );
        anyhow::ensure!(
            std::path::Path::new("/sys/devices/virtual/net")
                .join(interface)
                .is_dir(),
            "native emulator validation requires a virtual CAN interface"
        );
        let socket = socketcan::CanSocket::open(interface)?;
        socket.set_nonblocking(true)?;
        Ok(Self(socket))
    }
    pub fn read(&self) -> Result<Option<Frame>> {
        use socketcan::{EmbeddedFrame, Frame as _, Socket};
        match self.0.read_frame() {
            Ok(frame) => {
                anyhow::ensure!(
                    frame.is_extended() && !frame.is_error_frame() && !frame.is_remote_frame(),
                    "invalid CAN frame type"
                );
                Ok(Some(Frame {
                    id: frame.raw_id(),
                    data: frame.data().try_into()?,
                }))
            }
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => Ok(None),
            Err(e) => Err(e.into()),
        }
    }
    pub fn write(&self, frame: Frame) -> Result<()> {
        use socketcan::{CanFrame, EmbeddedFrame, ExtendedId, Socket};
        let id =
            ExtendedId::new(frame.id).ok_or_else(|| anyhow::anyhow!("invalid extended CAN id"))?;
        let frame =
            CanFrame::new(id, &frame.data).ok_or_else(|| anyhow::anyhow!("invalid CAN payload"))?;
        self.0.write_frame(&frame)?;
        Ok(())
    }
}

#[cfg(not(target_os = "linux"))]
pub struct CanEndpoint;

#[cfg(not(target_os = "linux"))]
impl CanEndpoint {
    pub fn open(_: &str) -> Result<Self> {
        anyhow::bail!("SocketCAN requires Linux; use Lima")
    }
    pub fn read(&self) -> Result<Option<Frame>> {
        anyhow::bail!("SocketCAN requires Linux")
    }
    pub fn write(&self, _: Frame) -> Result<()> {
        anyhow::bail!("SocketCAN requires Linux")
    }
}
