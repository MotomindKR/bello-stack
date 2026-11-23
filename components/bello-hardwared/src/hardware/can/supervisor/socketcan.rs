use std::io;
use std::time::{Duration, Instant};

use anyhow::{Context, Result};
use nix::errno::Errno;
use socketcan::id::CAN_EFF_FLAG;
use socketcan::{
    CanFrame as WireFrame, CanSocket, EmbeddedFrame, ExtendedId, Frame, Socket, SocketOptions,
};
use tracing::warn;

use super::blocking::BlockingSocket;
use super::{BusSpec, CanFrame, FrameWriter, ReceivedFrame, WriteFailure};
use crate::config::BusTransportConfig;
use crate::domain::BusName;

impl FrameWriter for CanSocket {
    fn write_frame(&self, frame: CanFrame) -> Result<(), WriteFailure> {
        let id = ExtendedId::new(frame.id)
            .ok_or_else(|| WriteFailure::fatal("invalid extended CAN id"))?;
        let frame = WireFrame::new(id, &frame.data)
            .ok_or_else(|| WriteFailure::fatal("could not construct CAN frame"))?;
        Socket::write_frame(self, &frame).map_err(|error| WriteFailure {
            queue_full: error.raw_os_error() == Some(Errno::ENOBUFS as i32),
            detail: format!("CAN write failed: {error}"),
        })
    }
}

impl BlockingSocket for CanSocket {
    fn open_pair(spec: &BusSpec) -> Result<(Self, Self)> {
        let BusTransportConfig::Socketcan { interface } = &spec.transport else {
            unreachable!("SocketCAN supervisor requires a SocketCAN transport")
        };
        let writer = CanSocket::open(interface)
            .with_context(|| format!("opening SocketCAN writer {} ({})", spec.name, interface))?;
        writer.set_write_timeout(Duration::from_millis(20))?;
        let reader = CanSocket::open(interface)
            .with_context(|| format!("opening SocketCAN reader {} ({})", spec.name, interface))?;
        reader
            .set_filters(&[(CAN_EFF_FLAG, CAN_EFF_FLAG)])
            .with_context(|| {
                format!(
                    "restricting SocketCAN reader {} ({}) to extended frames",
                    spec.name, interface
                )
            })?;
        reader.set_read_timeout(Duration::from_millis(100))?;
        Ok((writer, reader))
    }

    fn receive(&self, bus: &BusName) -> io::Result<Option<ReceivedFrame>> {
        let wire = self.read_frame()?;
        let received = decode_received(bus, &wire);
        if received.is_none() {
            warn!(bus=%bus, dlc=wire.data().len(), "discarding CAN frame with invalid payload length");
        }
        Ok(received)
    }
}

fn decode_received(bus: &BusName, wire: &WireFrame) -> Option<ReceivedFrame> {
    if !wire.is_extended() {
        return None;
    }
    Some(ReceivedFrame {
        bus: bus.clone(),
        frame: CanFrame {
            id: wire.raw_id(),
            data: wire.data().try_into().ok()?,
        },
        received: Instant::now(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn short_can_payload_is_rejected() {
        let id = ExtendedId::new(1).unwrap();
        let wire = WireFrame::new(id, &[1, 2, 3]).unwrap();
        assert!(decode_received(&BusName::from("can"), &wire).is_none());
    }

    #[test]
    fn standard_can_frame_is_not_routed_to_robstride() {
        let id = socketcan::StandardId::new(0x27).unwrap();
        let wire = WireFrame::new(id, &[0x01, 1, 2, 3, 4, 5, 6]).unwrap();
        assert!(decode_received(&BusName::from("can"), &wire).is_none());
    }
}
