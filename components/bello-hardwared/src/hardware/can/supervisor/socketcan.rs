use std::io;
use std::time::{Duration, Instant, SystemTime};

use anyhow::{Context, Result};
use nix::errno::Errno;
use nix::sys::time::TimeSpec;
use nix::sys::timerfd::{ClockId, Expiration, TimerFd, TimerFlags, TimerSetTimeFlags};
use socketcan::id::CAN_EFF_FLAG;
use socketcan::{
    CanFrame as WireFrame, CanSocket, EmbeddedFrame, ExtendedId, Frame, SOF_TIMESTAMPING_OPT_CMSG,
    SOF_TIMESTAMPING_RX_SOFTWARE, SOF_TIMESTAMPING_SOFTWARE, Socket, SocketOptions,
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

pub(super) struct SocketCanReader {
    socket: CanSocket,
    clock_changes: TimerFd,
}

impl BlockingSocket for SocketCanReader {
    type Writer = CanSocket;

    fn open_pair(spec: &BusSpec) -> Result<(CanSocket, Self)> {
        let BusTransportConfig::Socketcan { interface } = &spec.transport else {
            unreachable!("SocketCAN supervisor requires a SocketCAN transport")
        };
        let writer = CanSocket::open(interface)
            .with_context(|| format!("opening SocketCAN writer {} ({})", spec.name, interface))?;
        writer.set_write_timeout(Duration::from_millis(20))?;

        // A disarmed timer can report wall-clock changes without expiring.
        let clock_changes = TimerFd::new(
            ClockId::CLOCK_REALTIME,
            TimerFlags::TFD_NONBLOCK | TimerFlags::TFD_CLOEXEC,
        )?;
        clock_changes.set(
            Expiration::OneShot(TimeSpec::new(0, 0)),
            TimerSetTimeFlags::TFD_TIMER_ABSTIME | TimerSetTimeFlags::TFD_TIMER_CANCEL_ON_SET,
        )?;
        let reader = CanSocket::open(interface)
            .with_context(|| format!("opening SocketCAN reader {} ({})", spec.name, interface))?;
        // SO_TIMESTAMPNS can generate a dequeue timestamp if the skb has no timestamp.
        reader.set_timestamping(
            SOF_TIMESTAMPING_RX_SOFTWARE | SOF_TIMESTAMPING_SOFTWARE | SOF_TIMESTAMPING_OPT_CMSG,
        )?;
        reader
            .set_filters(&[(CAN_EFF_FLAG, CAN_EFF_FLAG)])
            .with_context(|| {
                format!(
                    "restricting SocketCAN reader {} ({}) to extended frames",
                    spec.name, interface
                )
            })?;
        reader.set_read_timeout(Duration::from_millis(100))?;
        Ok((
            writer,
            Self {
                socket: reader,
                clock_changes,
            },
        ))
    }

    fn receive(&self, bus: &BusName) -> io::Result<Option<ReceivedFrame>> {
        let (wire, timestamps) = self.socket.read_frame_with_timestamps()?;
        let Some(arrived_at) = timestamps.sw else {
            return Ok(None);
        };
        let monotonic_now = Instant::now();
        let realtime_now = SystemTime::now();
        match self.clock_changes.wait() {
            Err(Errno::EAGAIN) => {}
            // Reopening the socket discards frames whose wall-clock timestamps are ambiguous.
            Ok(()) => {
                return Err(io::Error::other(
                    "clock changed while receiving SocketCAN frames",
                ));
            }
            Err(error) => return Err(error.into()),
        }
        let timestamp = realtime_now
            .duration_since(arrived_at)
            .ok()
            .and_then(|age| monotonic_now.checked_sub(age))
            .ok_or_else(|| {
                io::Error::new(
                    io::ErrorKind::InvalidData,
                    "invalid SocketCAN receive timestamp",
                )
            })?;
        let received = decode_received(bus, &wire, timestamp);
        if received.is_none() {
            warn!(bus=%bus, dlc=wire.data().len(), "discarding CAN frame with invalid payload length");
        }
        Ok(received)
    }
}

fn decode_received(bus: &BusName, wire: &WireFrame, received: Instant) -> Option<ReceivedFrame> {
    if !wire.is_extended() {
        return None;
    }
    Some(ReceivedFrame {
        bus: bus.clone(),
        frame: CanFrame {
            id: wire.raw_id(),
            data: wire.data().try_into().ok()?,
        },
        received,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn short_can_payload_is_rejected() {
        let id = ExtendedId::new(1).unwrap();
        let wire = WireFrame::new(id, &[1, 2, 3]).unwrap();
        assert!(decode_received(&BusName::from("can"), &wire, Instant::now()).is_none());
    }

    #[test]
    fn standard_can_frame_is_not_routed_to_robstride() {
        let id = socketcan::StandardId::new(0x27).unwrap();
        let wire = WireFrame::new(id, &[0x01, 1, 2, 3, 4, 5, 6]).unwrap();
        assert!(decode_received(&BusName::from("can"), &wire, Instant::now()).is_none());
    }
}
