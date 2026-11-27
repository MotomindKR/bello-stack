use std::cell::Cell;
use std::io;
use std::mem::size_of;
use std::os::fd::AsRawFd;
use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};
use std::thread;
use std::time::{Duration, Instant, SystemTime};

use anyhow::{Context, Result};
use nix::sys::time::TimeSpec;
use nix::sys::timerfd::{ClockId, Expiration, TimerFd, TimerFlags, TimerSetTimeFlags};
use nix::{errno::Errno, libc};
use socketcan::frame::can_frame_default;
use socketcan::id::CAN_ERR_FLAG;
use socketcan::timestamp::{
    SOF_TIMESTAMPING_OPT_CMSG, SOF_TIMESTAMPING_RX_SOFTWARE, SOF_TIMESTAMPING_SOFTWARE,
};
use socketcan::{
    CanFrame as WireFrame, CanSocket, EmbeddedFrame, ExtendedId, Frame, Socket, SocketOptions,
};
use tracing::warn;

use super::super::BusAccess;
use super::blocking::BlockingSocket;
use super::{BusSpec, CanFrame, FrameWriter, ReceivedFrame, WriteFailure};
use crate::config::BusTransportConfig;
use crate::domain::BusName;

const CAN_ERR_CRTL: u32 = 0x04;
const CAN_ERR_BUSOFF: u32 = 0x40;
const CAN_ERR_CRTL_RX_OVERFLOW: u8 = 0x01;
const CONFIRMATION_TIMEOUT: Duration = Duration::from_millis(100);

pub(super) struct SocketCanWriter {
    socket: Arc<CanSocket>,
    sent: Cell<u64>,
    confirmed: Arc<AtomicU64>,
}

impl FrameWriter for SocketCanWriter {
    fn write_frame(&self, frame: CanFrame) -> Result<(), WriteFailure> {
        let id = ExtendedId::new(frame.id)
            .ok_or_else(|| WriteFailure::fatal("invalid extended CAN id"))?;
        let frame = WireFrame::new(id, &frame.data)
            .ok_or_else(|| WriteFailure::fatal("could not construct CAN frame"))?;
        self.socket
            .write_frame(&frame)
            .map_err(|error| WriteFailure {
                queue_full: error.raw_os_error() == Some(Errno::ENOBUFS as i32),
                detail: format!("CAN write failed: {error}"),
            })?;
        self.sent.set(self.sent.get() + 1);
        Ok(())
    }

    fn fence_writes(&self) -> Result<(), String> {
        let deadline = Instant::now() + CONFIRMATION_TIMEOUT;
        while self.confirmed.load(Ordering::Acquire) < self.sent.get() {
            if Instant::now() >= deadline {
                return Err("SocketCAN transmit confirmations timed out".into());
            }
            thread::sleep(Duration::from_micros(200));
        }
        Ok(())
    }
}

pub(super) struct SocketCanReader {
    socket: Arc<CanSocket>,
    confirmed: Arc<AtomicU64>,
    clock_changes: TimerFd,
}

impl BlockingSocket for SocketCanReader {
    type Writer = SocketCanWriter;

    fn open_pair(spec: &BusSpec) -> Result<(SocketCanWriter, Self)> {
        let BusTransportConfig::Socketcan { interface } = &spec.transport else {
            unreachable!("SocketCAN supervisor requires a SocketCAN transport")
        };

        // A disarmed timer can report wall-clock changes without expiring.
        let clock_changes = TimerFd::new(
            ClockId::CLOCK_REALTIME,
            TimerFlags::TFD_NONBLOCK | TimerFlags::TFD_CLOEXEC,
        )?;
        clock_changes.set(
            Expiration::OneShot(TimeSpec::new(0, 0)),
            TimerSetTimeFlags::TFD_TIMER_ABSTIME | TimerSetTimeFlags::TFD_TIMER_CANCEL_ON_SET,
        )?;
        let socket = Arc::new(
            CanSocket::open(interface)
                .with_context(|| format!("opening SocketCAN bus {} ({})", spec.name, interface))?,
        );
        socket.set_write_timeout(Duration::from_millis(20))?;
        socket.set_read_timeout(Duration::from_millis(100))?;
        socket.set_recv_own_msgs(true)?;
        socket.set_error_filter(CAN_ERR_CRTL | CAN_ERR_BUSOFF)?;
        socket.set_socket_option(libc::SOL_SOCKET, libc::SO_RXQ_OVFL, &1_i32)?;
        // SO_TIMESTAMPNS can generate a dequeue timestamp if the skb has no timestamp.
        socket.set_timestamping(
            SOF_TIMESTAMPING_RX_SOFTWARE | SOF_TIMESTAMPING_SOFTWARE | SOF_TIMESTAMPING_OPT_CMSG,
        )?;
        let confirmed = Arc::new(AtomicU64::new(0));
        Ok((
            SocketCanWriter {
                socket: socket.clone(),
                sent: Cell::new(0),
                confirmed: confirmed.clone(),
            },
            Self {
                socket,
                confirmed,
                clock_changes,
            },
        ))
    }

    fn receive(&self, bus: &BusName, access: &BusAccess) -> io::Result<Option<ReceivedFrame>> {
        let (wire, arrived_at, flags) = read_wire(&self.socket)?;
        if wire.id_word() & CAN_ERR_FLAG != 0 {
            if wire.id_word() & CAN_ERR_BUSOFF != 0 {
                return Err(io::Error::other("SocketCAN bus off"));
            }
            if wire.id_word() & CAN_ERR_CRTL != 0
                && wire
                    .data()
                    .get(1)
                    .is_some_and(|status| status & CAN_ERR_CRTL_RX_OVERFLOW != 0)
            {
                return Err(io::Error::other("SocketCAN controller receive overflow"));
            }
            return Ok(None);
        }
        if flags & libc::MSG_CONFIRM != 0 {
            self.confirmed.fetch_add(1, Ordering::Release);
            return Ok(None);
        }
        let Some(arrived_at) = arrived_at else {
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
            access
                .reject_unrepresentable_frame(timestamp, None)
                .map_err(io::Error::other)?;
            warn!(bus=%bus, dlc=wire.data().len(), "discarding unsupported CAN frame");
        }
        Ok(received)
    }
}

// Read both timestamps and loss notices. The socketcan timestamp helper
// does not expose the overflow counter or MSG_CONFIRM.
fn read_wire(socket: &CanSocket) -> io::Result<(WireFrame, Option<SystemTime>, libc::c_int)> {
    let mut frame = can_frame_default();
    let mut iov = libc::iovec {
        iov_base: std::ptr::from_mut(&mut frame).cast(),
        iov_len: size_of::<libc::can_frame>(),
    };
    let mut control = [0_usize; 16];
    // SAFETY: zero is valid for the unused name and control fields. The
    // initialized buffers live through recvmsg and are writable and aligned.
    let mut message: libc::msghdr = unsafe { std::mem::zeroed() };
    message.msg_iov = &mut iov;
    message.msg_iovlen = 1;
    message.msg_control = control.as_mut_ptr().cast();
    message.msg_controllen = size_of_val(&control);
    let length = unsafe { libc::recvmsg(socket.as_raw_fd(), &mut message, 0) };
    if length < 0 {
        return Err(io::Error::last_os_error());
    }
    if length as usize != iov.iov_len
        || message.msg_flags & (libc::MSG_TRUNC | libc::MSG_CTRUNC) != 0
    {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "truncated SocketCAN frame",
        ));
    }

    let mut timestamp = None;
    // SAFETY: libc walks the kernel-populated control buffer. Validate each
    // payload's declared length before loading its data.
    unsafe {
        let mut header = libc::CMSG_FIRSTHDR(&message);
        while let Some(cmsg) = header.as_ref() {
            if cmsg.cmsg_level == libc::SOL_SOCKET {
                match cmsg.cmsg_type {
                    libc::SO_RXQ_OVFL
                        if cmsg.cmsg_len >= libc::CMSG_LEN(size_of::<u32>() as u32) as usize =>
                    {
                        if libc::CMSG_DATA(header).cast::<u32>().read_unaligned() != 0 {
                            return Err(io::Error::other("SocketCAN socket receive overflow"));
                        }
                    }
                    libc::SCM_TIMESTAMPING
                        if cmsg.cmsg_len
                            >= libc::CMSG_LEN((3 * size_of::<libc::timespec>()) as u32)
                                as usize =>
                    {
                        let ts = libc::CMSG_DATA(header)
                            .cast::<libc::timespec>()
                            .read_unaligned();
                        if ts.tv_sec >= 0
                            && (0..1_000_000_000).contains(&ts.tv_nsec)
                            && (ts.tv_sec != 0 || ts.tv_nsec != 0)
                        {
                            timestamp = SystemTime::UNIX_EPOCH
                                .checked_add(Duration::new(ts.tv_sec as u64, ts.tv_nsec as u32));
                        }
                    }
                    _ => {}
                }
            }
            header = libc::CMSG_NXTHDR(&message, header);
        }
    }
    Ok((WireFrame::from(frame), timestamp, message.msg_flags))
}

fn decode_received(bus: &BusName, wire: &WireFrame, received: Instant) -> Option<ReceivedFrame> {
    if !wire.is_extended() || wire.is_remote_frame() {
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
    #[ignore = "requires an isolated SocketCAN vcan interface"]
    fn vcan_confirms_own_writes_and_delivers_peer_frames() {
        let interface = std::env::var("BELLO_E2E_CAN_INTERFACE").expect("set the vcan interface");
        let config: crate::config::BusConfig = serde_yaml::from_str(&format!(
            "name: test\ntransport: {{ type: socketcan, interface: {interface} }}"
        ))
        .unwrap();
        let spec = BusSpec::try_from(&config).unwrap();
        let (writer, reader) = SocketCanReader::open_pair(&spec).unwrap();
        let peer = CanSocket::open(&interface).unwrap();
        peer.set_read_timeout(Duration::from_secs(1)).unwrap();

        let access = BusAccess::default();
        let sent = CanFrame {
            id: 0x0b00fd01,
            data: [4; 8],
        };
        let reply = WireFrame::new(ExtendedId::new(0x0b0001fd).unwrap(), &[7; 8]).unwrap();
        thread::scope(|scope| {
            let receiver = scope.spawn(|| {
                assert!(reader.receive(&spec.name, &access).unwrap().is_none());
                reader.receive(&spec.name, &access).unwrap().unwrap()
            });
            writer.write_frame(sent).unwrap();
            let on_wire = peer.read_frame().unwrap();
            assert_eq!(on_wire.raw_id(), sent.id);
            assert_eq!(on_wire.data(), sent.data);
            writer.fence_writes().unwrap();
            peer.write_frame(&reply).unwrap();
            let received = receiver.join().unwrap();
            assert_eq!(received.frame.id, reply.raw_id());
            assert_eq!(received.frame.data, reply.data());
        });
    }

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
