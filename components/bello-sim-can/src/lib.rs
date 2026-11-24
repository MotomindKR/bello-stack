use std::io::{self, ErrorKind};
use std::net::{Ipv4Addr, SocketAddrV4, UdpSocket};
use std::time::{Duration, Instant};

use robstride_protocol::Frame;
use serde::Deserialize;

mod clock;

const MAX_EXTENDED_ID: u32 = 0x1fff_ffff;
const FRAME_BYTES: usize = 12;
const PACKET_BYTES: usize = FRAME_BYTES + size_of::<u64>();

#[derive(Clone, Copy, Debug, Deserialize, Eq, Hash, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct SimBus {
    pub daemon_port: u16,
    pub simulator_port: u16,
}

impl SimBus {
    pub fn validate(&self) -> io::Result<()> {
        if self.daemon_port == 0
            || self.simulator_port == 0
            || self.daemon_port == self.simulator_port
        {
            return Err(io::Error::new(
                ErrorKind::InvalidInput,
                "sim CAN requires two distinct nonzero UDP ports",
            ));
        }
        Ok(())
    }

    pub fn open_daemon(&self) -> io::Result<SimSocket> {
        self.open(self.daemon_port, self.simulator_port)
    }

    pub fn open_simulator(&self) -> io::Result<SimSocket> {
        self.open(self.simulator_port, self.daemon_port)
    }

    fn open(&self, local: u16, peer: u16) -> io::Result<SimSocket> {
        self.validate()?;
        let socket = UdpSocket::bind((Ipv4Addr::LOCALHOST, local))?;
        socket.set_write_timeout(Some(Duration::from_millis(2)))?;
        Ok(SimSocket {
            socket,
            peer: SocketAddrV4::new(Ipv4Addr::LOCALHOST, peer),
        })
    }
}

pub struct SimSocket {
    socket: UdpSocket,
    peer: SocketAddrV4,
}

impl SimSocket {
    pub fn try_clone(&self) -> io::Result<Self> {
        Ok(Self {
            socket: self.socket.try_clone()?,
            peer: self.peer,
        })
    }

    pub fn set_read_timeout(&self, timeout: Option<Duration>) -> io::Result<()> {
        self.socket.set_read_timeout(timeout)
    }

    pub fn set_nonblocking(&self, nonblocking: bool) -> io::Result<()> {
        self.socket.set_nonblocking(nonblocking)
    }

    pub fn send(&self, frame: Frame) -> io::Result<()> {
        if frame.id > MAX_EXTENDED_ID {
            return Err(io::Error::new(
                ErrorKind::InvalidInput,
                "invalid extended CAN id",
            ));
        }
        let mut packet = [0; PACKET_BYTES];
        packet[..4].copy_from_slice(&frame.id.to_be_bytes());
        packet[4..FRAME_BYTES].copy_from_slice(&frame.data);
        packet[FRAME_BYTES..].copy_from_slice(&clock::monotonic_ns()?.to_be_bytes());
        self.socket.send_to(&packet, self.peer)?;
        Ok(())
    }

    pub fn recv(&self) -> io::Result<Frame> {
        self.recv_packet().map(|(frame, _)| frame)
    }

    /// Receive a frame and its loopback peer's send time on the local monotonic timeline.
    pub fn recv_timestamped(&self) -> io::Result<(Frame, Instant)> {
        let (frame, sent_ns) = self.recv_packet()?;
        // Sample Instant first so a pause between clock reads can only increase the age.
        let now = Instant::now();
        let age_ns = clock::monotonic_ns()?
            .checked_sub(sent_ns)
            .ok_or(ErrorKind::WouldBlock)?;
        let sent = now
            .checked_sub(Duration::from_nanos(age_ns))
            .ok_or(ErrorKind::WouldBlock)?;
        Ok((frame, sent))
    }

    fn recv_packet(&self) -> io::Result<(Frame, u64)> {
        // One extra byte distinguishes an oversized datagram from a valid packet.
        let mut packet = [0; PACKET_BYTES + 1];
        let (size, source) = match self.socket.recv_from(&mut packet) {
            Ok(received) => received,
            // Winsock reports oversized UDP datagrams as WSAEMSGSIZE instead of truncating.
            #[cfg(windows)]
            Err(error) if error.raw_os_error() == Some(10040) => {
                return Err(ErrorKind::WouldBlock.into());
            }
            // Windows reports ICMP port unreachable from a prior send here.
            Err(error) if error.kind() == ErrorKind::ConnectionReset => {
                return Err(ErrorKind::WouldBlock.into());
            }
            Err(error) => return Err(error),
        };
        let id = u32::from_be_bytes(packet[..4].try_into().unwrap());
        if size != PACKET_BYTES || source != self.peer.into() || id > MAX_EXTENDED_ID {
            return Err(ErrorKind::WouldBlock.into());
        }
        Ok((
            Frame {
                id,
                data: packet[4..FRAME_BYTES].try_into().unwrap(),
            },
            u64::from_be_bytes(packet[FRAME_BYTES..PACKET_BYTES].try_into().unwrap()),
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn only_complete_extended_frames_from_the_configured_peer_are_delivered() {
        let bind = || UdpSocket::bind((Ipv4Addr::LOCALHOST, 0)).unwrap();
        let receiver = bind();
        let address = receiver.local_addr().unwrap();
        let peer = bind();
        let endpoint = SimSocket {
            socket: receiver,
            peer: SocketAddrV4::new(Ipv4Addr::LOCALHOST, peer.local_addr().unwrap().port()),
        };
        endpoint
            .set_read_timeout(Some(Duration::from_secs(1)))
            .unwrap();

        bind().send_to(&[0; PACKET_BYTES], address).unwrap();
        assert_eq!(endpoint.recv().unwrap_err().kind(), ErrorKind::WouldBlock);
        for invalid in [
            &[0; FRAME_BYTES][..],
            &[0; PACKET_BYTES + 4][..],
            &[0xff; PACKET_BYTES][..],
        ] {
            peer.send_to(invalid, address).unwrap();
            assert_eq!(endpoint.recv().unwrap_err().kind(), ErrorKind::WouldBlock);
        }
        let mut valid = [0; PACKET_BYTES];
        valid[..FRAME_BYTES].copy_from_slice(&[0x1f, 0xff, 0xff, 0xff, 0, 1, 2, 3, 4, 5, 6, 7]);
        valid[FRAME_BYTES..].copy_from_slice(&clock::monotonic_ns().unwrap().to_be_bytes());
        peer.send_to(&valid, address).unwrap();
        let received = endpoint.recv().unwrap();
        assert_eq!(received.id, MAX_EXTENDED_ID);
        assert_eq!(received.data, [0, 1, 2, 3, 4, 5, 6, 7]);
        endpoint.send(received).unwrap();
        peer.set_read_timeout(Some(Duration::from_secs(1))).unwrap();
        let mut packet = [0; PACKET_BYTES];
        assert_eq!(
            peer.recv_from(&mut packet).unwrap(),
            (PACKET_BYTES, address)
        );
        assert_eq!(packet[..FRAME_BYTES], valid[..FRAME_BYTES]);
    }

    #[test]
    fn timestamped_receive_includes_time_spent_in_the_queue() {
        let receiver = UdpSocket::bind((Ipv4Addr::LOCALHOST, 0)).unwrap();
        let sender = UdpSocket::bind((Ipv4Addr::LOCALHOST, 0)).unwrap();
        let endpoint = SimSocket {
            peer: SocketAddrV4::new(Ipv4Addr::LOCALHOST, sender.local_addr().unwrap().port()),
            socket: receiver,
        };
        let sender = SimSocket {
            peer: SocketAddrV4::new(
                Ipv4Addr::LOCALHOST,
                endpoint.socket.local_addr().unwrap().port(),
            ),
            socket: sender,
        };
        endpoint
            .set_read_timeout(Some(Duration::from_secs(1)))
            .unwrap();
        let mut future = [0; PACKET_BYTES];
        future[FRAME_BYTES..].copy_from_slice(&u64::MAX.to_be_bytes());
        sender.socket.send_to(&future, sender.peer).unwrap();
        assert_eq!(
            endpoint.recv_timestamped().unwrap_err().kind(),
            ErrorKind::WouldBlock
        );

        let frame = Frame {
            id: 1,
            data: [7; 8],
        };
        sender.send(frame).unwrap();
        let sent_by = Instant::now();
        drop(sender);
        std::thread::sleep(Duration::from_millis(20));
        let (received, timestamp) = endpoint.recv_timestamped().unwrap();
        assert_eq!(received, frame);
        assert!(timestamp <= sent_by);
    }
}
