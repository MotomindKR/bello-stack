use std::cmp::Ordering;
use std::collections::BinaryHeap;
use std::time::{Duration, Instant};

use anyhow::{Context, Result};
use bello_sim_can::SimSocket;
use robstride_protocol::Frame;
#[cfg(target_os = "linux")]
use socketcan::{CanFrame, CanSocket, EmbeddedFrame, ExtendedId, Frame as _, Socket as _};
use std::io;

use crate::config::{CanInterface, Config};
use crate::emulator::{Emulator, ScheduledFrame};

struct PendingFrame {
    transmit_at: Instant,
    sequence: u64,
    frame: Frame,
}

impl PartialEq for PendingFrame {
    fn eq(&self, other: &Self) -> bool {
        self.transmit_at == other.transmit_at && self.sequence == other.sequence
    }
}

impl Eq for PendingFrame {}

impl PartialOrd for PendingFrame {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for PendingFrame {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .transmit_at
            .cmp(&self.transmit_at)
            .then_with(|| other.sequence.cmp(&self.sequence))
    }
}

const MAX_PENDING_FRAMES: usize = 65_536;

enum Socket {
    Sim(SimSocket),
    #[cfg(target_os = "linux")]
    Socketcan(CanSocket),
}

impl Socket {
    fn open(interface: &CanInterface) -> Result<Self> {
        let timeout = Duration::from_millis(2);
        Ok(match interface {
            CanInterface::Sim(bus) => {
                let socket = bus.open_simulator()?;
                socket.set_read_timeout(Some(timeout))?;
                Self::Sim(socket)
            }
            #[cfg(target_os = "linux")]
            CanInterface::Socketcan(name) => {
                let socket = CanSocket::open(name)?;
                socket.set_read_timeout(timeout)?;
                Self::Socketcan(socket)
            }
            #[cfg(not(target_os = "linux"))]
            CanInterface::Socketcan(_) => anyhow::bail!("SocketCAN emulation requires Linux"),
        })
    }

    fn read(&self) -> io::Result<Option<Frame>> {
        match self {
            Self::Sim(socket) => socket.recv().map(Some),
            #[cfg(target_os = "linux")]
            Self::Socketcan(socket) => {
                let frame = socket.read_frame()?;
                let Ok(data) = frame.data().try_into() else {
                    tracing::warn!(
                        dlc = frame.data().len(),
                        "discarding CAN frame with invalid payload length"
                    );
                    return Ok(None);
                };
                Ok(Some(Frame {
                    id: frame.raw_id(),
                    data,
                }))
            }
        }
    }

    fn write(&self, frame: Frame) -> Result<()> {
        match self {
            Self::Sim(socket) => Ok(socket.send(frame)?),
            #[cfg(target_os = "linux")]
            Self::Socketcan(socket) => {
                let id = ExtendedId::new(frame.id).context("invalid extended CAN id")?;
                let frame = CanFrame::new(id, &frame.data).context("invalid CAN payload")?;
                Ok(socket.write_frame(&frame)?)
            }
        }
    }
}

pub fn run(config: Config) -> Result<()> {
    let interface = &config.interface;
    let socket = Socket::open(interface).with_context(|| format!("opening {interface}"))?;
    let actuator_count = config.actuators.len();
    tracing::info!(%interface, actuator_count, "Robstride emulator online");
    let mut emulator = Emulator::new(config)?;
    let mut last_step = Instant::now();
    let mut pending = BinaryHeap::<PendingFrame>::new();
    let mut next_sequence = 0_u64;

    loop {
        match socket.read() {
            Ok(Some(frame)) => emulator.handle(frame),
            Ok(None) => {}
            Err(error)
                if matches!(
                    error.kind(),
                    io::ErrorKind::WouldBlock
                        | io::ErrorKind::TimedOut
                        | io::ErrorKind::Interrupted
                ) => {}
            Err(error) => return Err(error).context("reading CAN frame"),
        }

        let now = Instant::now();
        let elapsed = now.saturating_duration_since(last_step);
        last_step = now;
        let mut dropped = 0_usize;
        for ScheduledFrame { delay, frame } in emulator.advance(elapsed) {
            if pending.len() >= MAX_PENDING_FRAMES {
                dropped += 1;
                continue;
            }
            pending.push(PendingFrame {
                transmit_at: now + delay,
                sequence: next_sequence,
                frame,
            });
            next_sequence = next_sequence.wrapping_add(1);
        }
        if dropped != 0 {
            tracing::warn!(
                dropped,
                limit = MAX_PENDING_FRAMES,
                "delayed feedback queue full"
            );
        }

        while pending.peek().is_some_and(|frame| frame.transmit_at <= now) {
            let pending_frame = pending.pop().expect("peeked pending frame");
            socket
                .write(pending_frame.frame)
                .context("writing CAN frame")?;
        }
    }
}
