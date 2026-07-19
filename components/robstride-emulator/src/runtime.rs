use std::cmp::Ordering;
use std::collections::BinaryHeap;
use std::time::{Duration, Instant};

use anyhow::{Context, Result, bail};
use robstride_protocol::Frame;
use socketcan::{CanFrame, CanSocket, EmbeddedFrame, ExtendedId, Frame as _, Socket};

use crate::config::Config;
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

pub fn run(config: Config) -> Result<()> {
    let socket = CanSocket::open(&config.interface)
        .with_context(|| format!("opening SocketCAN interface {}", config.interface))?;
    socket.set_read_timeout(Duration::from_millis(2))?;
    let interface = config.interface.clone();
    let actuator_count = config.actuators.len();
    let mut emulator = Emulator::new(config)?;
    let mut last_step = Instant::now();
    let mut pending = BinaryHeap::<PendingFrame>::new();
    let mut next_sequence = 0_u64;
    tracing::info!(%interface, actuator_count, "Robstride emulator online");

    loop {
        match socket.read_frame() {
            Ok(frame) => {
                let Ok(data) = frame.data().try_into() else {
                    tracing::warn!(
                        dlc = frame.data().len(),
                        "discarding CAN frame with invalid payload length"
                    );
                    continue;
                };
                emulator.handle(Frame {
                    id: frame.raw_id(),
                    data,
                });
            }
            Err(error)
                if matches!(
                    error.kind(),
                    std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut
                ) => {}
            Err(error) => return Err(error).context("reading SocketCAN frame"),
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
            write(&socket, pending_frame.frame)?;
        }
    }
}

fn write(socket: &CanSocket, frame: Frame) -> Result<()> {
    let Some(id) = ExtendedId::new(frame.id) else {
        bail!("invalid extended CAN id 0x{:08x}", frame.id);
    };
    let Some(frame) = CanFrame::new(id, &frame.data) else {
        bail!("failed to construct SocketCAN frame");
    };
    socket
        .write_frame(&frame)
        .context("writing SocketCAN frame")
}
