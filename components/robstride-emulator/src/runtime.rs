use std::time::{Duration, Instant};

use anyhow::{Context, Result, bail};
use robstride_protocol::Frame;
use socketcan::{CanFrame, CanSocket, EmbeddedFrame, ExtendedId, Frame as _, Socket};

use crate::config::Config;
use crate::emulator::{Emulator, ScheduledFrame};

struct PendingFrame {
    transmit_at: Instant,
    frame: Frame,
}

pub fn run(config: Config) -> Result<()> {
    let socket = CanSocket::open(&config.interface)
        .with_context(|| format!("opening SocketCAN interface {}", config.interface))?;
    socket.set_read_timeout(Duration::from_millis(2))?;
    let interface = config.interface.clone();
    let actuator_count = config.actuators.len();
    let mut emulator = Emulator::new(config)?;
    let mut last_step = Instant::now();
    let mut pending = Vec::<PendingFrame>::new();
    tracing::info!(%interface, actuator_count, "Robstride emulator online");

    loop {
        match socket.read_frame() {
            Ok(frame) => emulator.handle(Frame {
                id: frame.raw_id(),
                data: frame.data().try_into().unwrap_or([0; 8]),
            }),
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
        pending.extend(emulator.advance(elapsed).into_iter().map(
            |ScheduledFrame { delay, frame }| PendingFrame {
                transmit_at: now + delay,
                frame,
            },
        ));

        let mut index = 0;
        while index < pending.len() {
            if pending[index].transmit_at > now {
                index += 1;
                continue;
            }
            let pending_frame = pending.swap_remove(index);
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
