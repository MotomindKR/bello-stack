use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::{Duration, Instant};

use anyhow::{Context, Result};
use crossbeam_channel::{Sender, bounded};
use socketcan::id::CAN_EFF_FLAG;
use socketcan::{
    CanFrame as WireFrame, CanSocket, EmbeddedFrame, ExtendedId, Frame, Socket, SocketOptions,
};
use tracing::warn;

use super::{
    BusSpec, BusTxQueues, CanFrame, FrameWriter, ReceivedFrame, prepare_reconnect, reject_queued,
    run_tx,
};
use crate::config::BusTransportConfig;
use crate::domain::BusName;
use crate::hardware::lifecycle::DeviceLifecycle;

impl FrameWriter for CanSocket {
    fn write_frame(&self, frame: CanFrame) -> Result<(), String> {
        let id = ExtendedId::new(frame.id).ok_or_else(|| "invalid extended CAN id".to_owned())?;
        let frame = WireFrame::new(id, &frame.data)
            .ok_or_else(|| "could not construct CAN frame".to_owned())?;
        Socket::write_frame(self, &frame).map_err(|error| format!("CAN write failed: {error}"))
    }
}

pub(super) fn run(
    spec: BusSpec,
    queues: BusTxQueues,
    frames: Sender<ReceivedFrame>,
    lifecycle: DeviceLifecycle,
    shutdown: Arc<AtomicBool>,
) {
    let mut failures = 0_u32;
    lifecycle.mark_connecting(format!("opening {}", spec.transport.label()));
    while !shutdown.load(Ordering::Acquire) {
        match open_sockets(&spec) {
            Ok((writer, reader)) => {
                failures = 0;
                lifecycle.mark_online(format!("{} online", spec.transport.label()));
                if let Err(detail) = run_session(writer, reader, &spec, &queues, &frames, &shutdown)
                {
                    if shutdown.load(Ordering::Acquire) {
                        break;
                    }
                    prepare_reconnect(&spec, &lifecycle, &queues, detail, &shutdown, &mut failures);
                }
            }
            Err(error) => prepare_reconnect(
                &spec,
                &lifecycle,
                &queues,
                error.to_string(),
                &shutdown,
                &mut failures,
            ),
        }
    }
    reject_queued(&queues, "CAN supervisor stopped");
    lifecycle.shutdown("CAN supervisor stopped");
}

fn open_sockets(spec: &BusSpec) -> Result<(CanSocket, CanSocket)> {
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

fn run_session(
    writer: CanSocket,
    reader: CanSocket,
    spec: &BusSpec,
    queues: &BusTxQueues,
    frames: &Sender<ReceivedFrame>,
    shutdown: &Arc<AtomicBool>,
) -> Result<(), String> {
    let session_stop = Arc::new(AtomicBool::new(false));
    let (failure_tx, failure_rx) = bounded::<String>(2);

    let tx_queues = queues.clone();
    let tx_shutdown = shutdown.clone();
    let tx_stop = session_stop.clone();
    let tx_failure = failure_tx.clone();
    let tx_spec = spec.clone();
    let tx_bus = spec.name.to_string();
    let tx = thread::Builder::new()
        .name(format!("{tx_bus}-tx"))
        .spawn(move || {
            run_tx(
                writer,
                &tx_spec,
                &tx_queues,
                &tx_shutdown,
                &tx_stop,
                &tx_failure,
            )
        })
        .map_err(|error| error.to_string())?;

    let rx_shutdown = shutdown.clone();
    let rx_stop = session_stop.clone();
    let rx_failure = failure_tx;
    let rx_bus = spec.name.clone();
    let rx_frames = frames.clone();
    let rx = match thread::Builder::new()
        .name(format!("{rx_bus}-rx"))
        .spawn(move || {
            run_rx(
                reader,
                &rx_bus,
                &rx_frames,
                &rx_shutdown,
                &rx_stop,
                &rx_failure,
            )
        }) {
        Ok(thread) => thread,
        Err(error) => {
            session_stop.store(true, Ordering::Release);
            let _ = tx.join();
            return Err(error.to_string());
        }
    };

    let result = loop {
        if shutdown.load(Ordering::Acquire) {
            break Ok(());
        }
        match failure_rx.recv_timeout(Duration::from_millis(20)) {
            Ok(error) => break Err(error),
            Err(crossbeam_channel::RecvTimeoutError::Timeout) => {}
            Err(crossbeam_channel::RecvTimeoutError::Disconnected) => {
                break Err("CAN session workers disconnected".into());
            }
        }
    };
    session_stop.store(true, Ordering::Release);
    if tx.join().is_err() || rx.join().is_err() {
        return Err("CAN session worker panicked".into());
    }
    result
}

fn run_rx(
    reader: CanSocket,
    bus: &BusName,
    frames: &Sender<ReceivedFrame>,
    shutdown: &AtomicBool,
    session_stop: &AtomicBool,
    failure: &Sender<String>,
) {
    while !shutdown.load(Ordering::Acquire) && !session_stop.load(Ordering::Acquire) {
        let wire = match reader.read_frame() {
            Ok(frame) => frame,
            Err(error)
                if matches!(
                    error.kind(),
                    std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut
                ) =>
            {
                continue;
            }
            Err(error) => {
                let _ = failure.try_send(format!("CAN read failed: {error}"));
                break;
            }
        };
        let Some(received) = decode_received(bus, &wire) else {
            warn!(bus=%bus, dlc=wire.data().len(), "discarding CAN frame with invalid payload length");
            continue;
        };
        if frames.try_send(received).is_err() {
            let _ = failure.try_send("CAN receive queue is full".into());
            break;
        }
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
