use std::io;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::Duration;

use anyhow::Result;
use crossbeam_channel::{Sender, bounded};

use super::{
    BusSpec, BusTxQueues, FrameWriter, ReceivedFrame, prepare_reconnect, reject_queued, run_tx,
};
use crate::domain::BusName;
use crate::hardware::lifecycle::DeviceLifecycle;

pub(super) trait BlockingSocket: FrameWriter + Sized + Send + 'static {
    fn open_pair(spec: &BusSpec) -> Result<(Self, Self)>;
    fn receive(&self, bus: &BusName) -> io::Result<Option<ReceivedFrame>>;
}

pub(super) fn run<S: BlockingSocket>(
    spec: BusSpec,
    queues: BusTxQueues,
    frames: Sender<ReceivedFrame>,
    lifecycle: DeviceLifecycle,
    shutdown: Arc<AtomicBool>,
) {
    let mut failures = 0_u32;
    lifecycle.mark_connecting(format!("opening {}", spec.transport.label()));
    while !shutdown.load(Ordering::Acquire) {
        match S::open_pair(&spec) {
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

fn run_session<S: BlockingSocket>(
    writer: S,
    reader: S,
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

fn run_rx<S: BlockingSocket>(
    reader: S,
    bus: &BusName,
    frames: &Sender<ReceivedFrame>,
    shutdown: &AtomicBool,
    session_stop: &AtomicBool,
    failure: &Sender<String>,
) {
    while !shutdown.load(Ordering::Acquire) && !session_stop.load(Ordering::Acquire) {
        let received = match reader.receive(bus) {
            Ok(Some(frame)) => frame,
            Ok(None) => continue,
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
        if frames.try_send(received).is_err() {
            let _ = failure.try_send("CAN receive queue is full".into());
            break;
        }
    }
}
