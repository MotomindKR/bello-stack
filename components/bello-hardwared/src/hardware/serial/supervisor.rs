use std::io::{Read, Write};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::{Duration, Instant};

use anyhow::{Context, Result};
use crossbeam_channel::{Receiver, Sender, bounded};
use serialport::SerialPort;
use tracing::warn;

use super::{OutboundBytes, SerialChunk, SerialSpec};
use crate::domain::SerialDeviceName;
use crate::hardware::lifecycle::{DeviceLifecycle, wait_for_stop};

pub(super) fn run(
    spec: SerialSpec,
    outbound: Receiver<OutboundBytes>,
    chunks: Sender<SerialChunk>,
    chunk_queue: Receiver<SerialChunk>,
    reconnect: Receiver<String>,
    lifecycle: DeviceLifecycle,
    stop: Arc<AtomicBool>,
) {
    let mut failures = 0_u32;
    lifecycle.mark_connecting(format!("opening serial port {}", spec.port));
    while !stop.load(Ordering::Acquire) {
        match open_port(&spec) {
            Ok(port) => {
                failures = 0;
                // Reconnect requests raised against the previous transport
                // generation are satisfied by this newly opened session.
                while reconnect.try_recv().is_ok() {}
                lifecycle.mark_online(format!("serial port {} online", spec.port));
                if let Err(detail) =
                    run_session(port, &outbound, &chunks, &reconnect, &stop, &spec.name)
                {
                    if stop.load(Ordering::Acquire) {
                        break;
                    }
                    prepare_reconnect(
                        &spec,
                        &outbound,
                        &chunk_queue,
                        &lifecycle,
                        &stop,
                        &mut failures,
                        detail,
                    );
                }
            }
            Err(error) => prepare_reconnect(
                &spec,
                &outbound,
                &chunk_queue,
                &lifecycle,
                &stop,
                &mut failures,
                error.to_string(),
            ),
        }
    }
    reject_outbound(&outbound, "serial supervisor stopped");
    clear_chunks(&chunk_queue);
    lifecycle.shutdown("serial supervisor stopped");
}

fn open_port(spec: &SerialSpec) -> Result<Box<dyn SerialPort>> {
    serialport::new(&spec.port, spec.baud_rate)
        .timeout(spec.timeout)
        .open()
        .with_context(|| format!("opening serial device {} ({})", spec.name, spec.port))
}

fn run_session(
    reader: Box<dyn SerialPort>,
    outbound: &Receiver<OutboundBytes>,
    chunks: &Sender<SerialChunk>,
    reconnect: &Receiver<String>,
    stop: &Arc<AtomicBool>,
    name: &SerialDeviceName,
) -> Result<(), String> {
    let writer = reader.try_clone().map_err(|error| error.to_string())?;
    let session_stop = Arc::new(AtomicBool::new(false));
    let (failure_tx, failure_rx) = bounded::<String>(2);

    let tx_outbound = outbound.clone();
    let tx_stop = stop.clone();
    let tx_session_stop = session_stop.clone();
    let tx_failure = failure_tx.clone();
    let tx_name = name.to_string();
    let tx = thread::Builder::new()
        .name(format!("{tx_name}-serial-tx"))
        .spawn(move || {
            run_writer(
                writer,
                &tx_outbound,
                &tx_stop,
                &tx_session_stop,
                &tx_failure,
            )
        })
        .map_err(|error| error.to_string())?;

    let rx_chunks = chunks.clone();
    let rx_stop = stop.clone();
    let rx_session_stop = session_stop.clone();
    let rx_failure = failure_tx;
    let rx_name = name.to_string();
    let rx = match thread::Builder::new()
        .name(format!("{rx_name}-serial-rx"))
        .spawn(move || run_reader(reader, &rx_chunks, &rx_stop, &rx_session_stop, &rx_failure))
    {
        Ok(thread) => thread,
        Err(error) => {
            session_stop.store(true, Ordering::Release);
            let _ = tx.join();
            return Err(error.to_string());
        }
    };

    let result = loop {
        if stop.load(Ordering::Acquire) {
            break Ok(());
        }
        if let Ok(reason) = reconnect.try_recv() {
            break Err(reason);
        }
        match failure_rx.recv_timeout(Duration::from_millis(20)) {
            Ok(error) => break Err(error),
            Err(crossbeam_channel::RecvTimeoutError::Timeout) => {}
            Err(crossbeam_channel::RecvTimeoutError::Disconnected) => {
                break Err("serial session workers disconnected".into());
            }
        }
    };
    session_stop.store(true, Ordering::Release);
    if tx.join().is_err() || rx.join().is_err() {
        return Err("serial session worker panicked".into());
    }
    result
}

fn run_writer(
    mut writer: Box<dyn SerialPort>,
    outbound: &Receiver<OutboundBytes>,
    stop: &AtomicBool,
    session_stop: &AtomicBool,
    failure: &Sender<String>,
) {
    while !stop.load(Ordering::Acquire) && !session_stop.load(Ordering::Acquire) {
        let outbound = match outbound.recv_timeout(Duration::from_millis(20)) {
            Ok(outbound) => outbound,
            Err(crossbeam_channel::RecvTimeoutError::Timeout) => continue,
            Err(crossbeam_channel::RecvTimeoutError::Disconnected) => break,
        };
        if outbound.cancelled.load(Ordering::Acquire) || Instant::now() >= outbound.deadline {
            let _ = outbound
                .completion
                .send(Err("serial write deadline expired".into()));
            continue;
        }
        let result = writer
            .write_all(&outbound.bytes)
            .and_then(|()| writer.flush())
            .map_err(|error| error.to_string());
        let failed = result.as_ref().err().cloned();
        let _ = outbound.completion.send(result);
        if let Some(error) = failed {
            let _ = failure.try_send(format!("serial write failed: {error}"));
            break;
        }
    }
}

fn run_reader(
    mut reader: Box<dyn SerialPort>,
    chunks: &Sender<SerialChunk>,
    stop: &AtomicBool,
    session_stop: &AtomicBool,
    failure: &Sender<String>,
) {
    let mut buffer = [0_u8; 1024];
    while !stop.load(Ordering::Acquire) && !session_stop.load(Ordering::Acquire) {
        match reader.read(&mut buffer) {
            Ok(0) => continue,
            Ok(size) => {
                if chunks
                    .try_send(SerialChunk {
                        bytes: buffer[..size].to_vec(),
                        received: Instant::now(),
                    })
                    .is_err()
                {
                    let _ = failure.try_send("serial receive queue is full".into());
                    break;
                }
            }
            Err(error)
                if matches!(
                    error.kind(),
                    std::io::ErrorKind::TimedOut | std::io::ErrorKind::WouldBlock
                ) => {}
            Err(error) => {
                let _ = failure.try_send(format!("serial read failed: {error}"));
                break;
            }
        }
    }
}

fn prepare_reconnect(
    spec: &SerialSpec,
    outbound: &Receiver<OutboundBytes>,
    chunks: &Receiver<SerialChunk>,
    lifecycle: &DeviceLifecycle,
    stop: &AtomicBool,
    failures: &mut u32,
    detail: String,
) {
    reject_outbound(outbound, &detail);
    clear_chunks(chunks);
    if stop.load(Ordering::Acquire) {
        lifecycle.enter_fault(detail);
        return;
    }
    *failures = failures.saturating_add(1);
    let delay = spec.reconnect.delay(*failures);
    lifecycle.begin_reconnect(format!("{detail}; retrying in {} ms", delay.as_millis()));
    warn!(device=%spec.name, %detail, delay_ms=delay.as_millis(), "serial reconnect scheduled");
    if !wait_for_stop(stop, delay) {
        lifecycle.mark_connecting(format!("reopening serial port {}", spec.port));
    }
}

fn reject_outbound(outbound: &Receiver<OutboundBytes>, reason: &str) {
    while let Ok(outbound) = outbound.try_recv() {
        let _ = outbound.completion.send(Err(reason.into()));
    }
}

fn clear_chunks(chunks: &Receiver<SerialChunk>) {
    while chunks.try_recv().is_ok() {}
}
