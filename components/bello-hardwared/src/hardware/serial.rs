use std::collections::HashMap;
use std::io::{Read, Write};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc as std_mpsc;
use std::thread;
use std::time::{Duration, Instant};

use anyhow::{Context, Result, anyhow};
use crossbeam_channel::{Receiver, Sender, bounded};
use serialport::SerialPort;
use thiserror::Error;
use tracing::{error, warn};

use super::lifecycle::{DeviceLifecycle, DeviceState, DeviceStatus};
use crate::config::ResolvedTopology;
use crate::domain::SerialDeviceName;
use crate::telemetry::Telemetry;

mod spec;

pub(crate) use spec::SerialSpec;

#[derive(Clone, Debug)]
pub(crate) struct SerialChunk {
    pub(crate) bytes: Vec<u8>,
    pub(crate) received: Instant,
}

#[derive(Debug, Error)]
pub(crate) enum SerialError {
    #[error("serial device {device} is unavailable ({state})")]
    Unavailable {
        device: SerialDeviceName,
        state: DeviceState,
    },
    #[error("serial device {device} transmit queue is unavailable")]
    QueueUnavailable { device: SerialDeviceName },
    #[error("serial device {device} transmit timed out")]
    TransmitTimeout { device: SerialDeviceName },
    #[error("serial device {device} write failed: {detail}")]
    Write {
        device: SerialDeviceName,
        detail: String,
    },
}

#[derive(Clone)]
pub(crate) struct SerialHandle {
    pub(crate) spec: SerialSpec,
    lifecycle: DeviceLifecycle,
    outbound: Sender<OutboundBytes>,
    chunks: Receiver<SerialChunk>,
    reconnect: Sender<String>,
}

impl SerialHandle {
    pub(crate) fn status(&self) -> DeviceStatus {
        self.lifecycle.status()
    }

    pub(crate) fn send(&self, bytes: impl Into<Vec<u8>>) -> Result<(), SerialError> {
        let state = self.lifecycle.state();
        if !state.is_online() {
            return Err(SerialError::Unavailable {
                device: self.spec.name.clone(),
                state,
            });
        }
        let (completion, result) = std_mpsc::sync_channel(1);
        self.outbound
            .send_timeout(
                OutboundBytes {
                    bytes: bytes.into(),
                    completion,
                },
                Duration::from_millis(100),
            )
            .map_err(|_| SerialError::QueueUnavailable {
                device: self.spec.name.clone(),
            })?;
        result
            .recv_timeout(Duration::from_millis(100))
            .map_err(|_| SerialError::TransmitTimeout {
                device: self.spec.name.clone(),
            })?
            .map_err(|detail| SerialError::Write {
                device: self.spec.name.clone(),
                detail,
            })
    }

    pub(crate) fn recv_timeout(
        &self,
        timeout: Duration,
    ) -> Result<SerialChunk, crossbeam_channel::RecvTimeoutError> {
        self.chunks.recv_timeout(timeout)
    }

    pub(crate) fn request_reconnect(&self, reason: impl Into<String>) {
        let _ = self.reconnect.try_send(reason.into());
    }
}

struct OutboundBytes {
    bytes: Vec<u8>,
    completion: std_mpsc::SyncSender<Result<(), String>>,
}

pub(crate) struct SerialManager {
    handles: HashMap<SerialDeviceName, SerialHandle>,
    shutdown: Arc<AtomicBool>,
    threads: Vec<thread::JoinHandle<()>>,
}

impl SerialManager {
    pub(crate) fn start(topology: &ResolvedTopology, telemetry: Telemetry) -> Result<Self> {
        let shutdown = Arc::new(AtomicBool::new(false));
        let mut manager = Self {
            handles: HashMap::new(),
            shutdown,
            threads: Vec::new(),
        };
        for (_, spec) in topology.serial_devices() {
            let (outbound_tx, outbound_rx) = bounded(256);
            let (chunk_tx, chunk_rx) = bounded(4096);
            let (reconnect_tx, reconnect_rx) = bounded(1);
            let lifecycle = DeviceLifecycle::new(
                format!("serial/{}", spec.name),
                DeviceState::Starting,
                format!("starting serial transport on {}", spec.port),
                telemetry.clone(),
            );
            manager.handles.insert(
                spec.name.clone(),
                SerialHandle {
                    spec: spec.clone(),
                    lifecycle: lifecycle.clone(),
                    outbound: outbound_tx,
                    chunks: chunk_rx.clone(),
                    reconnect: reconnect_tx,
                },
            );
            let worker_spec = spec.clone();
            let worker_stop = manager.shutdown.clone();
            let worker_name = spec.name.to_string();
            let worker_chunk_queue = chunk_rx.clone();
            let worker_lifecycle = lifecycle.clone();
            manager.threads.push(
                thread::Builder::new()
                    .name(format!("{worker_name}-serial-supervisor"))
                    .spawn(move || {
                        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                            run_supervisor(
                                worker_spec,
                                outbound_rx,
                                chunk_tx,
                                worker_chunk_queue,
                                reconnect_rx,
                                lifecycle,
                                worker_stop,
                            );
                        }));
                        if result.is_err() {
                            worker_lifecycle.fail("serial supervisor panicked");
                        }
                    })?,
            );
        }
        Ok(manager)
    }

    pub(crate) fn handle(&self, name: &SerialDeviceName) -> Result<SerialHandle> {
        self.handles
            .get(name)
            .cloned()
            .ok_or_else(|| anyhow!("unknown serial device {name}"))
    }

    pub(crate) fn status(&self, name: &SerialDeviceName) -> Option<DeviceStatus> {
        self.handles.get(name).map(SerialHandle::status)
    }
}

impl Drop for SerialManager {
    fn drop(&mut self) {
        self.shutdown.store(true, Ordering::Release);
        while let Some(thread) = self.threads.pop() {
            if thread.join().is_err() {
                error!("serial transport worker panicked during shutdown");
            }
        }
    }
}

fn run_supervisor(
    spec: SerialSpec,
    outbound: Receiver<OutboundBytes>,
    chunks: Sender<SerialChunk>,
    chunk_queue: Receiver<SerialChunk>,
    reconnect: Receiver<String>,
    lifecycle: DeviceLifecycle,
    stop: Arc<AtomicBool>,
) {
    let mut failures = 0_u32;
    let _ = lifecycle.transition(
        DeviceState::Connecting,
        format!("opening serial port {}", spec.port),
    );
    while !stop.load(Ordering::Acquire) {
        match open_port(&spec) {
            Ok(port) => {
                failures = 0;
                let _ = lifecycle.transition(
                    DeviceState::Online,
                    format!("serial port {} online", spec.port),
                );
                if let Err(detail) =
                    run_session(port, &outbound, &chunks, &reconnect, &stop, &spec.name)
                {
                    if stop.load(Ordering::Acquire) {
                        break;
                    }
                    let _ = lifecycle.transition(DeviceState::Degraded, detail.clone());
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
    let _ = lifecycle.transition(DeviceState::Shutdown, "serial supervisor stopped");
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
    let tx_thread = thread::Builder::new()
        .name(format!("{tx_name}-serial-tx"))
        .spawn(move || {
            run_writer(
                writer,
                &tx_outbound,
                &tx_stop,
                &tx_session_stop,
                &tx_failure,
            );
        })
        .map_err(|error| error.to_string())?;

    let rx_chunks = chunks.clone();
    let rx_stop = stop.clone();
    let rx_session_stop = session_stop.clone();
    let rx_failure = failure_tx;
    let rx_name = name.to_string();
    let rx_thread = match thread::Builder::new()
        .name(format!("{rx_name}-serial-rx"))
        .spawn(move || {
            run_reader(reader, &rx_chunks, &rx_stop, &rx_session_stop, &rx_failure);
        }) {
        Ok(thread) => thread,
        Err(error) => {
            session_stop.store(true, Ordering::Release);
            let _ = tx_thread.join();
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
    if tx_thread.join().is_err() || rx_thread.join().is_err() {
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
    if lifecycle.state() != DeviceState::Fault {
        let _ = lifecycle.transition(DeviceState::Fault, detail.clone());
    }
    if stop.load(Ordering::Acquire) {
        return;
    }
    *failures = failures.saturating_add(1);
    let delay = spec.reconnect.delay(*failures);
    let _ = lifecycle.transition(
        DeviceState::Reconnecting,
        format!("{detail}; retrying in {} ms", delay.as_millis()),
    );
    warn!(device=%spec.name, %detail, delay_ms=delay.as_millis(), "serial reconnect scheduled");
    if !wait_for_stop(stop, delay) {
        let _ = lifecycle.transition(
            DeviceState::Connecting,
            format!("reopening serial port {}", spec.port),
        );
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

pub(crate) fn wait_for_stop(stop: &AtomicBool, duration: Duration) -> bool {
    let deadline = Instant::now() + duration;
    while !stop.load(Ordering::Acquire) && Instant::now() < deadline {
        thread::sleep(
            Duration::from_millis(10).min(deadline.saturating_duration_since(Instant::now())),
        );
    }
    stop.load(Ordering::Acquire)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::SerialDeviceConfig;

    #[test]
    fn rejects_invalid_serial_configuration() {
        let config: SerialDeviceConfig = serde_yaml::from_str(
            "name: imu_serial\nport: /dev/ttyUSB0\nbaud_rate: 0\ntimeout_ms: 100",
        )
        .unwrap();
        assert!(SerialSpec::try_from(&config).is_err());
    }
}
