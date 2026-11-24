use std::collections::HashMap;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc as std_mpsc;
use std::thread;
use std::time::{Duration, Instant};

use anyhow::{Result, anyhow};
use crossbeam_channel::{Receiver, Sender, bounded};
use thiserror::Error;
use tracing::error;

use crate::config::ResolvedTopology;
use crate::domain::SerialDeviceName;
use crate::hardware::lifecycle::{DeviceLifecycle, DeviceState, DeviceStatus};
use crate::telemetry::Telemetry;

mod spec;
mod supervisor;

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
    #[error("serial device {device} transmit cancelled")]
    TransmitCancelled { device: SerialDeviceName },
    #[error("serial device {device} write failed: {detail}")]
    Write {
        device: SerialDeviceName,
        detail: String,
    },
}

pub(super) struct OutboundBytes {
    pub(super) bytes: Vec<u8>,
    pub(super) completion: std_mpsc::SyncSender<Result<(), String>>,
    pub(super) cancelled: Arc<AtomicBool>,
    pub(super) deadline: Instant,
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

    pub(crate) fn send(
        &self,
        bytes: impl Into<Vec<u8>>,
        stop: &AtomicBool,
    ) -> Result<(), SerialError> {
        let state = self.lifecycle.state();
        if !state.is_online() {
            return Err(SerialError::Unavailable {
                device: self.spec.name.clone(),
                state,
            });
        }
        let (completion, result) = std_mpsc::sync_channel(1);
        let cancelled = Arc::new(AtomicBool::new(false));
        let deadline = Instant::now() + self.spec.timeout;
        let mut outbound = OutboundBytes {
            bytes: bytes.into(),
            completion,
            cancelled: cancelled.clone(),
            deadline,
        };
        loop {
            if stop.load(Ordering::Acquire) {
                cancelled.store(true, Ordering::Release);
                return Err(SerialError::TransmitCancelled {
                    device: self.spec.name.clone(),
                });
            }
            match self.outbound.try_send(outbound) {
                Ok(()) => break,
                Err(crossbeam_channel::TrySendError::Full(returned)) => {
                    outbound = returned;
                    let remaining = deadline.saturating_duration_since(Instant::now());
                    if remaining.is_zero() {
                        return Err(SerialError::QueueUnavailable {
                            device: self.spec.name.clone(),
                        });
                    }
                    thread::sleep(Duration::from_millis(5).min(remaining));
                }
                Err(crossbeam_channel::TrySendError::Disconnected(_)) => {
                    return Err(SerialError::QueueUnavailable {
                        device: self.spec.name.clone(),
                    });
                }
            }
        }
        loop {
            if stop.load(Ordering::Acquire) {
                cancelled.store(true, Ordering::Release);
                return Err(SerialError::TransmitCancelled {
                    device: self.spec.name.clone(),
                });
            }
            let remaining = deadline.saturating_duration_since(Instant::now());
            if remaining.is_zero() {
                cancelled.store(true, Ordering::Release);
                return Err(SerialError::TransmitTimeout {
                    device: self.spec.name.clone(),
                });
            }
            match result.recv_timeout(Duration::from_millis(5).min(remaining)) {
                Ok(result) => {
                    return result.map_err(|detail| SerialError::Write {
                        device: self.spec.name.clone(),
                        detail,
                    });
                }
                Err(std_mpsc::RecvTimeoutError::Timeout) => {}
                Err(std_mpsc::RecvTimeoutError::Disconnected) => {
                    return Err(SerialError::QueueUnavailable {
                        device: self.spec.name.clone(),
                    });
                }
            }
        }
    }

    pub(crate) fn recv_timeout(
        &self,
        timeout: std::time::Duration,
    ) -> Result<SerialChunk, crossbeam_channel::RecvTimeoutError> {
        self.chunks.recv_timeout(timeout)
    }

    pub(crate) fn request_reconnect(&self, reason: impl Into<String>) {
        let _ = self.reconnect.try_send(reason.into());
    }
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
            let (outbound_tx, outbound_rx) = bounded(16);
            let (chunk_tx, chunk_rx) = bounded(1024);
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
            let spec = spec.clone();
            let shutdown = manager.shutdown.clone();
            let panic_lifecycle = lifecycle.clone();
            manager.threads.push(
                thread::Builder::new()
                    .name(format!("{}-serial-supervisor", spec.name))
                    .spawn(move || {
                        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                            supervisor::run(
                                spec,
                                outbound_rx,
                                chunk_tx,
                                chunk_rx,
                                reconnect_rx,
                                lifecycle,
                                shutdown,
                            );
                        }));
                        if result.is_err() {
                            panic_lifecycle.enter_fault("serial supervisor panicked");
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
        for handle in self.handles.values() {
            handle.lifecycle.shutdown("serial manager stopped");
        }
    }
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
        assert!(SerialSpec::resolve(&config, false).is_err());
    }
}
