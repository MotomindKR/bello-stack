use std::collections::HashMap;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc as std_mpsc;
use std::thread;
use std::time::{Duration, Instant};

use anyhow::{Context, Result, anyhow};
use crossbeam_channel::{Receiver, Sender, bounded, select_biased};
use socketcan::{CanFrame as WireFrame, CanSocket, EmbeddedFrame, ExtendedId, Frame, Socket};
use thiserror::Error;
use tracing::{error, warn};

use crate::config::ResolvedTopology;
use crate::domain::BusName;
use crate::hardware::lifecycle::{DeviceLifecycle, DeviceState, DeviceStatus};
use crate::hardware::serial::wait_for_stop;
use crate::telemetry::Telemetry;

mod spec;

pub(crate) use spec::BusSpec;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CanFrame {
    pub id: u32,
    pub data: [u8; 8],
}

#[derive(Clone, Debug)]
pub(crate) struct ReceivedFrame {
    pub(crate) bus: BusName,
    pub(crate) frame: CanFrame,
    pub(crate) received: Instant,
}

#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub(crate) enum BusError {
    #[error("bus {bus} is unavailable ({state})")]
    Unavailable { bus: BusName, state: DeviceState },
    #[error("bus {bus} control queue is full")]
    ControlQueueFull { bus: BusName },
    #[error("bus {bus} safety queue did not drain within its deadline")]
    SafetyQueueTimeout { bus: BusName },
    #[error("bus {bus} did not confirm safety frame transmission")]
    SafetyDeliveryTimeout { bus: BusName },
    #[error("bus {bus} failed to transmit a safety frame: {detail}")]
    SafetyTransmit { bus: BusName, detail: String },
    #[error("bus {bus} transmit worker is disconnected")]
    Disconnected { bus: BusName },
}

#[derive(Clone)]
pub(crate) struct BusHandle {
    pub(crate) spec: BusSpec,
    lifecycle: DeviceLifecycle,
    control_tx: Sender<OutboundFrame>,
    safety_tx: Sender<OutboundFrame>,
}

struct OutboundFrame {
    frame: CanFrame,
    completion: Option<std_mpsc::SyncSender<Result<(), String>>>,
}
impl BusHandle {
    fn ensure_online(&self) -> Result<(), BusError> {
        let state = self.lifecycle.state();
        if state.is_online() {
            Ok(())
        } else {
            Err(BusError::Unavailable {
                bus: self.spec.name.clone(),
                state,
            })
        }
    }

    pub(crate) fn status(&self) -> DeviceStatus {
        self.lifecycle.status()
    }

    /// Enqueue an ordinary control frame without blocking the control loop.
    pub(crate) fn try_enqueue(&self, frame: CanFrame) -> Result<(), BusError> {
        self.ensure_online()?;
        self.control_tx
            .try_send(OutboundFrame {
                frame,
                completion: None,
            })
            .map_err(|error| match error {
                crossbeam_channel::TrySendError::Full(_) => BusError::ControlQueueFull {
                    bus: self.spec.name.clone(),
                },
                crossbeam_channel::TrySendError::Disconnected(_) => BusError::Disconnected {
                    bus: self.spec.name.clone(),
                },
            })
    }

    /// Enqueue a lifecycle or safety frame reliably, applying backpressure if needed.
    pub(crate) fn enqueue_safety(&self, frame: CanFrame) -> Result<(), BusError> {
        self.ensure_online()?;
        let (completion, result) = std_mpsc::sync_channel(1);
        self.safety_tx
            .send_timeout(
                OutboundFrame {
                    frame,
                    completion: Some(completion),
                },
                Duration::from_millis(20),
            )
            .map_err(|error| match error {
                crossbeam_channel::SendTimeoutError::Timeout(_) => BusError::SafetyQueueTimeout {
                    bus: self.spec.name.clone(),
                },
                crossbeam_channel::SendTimeoutError::Disconnected(_) => BusError::Disconnected {
                    bus: self.spec.name.clone(),
                },
            })?;
        result
            .recv_timeout(Duration::from_millis(20))
            .map_err(|_| BusError::SafetyDeliveryTimeout {
                bus: self.spec.name.clone(),
            })?
            .map_err(|detail| BusError::SafetyTransmit {
                bus: self.spec.name.clone(),
                detail,
            })
    }
}

pub(crate) struct BusManager {
    handles: HashMap<BusName, BusHandle>,
    _frame_tx: Sender<ReceivedFrame>,
    frames: Receiver<ReceivedFrame>,
    shutdown: Arc<AtomicBool>,
    threads: Vec<thread::JoinHandle<()>>,
}

impl BusManager {
    pub(crate) fn start(topology: &ResolvedTopology, telemetry: Telemetry) -> Result<Self> {
        let shutdown = Arc::new(AtomicBool::new(false));
        let (frame_tx, frame_rx) = bounded(8192);
        // Construct the manager before starting any threads. If a later bus fails,
        // Drop signals and joins every worker that was already started.
        let mut manager = Self {
            handles: HashMap::new(),
            _frame_tx: frame_tx.clone(),
            frames: frame_rx,
            shutdown,
            threads: Vec::new(),
        };
        for (_, bus) in topology.buses() {
            let (control_tx, control_rx) = bounded::<OutboundFrame>(4096);
            let (safety_tx, safety_rx) = bounded::<OutboundFrame>(128);
            let lifecycle = DeviceLifecycle::new(
                format!("can/{}", bus.name),
                DeviceState::Starting,
                format!("starting CAN supervisor for {}", bus.interface),
                telemetry.clone(),
            );
            manager.handles.insert(
                bus.name.clone(),
                BusHandle {
                    spec: bus.clone(),
                    lifecycle: lifecycle.clone(),
                    control_tx,
                    safety_tx,
                },
            );
            let bus_name = bus.name.clone();
            let stop = manager.shutdown.clone();
            let bus_spec = bus.clone();
            let bus_frames = frame_tx.clone();
            let worker_lifecycle = lifecycle.clone();
            manager.threads.push(
                thread::Builder::new()
                    .name(format!("{bus_name}-supervisor"))
                    .spawn(move || {
                        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                            run_can_supervisor(
                                bus_spec, safety_rx, control_rx, bus_frames, lifecycle, stop,
                            );
                        }));
                        if result.is_err() {
                            worker_lifecycle.fail("CAN supervisor panicked");
                        }
                    })?,
            );
        }
        Ok(manager)
    }
    pub(crate) fn handle(&self, name: &BusName) -> Result<BusHandle> {
        self.handles
            .get(name)
            .cloned()
            .ok_or_else(|| anyhow!("unknown bus {name}"))
    }
    pub(crate) fn frames(&self) -> Receiver<ReceivedFrame> {
        self.frames.clone()
    }
    pub(crate) fn status(&self, name: &BusName) -> Option<DeviceStatus> {
        self.handles.get(name).map(BusHandle::status)
    }
}

fn run_can_supervisor(
    spec: BusSpec,
    safety_rx: Receiver<OutboundFrame>,
    control_rx: Receiver<OutboundFrame>,
    frames: Sender<ReceivedFrame>,
    lifecycle: DeviceLifecycle,
    shutdown: Arc<AtomicBool>,
) {
    let mut failures = 0_u32;
    let _ = lifecycle.transition(
        DeviceState::Connecting,
        format!("opening SocketCAN interface {}", spec.interface),
    );
    while !shutdown.load(Ordering::Acquire) {
        match open_sockets(&spec) {
            Ok((writer, reader)) => {
                failures = 0;
                let _ = lifecycle.transition(
                    DeviceState::Online,
                    format!("SocketCAN interface {} online", spec.interface),
                );
                if let Err(detail) = run_bus_session(
                    writer,
                    reader,
                    &spec.name,
                    &safety_rx,
                    &control_rx,
                    &frames,
                    &shutdown,
                ) {
                    if shutdown.load(Ordering::Acquire) {
                        break;
                    }
                    let _ = lifecycle.transition(DeviceState::Degraded, detail.clone());
                    fault_and_prepare_reconnect(
                        &spec,
                        &lifecycle,
                        &safety_rx,
                        &control_rx,
                        detail,
                        &shutdown,
                        &mut failures,
                    );
                }
            }
            Err(error) => fault_and_prepare_reconnect(
                &spec,
                &lifecycle,
                &safety_rx,
                &control_rx,
                error.to_string(),
                &shutdown,
                &mut failures,
            ),
        }
    }
    reject_queued(&safety_rx, &control_rx, "CAN supervisor stopped");
    let _ = lifecycle.transition(DeviceState::Shutdown, "CAN supervisor stopped");
}

fn open_sockets(spec: &BusSpec) -> Result<(CanSocket, CanSocket)> {
    let writer = CanSocket::open(&spec.interface).with_context(|| {
        format!(
            "opening SocketCAN writer {} ({})",
            spec.name, spec.interface
        )
    })?;
    let reader = CanSocket::open(&spec.interface).with_context(|| {
        format!(
            "opening SocketCAN reader {} ({})",
            spec.name, spec.interface
        )
    })?;
    reader.set_read_timeout(Duration::from_millis(100))?;
    Ok((writer, reader))
}

fn run_bus_session(
    writer: CanSocket,
    reader: CanSocket,
    bus_name: &BusName,
    safety_rx: &Receiver<OutboundFrame>,
    control_rx: &Receiver<OutboundFrame>,
    frames: &Sender<ReceivedFrame>,
    shutdown: &Arc<AtomicBool>,
) -> Result<(), String> {
    let session_stop = Arc::new(AtomicBool::new(false));
    let (failure_tx, failure_rx) = bounded::<String>(2);

    let tx_safety = safety_rx.clone();
    let tx_control = control_rx.clone();
    let tx_shutdown = shutdown.clone();
    let tx_session_stop = session_stop.clone();
    let tx_failure = failure_tx.clone();
    let tx_bus = bus_name.to_string();
    let tx_thread = thread::Builder::new()
        .name(format!("{tx_bus}-tx"))
        .spawn(move || {
            run_tx_session(
                writer,
                &tx_safety,
                &tx_control,
                &tx_shutdown,
                &tx_session_stop,
                &tx_failure,
                &tx_bus,
            );
        })
        .map_err(|error| error.to_string())?;

    let rx_shutdown = shutdown.clone();
    let rx_session_stop = session_stop.clone();
    let rx_failure = failure_tx;
    let rx_bus = bus_name.clone();
    let rx_frames = frames.clone();
    let rx_thread = match thread::Builder::new()
        .name(format!("{rx_bus}-rx"))
        .spawn(move || {
            run_rx_session(
                reader,
                &rx_bus,
                &rx_frames,
                &rx_shutdown,
                &rx_session_stop,
                &rx_failure,
            );
        }) {
        Ok(thread) => thread,
        Err(error) => {
            session_stop.store(true, Ordering::Release);
            let _ = tx_thread.join();
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
    if tx_thread.join().is_err() || rx_thread.join().is_err() {
        return Err("CAN session worker panicked".into());
    }
    result
}

fn run_tx_session(
    writer: CanSocket,
    safety_rx: &Receiver<OutboundFrame>,
    control_rx: &Receiver<OutboundFrame>,
    shutdown: &AtomicBool,
    session_stop: &AtomicBool,
    failure: &Sender<String>,
    bus: &str,
) {
    while let Some(outbound) =
        receive_prioritized(safety_rx, control_rx, shutdown, Some(session_stop))
    {
        let frame = outbound.frame;
        let Some(id) = ExtendedId::new(frame.id) else {
            complete(outbound, Err("invalid extended CAN id".into()), bus);
            continue;
        };
        let Some(frame) = WireFrame::new(id, &frame.data) else {
            complete(outbound, Err("could not construct CAN frame".into()), bus);
            continue;
        };
        match writer.write_frame(&frame) {
            Ok(()) => complete(outbound, Ok(()), bus),
            Err(error) => {
                let detail = format!("CAN write failed: {error}");
                complete(outbound, Err(detail.clone()), bus);
                let _ = failure.try_send(detail);
                break;
            }
        }
    }
}

fn run_rx_session(
    reader: CanSocket,
    bus_name: &BusName,
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
        let frame = ReceivedFrame {
            bus: bus_name.clone(),
            frame: CanFrame {
                id: wire.raw_id(),
                data: wire.data().try_into().unwrap_or([0; 8]),
            },
            received: Instant::now(),
        };
        if frames.try_send(frame).is_err() {
            let _ = failure.try_send("CAN receive queue is full".into());
            break;
        }
    }
}

fn fault_and_prepare_reconnect(
    spec: &BusSpec,
    lifecycle: &DeviceLifecycle,
    safety_rx: &Receiver<OutboundFrame>,
    control_rx: &Receiver<OutboundFrame>,
    detail: String,
    shutdown: &AtomicBool,
    failures: &mut u32,
) {
    reject_queued(safety_rx, control_rx, &detail);
    if lifecycle.state() != DeviceState::Fault {
        let _ = lifecycle.transition(DeviceState::Fault, detail.clone());
    }
    if shutdown.load(Ordering::Acquire) {
        return;
    }
    *failures = failures.saturating_add(1);
    let delay = spec.reconnect.delay(*failures);
    let _ = lifecycle.transition(
        DeviceState::Reconnecting,
        format!("{detail}; retrying in {} ms", delay.as_millis()),
    );
    warn!(bus=%spec.name, %detail, delay_ms=delay.as_millis(), "CAN bus reconnect scheduled");
    if !wait_for_stop(shutdown, delay) {
        let _ = lifecycle.transition(
            DeviceState::Connecting,
            format!("reopening SocketCAN interface {}", spec.interface),
        );
    }
}

fn reject_queued(
    safety_rx: &Receiver<OutboundFrame>,
    control_rx: &Receiver<OutboundFrame>,
    reason: &str,
) {
    while let Ok(outbound) = safety_rx.try_recv() {
        if let Some(completion) = outbound.completion {
            let _ = completion.send(Err(reason.into()));
        }
    }
    while control_rx.try_recv().is_ok() {}
}

fn receive_prioritized(
    safety: &Receiver<OutboundFrame>,
    control: &Receiver<OutboundFrame>,
    shutdown: &AtomicBool,
    session_stop: Option<&AtomicBool>,
) -> Option<OutboundFrame> {
    loop {
        if shutdown.load(Ordering::Acquire)
            || session_stop.is_some_and(|stop| stop.load(Ordering::Acquire))
        {
            return None;
        }
        select_biased! {
            recv(safety) -> frame => match frame {
                Ok(frame) => return Some(frame),
                Err(_) => return None,
            },
            recv(control) -> frame => match frame {
                Ok(frame) => return Some(frame),
                Err(_) => return None,
            },
            default(Duration::from_millis(20)) => {}
        }
    }
}

fn complete(outbound: OutboundFrame, result: Result<(), String>, bus: &str) {
    if let Err(error) = &result {
        error!(bus, %error, "CAN write failed");
    }
    if let Some(completion) = outbound.completion {
        let _ = completion.send(result);
    }
}

impl Drop for BusManager {
    fn drop(&mut self) {
        self.shutdown.store(true, Ordering::Relaxed);
        while let Some(worker) = self.threads.pop() {
            if worker.join().is_err() {
                error!("CAN worker panicked during shutdown");
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn safety_frames_are_received_before_queued_control_frames() {
        let (control_tx, control_rx) = bounded(4);
        let (safety_tx, safety_rx) = bounded(4);
        let shutdown = AtomicBool::new(false);
        let control = CanFrame {
            id: 1,
            data: [1; 8],
        };
        let safety = CanFrame {
            id: 2,
            data: [2; 8],
        };
        control_tx
            .send(OutboundFrame {
                frame: control,
                completion: None,
            })
            .unwrap();
        safety_tx
            .send(OutboundFrame {
                frame: safety,
                completion: None,
            })
            .unwrap();

        assert_eq!(
            receive_prioritized(&safety_rx, &control_rx, &shutdown, None)
                .map(|outbound| outbound.frame),
            Some(safety),
        );
        assert_eq!(
            receive_prioritized(&safety_rx, &control_rx, &shutdown, None)
                .map(|outbound| outbound.frame),
            Some(control),
        );
    }

    #[test]
    fn reconnect_discards_control_and_rejects_safety_frames() {
        let (control_tx, control_rx) = bounded(4);
        let (safety_tx, safety_rx) = bounded(4);
        let (completion_tx, completion_rx) = std_mpsc::sync_channel(1);
        control_tx
            .send(OutboundFrame {
                frame: CanFrame {
                    id: 1,
                    data: [0; 8],
                },
                completion: None,
            })
            .unwrap();
        safety_tx
            .send(OutboundFrame {
                frame: CanFrame {
                    id: 2,
                    data: [0; 8],
                },
                completion: Some(completion_tx),
            })
            .unwrap();

        reject_queued(&safety_rx, &control_rx, "reconnecting");

        assert!(control_rx.is_empty());
        assert!(safety_rx.is_empty());
        assert_eq!(completion_rx.recv().unwrap(), Err("reconnecting".into()));
    }
}
