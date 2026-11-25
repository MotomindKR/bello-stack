use std::collections::{BTreeMap, HashMap};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::mpsc as std_mpsc;
use std::thread;
use std::time::{Duration, Instant};

use anyhow::{Result, anyhow};
use crossbeam_channel::{Receiver, Sender, bounded};
use parking_lot::Mutex;
use thiserror::Error;
use tracing::error;

use crate::config::ResolvedTopology;
use crate::domain::{BusName, GroupName};
use crate::hardware::lifecycle::{DeviceLifecycle, DeviceState, DeviceStatus};
use crate::telemetry::Telemetry;

mod maintenance;
mod periodic;
mod spec;
mod supervisor;

use maintenance::{BusAccess, BusGeneration, MaintenanceRequest};
pub(crate) use maintenance::{FeedbackWindow, MaintenanceBus};
pub(crate) use periodic::PeriodicControl;
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
    #[error("bus {bus} ordinary traffic interrupted by maintenance")]
    Maintenance { bus: BusName },
    #[error("bus {bus} control batch expired before transmission")]
    ControlBatchExpired { bus: BusName },
    #[error("group {group} started a new control batch before completing the previous batch")]
    ControlBatchInterrupted { group: GroupName },
    #[error("bus {bus} {kind} batch expired before transmission")]
    PriorityBatchExpired {
        bus: BusName,
        kind: PriorityBatchKind,
    },
    #[error("bus {bus} {kind} queue is full")]
    PriorityQueueFull {
        bus: BusName,
        kind: PriorityBatchKind,
    },
    #[error("bus {bus} failed to transmit {kind} batch: {detail}")]
    PriorityBatchTransmit {
        bus: BusName,
        kind: PriorityBatchKind,
        detail: String,
    },
    #[error("bus {bus} transmit worker is disconnected")]
    Disconnected { bus: BusName },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum PriorityBatchKind {
    /// Ordered ahead of control traffic without invalidating it.
    Configuration,
    /// Ordered ahead of control traffic and discards every queued control batch.
    SafetyBarrier,
}

impl std::fmt::Display for PriorityBatchKind {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter.write_str(match self {
            Self::Configuration => "configuration",
            Self::SafetyBarrier => "safety barrier",
        })
    }
}

pub(super) struct ControlBatch {
    generation: BusGeneration,
    pub(super) frames: Vec<CanFrame>,
    pub(super) expires_at: Instant,
}

#[derive(Debug, PartialEq, Eq)]
pub(super) enum BatchFailure {
    Maintenance,
    Expired,
    Transmit(String),
}

pub(super) struct PriorityBatch {
    generation: BusGeneration,
    pub(super) kind: PriorityBatchKind,
    pub(super) frames: Vec<CanFrame>,
    pub(super) expires_at: Instant,
    pub(super) completion: std_mpsc::SyncSender<Result<(), BatchFailure>>,
}

#[derive(Clone)]
pub(super) struct BusTxQueues {
    access: Arc<BusAccess>,
    maintenance: Receiver<MaintenanceRequest>,
    pub(super) safety: Receiver<PriorityBatch>,
    pub(super) configuration: Receiver<PriorityBatch>,
    pub(super) control: Receiver<ControlBatch>,
    pub(super) periodic: Arc<Mutex<BTreeMap<GroupName, PeriodicControl>>>,
    pub(super) wake: Receiver<()>,
}

#[must_use = "a submitted CAN batch must be awaited with PendingBatch::wait"]
pub(crate) struct PendingBatch {
    bus: BusName,
    kind: PriorityBatchKind,
    completion: Option<std_mpsc::Receiver<Result<(), BatchFailure>>>,
}

impl PendingBatch {
    pub(crate) fn wait(self) -> Result<(), BusError> {
        let Some(completion) = self.completion else {
            return Ok(());
        };
        match completion.recv() {
            Ok(Ok(())) => Ok(()),
            Ok(Err(BatchFailure::Maintenance)) => Err(BusError::Maintenance { bus: self.bus }),
            Ok(Err(BatchFailure::Expired)) => Err(BusError::PriorityBatchExpired {
                bus: self.bus,
                kind: self.kind,
            }),
            Ok(Err(BatchFailure::Transmit(detail))) => Err(BusError::PriorityBatchTransmit {
                bus: self.bus,
                kind: self.kind,
                detail,
            }),
            Err(_) => Err(BusError::Disconnected { bus: self.bus }),
        }
    }
}

#[derive(Clone)]
pub(crate) struct BusHandle {
    pub(crate) spec: BusSpec,
    lifecycle: DeviceLifecycle,
    access: Arc<BusAccess>,
    maintenance_tx: Sender<MaintenanceRequest>,
    control_tx: Sender<ControlBatch>,
    configuration_tx: Sender<PriorityBatch>,
    safety_tx: Sender<PriorityBatch>,
    periodic: Arc<Mutex<BTreeMap<GroupName, PeriodicControl>>>,
    wake: Sender<()>,
}

impl BusHandle {
    pub(crate) fn reserve_maintenance(&self) -> anyhow::Result<MaintenanceBus> {
        self.access
            .reserve(self.spec.name.clone(), self.maintenance_tx.clone())
    }

    pub(crate) fn feedback_window(&self) -> Option<FeedbackWindow> {
        self.access.feedback_window()
    }

    pub(crate) fn maintenance_error(&self) -> BusError {
        BusError::Maintenance {
            bus: self.spec.name.clone(),
        }
    }

    pub(crate) fn periodic_control(
        &self,
        group: &GroupName,
        period: Duration,
        lease: Arc<AtomicU64>,
        shutdown: Arc<AtomicBool>,
    ) -> PeriodicControl {
        self.periodic
            .lock()
            .entry(group.clone())
            .or_insert_with(|| {
                PeriodicControl::new(
                    group.clone(),
                    self.spec.name.clone(),
                    period,
                    lease,
                    shutdown,
                    self.wake.clone(),
                    self.access.clone(),
                )
            })
            .clone()
    }

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

    pub(crate) fn enqueue_control_batch(
        &self,
        frames: Vec<CanFrame>,
        started: Instant,
        expires_at: Instant,
    ) -> Result<(), BusError> {
        let (generation, _) = self
            .access
            .ordinary_traffic()
            .filter(|(_, window)| window.accepts(started))
            .ok_or_else(|| self.maintenance_error())?;
        self.ensure_online()?;
        if frames.is_empty() {
            return Ok(());
        }
        let Some(enqueue_by) = expires_at.checked_sub(self.spec.control_batch_budget(frames.len()))
        else {
            return Err(BusError::ControlBatchExpired {
                bus: self.spec.name.clone(),
            });
        };
        enqueue_control(
            &self.control_tx,
            ControlBatch {
                generation,
                frames,
                expires_at,
            },
            enqueue_by,
        )
        .map_err(|error| match error {
            crossbeam_channel::SendTimeoutError::Timeout(_) => BusError::ControlBatchExpired {
                bus: self.spec.name.clone(),
            },
            crossbeam_channel::SendTimeoutError::Disconnected(_) => BusError::Disconnected {
                bus: self.spec.name.clone(),
            },
        })
    }

    pub(crate) fn enqueue_configuration_batch(
        &self,
        window: FeedbackWindow,
        frames: Vec<CanFrame>,
    ) -> Result<(), BusError> {
        self.submit_configuration_batch(window, frames)?.wait()
    }

    pub(crate) fn submit_configuration_batch(
        &self,
        window: FeedbackWindow,
        frames: Vec<CanFrame>,
    ) -> Result<PendingBatch, BusError> {
        self.submit_priority_batch(PriorityBatchKind::Configuration, Some(window), frames)
    }

    pub(crate) fn submit_safety_batch(
        &self,
        frames: Vec<CanFrame>,
    ) -> Result<PendingBatch, BusError> {
        self.submit_priority_batch(PriorityBatchKind::SafetyBarrier, None, frames)
    }

    fn submit_priority_batch(
        &self,
        kind: PriorityBatchKind,
        window: Option<FeedbackWindow>,
        frames: Vec<CanFrame>,
    ) -> Result<PendingBatch, BusError> {
        let (generation, _) = self
            .access
            .ordinary_traffic()
            .filter(|(_, current)| window.is_none_or(|expected| expected == *current))
            .ok_or_else(|| self.maintenance_error())?;
        self.ensure_online()?;
        if frames.is_empty() {
            return Ok(PendingBatch {
                bus: self.spec.name.clone(),
                kind,
                completion: None,
            });
        }
        let lifetime = self.spec.priority_batch_lifetime(frames.len());
        let expires_at = Instant::now() + lifetime;
        let (completion, result) = std_mpsc::sync_channel(1);
        let queue = match kind {
            PriorityBatchKind::Configuration => &self.configuration_tx,
            PriorityBatchKind::SafetyBarrier => &self.safety_tx,
        };
        queue
            .try_send(PriorityBatch {
                generation,
                kind,
                frames,
                expires_at,
                completion,
            })
            .map_err(|error| match error {
                crossbeam_channel::TrySendError::Full(_) => BusError::PriorityQueueFull {
                    bus: self.spec.name.clone(),
                    kind,
                },
                crossbeam_channel::TrySendError::Disconnected(_) => BusError::Disconnected {
                    bus: self.spec.name.clone(),
                },
            })?;
        Ok(PendingBatch {
            bus: self.spec.name.clone(),
            kind,
            completion: Some(result),
        })
    }
}

fn enqueue_control(
    queue: &Sender<ControlBatch>,
    batch: ControlBatch,
    enqueue_by: Instant,
) -> Result<(), crossbeam_channel::SendTimeoutError<ControlBatch>> {
    if Instant::now() >= enqueue_by {
        return Err(crossbeam_channel::SendTimeoutError::Timeout(batch));
    }
    // Give the TX worker a chance to drain after scheduler jitter, but reserve
    // the wire budget inside the original deadline. Never grow the queue.
    queue.send_deadline(batch, enqueue_by)
}

fn control_queue_capacity(topology: &ResolvedTopology, bus: &BusName) -> usize {
    // Each group contributes at most one atomic batch to a bus per control tick.
    topology
        .groups()
        .filter(|(_, group)| {
            group.motors.iter().any(|motor| {
                topology
                    .motor(motor)
                    .is_ok_and(|motor| &motor.spec.address.bus == bus)
            })
        })
        .count()
        .max(1)
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
        let mut manager = Self {
            handles: HashMap::new(),
            _frame_tx: frame_tx.clone(),
            frames: frame_rx,
            shutdown,
            threads: Vec::new(),
        };
        for (_, bus) in topology.buses() {
            let control_capacity = control_queue_capacity(topology, &bus.name);
            let (control_tx, control_rx) = bounded(control_capacity);
            let priority_capacity = control_capacity.max(4);
            let (configuration_tx, configuration_rx) = bounded(priority_capacity);
            let (safety_tx, safety_rx) = bounded(priority_capacity);
            let (wake_tx, wake_rx) = bounded(1);
            let periodic = Arc::new(Mutex::new(BTreeMap::new()));
            let access = Arc::new(BusAccess::default());
            let (maintenance_tx, maintenance_rx) = bounded(1);
            let lifecycle = DeviceLifecycle::new(
                format!("can/{}", bus.name),
                DeviceState::Starting,
                format!("starting CAN supervisor for {}", bus.transport.label()),
                telemetry.clone(),
            );
            manager.handles.insert(
                bus.name.clone(),
                BusHandle {
                    spec: bus.clone(),
                    lifecycle: lifecycle.clone(),
                    access: access.clone(),
                    maintenance_tx,
                    control_tx,
                    configuration_tx,
                    safety_tx,
                    periodic: periodic.clone(),
                    wake: wake_tx,
                },
            );
            let spec = bus.clone();
            let frames = frame_tx.clone();
            let shutdown = manager.shutdown.clone();
            let panic_lifecycle = lifecycle.clone();
            let worker_access = access.clone();
            manager.threads.push(
                thread::Builder::new()
                    .name(format!("{}-supervisor", bus.name))
                    .spawn(move || {
                        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                            supervisor::run(
                                spec,
                                BusTxQueues {
                                    access: worker_access,
                                    maintenance: maintenance_rx,
                                    safety: safety_rx,
                                    configuration: configuration_rx,
                                    control: control_rx,
                                    periodic,
                                    wake: wake_rx,
                                },
                                frames,
                                lifecycle,
                                shutdown,
                            );
                        }));
                        if result.is_err() {
                            access.disconnected("CAN supervisor panicked");
                            panic_lifecycle.enter_fault("CAN supervisor panicked");
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

impl Drop for BusManager {
    fn drop(&mut self) {
        self.shutdown.store(true, Ordering::Release);
        while let Some(worker) = self.threads.pop() {
            if worker.join().is_err() {
                error!("CAN worker panicked during shutdown");
            }
        }
        for handle in self.handles.values() {
            handle.lifecycle.shutdown("CAN manager stopped");
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn batch(id: u32) -> ControlBatch {
        ControlBatch {
            generation: BusGeneration::INITIAL,
            frames: vec![CanFrame { id, data: [0; 8] }],
            expires_at: Instant::now() + std::time::Duration::from_secs(1),
        }
    }

    #[test]
    fn control_enqueue_waits_for_transient_backpressure_without_growing_queue() {
        let (tx, rx) = bounded(1);
        tx.send(batch(1)).unwrap();
        let worker = thread::spawn(move || {
            assert_eq!(rx.recv().unwrap().frames[0].id, 1);
            assert_eq!(rx.recv().unwrap().frames[0].id, 2);
        });
        assert!(
            enqueue_control(
                &tx,
                batch(2),
                Instant::now() + std::time::Duration::from_secs(1)
            )
            .is_ok()
        );
        worker.join().unwrap();
    }

    #[test]
    fn stalled_control_queue_expires_and_keeps_the_original_batch() {
        let (tx, rx) = bounded(1);
        tx.send(batch(1)).unwrap();
        assert!(matches!(
            enqueue_control(
                &tx,
                batch(2),
                Instant::now() + std::time::Duration::from_millis(10)
            ),
            Err(crossbeam_channel::SendTimeoutError::Timeout(_))
        ));
        assert_eq!(rx.recv().unwrap().frames[0].id, 1);
        assert!(rx.is_empty());
        assert!(enqueue_control(&tx, batch(3), Instant::now()).is_err());
        assert!(rx.is_empty());
    }

    #[test]
    fn pending_batch_preserves_transport_expiration() {
        let (completion, result) = std_mpsc::sync_channel(1);
        completion.send(Err(BatchFailure::Expired)).unwrap();
        let pending = PendingBatch {
            bus: BusName::from("can"),
            kind: PriorityBatchKind::SafetyBarrier,
            completion: Some(result),
        };

        assert!(matches!(
            pending.wait(),
            Err(BusError::PriorityBatchExpired {
                kind: PriorityBatchKind::SafetyBarrier,
                ..
            })
        ));
    }
}

#[cfg(test)]
mod maintenance_tests;
