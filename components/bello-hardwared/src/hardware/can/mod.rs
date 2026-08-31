use std::collections::HashMap;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc as std_mpsc;
use std::thread;
use std::time::Instant;

use anyhow::{Result, anyhow};
use crossbeam_channel::{Receiver, Sender, bounded};
use thiserror::Error;
use tracing::error;

use crate::config::ResolvedTopology;
use crate::domain::{BusName, GroupName};
use crate::hardware::lifecycle::{DeviceLifecycle, DeviceState, DeviceStatus};
use crate::telemetry::Telemetry;

mod spec;
mod supervisor;

pub(crate) use spec::BusSpec;

// Keep several control ticks so a brief host scheduling delay cannot fault an
// otherwise healthy CAN bus. Every batch still carries its original one-tick
// expiry, so the transmit worker drops stale commands instead of replaying them.
const CONTROL_QUEUE_TICKS_PER_GROUP: usize = 4;

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
    pub(super) frames: Vec<CanFrame>,
    pub(super) expires_at: Instant,
}

#[derive(Debug, PartialEq, Eq)]
pub(super) enum BatchFailure {
    Expired,
    Transmit(String),
}

pub(super) struct PriorityBatch {
    pub(super) kind: PriorityBatchKind,
    pub(super) frames: Vec<CanFrame>,
    pub(super) expires_at: Instant,
    pub(super) completion: std_mpsc::SyncSender<Result<(), BatchFailure>>,
}

#[derive(Clone)]
pub(super) struct BusTxQueues {
    pub(super) safety: Receiver<PriorityBatch>,
    pub(super) configuration: Receiver<PriorityBatch>,
    pub(super) control: Receiver<ControlBatch>,
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
    control_tx: Sender<ControlBatch>,
    configuration_tx: Sender<PriorityBatch>,
    safety_tx: Sender<PriorityBatch>,
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

    pub(crate) fn try_enqueue_control_batch(
        &self,
        frames: Vec<CanFrame>,
        expires_at: Instant,
    ) -> Result<(), BusError> {
        self.ensure_online()?;
        if frames.is_empty() {
            return Ok(());
        }
        if Instant::now()
            .checked_add(self.spec.control_batch_budget(frames.len()))
            .is_none_or(|completion| completion > expires_at)
        {
            return Err(BusError::ControlBatchExpired {
                bus: self.spec.name.clone(),
            });
        }
        self.control_tx
            .try_send(ControlBatch { frames, expires_at })
            .map_err(|error| match error {
                crossbeam_channel::TrySendError::Full(_) => BusError::ControlQueueFull {
                    bus: self.spec.name.clone(),
                },
                crossbeam_channel::TrySendError::Disconnected(_) => BusError::Disconnected {
                    bus: self.spec.name.clone(),
                },
            })
    }

    pub(crate) fn enqueue_configuration_batch(
        &self,
        frames: Vec<CanFrame>,
    ) -> Result<(), BusError> {
        self.submit_priority_batch(PriorityBatchKind::Configuration, frames)?
            .wait()
    }

    pub(crate) fn submit_safety_batch(
        &self,
        frames: Vec<CanFrame>,
    ) -> Result<PendingBatch, BusError> {
        self.submit_priority_batch(PriorityBatchKind::SafetyBarrier, frames)
    }

    fn submit_priority_batch(
        &self,
        kind: PriorityBatchKind,
        frames: Vec<CanFrame>,
    ) -> Result<PendingBatch, BusError> {
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

fn control_queue_capacity(topology: &ResolvedTopology, bus: &BusName) -> usize {
    let groups_on_bus = topology
        .groups()
        .filter(|(_, group)| {
            group.motors.iter().any(|motor| {
                topology
                    .motor(motor)
                    .is_ok_and(|motor| &motor.spec.address.bus == bus)
            })
        })
        .count();
    control_queue_capacity_for_groups(groups_on_bus)
}

fn control_queue_capacity_for_groups(groups_on_bus: usize) -> usize {
    groups_on_bus
        .max(1)
        .saturating_mul(CONTROL_QUEUE_TICKS_PER_GROUP)
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
                    configuration_tx,
                    safety_tx,
                },
            );
            let spec = bus.clone();
            let frames = frame_tx.clone();
            let shutdown = manager.shutdown.clone();
            let panic_lifecycle = lifecycle.clone();
            manager.threads.push(
                thread::Builder::new()
                    .name(format!("{}-supervisor", bus.name))
                    .spawn(move || {
                        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                            supervisor::run(
                                spec,
                                BusTxQueues {
                                    safety: safety_rx,
                                    configuration: configuration_rx,
                                    control: control_rx,
                                },
                                frames,
                                lifecycle,
                                shutdown,
                            );
                        }));
                        if result.is_err() {
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

    #[test]
    fn control_queue_keeps_four_ticks_per_group() {
        assert_eq!(control_queue_capacity_for_groups(0), 4);
        assert_eq!(control_queue_capacity_for_groups(1), 4);
        assert_eq!(control_queue_capacity_for_groups(2), 8);
    }
}
