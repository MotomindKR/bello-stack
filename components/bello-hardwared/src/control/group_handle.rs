use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::mpsc::SyncSender;
use std::time::Instant;

use parking_lot::{Mutex, RwLock};
use thiserror::Error;
use tokio::sync::oneshot;

use crate::domain::{CommandBatch, CommandError};

use super::fsm::{GroupCause, GroupState, Transition};

#[derive(Clone, Debug, Default)]
pub struct GroupSnapshot {
    pub state: GroupState,
    pub cause: GroupCause,
    pub detail: String,
    pub command_count: u64,
    pub control_tick: u64,
    pub jitter_us: i64,
    pub updated: Option<Instant>,
}

pub(crate) enum LifecycleAction {
    Enable,
    Start {
        batch: CommandBatch,
        submitted_at: Instant,
    },
    Stop,
    Disable,
    Recover,
}

#[derive(Clone, Copy, Debug, Error)]
pub(crate) enum GroupQueueError {
    #[error("group controller stopped")]
    Stopped,
    #[error("group controller operation queue is full")]
    Full,
    #[error("group controller operation queue is disconnected")]
    Disconnected,
}

pub(super) enum LifecycleRequest {
    Operation {
        action: LifecycleAction,
        lease_generation: u64,
        reply: oneshot::Sender<Transition>,
    },
}

pub(super) struct CommandSubmission {
    pub batch: CommandBatch,
    pub lease_generation: u64,
    pub submitted_at: Instant,
    pub reply: oneshot::Sender<Result<(), CommandError>>,
}

#[derive(Clone)]
pub struct GroupHandle {
    pub(super) lifecycle_tx: SyncSender<LifecycleRequest>,
    pub(super) wake_tx: SyncSender<()>,
    pub(super) command_mailbox: Arc<Mutex<Option<CommandSubmission>>>,
    pub(super) lease_lost: Arc<AtomicBool>,
    pub(super) lease_generation: Arc<AtomicU64>,
    pub(super) lease_loss_waiters: Arc<Mutex<Vec<oneshot::Sender<Transition>>>>,
    pub(super) shutdown: Arc<AtomicBool>,
    pub snapshot: Arc<RwLock<GroupSnapshot>>,
}

impl GroupHandle {
    pub(crate) fn activate_lease(&self, generation: u64) {
        self.lease_generation.store(generation, Ordering::Release);
    }

    pub(crate) fn enqueue_lifecycle(
        &self,
        action: LifecycleAction,
        lease_generation: u64,
    ) -> Result<oneshot::Receiver<Transition>, GroupQueueError> {
        if self.shutdown.load(Ordering::Acquire) {
            return Err(GroupQueueError::Stopped);
        }
        let (reply, receiver) = oneshot::channel();
        self.lifecycle_tx
            .try_send(LifecycleRequest::Operation {
                action,
                lease_generation,
                reply,
            })
            .map_err(|error| match error {
                std::sync::mpsc::TrySendError::Full(_) => GroupQueueError::Full,
                std::sync::mpsc::TrySendError::Disconnected(_) => GroupQueueError::Disconnected,
            })?;
        let _ = self.wake_tx.try_send(());
        Ok(receiver)
    }

    pub(crate) fn invalidate_lease(&self) -> oneshot::Receiver<Transition> {
        let (reply, receiver) = oneshot::channel();
        let mut waiters = self.lease_loss_waiters.lock();
        if self.shutdown.load(Ordering::Acquire) {
            drop(waiters);
            let snapshot = self.snapshot.read();
            let _ = reply.send(Transition {
                ok: false,
                to: snapshot.state,
                message: "group controller stopped".into(),
            });
            return receiver;
        }
        waiters.push(reply);
        self.lease_generation.store(0, Ordering::Release);
        self.lease_lost.store(true, Ordering::Release);
        drop(waiters);
        let _ = self.wake_tx.try_send(());
        receiver
    }

    pub(crate) fn expire_lease(&self) {
        self.lease_generation.store(0, Ordering::Release);
        self.lease_lost.store(true, Ordering::Release);
        let _ = self.wake_tx.try_send(());
    }

    pub(crate) fn enqueue_command(
        &self,
        batch: CommandBatch,
        lease_generation: u64,
        submitted_at: Instant,
    ) -> Result<oneshot::Receiver<Result<(), CommandError>>, CommandError> {
        let (reply, receiver) = oneshot::channel();
        let mut mailbox = self.command_mailbox.lock();
        if self.shutdown.load(Ordering::Acquire) {
            return Err(CommandError::ControllerStopped);
        }
        if let Some(previous) = mailbox.replace(CommandSubmission {
            batch,
            lease_generation,
            submitted_at,
            reply,
        }) {
            let _ = previous.reply.send(Err(CommandError::Superseded));
        }
        drop(mailbox);
        let _ = self.wake_tx.try_send(());
        Ok(receiver)
    }
}
