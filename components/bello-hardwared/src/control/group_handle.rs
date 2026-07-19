use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::SyncSender;

use anyhow::{Result, anyhow};
use parking_lot::{Mutex, RwLock};
use tokio::sync::oneshot;

use crate::domain::{CommandBatch, CommandError};

use super::fsm::{GroupState, Transition};

#[derive(Clone, Debug, Default)]
pub struct GroupSnapshot {
    pub state: GroupState,
    pub fault_reason: String,
}

pub(super) enum LifecycleRequest {
    Enable(oneshot::Sender<Transition>),
    Start(oneshot::Sender<Transition>),
    Stop(oneshot::Sender<Transition>),
    Disable(oneshot::Sender<Transition>),
    Recover(oneshot::Sender<Transition>),
    Fault(String, oneshot::Sender<Transition>),
    Wake,
}

pub(super) struct CommandSubmission {
    pub batch: CommandBatch,
    pub reply: oneshot::Sender<Result<(), CommandError>>,
}

#[derive(Clone)]
pub struct GroupHandle {
    pub(super) lifecycle_tx: SyncSender<LifecycleRequest>,
    pub(super) command_mailbox: Arc<Mutex<Option<CommandSubmission>>>,
    pub(super) lease_lost: Arc<AtomicBool>,
    pub(super) shutdown: Arc<AtomicBool>,
    pub snapshot: Arc<RwLock<GroupSnapshot>>,
}

impl GroupHandle {
    async fn transition(
        &self,
        request: impl FnOnce(oneshot::Sender<Transition>) -> LifecycleRequest,
    ) -> Result<Transition> {
        let (tx, rx) = oneshot::channel();
        self.lifecycle_tx
            .try_send(request(tx))
            .map_err(|error| anyhow!("group controller unavailable: {error}"))?;
        Ok(rx.await?)
    }

    pub async fn enable(&self) -> Result<Transition> {
        self.transition(LifecycleRequest::Enable).await
    }

    pub async fn start(&self) -> Result<Transition> {
        self.transition(LifecycleRequest::Start).await
    }

    pub async fn stop(&self) -> Result<Transition> {
        self.transition(LifecycleRequest::Stop).await
    }

    pub async fn disable(&self) -> Result<Transition> {
        self.transition(LifecycleRequest::Disable).await
    }

    pub async fn recover(&self) -> Result<Transition> {
        self.transition(LifecycleRequest::Recover).await
    }

    pub async fn fault(&self, reason: String) -> Result<Transition> {
        self.transition(|reply| LifecycleRequest::Fault(reason, reply))
            .await
    }

    pub fn lease_lost(&self) {
        self.lease_lost.store(true, Ordering::Release);
        let _ = self.lifecycle_tx.try_send(LifecycleRequest::Wake);
    }

    pub(crate) fn enqueue_command(
        &self,
        batch: CommandBatch,
    ) -> Result<oneshot::Receiver<Result<(), CommandError>>, CommandError> {
        let (tx, rx) = oneshot::channel();
        if self.shutdown.load(Ordering::Acquire) {
            return Err(CommandError::ControllerStopped);
        }
        if let Some(previous) = self
            .command_mailbox
            .lock()
            .replace(CommandSubmission { batch, reply: tx })
        {
            let _ = previous.reply.send(Err(CommandError::Superseded));
        }
        let _ = self.lifecycle_tx.try_send(LifecycleRequest::Wake);
        Ok(rx)
    }
}
