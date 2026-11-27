use std::collections::BTreeMap;
use std::future::Future;
use std::pin::Pin;
use std::sync::{Arc, Mutex};
use std::task::{Context, Poll};
use std::time::Duration;

use bello_hardware_api::{
    CommandResult, GroupState, StreamCommandsRequest, StreamCommandsResponse,
};
use tokio::sync::{Notify, OwnedSemaphorePermit, Semaphore, TryAcquireError, mpsc, oneshot};
use tokio::task::JoinHandle;
use tokio::time::Instant;
use tokio_stream::wrappers::ReceiverStream;

use crate::control::RenewalStatus;
use crate::lease::{GroupLease, LeaseHealth, active_health, mark_lease_lost};
use crate::{ControlState, EnumValue, Error};

/// Resolves when the daemon acknowledges this command. Dropping it only stops observation.
#[derive(Debug)]
#[must_use = "await the receipt to observe the command result"]
pub struct CommandReceipt(oneshot::Receiver<Result<(), Error>>);

impl Future for CommandReceipt {
    type Output = Result<(), Error>;

    fn poll(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        Pin::new(&mut self.0)
            .poll(cx)
            .map(|reply| reply.unwrap_or(Err(Error::CommandStreamClosed)))
    }
}

struct Pending {
    deadline: Instant,
    result: oneshot::Sender<Result<(), Error>>,
    slot: OwnedSemaphorePermit,
}

struct PendingCommands {
    entries: BTreeMap<u64, Pending>,
    state: ControlState,
}

pub(crate) struct CommandStream {
    requests: mpsc::Sender<StreamCommandsRequest>,
    pending: Arc<Mutex<PendingCommands>>,
    slots: Arc<Semaphore>,
    changed: Arc<Notify>,
    receiver: JoinHandle<()>,
}

impl CommandStream {
    pub(crate) async fn open(
        lease: &GroupLease,
        capacity: usize,
        renewal: tokio::sync::watch::Receiver<RenewalStatus>,
    ) -> Result<Self, Error> {
        active_health(&lease.health)?;
        let (requests, rx) = mpsc::channel(capacity);
        let responses = lease
            .client
            .clone()
            .stream_commands(ReceiverStream::new(rx))
            .await?
            .into_inner();
        let pending = Arc::new(Mutex::new(PendingCommands {
            entries: BTreeMap::new(),
            state: ControlState::Running,
        }));
        let slots = Arc::new(Semaphore::new(capacity));
        let changed = Arc::new(Notify::new());
        let receiver = tokio::spawn(receive(
            responses,
            lease.group.clone(),
            lease.health.clone(),
            renewal,
            Arc::clone(&pending),
            Arc::clone(&slots),
            Arc::clone(&changed),
        ));
        Ok(Self {
            requests,
            pending,
            slots,
            changed,
            receiver,
        })
    }

    pub(crate) fn state(&self) -> ControlState {
        self.pending.lock().unwrap().state
    }

    pub(crate) fn try_slot(&self) -> Result<OwnedSemaphorePermit, Error> {
        self.slots
            .clone()
            .try_acquire_owned()
            .map_err(|error| match error {
                TryAcquireError::NoPermits => Error::QueueFull,
                TryAcquireError::Closed => Error::CommandStreamClosed,
            })
    }

    pub(crate) async fn slot(&self) -> Result<OwnedSemaphorePermit, Error> {
        self.slots
            .clone()
            .acquire_owned()
            .await
            .map_err(|_| Error::CommandStreamClosed)
    }

    pub(crate) fn submit(
        &self,
        slot: OwnedSemaphorePermit,
        request: StreamCommandsRequest,
        timeout: Duration,
    ) -> Result<CommandReceipt, Error> {
        let mut pending = self.pending.lock().unwrap();
        if pending.state != ControlState::Running {
            return Err(Error::CommandStreamClosed);
        }
        let permit = self.requests.try_reserve().map_err(|err| match err {
            mpsc::error::TrySendError::Full(_) => Error::QueueFull,
            mpsc::error::TrySendError::Closed(_) => Error::CommandStreamClosed,
        })?;
        let (result, reply) = oneshot::channel();
        let deadline = Instant::now().checked_add(timeout).ok_or_else(|| {
            Error::InvalidArgument("acknowledgement deadline exceeds local clock range".into())
        })?;
        pending.entries.insert(
            request.sequence,
            Pending {
                deadline,
                result,
                slot,
            },
        );
        permit.send(request);
        self.changed.notify_one();
        Ok(CommandReceipt(reply))
    }
}

impl Drop for CommandStream {
    fn drop(&mut self) {
        self.receiver.abort();
        self.pending.lock().unwrap().entries.clear();
    }
}

async fn receive(
    mut responses: tonic::Streaming<StreamCommandsResponse>,
    group: String,
    health: tokio::sync::watch::Sender<LeaseHealth>,
    mut renewal: tokio::sync::watch::Receiver<RenewalStatus>,
    pending: Arc<Mutex<PendingCommands>>,
    slots: Arc<Semaphore>,
    changed: Arc<Notify>,
) {
    let mut health_changed = health.subscribe();
    let (failed_sequence, error) = loop {
        let lease = match active_health(&health) {
            Ok(lease) => lease,
            Err(error) => break (None, error),
        };
        let lease_deadline = Instant::from_std(lease.expires_at);
        let next = pending
            .lock()
            .unwrap()
            .entries
            .first_key_value()
            .map(|(&seq, cmd)| (seq, cmd.deadline));
        let deadline = next.map_or(lease_deadline, |(_, deadline)| deadline.min(lease_deadline));
        if let Some((seq, deadline)) = next
            && Instant::now() >= deadline
        {
            break (
                Some(seq),
                Error::AcknowledgementTimeout {
                    group: group.clone(),
                },
            );
        }
        let response = tokio::select! {
            biased;
            _ = tokio::time::sleep_until(deadline) => continue,
            _ = health_changed.changed() => continue,
            changed = renewal.changed() => {
                let detail = match (changed, &*renewal.borrow()) {
                    (Ok(()), RenewalStatus::Failed(detail)) => detail.clone(),
                    _ => "renewal task stopped unexpectedly".into(),
                };
                break (None, Error::RenewalFailed { group: group.clone(), detail });
            }
            response = responses.message() => response,
            _ = changed.notified() => continue,
        };
        let ack = match response {
            Ok(Some(ack)) => ack,
            Ok(None) => break (None, Error::AcknowledgementStreamClosed(group.clone())),
            Err(status) => {
                if status.code() == tonic::Code::PermissionDenied {
                    mark_lease_lost(&health);
                }
                break (None, status.into());
            }
        };
        let sequence = ack.sequence;
        let mut commands = pending.lock().unwrap();
        if let Err(error) = commands.acknowledge(ack, &group, &health) {
            commands.close(Some(sequence), error, &slots);
            return;
        }
    };
    pending
        .lock()
        .unwrap()
        .close(failed_sequence, error, &slots);
}

impl PendingCommands {
    fn acknowledge(
        &mut self,
        ack: StreamCommandsResponse,
        group: &str,
        health: &tokio::sync::watch::Sender<LeaseHealth>,
    ) -> Result<(), Error> {
        if ack.group != group {
            return Err(Error::Protocol(format!(
                "command acknowledgement for unexpected group {:?}",
                ack.group
            )));
        }
        let command = self.entries.get(&ack.sequence).ok_or_else(|| {
            Error::Protocol(format!(
                "unsolicited command acknowledgement {}",
                ack.sequence
            ))
        })?;
        if Instant::now() >= command.deadline {
            return Err(Error::AcknowledgementTimeout {
                group: group.into(),
            });
        }
        if ack.result != CommandResult::Accepted as i32 {
            if ack.result == CommandResult::LeaseLost as i32 {
                mark_lease_lost(health);
            }
            return Err(Error::RejectedCommand {
                group: ack.group,
                sequence: ack.sequence,
                result: EnumValue::from_raw(ack.result),
                detail: ack.detail,
                state: EnumValue::from_raw(ack.state),
            });
        }
        if GroupState::try_from(ack.state).is_err() {
            return Err(Error::Protocol(format!(
                "accepted command returned unknown state {}",
                ack.state
            )));
        }
        active_health(health)?;
        let entry = self.entries.remove(&ack.sequence).expect("matched command");
        drop(entry.slot);
        let _ = entry.result.send(Ok(()));
        Ok(())
    }

    fn close(&mut self, failed_sequence: Option<u64>, error: Error, slots: &Semaphore) {
        self.state = match &error {
            Error::RejectedCommand {
                state: EnumValue::Known(GroupState::Damped),
                result: EnumValue::Known(result),
                ..
            } if *result != CommandResult::LeaseLost => ControlState::Damped,
            _ => ControlState::Uncertain,
        };
        slots.close();
        let sequence = failed_sequence
            .filter(|seq| self.entries.contains_key(seq))
            .or_else(|| self.entries.keys().next().copied());
        if let Some(cmd) = sequence.and_then(|seq| self.entries.remove(&seq)) {
            let _ = cmd.result.send(Err(error));
        }
        self.entries.clear();
    }
}
