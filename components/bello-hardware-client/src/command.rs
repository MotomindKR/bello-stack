use std::fmt;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::time::Duration;

use bello_hardware_api::{
    CommandResult, GroupState, MotorCommand, StreamCommandsRequest, StreamCommandsResponse,
};
use thiserror::Error as ThisError;
use tokio::sync::{mpsc, watch};
use tokio_stream::wrappers::ReceiverStream;
use tonic::Streaming;

use crate::duration::positive_milliseconds;
use crate::lease::{GroupLease, LeaseHealth, LeaseStatus, mark_lease_lost, observe_lease_health};
use crate::{EnumValue, Error};

/// Freshness assigned to a command batch when it reaches the daemon.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum CommandValidity {
    /// Use the motor group's configured command timeout.
    #[default]
    GroupDefault,
    For(Duration),
}

impl CommandValidity {
    pub(crate) fn to_milliseconds(self) -> Result<u64, Error> {
        match self {
            Self::GroupDefault => Ok(0),
            Self::For(duration) => positive_milliseconds(duration, "command validity"),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CommandStreamOptions {
    queue_capacity: usize,
}

impl Default for CommandStreamOptions {
    fn default() -> Self {
        Self { queue_capacity: 1 }
    }
}

impl CommandStreamOptions {
    /// Queued commands begin their validity period when the daemon receives them.
    pub fn queue_capacity(mut self, capacity: usize) -> Self {
        self.queue_capacity = capacity;
        self
    }
}

/// A bound command sender and its independent acknowledgement stream.
pub struct CommandStream {
    sender: CommandSender,
    responses: CommandResponses,
}

impl CommandStream {
    pub fn split(self) -> (CommandSender, CommandResponses) {
        (self.sender, self.responses)
    }

    pub async fn send(
        &mut self,
        validity: CommandValidity,
        commands: Vec<MotorCommand>,
    ) -> Result<u64, Error> {
        self.sender.send(validity, commands).await
    }

    pub async fn message(&mut self) -> Result<Option<CommandAcknowledgement>, Error> {
        self.responses.message().await
    }

    /// Closes the request side while retaining responses already in flight.
    pub fn close(self) -> CommandResponses {
        let Self { sender, responses } = self;
        drop(sender);
        responses
    }
}

impl fmt::Debug for CommandStream {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str("CommandStream { .. }")
    }
}

/// Single-producer sender that owns monotonically increasing command sequence
/// allocation for one group lease.
pub struct CommandSender {
    inner: mpsc::Sender<StreamCommandsRequest>,
    group: String,
    token: String,
    next_sequence: Arc<AtomicU64>,
    sequence_exhausted: Arc<AtomicBool>,
    lease_health: watch::Sender<LeaseHealth>,
    _claim: Arc<CommandStreamClaim>,
}

impl CommandSender {
    pub async fn send(
        &mut self,
        validity: CommandValidity,
        commands: Vec<MotorCommand>,
    ) -> Result<u64, Error> {
        let permit = loop {
            let health = observe_lease_health(&self.lease_health);
            ensure_lease_active(&self.lease_health)?;
            let mut health_changed = self.lease_health.subscribe();
            ensure_lease_active(&self.lease_health)?;
            tokio::select! {
                permit = self.inner.reserve() => {
                    break permit.map_err(|_| Error::CommandStreamClosed)?;
                }
                changed = health_changed.changed() => {
                    changed.map_err(|_| Error::CommandStreamClosed)?;
                    ensure_lease_active(&self.lease_health)?;
                }
                _ = tokio::time::sleep_until(tokio::time::Instant::from_std(health.expires_at)) => {
                    ensure_lease_active(&self.lease_health)?;
                }
            }
        };
        ensure_lease_active(&self.lease_health)?;
        if self.sequence_exhausted.load(Ordering::Relaxed) {
            return Err(Error::SequenceExhausted);
        }
        let sequence = self.next_sequence.load(Ordering::Relaxed);
        let request = StreamCommandsRequest {
            group: self.group.clone(),
            lease_token: self.token.clone(),
            sequence,
            valid_for_ms: validity.to_milliseconds()?,
            commands,
        };
        permit.send(request);
        if let Some(next) = sequence.checked_add(1) {
            self.next_sequence.store(next, Ordering::Relaxed);
        } else {
            self.sequence_exhausted.store(true, Ordering::Relaxed);
        }
        Ok(sequence)
    }

    pub fn next_sequence(&self) -> Option<u64> {
        (!self.sequence_exhausted.load(Ordering::Relaxed))
            .then(|| self.next_sequence.load(Ordering::Relaxed))
    }
}

impl fmt::Debug for CommandSender {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter
            .debug_struct("CommandSender")
            .field("group", &self.group)
            .field("token", &"<redacted>")
            .field("next_sequence", &self.next_sequence())
            .finish()
    }
}

pub struct CommandResponses {
    inner: Streaming<StreamCommandsResponse>,
    group: String,
    lease_health: watch::Sender<LeaseHealth>,
    claim: Option<Arc<CommandStreamClaim>>,
}

impl CommandResponses {
    /// Receives the next acknowledgement. Domain rejection is represented by
    /// `acknowledgement.result`; it does not become a gRPC error.
    pub async fn message(&mut self) -> Result<Option<CommandAcknowledgement>, Error> {
        let response = match self.inner.message().await {
            Ok(response) => response,
            Err(status) => {
                self.claim.take();
                if status.code() == tonic::Code::PermissionDenied {
                    mark_lease_lost(&self.lease_health);
                }
                return Err(status.into());
            }
        };
        let Some(response) = response else {
            self.claim.take();
            return Ok(None);
        };
        if response.group != self.group {
            return Err(Error::Protocol(format!(
                "command response group {:?} does not match stream group {:?}",
                response.group, self.group
            )));
        }
        let result = EnumValue::from_raw(response.result);
        if result == EnumValue::Known(CommandResult::LeaseLost) {
            mark_lease_lost(&self.lease_health);
        }
        Ok(Some(CommandAcknowledgement {
            group: response.group,
            sequence: response.sequence,
            result,
            detail: response.detail,
            state: EnumValue::from_raw(response.state),
        }))
    }

    pub fn into_inner(self) -> Streaming<StreamCommandsResponse> {
        self.inner
    }
}

impl fmt::Debug for CommandResponses {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter
            .debug_struct("CommandResponses")
            .field("group", &self.group)
            .finish_non_exhaustive()
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CommandAcknowledgement {
    pub group: String,
    pub sequence: u64,
    pub result: EnumValue<CommandResult>,
    pub detail: String,
    pub state: EnumValue<GroupState>,
}

impl CommandAcknowledgement {
    pub fn is_accepted(&self) -> bool {
        self.result == EnumValue::Known(CommandResult::Accepted)
    }

    pub fn require_accepted(self) -> Result<Self, CommandRejected> {
        if self.is_accepted() {
            Ok(self)
        } else {
            Err(CommandRejected {
                acknowledgement: self,
            })
        }
    }
}

#[derive(Clone, Debug, ThisError)]
#[error(
    "command {sequence} for {group} was rejected as {result:?}: {detail}",
    group = .acknowledgement.group,
    sequence = .acknowledgement.sequence,
    result = .acknowledgement.result,
    detail = .acknowledgement.detail
)]
pub struct CommandRejected {
    pub acknowledgement: CommandAcknowledgement,
}

pub(crate) async fn open(
    lease: &GroupLease,
    options: CommandStreamOptions,
) -> Result<CommandStream, Error> {
    if options.queue_capacity == 0 {
        return Err(Error::InvalidArgument(
            "command queue capacity must be positive".into(),
        ));
    }
    let open = lease.command_stream_open_flag();
    if open
        .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
        .is_err()
    {
        return Err(Error::CommandStreamAlreadyOpen);
    }
    let claim = Arc::new(CommandStreamClaim { open });
    let (sender, receiver) = mpsc::channel(options.queue_capacity);
    let mut client = lease.client().inner.clone();
    let inner = match client.stream_commands(ReceiverStream::new(receiver)).await {
        Ok(response) => response.into_inner(),
        Err(status) => {
            drop(claim);
            return Err(status.into());
        }
    };
    let group = lease.group().to_owned();
    let (next_sequence, sequence_exhausted) = lease.command_sequence();
    Ok(CommandStream {
        sender: CommandSender {
            inner: sender,
            group: group.clone(),
            token: lease.token().to_owned(),
            next_sequence,
            sequence_exhausted,
            lease_health: lease.health_sender(),
            _claim: Arc::clone(&claim),
        },
        responses: CommandResponses {
            inner,
            group,
            lease_health: lease.health_sender(),
            claim: Some(claim),
        },
    })
}

struct CommandStreamClaim {
    open: Arc<AtomicBool>,
}

impl Drop for CommandStreamClaim {
    fn drop(&mut self) {
        self.open.store(false, Ordering::Release);
    }
}

fn ensure_lease_active(health: &watch::Sender<LeaseHealth>) -> Result<(), Error> {
    let status = observe_lease_health(health).status;
    if status == LeaseStatus::Active {
        Ok(())
    } else {
        Err(Error::LeaseInactive(status))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rejection_preserves_the_complete_acknowledgement() {
        let acknowledgement = CommandAcknowledgement {
            group: "leg".into(),
            sequence: 7,
            result: EnumValue::Known(CommandResult::Unsafe),
            detail: "limit".into(),
            state: EnumValue::Known(GroupState::Fault),
        };
        let rejected = acknowledgement.require_accepted().unwrap_err();
        assert_eq!(rejected.acknowledgement.sequence, 7);
        assert_eq!(
            rejected.acknowledgement.result,
            EnumValue::Known(CommandResult::Unsafe)
        );
    }

    #[test]
    fn unknown_enums_are_not_rewritten_as_unspecified() {
        assert_eq!(
            EnumValue::<CommandResult>::from_raw(10_000),
            EnumValue::Unknown(10_000)
        );
    }
}
