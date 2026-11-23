use thiserror::Error;

use crate::lease::LeaseStatus;
use crate::{CommandResult, GroupState};

#[derive(Debug, Error)]
#[non_exhaustive]
pub enum Error {
    #[error("hardware transport error: {0}")]
    Transport(#[from] tonic::transport::Error),
    #[error("hardware gRPC request failed: {0}")]
    Rpc(#[from] tonic::Status),
    #[error("invalid client argument: {0}")]
    InvalidArgument(String),
    #[error("hardware server violated the client protocol: {0}")]
    Protocol(String),
    #[error("group lease is not active ({0:?})")]
    LeaseInactive(LeaseStatus),
    #[error("command request stream is closed")]
    CommandStreamClosed,
    #[error("this group lease already has an open command stream")]
    CommandStreamAlreadyOpen,
    #[error("command sequence is exhausted")]
    SequenceExhausted,
    #[error("hardware inventory is inconsistent: {0}")]
    InvalidInventory(String),
    #[error("unknown motor group {0:?}")]
    UnknownGroup(String),
    #[error("unknown coordinate {coordinate:?} in motor group {group:?}")]
    UnknownCoordinate { group: String, coordinate: String },
    #[error("coordinate {coordinate:?} was assigned more than once in motor group {group:?}")]
    DuplicateCoordinate { group: String, coordinate: String },
    #[error("command frame for {group:?} is incomplete; missing coordinates: {missing:?}")]
    IncompleteFrame { group: String, missing: Vec<String> },
    #[error("command frame belongs to {actual:?}, not control group {expected:?}")]
    FrameGroupMismatch { expected: String, actual: String },
    #[error("invalid command target for coordinate {coordinate:?}: {detail}")]
    InvalidTarget { coordinate: String, detail: String },
    #[error("control session for {group:?} cannot {operation} while it is {state}")]
    InvalidSessionState {
        group: String,
        operation: &'static str,
        state: &'static str,
    },
    #[error("{operation} for motor group {group:?} returned {actual:?}, expected {expected:?}")]
    UnexpectedGroupState {
        group: String,
        operation: &'static str,
        expected: GroupState,
        actual: GroupState,
    },
    #[error(
        "command {sequence} for motor group {group:?} was rejected as {result:?} in {state:?}: {detail}"
    )]
    RejectedCommand {
        group: String,
        sequence: u64,
        result: CommandResult,
        detail: String,
        state: GroupState,
    },
    #[error("command acknowledgement stream for motor group {0:?} closed")]
    AcknowledgementStreamClosed(String),
    #[error("timed out waiting for command acknowledgement from motor group {group:?}")]
    AcknowledgementTimeout { group: String },
    #[error(
        "motor group {group:?} acknowledged command {actual}, but command {expected} was pending"
    )]
    AcknowledgementMismatch {
        group: String,
        expected: u64,
        actual: u64,
    },
    #[error("automatic lease renewal for motor group {group:?} failed: {detail}")]
    RenewalFailed { group: String, detail: String },
    #[error("control cleanup encountered errors: {0:?}")]
    CleanupFailed(Vec<String>),
    #[error("dynamic state for {kind} {name:?} is not available yet")]
    StateUnavailable { kind: &'static str, name: String },
}

impl Error {
    /// Returns the original gRPC status when the failure came from an RPC.
    pub fn rpc_status(&self) -> Option<&tonic::Status> {
        match self {
            Self::Rpc(status) => Some(status),
            _ => None,
        }
    }
}
