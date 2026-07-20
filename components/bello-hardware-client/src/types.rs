use bello_hardware_api as proto;

use crate::EnumValue;

/// Motor-group lifecycle state without exposing protobuf representation details.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum GroupState {
    Unspecified,
    Idle,
    Enabled,
    Running,
    Fault,
    Shutdown,
    Unknown(i32),
}

impl GroupState {
    pub(crate) fn from_raw(raw: i32) -> Self {
        match proto::GroupState::try_from(raw) {
            Ok(proto::GroupState::Unspecified) => Self::Unspecified,
            Ok(proto::GroupState::Idle) => Self::Idle,
            Ok(proto::GroupState::Enabled) => Self::Enabled,
            Ok(proto::GroupState::Running) => Self::Running,
            Ok(proto::GroupState::Fault) => Self::Fault,
            Ok(proto::GroupState::Shutdown) => Self::Shutdown,
            Err(_) => Self::Unknown(raw),
        }
    }

    pub(crate) fn from_wire(value: EnumValue<proto::GroupState>) -> Self {
        match value {
            EnumValue::Known(value) => Self::from_raw(value as i32),
            EnumValue::Unknown(raw) => Self::Unknown(raw),
        }
    }
}

/// Transport or device lifecycle state.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum DeviceState {
    Unspecified,
    Starting,
    Connecting,
    Online,
    Degraded,
    Reconnecting,
    Fault,
    Shutdown,
    Unknown(i32),
}

impl DeviceState {
    pub(crate) fn from_raw(raw: i32) -> Self {
        match proto::DeviceState::try_from(raw) {
            Ok(proto::DeviceState::Unspecified) => Self::Unspecified,
            Ok(proto::DeviceState::Starting) => Self::Starting,
            Ok(proto::DeviceState::Connecting) => Self::Connecting,
            Ok(proto::DeviceState::Online) => Self::Online,
            Ok(proto::DeviceState::Degraded) => Self::Degraded,
            Ok(proto::DeviceState::Reconnecting) => Self::Reconnecting,
            Ok(proto::DeviceState::Fault) => Self::Fault,
            Ok(proto::DeviceState::Shutdown) => Self::Shutdown,
            Err(_) => Self::Unknown(raw),
        }
    }
}

/// Cause attached to the latest motor-group fault telemetry.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum FaultCause {
    Unspecified,
    CommandLimit,
    CommandPositionStep,
    CommandTimeout,
    SafePosition,
    StaleFeedback,
    Temperature,
    ReportedMotorFault,
    ModeStatus,
    Communication,
    Enable,
    Start,
    Stop,
    Transmission,
    LeaseLoss,
    Disable,
    Recover,
    Exit,
    Internal,
    Unknown(i32),
}

impl FaultCause {
    pub(crate) fn from_raw(raw: i32) -> Self {
        match proto::FaultCause::try_from(raw) {
            Ok(proto::FaultCause::Unspecified) => Self::Unspecified,
            Ok(proto::FaultCause::CommandLimit) => Self::CommandLimit,
            Ok(proto::FaultCause::CommandPositionStep) => Self::CommandPositionStep,
            Ok(proto::FaultCause::CommandTimeout) => Self::CommandTimeout,
            Ok(proto::FaultCause::SafePosition) => Self::SafePosition,
            Ok(proto::FaultCause::StaleFeedback) => Self::StaleFeedback,
            Ok(proto::FaultCause::Temperature) => Self::Temperature,
            Ok(proto::FaultCause::ReportedMotorFault) => Self::ReportedMotorFault,
            Ok(proto::FaultCause::ModeStatus) => Self::ModeStatus,
            Ok(proto::FaultCause::Communication) => Self::Communication,
            Ok(proto::FaultCause::Enable) => Self::Enable,
            Ok(proto::FaultCause::Start) => Self::Start,
            Ok(proto::FaultCause::Stop) => Self::Stop,
            Ok(proto::FaultCause::Transmission) => Self::Transmission,
            Ok(proto::FaultCause::LeaseLoss) => Self::LeaseLoss,
            Ok(proto::FaultCause::Disable) => Self::Disable,
            Ok(proto::FaultCause::Recover) => Self::Recover,
            Ok(proto::FaultCause::Exit) => Self::Exit,
            Ok(proto::FaultCause::Internal) => Self::Internal,
            Err(_) => Self::Unknown(raw),
        }
    }
}

/// Safety check that produced a command-policy telemetry event.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum CommandPolicyCheck {
    Unspecified,
    Limit,
    PositionStep,
    Unknown(i32),
}

impl CommandPolicyCheck {
    pub(crate) fn from_raw(raw: i32) -> Self {
        match proto::CommandPolicyCheck::try_from(raw) {
            Ok(proto::CommandPolicyCheck::Unspecified) => Self::Unspecified,
            Ok(proto::CommandPolicyCheck::Limit) => Self::Limit,
            Ok(proto::CommandPolicyCheck::PositionStep) => Self::PositionStep,
            Err(_) => Self::Unknown(raw),
        }
    }
}

/// Configured reaction associated with a command-policy event.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum CommandBehavior {
    Unspecified,
    Clamp,
    Reject,
    Fault,
    Unknown(i32),
}

impl CommandBehavior {
    pub(crate) fn from_raw(raw: i32) -> Self {
        match proto::CommandBehavior::try_from(raw) {
            Ok(proto::CommandBehavior::Unspecified) => Self::Unspecified,
            Ok(proto::CommandBehavior::Clamp) => Self::Clamp,
            Ok(proto::CommandBehavior::Reject) => Self::Reject,
            Ok(proto::CommandBehavior::Fault) => Self::Fault,
            Err(_) => Self::Unknown(raw),
        }
    }
}

/// Motor reaction requested when command policy transitions a group to fault.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum FaultResponse {
    Unspecified,
    Damping,
    Disable,
    Unknown(i32),
}

impl FaultResponse {
    pub(crate) fn from_raw(raw: i32) -> Self {
        match proto::FaultResponse::try_from(raw) {
            Ok(proto::FaultResponse::Unspecified) => Self::Unspecified,
            Ok(proto::FaultResponse::Damping) => Self::Damping,
            Ok(proto::FaultResponse::Disable) => Self::Disable,
            Err(_) => Self::Unknown(raw),
        }
    }
}

/// Result of a submitted command snapshot.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum CommandResult {
    Unspecified,
    Accepted,
    UnknownGroup,
    LeaseLost,
    InvalidState,
    IncompleteGroup,
    StaleSequence,
    Unsafe,
    Superseded,
    ControllerStopped,
    Expired,
    Unknown(i32),
}

impl CommandResult {
    pub(crate) fn from_wire(value: EnumValue<proto::CommandResult>) -> Self {
        let raw = match value {
            EnumValue::Known(value) => value as i32,
            EnumValue::Unknown(raw) => return Self::Unknown(raw),
        };
        match proto::CommandResult::try_from(raw) {
            Ok(proto::CommandResult::Unspecified) => Self::Unspecified,
            Ok(proto::CommandResult::Accepted) => Self::Accepted,
            Ok(proto::CommandResult::UnknownGroup) => Self::UnknownGroup,
            Ok(proto::CommandResult::LeaseLost) => Self::LeaseLost,
            Ok(proto::CommandResult::InvalidState) => Self::InvalidState,
            Ok(proto::CommandResult::IncompleteGroup) => Self::IncompleteGroup,
            Ok(proto::CommandResult::StaleSequence) => Self::StaleSequence,
            Ok(proto::CommandResult::Unsafe) => Self::Unsafe,
            Ok(proto::CommandResult::Superseded) => Self::Superseded,
            Ok(proto::CommandResult::ControllerStopped) => Self::ControllerStopped,
            Ok(proto::CommandResult::Expired) => Self::Expired,
            Err(_) => Self::Unknown(raw),
        }
    }
}
