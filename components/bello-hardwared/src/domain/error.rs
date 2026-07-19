use thiserror::Error;

use super::MotorName;

#[derive(Clone, Debug, Error, PartialEq)]
pub enum SafetyViolation {
    #[error("non-finite {field}")]
    NonFinite { field: &'static str },
    #[error("position outside safe limit")]
    PositionLimit,
    #[error("velocity outside safe limit")]
    VelocityLimit,
    #[error("torque outside safe limit")]
    TorqueLimit,
    #[error("gain outside safe limit")]
    GainLimit,
    #[error("position step exceeds limit")]
    PositionStep,
}

#[derive(Clone, Debug, Error, PartialEq)]
pub enum CommandError {
    #[error("unknown motor group")]
    UnknownGroup,
    #[error("control lease was lost before the command could be applied")]
    LeaseLost,
    #[error("group must be Enabled or Running")]
    InvalidState,
    #[error("command batch must contain every group motor exactly once")]
    IncompleteGroup,
    #[error("command sequence {received} is not newer than {previous}")]
    StaleSequence { received: u64, previous: u64 },
    #[error("unsafe command for {motor}: {violation}")]
    Unsafe {
        motor: MotorName,
        violation: SafetyViolation,
    },
    #[error("command was superseded by a newer pending batch")]
    Superseded,
    #[error("group controller stopped")]
    ControllerStopped,
}
