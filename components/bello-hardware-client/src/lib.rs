//! Use [`Robot`] for group control and [`raw`] for protobuf and lease access.

mod client;
mod command;
mod control;
mod duration;
mod endpoint;
mod enum_value;
mod error;
mod frame;
mod inventory;
mod lease;
mod power;
mod robot;
mod state;
mod telemetry;
mod types;

pub(crate) use client::HardwareClient;
pub(crate) use command::{CommandStream, CommandStreamOptions, CommandValidity};
pub use control::{ControlOptions, ControlSession, ControlState};
pub use endpoint::{ConnectOptions, TlsOptions};
pub(crate) use enum_value::EnumValue;
pub use error::Error;
pub use frame::{CommandFrame, CommandFrameBuilder, CoordinateTarget};
pub use inventory::{Bus, DeviceStatus, Imu, Motor, MotorLimits, SerialDevice};
pub(crate) use lease::{GroupLease, LeaseDuration};
pub use power::{
    PowerBoard, PowerBoardSnapshot, PowerFlags, PowerReading, PowerSample, VoltageCurrent,
};
pub use robot::{MotorGroup, Robot};
pub use state::{
    CommandPolicySnapshot, GroupSnapshot, HealthSnapshot, ImuSnapshot, MotorSnapshot, Quaternion,
    RobotState, RobotStateChange, RobotStates, StateStreamOptions, Vector3,
};
pub(crate) use telemetry::{TelemetryEvent, TelemetryFilter, TelemetryStream};
pub use types::{
    CommandBehavior, CommandPolicyCheck, CommandResult, DeviceState, FaultResponse, GroupCause,
    GroupState,
};

/// Low-level protobuf, lease, command-stream, and telemetry primitives.
pub mod raw {
    pub use bello_hardware_api as proto;

    pub use crate::client::HardwareClient;
    pub use crate::command::{
        CommandAcknowledgement, CommandRejected, CommandResponses, CommandSender, CommandStream,
        CommandStreamOptions, CommandValidity,
    };
    pub use crate::enum_value::EnumValue;
    pub use crate::lease::{GroupLease, LeaseDeadline, LeaseDuration, LeaseRenewer, LeaseStatus};
    pub use crate::telemetry::{TelemetryEvent, TelemetryFilter, TelemetryStream};
}
