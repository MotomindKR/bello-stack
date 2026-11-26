//! Use [`Robot`] for managed group control and [`Robot::grpc_client`] for direct RPCs.

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

pub use bello_hardware_api as proto;
pub use bello_hardware_api::{
    CommandBehavior, CommandPolicyCheck, CommandResult, DeviceState, FaultResponse, GroupCause,
    GroupState,
};
pub use command::CommandReceipt;
pub use control::{ControlOptions, ControlSession, ControlState};
pub use endpoint::{ConnectOptions, TlsOptions};
pub use enum_value::EnumValue;
pub use error::Error;
pub use frame::{CommandFrame, CommandFrameBuilder, CoordinateTarget};
pub use inventory::{DeviceStatus, Motor, MotorLimits};
pub use lease::LeaseStatus;
pub(crate) use lease::{GroupLease, LeaseDuration};
pub use power::{
    PowerBoard, PowerBoardSnapshot, PowerFlags, PowerReading, PowerSample, VoltageCurrent,
};
pub use robot::{MotorGroup, Robot};
pub use state::{
    CommandPolicySnapshot, GroupSnapshot, HealthSnapshot, ImuSnapshot, MotorSnapshot, Quaternion,
    RobotState, RobotStateChange, RobotStates, StateStreamOptions, Vector3,
};
