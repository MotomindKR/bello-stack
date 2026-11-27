//! Managed hardware inventory, motor-group control, and telemetry.

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

pub use bello_hardware_api::{
    CommandBehavior, CommandPolicyCheck, CommandResult, DeviceState, FaultResponse, GroupCause,
    GroupState,
};
pub use command::CommandReceipt;
pub use control::{ControlOptions, ControlSession, ControlState};
pub use endpoint::{ConnectOptions, TlsOptions};
pub use enum_value::EnumValue;
pub use error::Error;
pub use frame::{CommandFrame, CoordinateTarget};
pub use inventory::{BusInfo, DeviceStatus, ImuInfo, Motor, MotorLimits, SerialDeviceInfo};
pub(crate) use lease::GroupLease;
pub use lease::LeaseStatus;
pub use power::{PowerBoard, PowerBoardSnapshot, PowerReading, PowerSample};
pub use robot::{MotorGroup, Robot};
pub use state::{
    CommandPolicySnapshot, GroupSnapshot, HealthSnapshot, ImuSnapshot, MotorSnapshot, Quaternion,
    RobotState, RobotStateChange, RobotStates, StateStreamOptions, Vector3,
};
