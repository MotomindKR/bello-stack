use super::{DeviceStatus, GroupName, ImuName, MotorName};
use crate::config::{CommandBehavior, FaultCause, FaultResponse};
use crate::control::CommandPolicyCheck;
use crate::control::GroupState;

#[derive(Clone, Debug)]
pub struct MotorTelemetry {
    pub group: GroupName,
    pub motor: MotorName,
    pub position: f64,
    pub velocity: f64,
    pub torque: f64,
    pub temperature: f64,
    pub fault_bits: u16,
    pub mode_status: u8,
    pub feedback_age_ms: u64,
}

#[derive(Clone, Debug)]
pub struct ImuTelemetry {
    pub imu: ImuName,
    pub orientation_xyzw: [f64; 4],
    pub angular_velocity_xyz: [f64; 3],
    pub linear_acceleration_xyz: [f64; 3],
    pub device_timestamp_ms: u64,
    pub sample_age_ms: u64,
}

#[derive(Clone, Debug)]
pub struct GroupTelemetry {
    pub group: GroupName,
    pub state: GroupState,
    pub fault_cause: Option<FaultCause>,
    pub fault_detail: String,
    pub command_count: u64,
    pub control_tick: u64,
    pub jitter_us: i64,
}

#[derive(Clone, Debug)]
pub struct CommandPolicyTelemetry {
    pub group: GroupName,
    pub motor: MotorName,
    pub check: CommandPolicyCheck,
    pub behavior: CommandBehavior,
    pub detail: String,
    pub command_modified: bool,
    pub command_rejected: bool,
    pub response: Option<FaultResponse>,
}

#[derive(Clone, Debug)]
pub struct ComponentHealth {
    pub component: String,
    pub status: DeviceStatus,
    pub dropped_events: u64,
}

#[derive(Clone, Debug)]
pub enum TelemetryPayload {
    Motor(MotorTelemetry),
    Imu(ImuTelemetry),
    Group(GroupTelemetry),
    CommandPolicy(CommandPolicyTelemetry),
    Health(ComponentHealth),
}

#[derive(Clone, Debug)]
pub struct TelemetryEvent {
    pub monotonic_ns: u64,
    pub payload: TelemetryPayload,
}
