use super::{
    DeviceState, DeviceStatus, GroupName, ImuName, MotorAddress, MotorName, SerialDeviceName,
};
use crate::control::GroupState;

#[derive(Clone, Debug)]
pub struct MotorTelemetry {
    pub group: GroupName,
    pub motor: MotorName,
    pub address: MotorAddress,
    pub status: DeviceStatus,
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
    pub serial_device: SerialDeviceName,
    pub status: DeviceStatus,
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
    pub fault_reason: String,
    pub lease_owner: Option<String>,
    pub command_count: u64,
    pub control_tick: u64,
    pub jitter_us: i64,
}

#[derive(Clone, Debug)]
pub struct CommandPolicyTelemetry {
    pub group: GroupName,
    pub motor: MotorName,
    pub check: String,
    pub severity: String,
    pub detail: String,
    pub command_modified: bool,
    pub command_rejected: bool,
    pub response: String,
}

#[derive(Clone, Debug)]
pub struct ComponentHealth {
    pub component: String,
    pub status: DeviceState,
    pub detail: String,
    pub transition_count: u64,
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
