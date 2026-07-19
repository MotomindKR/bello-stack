use std::net::SocketAddr;

use serde::Deserialize;

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Config {
    #[serde(default)]
    pub runtime: RuntimeConfig,
    #[serde(default)]
    pub grpc: GrpcConfig,
    #[serde(default)]
    pub rerun: RerunConfig,
    #[serde(default)]
    pub serial_devices: Vec<SerialDeviceConfig>,
    #[serde(default)]
    pub imus: Vec<ImuConfig>,
    pub buses: Vec<BusConfig>,
    pub motors: Vec<MotorConfig>,
    pub motor_groups: Vec<GroupConfig>,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct RuntimeConfig {
    pub telemetry_capacity: usize,
}
impl Default for RuntimeConfig {
    fn default() -> Self {
        Self {
            telemetry_capacity: 4096,
        }
    }
}

#[derive(Clone, Debug, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct GrpcConfig {
    pub listen: SocketAddr,
    pub default_lease_ms: u64,
    pub maximum_lease_ms: u64,
}
impl Default for GrpcConfig {
    fn default() -> Self {
        Self {
            listen: "0.0.0.0:50051".parse().expect("valid default address"),
            default_lease_ms: 1000,
            maximum_lease_ms: 5000,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RerunMode {
    #[default]
    Off,
    Save,
    Connect,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct RerunConfig {
    pub mode: RerunMode,
    pub path: String,
    pub address: String,
}
impl Default for RerunConfig {
    fn default() -> Self {
        Self {
            mode: RerunMode::Off,
            path: "bello_hardware.rrd".into(),
            address: "127.0.0.1:9876".into(),
        }
    }
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BusConfig {
    pub name: String,
    pub interface: String,
    #[serde(default = "default_host_id")]
    pub host_id: u8,
    #[serde(default = "default_bitrate")]
    pub bitrate: u32,
    #[serde(default = "default_bus_utilization")]
    pub maximum_utilization: f64,
    #[serde(default)]
    pub reconnect: ReconnectConfig,
}
fn default_host_id() -> u8 {
    0xFD
}
fn default_bitrate() -> u32 {
    1_000_000
}
fn default_bus_utilization() -> f64 {
    0.75
}

#[derive(Clone, Debug, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct ReconnectConfig {
    pub initial_delay_ms: u64,
    pub maximum_delay_ms: u64,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SerialDeviceConfig {
    pub name: String,
    pub port: String,
    pub baud_rate: u32,
    #[serde(default = "default_serial_timeout")]
    pub timeout_ms: u64,
    #[serde(default)]
    pub reconnect: ReconnectConfig,
}
fn default_serial_timeout() -> u64 {
    100
}
impl Default for ReconnectConfig {
    fn default() -> Self {
        Self {
            initial_delay_ms: 100,
            maximum_delay_ms: 5_000,
        }
    }
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MotorConfig {
    pub name: String,
    pub joint: String,
    pub bus: String,
    pub id: u8,
    pub model: String,
    #[serde(default = "default_direction")]
    pub direction: i8,
    #[serde(default)]
    pub limits: MotorLimits,
    #[serde(default)]
    pub health: MotorHealth,
    #[serde(default)]
    pub command_policy: MotorCommandPolicy,
    #[serde(default)]
    pub fault_damping: MotorFaultDamping,
}
fn default_direction() -> i8 {
    1
}

#[derive(Clone, Debug, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct MotorLimits {
    pub position_min: f64,
    pub position_max: f64,
    pub velocity_abs: Option<f64>,
    pub torque_abs: Option<f64>,
    pub kp_max: Option<f64>,
    pub kd_max: Option<f64>,
}
impl Default for MotorLimits {
    fn default() -> Self {
        Self {
            position_min: -12.57,
            position_max: 12.57,
            velocity_abs: None,
            torque_abs: None,
            kp_max: None,
            kd_max: None,
        }
    }
}

#[derive(Clone, Debug, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct MotorHealth {
    pub safe_position_min: f64,
    pub safe_position_max: f64,
    pub temperature_warning_c: f64,
    pub maximum_temperature_c: f64,
    pub temperature_recovery_c: f64,
    pub temperature_recovery_duration_ms: u64,
}
impl Default for MotorHealth {
    fn default() -> Self {
        Self {
            safe_position_min: -12.57,
            safe_position_max: 12.57,
            temperature_warning_c: 60.0,
            maximum_temperature_c: 80.0,
            temperature_recovery_c: 75.0,
            temperature_recovery_duration_ms: 2_000,
        }
    }
}

#[derive(Clone, Debug, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct MotorCommandPolicy {
    pub maximum_position_step: f64,
    pub limit_action: CommandLimitAction,
    pub limit_severity: CommandViolationSeverity,
    pub position_step_action: CommandLimitAction,
    pub position_step_severity: CommandViolationSeverity,
}

impl Default for MotorCommandPolicy {
    fn default() -> Self {
        Self {
            maximum_position_step: 0.5,
            limit_action: CommandLimitAction::Reject,
            limit_severity: CommandViolationSeverity::Warning,
            position_step_action: CommandLimitAction::Reject,
            position_step_severity: CommandViolationSeverity::Fault,
        }
    }
}

#[derive(Clone, Debug, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct MotorFaultDamping {
    pub kd: f64,
}

impl Default for MotorFaultDamping {
    fn default() -> Self {
        Self { kd: 2.0 }
    }
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum CommandLimitAction {
    Clamp,
    Reject,
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum CommandViolationSeverity {
    Warning,
    Fault,
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum FaultResponse {
    None,
    Damping,
    Disable,
}
fn default_response() -> FaultResponse {
    FaultResponse::Damping
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GroupConfig {
    pub name: String,
    pub motors: Vec<String>,
    #[serde(default = "default_control_rate")]
    pub control_rate_hz: f64,
    #[serde(default = "default_command_timeout")]
    pub command_timeout_ms: u64,
    #[serde(default = "default_feedback_timeout")]
    pub feedback_timeout_ms: u64,
    #[serde(default = "default_enable_timeout")]
    pub enable_timeout_ms: u64,
    #[serde(default = "default_retries")]
    pub enable_retries: u32,
    #[serde(default)]
    pub fault_responses: GroupFaultResponses,
    #[serde(default)]
    pub damping_response: DampingResponseConfig,
}
fn default_control_rate() -> f64 {
    500.0
}
fn default_command_timeout() -> u64 {
    100
}
fn default_feedback_timeout() -> u64 {
    100
}
fn default_enable_timeout() -> u64 {
    500
}
fn default_retries() -> u32 {
    3
}

/// Group-level policy selected from typed fault causes.
#[derive(Clone, Debug, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct GroupFaultResponses {
    pub command_limit: FaultResponse,
    pub command_position_step: FaultResponse,
    pub command_timeout: FaultResponse,
    pub safe_position: FaultResponse,
    pub stale_feedback: FaultResponse,
    pub temperature: FaultResponse,
    pub reported_motor_fault: FaultResponse,
    pub mode_status: FaultResponse,
    pub communication: FaultResponse,
    pub enable: FaultResponse,
    pub start: FaultResponse,
    pub stop: FaultResponse,
    pub transmission: FaultResponse,
    pub lease_loss: FaultResponse,
    pub manual: FaultResponse,
    pub disable: FaultResponse,
    pub exit: FaultResponse,
}

impl Default for GroupFaultResponses {
    fn default() -> Self {
        Self {
            command_limit: default_response(),
            command_position_step: default_response(),
            command_timeout: default_response(),
            safe_position: default_response(),
            stale_feedback: default_response(),
            temperature: default_response(),
            reported_motor_fault: default_response(),
            mode_status: default_response(),
            communication: default_response(),
            enable: default_response(),
            start: default_response(),
            stop: default_response(),
            transmission: default_response(),
            lease_loss: default_response(),
            manual: default_response(),
            disable: default_response(),
            exit: default_response(),
        }
    }
}

#[derive(Clone, Debug, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct DampingResponseConfig {
    pub send_count: u32,
    pub send_rate_hz: f64,
}

impl Default for DampingResponseConfig {
    fn default() -> Self {
        Self {
            send_count: 5,
            send_rate_hz: 100.0,
        }
    }
}

#[derive(Clone, Debug, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct ImuConfig {
    pub name: String,
    pub enabled: bool,
    pub serial_device: String,
    pub model: String,
    pub stale_timeout_ms: u64,
    pub output_rate_hz: u32,
    pub quaternion: bool,
    pub gyro: bool,
    pub acceleration: bool,
    pub device_timestamp: bool,
    pub configure_on_start: bool,
}
impl Default for ImuConfig {
    fn default() -> Self {
        Self {
            name: "imu".into(),
            enabled: true,
            serial_device: "imu_serial".into(),
            model: "EBIMU".into(),
            stale_timeout_ms: 500,
            output_rate_hz: 500,
            quaternion: true,
            gyro: true,
            acceleration: true,
            device_timestamp: true,
            configure_on_start: true,
        }
    }
}
