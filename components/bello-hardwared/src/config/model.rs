use std::collections::HashMap;
use std::net::SocketAddr;
use std::path::PathBuf;

use robstride_protocol::{CONTROL_POSITION_MAX, CONTROL_POSITION_MIN};
use serde::Deserialize;

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Config {
    #[serde(default)]
    pub runtime: RuntimeConfig,
    #[serde(default)]
    pub grpc: GrpcConfig,
    #[serde(default)]
    pub serial_devices: Vec<SerialDeviceConfig>,
    #[serde(default)]
    pub imus: Vec<ImuConfig>,
    pub buses: Vec<BusConfig>,
    pub motors: Vec<MotorConfig>,
    pub motor_groups: Vec<GroupConfig>,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct DeploymentConfig {
    pub description: PathBuf,
    #[serde(default)]
    pub runtime: RuntimeConfig,
    #[serde(default)]
    pub grpc: GrpcConfig,
    #[serde(default)]
    pub bus_interfaces: HashMap<String, String>,
    #[serde(default)]
    pub serial_ports: HashMap<String, String>,
    pub enabled_groups: Vec<String>,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct HardwareDescription {
    pub logical_buses: Vec<LogicalBusConfig>,
    #[serde(default)]
    pub serial_devices: Vec<LogicalSerialDeviceConfig>,
    #[serde(default)]
    pub imus: Vec<ImuConfig>,
    pub motors: Vec<MotorConfig>,
    pub motor_groups: Vec<GroupConfig>,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct LogicalBusConfig {
    pub name: String,
    #[serde(default = "default_host_id")]
    pub host_id: u8,
    #[serde(default = "default_bitrate")]
    pub bitrate: u32,
    #[serde(default = "default_bus_utilization")]
    pub maximum_utilization: f64,
    #[serde(default)]
    pub reconnect: ReconnectConfig,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct LogicalSerialDeviceConfig {
    pub name: String,
    pub baud_rate: u32,
    #[serde(default = "default_serial_timeout")]
    pub timeout_ms: u64,
    #[serde(default)]
    pub reconnect: ReconnectConfig,
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
    pub listen: GrpcListenConfig,
    pub tls: Option<GrpcTlsConfig>,
    pub default_lease_ms: u64,
    pub maximum_lease_ms: u64,
}
impl Default for GrpcConfig {
    fn default() -> Self {
        Self {
            listen: GrpcListenConfig::default(),
            tls: None,
            default_lease_ms: 1000,
            maximum_lease_ms: 5000,
        }
    }
}

#[derive(Clone, Debug, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case", deny_unknown_fields)]
pub enum GrpcListenConfig {
    Tcp {
        address: SocketAddr,
    },
    Unix {
        path: PathBuf,
        #[serde(default = "default_unix_socket_mode")]
        mode: String,
    },
}

impl Default for GrpcListenConfig {
    fn default() -> Self {
        Self::Tcp {
            address: "127.0.0.1:50051".parse().expect("valid default address"),
        }
    }
}

fn default_unix_socket_mode() -> String {
    "0660".to_owned()
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GrpcTlsConfig {
    pub certificate: PathBuf,
    pub private_key: PathBuf,
    pub client_ca: Option<PathBuf>,
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
    pub coordinate: String,
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
    #[serde(default = "default_fault_damping_kd")]
    pub fault_damping_kd: f64,
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
            position_min: CONTROL_POSITION_MIN,
            position_max: CONTROL_POSITION_MAX,
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
            safe_position_min: CONTROL_POSITION_MIN,
            safe_position_max: CONTROL_POSITION_MAX,
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
    pub limit: CommandBehavior,
    pub position_step: CommandBehavior,
}

impl Default for MotorCommandPolicy {
    fn default() -> Self {
        Self {
            maximum_position_step: 0.5,
            limit: CommandBehavior::Reject,
            position_step: CommandBehavior::Fault,
        }
    }
}

fn default_fault_damping_kd() -> f64 {
    2.0
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum CommandBehavior {
    Clamp,
    Reject,
    Fault,
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum FaultResponse {
    Damping,
    Disable,
}

#[derive(Clone, Copy, Debug, Deserialize, Hash, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum FaultCause {
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
    pub fault_policy: FaultPolicyConfig,
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

#[derive(Clone, Debug, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct FaultPolicyConfig {
    pub default: FaultResponse,
    pub overrides: HashMap<FaultCause, FaultResponse>,
}

impl Default for FaultPolicyConfig {
    fn default() -> Self {
        Self {
            default: FaultResponse::Damping,
            overrides: HashMap::new(),
        }
    }
}

impl FaultPolicyConfig {
    pub fn response(&self, cause: FaultCause) -> FaultResponse {
        self.overrides.get(&cause).copied().unwrap_or(self.default)
    }
}

#[derive(Clone, Debug, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct ImuConfig {
    pub name: String,
    pub serial_device: String,
    pub stale_timeout_ms: u64,
    pub output_rate_hz: u32,
}
impl Default for ImuConfig {
    fn default() -> Self {
        Self {
            name: "imu".into(),
            serial_device: "imu_serial".into(),
            stale_timeout_ms: 500,
            output_rate_hz: 500,
        }
    }
}
