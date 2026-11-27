use std::collections::{HashMap, HashSet};
use std::net::SocketAddr;
use std::path::PathBuf;

use garde::Validate;
use robstride_protocol::{
    CONTROL_POSITION_MAX, CONTROL_POSITION_MIN, CONTROL_POSITION_SPAN, MAX_MOTOR_ID,
};
use serde::Deserialize;

fn finite(value: &f64, _: &()) -> garde::Result {
    if !value.is_finite() {
        return Err(garde::Error::new("must be finite"));
    }
    Ok(())
}

#[derive(Clone, Copy, Debug, Default, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub(crate) enum EmergencyStopMode {
    #[default]
    Hardware,
    Damping,
}

#[derive(Clone, Debug, Deserialize, Validate)]
#[garde(allow_unvalidated)]
#[serde(deny_unknown_fields)]
pub(crate) struct PowerBoardConfig {
    #[garde(length(min = 1))]
    pub name: String,
    #[garde(length(min = 1))]
    pub serial_device: String,
    pub profile: crate::hardware::power_board::protocol::Profile,
    pub sync_period_ms: Option<u64>,
    #[serde(default)]
    pub emergency_stop: EmergencyStopMode,
    #[serde(default = "default_power_stale_timeout")]
    #[garde(range(min = 250, max = 60_000))]
    pub stale_timeout_ms: u64,
}

fn default_power_stale_timeout() -> u64 {
    3000
}

#[derive(Clone, Debug, Deserialize, Validate)]
#[garde(allow_unvalidated)]
#[serde(deny_unknown_fields)]
pub(crate) struct Config {
    #[serde(skip)]
    pub(crate) robot_description: PathBuf,
    #[serde(skip)]
    pub(crate) disabled_groups: HashSet<String>,
    #[serde(default)]
    pub simulation: bool,
    #[serde(default)]
    #[garde(dive)]
    pub runtime: RuntimeConfig,
    #[serde(default)]
    pub grpc: GrpcConfig,
    #[serde(default)]
    pub cameras: Option<bello_hardware_api::camera::config::Source>,
    #[serde(default)]
    pub serial_devices: Vec<SerialDeviceConfig>,
    #[serde(default)]
    pub imus: Vec<ImuConfig>,
    #[serde(default)]
    pub power_boards: Vec<PowerBoardConfig>,
    #[garde(length(min = 1))]
    pub buses: Vec<BusConfig>,
    #[garde(length(min = 1))]
    pub motors: Vec<MotorConfig>,
    #[garde(length(min = 1))]
    pub motor_groups: Vec<GroupConfig>,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct DeploymentConfig {
    pub description: PathBuf,
    #[serde(default)]
    pub simulation: bool,
    #[serde(default)]
    pub runtime: RuntimeConfig,
    #[serde(default)]
    pub grpc: GrpcConfig,
    #[serde(default)]
    pub cameras: Option<bello_hardware_api::camera::config::Source>,
    #[serde(default)]
    pub bus_transports: HashMap<String, BusTransportConfig>,
    #[serde(default)]
    pub serial_ports: HashMap<String, String>,
    pub enabled_groups: Vec<String>,
    #[serde(default)]
    pub enabled_power_boards: Vec<String>,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct HardwareDescription {
    pub robot_description: PathBuf,
    pub logical_buses: Vec<LogicalBusConfig>,
    #[serde(default)]
    pub serial_devices: Vec<LogicalSerialDeviceConfig>,
    #[serde(default)]
    pub imus: Vec<ImuConfig>,
    #[serde(default)]
    pub power_boards: Vec<PowerBoardConfig>,
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

#[derive(Clone, Debug, Deserialize, Validate)]
#[serde(default, deny_unknown_fields)]
pub struct RuntimeConfig {
    #[garde(range(min = 64, max = 1_000_000))]
    pub telemetry_capacity: usize,
    #[garde(dive)]
    pub motor_velocity_filter: MotorVelocityFilterConfig,
    #[garde(dive)]
    pub imu_filter: ImuFilterConfig,
}
impl Default for RuntimeConfig {
    fn default() -> Self {
        Self {
            telemetry_capacity: 4096,
            motor_velocity_filter: MotorVelocityFilterConfig::default(),
            imu_filter: ImuFilterConfig::default(),
        }
    }
}

#[derive(Clone, Copy, Debug, Deserialize, Validate)]
#[serde(default, deny_unknown_fields)]
pub struct MotorVelocityFilterConfig {
    #[garde(custom(finite), range(min = 0.0_f64.next_up()))]
    pub cutoff_hz: f64,
}

impl Default for MotorVelocityFilterConfig {
    fn default() -> Self {
        Self { cutoff_hz: 10.0 }
    }
}

#[derive(Clone, Copy, Debug, Deserialize, Validate)]
#[serde(default, deny_unknown_fields)]
pub struct ImuFilterConfig {
    #[garde(custom(finite), range(min = 0.0_f64.next_up()))]
    pub orientation_cutoff_hz: f64,
    #[garde(custom(finite), range(min = 0.0_f64.next_up()))]
    pub gyro_cutoff_hz: f64,
    #[garde(custom(finite), range(min = 0.0_f64.next_up()))]
    pub acceleration_cutoff_hz: f64,
}

impl Default for ImuFilterConfig {
    fn default() -> Self {
        Self {
            orientation_cutoff_hz: 10.0,
            gyro_cutoff_hz: 10.0,
            acceleration_cutoff_hz: 10.0,
        }
    }
}

#[derive(Clone, Debug, Deserialize, Validate)]
#[garde(allow_unvalidated)]
#[serde(default, deny_unknown_fields)]
pub struct GrpcConfig {
    #[garde(length(min = 1))]
    pub listen: Vec<GrpcListenConfig>,
    pub tls: Option<GrpcTlsConfig>,
    #[garde(range(min = 20, max = 60_000))]
    pub default_lease_ms: u64,
    #[garde(range(max = 300_000))]
    pub maximum_lease_ms: u64,
}
impl Default for GrpcConfig {
    fn default() -> Self {
        Self {
            listen: vec![GrpcListenConfig::default()],
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

#[derive(Clone, Debug, Deserialize, Validate)]
#[garde(allow_unvalidated)]
#[serde(deny_unknown_fields)]
pub struct BusConfig {
    #[garde(length(min = 1))]
    pub name: String,
    pub transport: BusTransportConfig,
    #[serde(default = "default_host_id")]
    pub host_id: u8,
    #[serde(default = "default_bitrate")]
    #[garde(range(min = 10_000, max = 1_000_000))]
    pub bitrate: u32,
    #[serde(default = "default_bus_utilization")]
    #[garde(custom(finite), range(min = 0.0_f64.next_up(), max = 1.0))]
    pub maximum_utilization: f64,
    #[serde(default)]
    pub reconnect: ReconnectConfig,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Eq, Hash)]
#[serde(tag = "type", rename_all = "snake_case", deny_unknown_fields)]
pub enum BusTransportConfig {
    Socketcan { interface: String },
    Gsusb { serial: String, channel: u8 },
    Sim(bello_sim_can::SimBus),
}

impl BusTransportConfig {
    pub fn label(&self) -> String {
        match self {
            Self::Socketcan { interface } => format!("socketcan:{interface}"),
            Self::Sim(bus) => format!("sim:{}:{}", bus.daemon_port, bus.simulator_port),
            Self::Gsusb { serial, channel } => format!("gsusb:{serial}:{channel}"),
        }
    }
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

#[derive(Clone, Debug, Deserialize, Validate)]
#[serde(default, deny_unknown_fields)]
pub struct ReconnectConfig {
    #[garde(range(min = 1))]
    pub initial_delay_ms: u64,
    #[garde(range(max = 60_000))]
    pub maximum_delay_ms: u64,
}

#[derive(Clone, Debug, Deserialize, Validate)]
#[garde(allow_unvalidated)]
#[serde(deny_unknown_fields)]
pub struct SerialDeviceConfig {
    #[garde(length(min = 1))]
    pub name: String,
    #[garde(length(min = 1))]
    pub port: String,
    #[garde(range(min = 1))]
    pub baud_rate: u32,
    #[serde(default = "default_serial_timeout")]
    #[garde(range(min = 1, max = 500))]
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

#[derive(Clone, Debug, Deserialize, Validate)]
#[garde(allow_unvalidated)]
#[serde(deny_unknown_fields)]
pub struct MotorConfig {
    #[garde(length(min = 1))]
    pub name: String,
    #[garde(length(min = 1))]
    pub coordinate: String,
    #[garde(length(min = 1))]
    pub bus: String,
    #[garde(range(max = MAX_MOTOR_ID))]
    pub id: u8,
    pub model: String,
    #[serde(default = "default_direction")]
    pub direction: i8,
    #[serde(default)]
    #[garde(dive)]
    pub limits: MotorLimits,
    #[serde(default)]
    #[garde(dive)]
    pub health: MotorHealth,
    #[serde(default)]
    #[garde(dive)]
    pub command_policy: MotorCommandPolicy,
    #[serde(default = "default_fault_damping_kd")]
    #[garde(custom(finite), range(min = 0.0_f64.next_up()))]
    pub fault_damping_kd: f64,
}
fn default_direction() -> i8 {
    1
}

#[derive(Clone, Debug, Deserialize, Validate)]
#[serde(default, deny_unknown_fields)]
pub struct MotorLimits {
    #[garde(custom(finite), range(min = CONTROL_POSITION_MIN, max = CONTROL_POSITION_MAX))]
    pub position_min: f64,
    #[garde(custom(finite), range(min = CONTROL_POSITION_MIN, max = CONTROL_POSITION_MAX))]
    pub position_max: f64,
    #[garde(inner(custom(finite), range(min = 0.0_f64.next_up())))]
    pub velocity_abs: Option<f64>,
    #[garde(inner(custom(finite), range(min = 0.0_f64.next_up())))]
    pub torque_abs: Option<f64>,
}
impl Default for MotorLimits {
    fn default() -> Self {
        Self {
            position_min: CONTROL_POSITION_MIN,
            position_max: CONTROL_POSITION_MAX,
            velocity_abs: None,
            torque_abs: None,
        }
    }
}

#[derive(Clone, Debug, Deserialize, Validate)]
#[serde(default, deny_unknown_fields)]
pub struct MotorHealth {
    #[garde(custom(finite), range(min = CONTROL_POSITION_MIN, max = CONTROL_POSITION_MAX))]
    pub safe_position_min: f64,
    #[garde(custom(finite), range(min = CONTROL_POSITION_MIN, max = CONTROL_POSITION_MAX))]
    pub safe_position_max: f64,
    #[garde(custom(finite), range(min = CONTROL_POSITION_SPAN / u16::MAX as f64, max = 0.4))]
    pub feedback_position_tolerance: f64,
    #[garde(custom(finite), range(min = 0.0_f64.next_up()))]
    pub temperature_warning_c: f64,
    #[garde(custom(finite), range(min = 0.0_f64.next_up(), max = 200.0))]
    pub maximum_temperature_c: f64,
    #[garde(custom(finite), range(min = 0.0_f64.next_up()))]
    pub temperature_recovery_c: f64,
    #[garde(range(min = 1, max = 60_000))]
    pub temperature_recovery_duration_ms: u64,
}
impl Default for MotorHealth {
    fn default() -> Self {
        Self {
            safe_position_min: CONTROL_POSITION_MIN,
            safe_position_max: CONTROL_POSITION_MAX,
            feedback_position_tolerance: default_feedback_position_tolerance(),
            temperature_warning_c: 60.0,
            maximum_temperature_c: 80.0,
            temperature_recovery_c: 75.0,
            temperature_recovery_duration_ms: 2_000,
        }
    }
}

fn default_feedback_position_tolerance() -> f64 {
    2.0 * CONTROL_POSITION_SPAN / u16::MAX as f64
}

#[derive(Clone, Debug, Deserialize, Validate)]
#[garde(allow_unvalidated)]
#[serde(default, deny_unknown_fields)]
pub struct MotorCommandPolicy {
    #[garde(custom(finite), range(min = 0.0_f64.next_up(), max = CONTROL_POSITION_SPAN))]
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
    10.0
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum CommandBehavior {
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
    SafePosition,
    StaleFeedback,
    Temperature,
    ReportedMotorFault,
    ModeStatus,
    Communication,
    Enable,
    Transmission,
    Disable,
    Recover,
    Exit,
    Internal,
}

#[derive(Clone, Copy, Debug, Default, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum CommandTransmission {
    #[default]
    OnUpdate,
    Periodic,
}

#[derive(Clone, Debug, Deserialize, Validate)]
#[garde(allow_unvalidated)]
#[serde(deny_unknown_fields)]
pub struct GroupConfig {
    #[garde(length(min = 1))]
    pub name: String,
    #[garde(length(min = 1))]
    pub motors: Vec<String>,
    #[serde(default = "default_parallel_feedback_position_tolerance")]
    #[garde(custom(finite), range(min = 0.0_f64.next_up(), max = 0.4))]
    pub parallel_feedback_position_tolerance: f64,
    #[serde(default = "default_control_rate")]
    #[garde(custom(finite), range(min = 1.0, max = 2_000.0))]
    pub control_rate_hz: f64,
    #[serde(default)]
    pub command_refresh_rate_hz: Option<f64>,
    #[serde(default)]
    pub command_transmission: CommandTransmission,
    #[serde(default = "default_periodic_feedback")]
    pub periodic_feedback_during_control: bool,
    #[serde(default = "default_command_timeout")]
    #[garde(range(min = 1, max = 10_000))]
    pub command_timeout_ms: u64,
    #[serde(default = "default_feedback_timeout")]
    #[garde(range(min = 1, max = 10_000))]
    pub feedback_timeout_ms: u64,
    #[serde(default = "default_enable_timeout")]
    #[garde(range(min = 1, max = 30_000))]
    pub enable_timeout_ms: u64,
    #[serde(default = "default_retries")]
    #[garde(range(min = 1, max = 100))]
    pub enable_retries: u32,
    #[serde(default)]
    pub fault_policy: FaultPolicyConfig,
}
fn default_parallel_feedback_position_tolerance() -> f64 {
    0.02
}
fn default_periodic_feedback() -> bool {
    true
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

#[derive(Clone, Debug, Deserialize, Validate)]
#[serde(default, deny_unknown_fields)]
pub struct ImuConfig {
    #[garde(length(min = 1))]
    pub name: String,
    #[garde(length(min = 1))]
    pub serial_device: String,
    #[garde(length(min = 1))]
    pub source_frame: String,
    #[garde(length(min = 1))]
    pub output_frame: String,
    #[garde(range(min = 1, max = 60_000))]
    pub stale_timeout_ms: u64,
    #[garde(range(min = 1, max = 1000))]
    pub output_rate_hz: u32,
}
impl Default for ImuConfig {
    fn default() -> Self {
        Self {
            name: "imu".into(),
            serial_device: "imu_serial".into(),
            source_frame: String::new(),
            output_frame: String::new(),
            stale_timeout_ms: 500,
            output_rate_hz: 500,
        }
    }
}
