use std::collections::BTreeMap;
use std::time::Duration;

use bello_hardware_api::watch_telemetry_response::Payload;
use tonic::Code;

use crate::inventory::DeviceStatus;
use crate::{Error, FaultCause, GroupState, Robot, TelemetryFilter, TelemetryStream};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Quaternion {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Vector3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

/// Latest feedback received for one configured motor.
#[derive(Clone, Debug, PartialEq)]
pub struct MotorSnapshot {
    pub monotonic_ns: u64,
    pub group: String,
    pub motor: String,
    pub joint: String,
    pub position: f64,
    pub velocity: f64,
    pub torque: f64,
    pub temperature: f64,
    pub fault_bits: u32,
    pub mode_status: u32,
    pub feedback_age: Duration,
}

/// Latest feedback received for one configured IMU.
#[derive(Clone, Debug, PartialEq)]
pub struct ImuSnapshot {
    pub monotonic_ns: u64,
    pub imu: String,
    pub orientation: Quaternion,
    pub angular_velocity: Vector3,
    pub linear_acceleration: Vector3,
    pub device_timestamp: Duration,
    pub sample_age: Duration,
}

/// Latest lifecycle and timing event received for one motor group.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct GroupSnapshot {
    pub monotonic_ns: u64,
    pub group: String,
    pub state: GroupState,
    pub fault_detail: String,
    pub fault_cause: FaultCause,
    pub command_count: u64,
    pub control_tick: u64,
    pub jitter_us: i64,
}

/// Latest health event for one daemon component.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct HealthSnapshot {
    pub monotonic_ns: u64,
    pub component: String,
    pub status: Option<DeviceStatus>,
    pub dropped_events: u64,
}

/// Latest-value aggregate built from the event-based telemetry stream.
#[derive(Clone, Debug)]
pub struct RobotState {
    robot: Robot,
    generation: u64,
    monotonic_ns: u64,
    motors: BTreeMap<String, MotorSnapshot>,
    groups: BTreeMap<String, GroupSnapshot>,
    imus: BTreeMap<String, ImuSnapshot>,
    health: BTreeMap<String, HealthSnapshot>,
}

impl RobotState {
    fn new(robot: Robot) -> Self {
        Self {
            robot,
            generation: 0,
            monotonic_ns: 0,
            motors: BTreeMap::new(),
            groups: BTreeMap::new(),
            imus: BTreeMap::new(),
            health: BTreeMap::new(),
        }
    }

    pub fn generation(&self) -> u64 {
        self.generation
    }

    pub fn monotonic_ns(&self) -> u64 {
        self.monotonic_ns
    }

    pub fn motor(&self, name: impl AsRef<str>) -> Result<&MotorSnapshot, Error> {
        let name = name.as_ref();
        if !self.robot.inner.inventory.motors.contains_key(name) {
            return Err(Error::StateUnavailable {
                kind: "motor",
                name: name.into(),
            });
        }
        self.motors
            .get(name)
            .ok_or_else(|| Error::StateUnavailable {
                kind: "motor",
                name: name.into(),
            })
    }

    pub fn joint(&self, name: impl AsRef<str>) -> Result<&MotorSnapshot, Error> {
        let name = name.as_ref();
        let motor =
            self.robot
                .inner
                .inventory
                .joints
                .get(name)
                .ok_or_else(|| Error::StateUnavailable {
                    kind: "joint",
                    name: name.into(),
                })?;
        self.motors
            .get(motor)
            .ok_or_else(|| Error::StateUnavailable {
                kind: "joint",
                name: name.into(),
            })
    }

    pub fn group(&self, name: impl AsRef<str>) -> Result<&GroupSnapshot, Error> {
        let name = name.as_ref();
        self.groups
            .get(name)
            .ok_or_else(|| Error::StateUnavailable {
                kind: "motor group",
                name: name.into(),
            })
    }

    pub fn imu(&self, name: impl AsRef<str>) -> Result<&ImuSnapshot, Error> {
        let name = name.as_ref();
        self.imus.get(name).ok_or_else(|| Error::StateUnavailable {
            kind: "IMU",
            name: name.into(),
        })
    }

    pub fn health(&self, component: impl AsRef<str>) -> Option<&HealthSnapshot> {
        self.health.get(component.as_ref())
    }

    pub fn motors(&self) -> impl Iterator<Item = &MotorSnapshot> {
        self.motors.values()
    }

    pub fn groups(&self) -> impl Iterator<Item = &GroupSnapshot> {
        self.groups.values()
    }

    pub fn imus(&self) -> impl Iterator<Item = &ImuSnapshot> {
        self.imus.values()
    }

    fn clear_after_reconnect(&mut self) {
        self.generation = self.generation.saturating_add(1);
        self.monotonic_ns = 0;
        self.motors.clear();
        self.groups.clear();
        self.imus.clear();
        self.health.clear();
    }

    fn apply(&mut self, event: crate::TelemetryEvent) -> Result<(), Error> {
        self.monotonic_ns = event.monotonic_ns;
        match event.payload {
            Payload::Motor(motor) => {
                let configured = self
                    .robot
                    .inner
                    .inventory
                    .motors
                    .get(&motor.motor_name)
                    .ok_or_else(|| {
                        Error::Protocol(format!(
                            "telemetry references unknown motor {:?}",
                            motor.motor_name
                        ))
                    })?;
                if configured.group() != motor.group {
                    return Err(Error::Protocol(format!(
                        "motor {:?} telemetry names group {:?}, expected {:?}",
                        motor.motor_name,
                        motor.group,
                        configured.group()
                    )));
                }
                self.motors.insert(
                    motor.motor_name.clone(),
                    MotorSnapshot {
                        monotonic_ns: event.monotonic_ns,
                        group: motor.group,
                        motor: motor.motor_name,
                        joint: configured.joint().into(),
                        position: motor.position,
                        velocity: motor.velocity,
                        torque: motor.torque,
                        temperature: motor.temperature,
                        fault_bits: motor.fault_bits,
                        mode_status: motor.mode_status,
                        feedback_age: Duration::from_millis(motor.feedback_age_ms),
                    },
                );
            }
            Payload::Imu(imu) => {
                if !self.robot.inner.inventory.imus.contains(&imu.imu_name) {
                    return Err(Error::Protocol(format!(
                        "telemetry references unknown IMU {:?}",
                        imu.imu_name
                    )));
                }
                let orientation = imu.orientation.ok_or_else(|| {
                    Error::Protocol(format!("IMU {:?} has no orientation", imu.imu_name))
                })?;
                let angular_velocity = imu.angular_velocity.ok_or_else(|| {
                    Error::Protocol(format!("IMU {:?} has no angular velocity", imu.imu_name))
                })?;
                let linear_acceleration = imu.linear_acceleration.ok_or_else(|| {
                    Error::Protocol(format!("IMU {:?} has no linear acceleration", imu.imu_name))
                })?;
                self.imus.insert(
                    imu.imu_name.clone(),
                    ImuSnapshot {
                        monotonic_ns: event.monotonic_ns,
                        imu: imu.imu_name,
                        orientation: Quaternion {
                            x: orientation.x,
                            y: orientation.y,
                            z: orientation.z,
                            w: orientation.w,
                        },
                        angular_velocity: Vector3 {
                            x: angular_velocity.x,
                            y: angular_velocity.y,
                            z: angular_velocity.z,
                        },
                        linear_acceleration: Vector3 {
                            x: linear_acceleration.x,
                            y: linear_acceleration.y,
                            z: linear_acceleration.z,
                        },
                        device_timestamp: Duration::from_millis(imu.device_timestamp_ms),
                        sample_age: Duration::from_millis(imu.sample_age_ms),
                    },
                );
            }
            Payload::Group(group) => {
                if !self.robot.inner.inventory.groups.contains_key(&group.group) {
                    return Err(Error::Protocol(format!(
                        "telemetry references unknown motor group {:?}",
                        group.group
                    )));
                }
                self.groups.insert(
                    group.group.clone(),
                    GroupSnapshot {
                        monotonic_ns: event.monotonic_ns,
                        group: group.group,
                        state: GroupState::from_raw(group.state),
                        fault_detail: group.fault_detail,
                        fault_cause: FaultCause::from_raw(group.fault_cause),
                        command_count: group.command_count,
                        control_tick: group.control_tick,
                        jitter_us: group.jitter_us,
                    },
                );
            }
            Payload::Health(health) => {
                self.health.insert(
                    health.component.clone(),
                    HealthSnapshot {
                        monotonic_ns: event.monotonic_ns,
                        component: health.component,
                        status: health.device_status.map(Into::into),
                        dropped_events: health.dropped_events,
                    },
                );
            }
            Payload::CommandPolicy(_) => {}
        }
        Ok(())
    }
}

/// Read-only stream behavior. Motion sessions never use this reconnect policy.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct StateStreamOptions {
    reconnect_on_unavailable: bool,
    reconnect_delay: Duration,
}

impl Default for StateStreamOptions {
    fn default() -> Self {
        Self {
            reconnect_on_unavailable: true,
            reconnect_delay: Duration::from_millis(200),
        }
    }
}

impl StateStreamOptions {
    pub fn reconnect_on_unavailable(mut self, enabled: bool) -> Self {
        self.reconnect_on_unavailable = enabled;
        self
    }

    pub fn reconnect_delay(mut self, delay: Duration) -> Self {
        self.reconnect_delay = delay;
        self
    }

    fn validate(self) -> Result<Self, Error> {
        if self.reconnect_on_unavailable && self.reconnect_delay.is_zero() {
            Err(Error::InvalidArgument(
                "telemetry reconnect delay must be positive".into(),
            ))
        } else {
            Ok(self)
        }
    }
}

/// Aggregates every telemetry event into a latest-value [`RobotState`].
pub struct RobotStates {
    robot: Robot,
    options: StateStreamOptions,
    stream: TelemetryStream,
    state: RobotState,
}

impl RobotStates {
    pub(crate) async fn open(robot: Robot, options: StateStreamOptions) -> Result<Self, Error> {
        let options = options.validate()?;
        let stream = robot.inner.hardware.watch(state_filter()).await?;
        let state = RobotState::new(robot.clone());
        Ok(Self {
            robot,
            options,
            stream,
            state,
        })
    }

    pub fn latest(&self) -> &RobotState {
        &self.state
    }

    /// Applies the next event and returns the updated aggregate. Normal server EOF returns `None`.
    pub async fn changed(&mut self) -> Result<Option<&RobotState>, Error> {
        loop {
            match self.stream.message().await {
                Ok(Some(event)) => {
                    self.state.apply(event)?;
                    return Ok(Some(&self.state));
                }
                Ok(None) => return Ok(None),
                Err(error)
                    if self.options.reconnect_on_unavailable
                        && is_retryable_stream_error(&error) =>
                {
                    self.reconnect().await?;
                }
                Err(error) => return Err(error),
            }
        }
    }

    async fn reconnect(&mut self) -> Result<(), Error> {
        loop {
            tokio::time::sleep(self.options.reconnect_delay).await;
            match self.robot.inner.hardware.watch(state_filter()).await {
                Ok(stream) => {
                    self.stream = stream;
                    self.state.clear_after_reconnect();
                    return Ok(());
                }
                Err(error) if is_retryable_stream_error(&error) => {}
                Err(error) => return Err(error),
            }
        }
    }
}

fn state_filter() -> TelemetryFilter {
    TelemetryFilter::default()
        .include_motors(true)
        .include_imu(true)
        .include_health(true)
}

impl std::fmt::Debug for RobotStates {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter
            .debug_struct("RobotStates")
            .field("options", &self.options)
            .field("state", &self.state)
            .finish_non_exhaustive()
    }
}

fn is_retryable_stream_error(error: &Error) -> bool {
    error.rpc_status().is_some_and(|status| {
        matches!(
            status.code(),
            Code::Unavailable | Code::Cancelled | Code::DeadlineExceeded | Code::Unknown
        )
    })
}
