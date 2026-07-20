use std::fmt;
use std::sync::Arc;

use crate::inventory::{Bus, GroupDefinition, Imu, Inventory, Motor, SerialDevice};
use crate::{ConnectOptions, Error, GroupState, HardwareClient, RobotStates, StateStreamOptions};

/// Connected robot with a validated, cached hardware inventory.
#[derive(Clone)]
pub struct Robot {
    pub(crate) inner: Arc<RobotInner>,
}

pub(crate) struct RobotInner {
    pub(crate) hardware: HardwareClient,
    pub(crate) inventory: Inventory,
}

impl Robot {
    /// Connects to the daemon and validates its complete inventory before returning.
    pub async fn connect(endpoint: impl Into<String>) -> Result<Self, Error> {
        let hardware = HardwareClient::connect(endpoint).await?;
        Self::from_hardware_client(hardware).await
    }

    pub async fn connect_with(options: ConnectOptions) -> Result<Self, Error> {
        let hardware = HardwareClient::connect_with(options).await?;
        Self::from_hardware_client(hardware).await
    }

    /// Constructs the ergonomic API over an already connected low-level client.
    pub async fn from_hardware_client(hardware: HardwareClient) -> Result<Self, Error> {
        let inventory = Inventory::from_proto(hardware.inventory().await?)?;
        Ok(Self {
            inner: Arc::new(RobotInner {
                hardware,
                inventory,
            }),
        })
    }

    pub fn group(&self, name: impl AsRef<str>) -> Result<MotorGroup, Error> {
        let name = name.as_ref();
        let definition = self
            .inner
            .inventory
            .groups
            .get(name)
            .cloned()
            .ok_or_else(|| Error::UnknownGroup(name.into()))?;
        Ok(MotorGroup {
            robot: self.clone(),
            definition,
        })
    }

    pub fn group_names(&self) -> impl ExactSizeIterator<Item = &str> {
        self.inner.inventory.groups.keys().map(String::as_str)
    }

    pub fn motors(&self) -> impl ExactSizeIterator<Item = &Motor> {
        self.inner.inventory.motors.values()
    }

    /// Returns configured CAN buses in stable name order.
    pub fn buses(&self) -> impl ExactSizeIterator<Item = &Bus> {
        self.inner.inventory.buses.values()
    }

    /// Returns configured serial transports in stable name order.
    pub fn serial_devices(&self) -> impl ExactSizeIterator<Item = &SerialDevice> {
        self.inner.inventory.serial_devices.values()
    }

    /// Returns configured IMUs in stable name order.
    pub fn imus(&self) -> impl ExactSizeIterator<Item = &Imu> {
        self.inner.inventory.imus.values()
    }

    /// Opens an aggregated stream of the latest robot state.
    pub async fn states(&self) -> Result<RobotStates, Error> {
        self.states_with(StateStreamOptions::default()).await
    }

    pub async fn states_with(&self, options: StateStreamOptions) -> Result<RobotStates, Error> {
        RobotStates::open(self.clone(), options).await
    }

    /// Explicit escape hatch for generated gRPC/protobuf-oriented workflows.
    pub fn raw_client(&self) -> HardwareClient {
        self.inner.hardware.clone()
    }
}

impl fmt::Debug for Robot {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter
            .debug_struct("Robot")
            .field("buses", &self.inner.inventory.buses.len())
            .field("groups", &self.inner.inventory.groups.len())
            .field("motors", &self.inner.inventory.motors.len())
            .field("serial_devices", &self.inner.inventory.serial_devices.len())
            .field("imus", &self.inner.inventory.imus.len())
            .finish_non_exhaustive()
    }
}

/// Validated view of one configured motor group.
#[derive(Clone)]
pub struct MotorGroup {
    pub(crate) robot: Robot,
    pub(crate) definition: Arc<GroupDefinition>,
}

impl MotorGroup {
    pub fn name(&self) -> &str {
        &self.definition.name
    }

    pub fn control_rate_hz(&self) -> f64 {
        self.definition.control_rate_hz
    }

    pub fn initial_state(&self) -> GroupState {
        self.definition.initial_state
    }

    pub fn motors(&self) -> &[Motor] {
        &self.definition.motors
    }

    pub fn coordinate_names(&self) -> impl ExactSizeIterator<Item = &str> {
        self.definition.motors.iter().map(Motor::coordinate)
    }
}

impl fmt::Debug for MotorGroup {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter
            .debug_struct("MotorGroup")
            .field("name", &self.name())
            .field("control_rate_hz", &self.control_rate_hz())
            .field("motors", &self.motors())
            .finish()
    }
}
