use std::fmt;
use std::sync::Arc;

use bello_hardware_api::ListHardwareRequest;
use bello_hardware_api::hardware_service_client::HardwareServiceClient;
use tonic::transport::Channel;

use crate::inventory::{GroupDefinition, Inventory, Motor};
use crate::{ConnectOptions, EnumValue, Error, GroupState, RobotStates, StateStreamOptions};

/// Connected robot with a cached hardware inventory.
#[derive(Clone)]
pub struct Robot {
    pub(crate) hardware: HardwareServiceClient<Channel>,
    pub(crate) inventory: Arc<Inventory>,
}

impl Robot {
    /// Connects to the daemon and caches its hardware inventory before returning.
    pub async fn connect(options: impl Into<ConnectOptions>) -> Result<Self, Error> {
        let channel = options.into().into_endpoint()?.connect().await?;
        let mut hardware = HardwareServiceClient::new(channel);
        let inventory = Arc::new(load_inventory(&mut hardware).await?);
        Ok(Self {
            hardware,
            inventory,
        })
    }

    /// Refreshes this handle's inventory. Earlier clones, groups, and readers retain their snapshot.
    pub async fn refresh_inventory(&mut self) -> Result<(), Error> {
        self.inventory = Arc::new(load_inventory(&mut self.hardware).await?);
        Ok(())
    }

    pub fn group(&self, name: impl AsRef<str>) -> Result<MotorGroup, Error> {
        let name = name.as_ref();
        let definition = self
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
        self.inventory.groups.keys().map(String::as_str)
    }

    pub fn motors(&self) -> impl ExactSizeIterator<Item = &Motor> {
        self.inventory.motors.values()
    }

    /// Returns configured CAN buses in stable name order.
    pub fn buses(&self) -> impl ExactSizeIterator<Item = &crate::BusInfo> {
        self.inventory.buses.values()
    }

    /// Returns configured serial transports in stable name order.
    pub fn serial_devices(&self) -> impl ExactSizeIterator<Item = &crate::SerialDeviceInfo> {
        self.inventory.serial_devices.values()
    }

    /// Returns configured IMUs in stable name order.
    pub fn imus(&self) -> impl ExactSizeIterator<Item = &crate::ImuInfo> {
        self.inventory.imus.values()
    }

    pub fn power_boards(&self) -> impl ExactSizeIterator<Item = &crate::PowerBoard> {
        self.inventory.power_boards.values()
    }

    /// Opens an aggregated stream of the latest robot state.
    pub async fn states(&self, options: StateStreamOptions) -> Result<RobotStates, Error> {
        RobotStates::open(self.clone(), options).await
    }
}

async fn load_inventory(client: &mut HardwareServiceClient<Channel>) -> Result<Inventory, Error> {
    Inventory::from_proto(
        client
            .list_hardware(ListHardwareRequest {})
            .await?
            .into_inner(),
    )
}

impl fmt::Debug for Robot {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter
            .debug_struct("Robot")
            .field("buses", &self.inventory.buses.len())
            .field("groups", &self.inventory.groups.len())
            .field("motors", &self.inventory.motors.len())
            .field("serial_devices", &self.inventory.serial_devices.len())
            .field("imus", &self.inventory.imus.len())
            .finish_non_exhaustive()
    }
}

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

    pub fn initial_state(&self) -> EnumValue<GroupState> {
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
