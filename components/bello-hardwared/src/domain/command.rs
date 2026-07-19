use std::time::Duration;

use super::{GroupName, LeaseToken, MotorName};

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ImpedanceCommand {
    pub position: f64,
    pub velocity: f64,
    pub torque: f64,
    pub kp: f64,
    pub kd: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct MotorCommand {
    pub motor: MotorName,
    pub impedance: ImpedanceCommand,
}

#[derive(Clone, Debug, PartialEq)]
pub struct CommandBatch {
    pub group: GroupName,
    pub lease_token: LeaseToken,
    pub sequence: u64,
    pub valid_for: Option<Duration>,
    pub commands: Vec<MotorCommand>,
}
