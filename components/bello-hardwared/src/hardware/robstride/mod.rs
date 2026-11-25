mod controller;
mod firmware;
mod health;
mod manager;
mod spec;
mod velocity;

pub(crate) use controller::{MotorController, TimedFeedback};
pub(crate) use firmware::FirmwareVersion;
pub(crate) use health::MotorFaultCause;
pub(crate) use manager::MotorManager;
pub(crate) use robstride_protocol::Feedback;
pub(crate) use spec::MotorSpec;

impl From<robstride_protocol::Frame> for crate::hardware::can::CanFrame {
    fn from(frame: robstride_protocol::Frame) -> Self {
        Self {
            id: frame.id,
            data: frame.data,
        }
    }
}

impl From<crate::hardware::can::CanFrame> for robstride_protocol::Frame {
    fn from(frame: crate::hardware::can::CanFrame) -> Self {
        Self {
            id: frame.id,
            data: frame.data,
        }
    }
}
