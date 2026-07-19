#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Frame {
    pub id: u32,
    pub data: [u8; 8],
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub(crate) enum CommType {
    MotorControl = 1,
    MotorFeedback = 2,
    MotorEnable = 3,
    MotorStop = 4,
    ParamWrite = 18,
    ActiveReport = 24,
}

impl CommType {
    pub(crate) fn from_id(id: u32) -> Option<Self> {
        match ((id >> 24) & 0x1f) as u8 {
            1 => Some(Self::MotorControl),
            2 => Some(Self::MotorFeedback),
            3 => Some(Self::MotorEnable),
            4 => Some(Self::MotorStop),
            18 => Some(Self::ParamWrite),
            24 => Some(Self::ActiveReport),
            _ => None,
        }
    }
}

pub(crate) fn build_can_id(kind: CommType, data_field: u16, target_id: u8) -> u32 {
    ((kind as u32 & 0x1f) << 24) | ((data_field as u32) << 8) | target_id as u32
}

pub(crate) fn data_field(id: u32) -> u16 {
    ((id >> 8) & 0xffff) as u16
}

pub fn frame_target(frame: &Frame) -> u8 {
    frame.id as u8
}

pub(crate) fn frame(kind: CommType, field: u16, target: u8) -> Frame {
    Frame {
        id: build_can_id(kind, field, target),
        data: [0; 8],
    }
}
