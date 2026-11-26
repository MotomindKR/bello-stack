mod codec;
mod frame;
mod model;

pub use codec::{
    ControlCommand, Feedback, Request, active_report_frame, control_frame, enable_frame,
    feedback_frame, feedback_motor_id, firmware_version_reply, firmware_version_request,
    param_write_u8_frame, parse_control, parse_feedback, parse_request, stop_frame,
};
pub use frame::{Frame, frame_target, is_control_frame};
pub use model::{
    CONTROL_POSITION_MAX, CONTROL_POSITION_MIN, CONTROL_POSITION_SPAN, MAX_MOTOR_ID, MODEL_NAMES,
    ModelHardware, ModelLimits, MotorParams, model_limits,
};

pub const OTA_HOST_ID: u8 = 0xfd;
