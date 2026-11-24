use crate::frame::{CommType, Frame, data_field, frame, frame_target};
use crate::model::MotorParams;

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ControlCommand {
    pub motor_id: u8,
    pub position: f64,
    pub velocity: f64,
    pub torque: f64,
    pub kp: f64,
    pub kd: f64,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Feedback {
    pub motor_id: u8,
    pub mode_status: u8,
    pub fault_bits: u16,
    pub position: f64,
    pub velocity: f64,
    pub torque: f64,
    pub temperature: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Request {
    Enable {
        host_id: u8,
        motor_id: u8,
    },
    Stop {
        host_id: u8,
        motor_id: u8,
        clear_fault: bool,
    },
    ActiveReport {
        host_id: u8,
        motor_id: u8,
        enabled: bool,
    },
    ParamWriteU8 {
        host_id: u8,
        motor_id: u8,
        index: u16,
        value: u8,
    },
}

pub fn enable_frame(host: u8, motor: u8) -> Frame {
    frame(CommType::MotorEnable, host as u16, motor)
}

pub fn stop_frame(host: u8, motor: u8, clear_fault: bool) -> Frame {
    let mut frame = frame(CommType::MotorStop, host as u16, motor);
    frame.data[0] = u8::from(clear_fault);
    frame
}

pub fn active_report_frame(host: u8, motor: u8, enabled: bool) -> Frame {
    let mut frame = frame(CommType::ActiveReport, host as u16, motor);
    frame.data = [1, 2, 3, 4, 5, 6, u8::from(enabled), 0];
    frame
}

pub fn param_write_u8_frame(host: u8, motor: u8, index: u16, value: u8) -> Frame {
    let mut frame = frame(CommType::ParamWrite, host as u16, motor);
    frame.data[..2].copy_from_slice(&index.to_le_bytes());
    frame.data[4] = value;
    frame
}

fn float_to_u16(value: f64, min: f64, max: f64) -> u16 {
    ((value.clamp(min, max) - min) * u16::MAX as f64 / (max - min)) as u16
}

fn u16_to_float(value: u16, min: f64, max: f64) -> f64 {
    value as f64 * (max - min) / u16::MAX as f64 + min
}

pub fn control_frame(params: MotorParams, command: ControlCommand) -> Frame {
    let direction = params.direction as f64;
    let torque = float_to_u16(
        command.torque * direction,
        params.torque_min,
        params.torque_max,
    );
    let mut frame = frame(CommType::MotorControl, torque, command.motor_id);
    for (offset, value) in [
        (
            0,
            float_to_u16(
                command.position * direction,
                params.position_min,
                params.position_max,
            ),
        ),
        (
            2,
            float_to_u16(
                command.velocity * direction,
                params.velocity_min,
                params.velocity_max,
            ),
        ),
        (4, float_to_u16(command.kp, params.kp_min, params.kp_max)),
        (6, float_to_u16(command.kd, params.kd_min, params.kd_max)),
    ] {
        frame.data[offset..offset + 2].copy_from_slice(&value.to_be_bytes());
    }
    frame
}

pub fn parse_control(frame: &Frame, params: MotorParams) -> Option<ControlCommand> {
    if CommType::from_id(frame.id) != Some(CommType::MotorControl) {
        return None;
    }
    let direction = params.direction as f64;
    let read = |offset| u16::from_be_bytes([frame.data[offset], frame.data[offset + 1]]);
    Some(ControlCommand {
        motor_id: frame_target(frame),
        position: u16_to_float(read(0), params.position_min, params.position_max) * direction,
        velocity: u16_to_float(read(2), params.velocity_min, params.velocity_max) * direction,
        torque: u16_to_float(data_field(frame.id), params.torque_min, params.torque_max)
            * direction,
        kp: u16_to_float(read(4), params.kp_min, params.kp_max),
        kd: u16_to_float(read(6), params.kd_min, params.kd_max),
    })
}

pub fn feedback_frame(host: u8, feedback: Feedback, params: MotorParams) -> Frame {
    let status = ((feedback.mode_status as u16 & 0x03) << 14)
        | ((feedback.fault_bits & 0x3f) << 8)
        | feedback.motor_id as u16;
    let direction = params.direction as f64;
    let mut frame = frame(CommType::MotorFeedback, status, host);
    for (offset, value) in [
        (
            0,
            float_to_u16(
                feedback.position * direction,
                params.position_min,
                params.position_max,
            ),
        ),
        (
            2,
            float_to_u16(
                feedback.velocity * direction,
                params.velocity_min,
                params.velocity_max,
            ),
        ),
        (
            4,
            float_to_u16(
                feedback.torque * direction,
                params.torque_min,
                params.torque_max,
            ),
        ),
        (6, (feedback.temperature.clamp(0.0, 6553.5) * 10.0) as u16),
    ] {
        frame.data[offset..offset + 2].copy_from_slice(&value.to_be_bytes());
    }
    frame
}

pub fn feedback_motor_id(frame: &Frame) -> Option<u8> {
    matches!(
        CommType::from_id(frame.id),
        Some(CommType::MotorFeedback | CommType::ActiveReport)
    )
    .then(|| data_field(frame.id) as u8)
}

pub fn parse_feedback(frame: &Frame, params: MotorParams) -> Option<Feedback> {
    let motor_id = feedback_motor_id(frame)?;
    let field = data_field(frame.id);
    let direction = params.direction as f64;
    let read = |offset| u16::from_be_bytes([frame.data[offset], frame.data[offset + 1]]);
    Some(Feedback {
        motor_id,
        fault_bits: (field >> 8) & 0x3f,
        mode_status: ((field >> 14) & 3) as u8,
        position: u16_to_float(read(0), params.position_min, params.position_max) * direction,
        velocity: u16_to_float(read(2), params.velocity_min, params.velocity_max) * direction,
        torque: u16_to_float(read(4), params.torque_min, params.torque_max) * direction,
        temperature: read(6) as f64 / 10.0,
    })
}

pub fn parse_request(frame: &Frame) -> Option<Request> {
    let host_id = data_field(frame.id) as u8;
    let motor_id = frame_target(frame);
    match CommType::from_id(frame.id)? {
        CommType::MotorEnable => Some(Request::Enable { host_id, motor_id }),
        CommType::MotorStop => Some(Request::Stop {
            host_id,
            motor_id,
            clear_fault: frame.data[0] != 0,
        }),
        CommType::ActiveReport => Some(Request::ActiveReport {
            host_id,
            motor_id,
            enabled: frame.data[6] != 0,
        }),
        CommType::ParamWrite => Some(Request::ParamWriteU8 {
            host_id,
            motor_id,
            index: u16::from_le_bytes([frame.data[0], frame.data[1]]),
            value: frame.data[4],
        }),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn params() -> MotorParams {
        MotorParams::for_model("RS_03", -1).expect("known model")
    }

    #[test]
    fn control_round_trip_preserves_physical_values_and_direction() {
        let command = ControlCommand {
            motor_id: 7,
            position: 1.2,
            velocity: -2.5,
            torque: 3.0,
            kp: 25.0,
            kd: 1.5,
        };
        let decoded = parse_control(&control_frame(params(), command), params()).unwrap();
        assert_eq!(decoded.motor_id, 7);
        assert!((decoded.position - command.position).abs() < 0.001);
        assert!((decoded.velocity - command.velocity).abs() < 0.001);
        assert!((decoded.torque - command.torque).abs() < 0.01);
    }

    #[test]
    fn feedback_round_trip_preserves_status_and_measurements() {
        let feedback = Feedback {
            motor_id: 7,
            mode_status: 2,
            fault_bits: 0x15,
            position: -0.4,
            velocity: 1.0,
            torque: -2.0,
            temperature: 63.4,
        };
        let decoded = parse_feedback(&feedback_frame(0xfd, feedback, params()), params()).unwrap();
        assert_eq!(decoded.motor_id, feedback.motor_id);
        assert_eq!(decoded.mode_status, feedback.mode_status);
        assert_eq!(decoded.fault_bits, feedback.fault_bits);
        assert!((decoded.position - feedback.position).abs() < 0.001);
    }
}
