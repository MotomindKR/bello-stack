pub const CONTROL_POSITION_MIN: f64 = -12.57;
pub const CONTROL_POSITION_MAX: f64 = 12.57;
pub const CONTROL_POSITION_SPAN: f64 = CONTROL_POSITION_MAX - CONTROL_POSITION_MIN;
pub const MAX_MOTOR_ID: u8 = 127;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ModelLimits {
    pub velocity_abs: f64,
    pub torque_abs: f64,
    pub kp_max: f64,
    pub kd_max: f64,
}

pub fn model_limits(model: &str) -> Option<ModelLimits> {
    let (velocity_abs, torque_abs, kp_max, kd_max) = match model {
        "ROBSTRIDE_00" => (33.0, 14.0, 500.0, 5.0),
        "ROBSTRIDE_01" | "ROBSTRIDE_02" => (44.0, 17.0, 500.0, 5.0),
        "ROBSTRIDE_03" => (20.0, 60.0, 5000.0, 100.0),
        "ROBSTRIDE_04" => (15.0, 120.0, 5000.0, 100.0),
        "ROBSTRIDE_05" => (50.0, 5.5, 500.0, 5.0),
        "ROBSTRIDE_06" => (50.0, 36.0, 5000.0, 100.0),
        _ => return None,
    };
    Some(ModelLimits {
        velocity_abs,
        torque_abs,
        kp_max,
        kd_max,
    })
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotorParams {
    pub position_min: f64,
    pub position_max: f64,
    pub velocity_min: f64,
    pub velocity_max: f64,
    pub torque_min: f64,
    pub torque_max: f64,
    pub kp_min: f64,
    pub kp_max: f64,
    pub kd_min: f64,
    pub kd_max: f64,
    pub direction: i8,
}

impl MotorParams {
    pub fn for_model(model: &str, direction: i8) -> Option<Self> {
        if !matches!(direction, -1 | 1) {
            return None;
        }
        let limits = model_limits(model)?;
        Some(Self {
            position_min: CONTROL_POSITION_MIN,
            position_max: CONTROL_POSITION_MAX,
            velocity_min: -limits.velocity_abs,
            velocity_max: limits.velocity_abs,
            torque_min: -limits.torque_abs,
            torque_max: limits.torque_abs,
            kp_min: 0.0,
            kp_max: limits.kp_max,
            kd_min: 0.0,
            kd_max: limits.kd_max,
            direction,
        })
    }
}
