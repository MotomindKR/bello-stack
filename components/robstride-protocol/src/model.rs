pub const MAX_MOTOR_ID: u8 = 127;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ModelHardware {
    pub no_load_speed_rpm: f64,
    pub rated_voltage: f64,
    pub peak_phase_current: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ModelLimits {
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
    pub hardware: Option<ModelHardware>,
}

include!(concat!(env!("OUT_DIR"), "/model_catalog.rs"));

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
            position_min: limits.position_min,
            position_max: limits.position_max,
            velocity_min: limits.velocity_min,
            velocity_max: limits.velocity_max,
            torque_min: limits.torque_min,
            torque_max: limits.torque_max,
            kp_min: limits.kp_min,
            kp_max: limits.kp_max,
            kd_min: limits.kd_min,
            kd_max: limits.kd_max,
            direction,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn generated_catalog_contains_every_supported_model() {
        assert_eq!(
            MODEL_NAMES,
            [
                "ROBSTRIDE_00",
                "ROBSTRIDE_01",
                "ROBSTRIDE_02",
                "ROBSTRIDE_03",
                "ROBSTRIDE_04",
                "ROBSTRIDE_05",
                "ROBSTRIDE_06",
            ]
        );
        assert_eq!(model_limits("ROBSTRIDE_04").unwrap().torque_max, 120.0);
        assert_eq!(
            model_limits("ROBSTRIDE_04")
                .unwrap()
                .hardware
                .unwrap()
                .peak_phase_current,
            90.0
        );
        assert!(model_limits("ROBSTRIDE_01").unwrap().hardware.is_none());
        assert!(model_limits("unknown").is_none());
        for model in MODEL_NAMES {
            let limits = model_limits(model).unwrap();
            assert_eq!(limits.kp_min, 0.0);
            assert_eq!(limits.kd_min, 0.0);
        }
    }
}
