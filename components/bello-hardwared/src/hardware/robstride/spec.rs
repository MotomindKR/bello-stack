use anyhow::{Result, bail};
use robstride_protocol::{
    CONTROL_POSITION_MAX, CONTROL_POSITION_MIN, CONTROL_POSITION_SPAN, MAX_MOTOR_ID, ModelLimits,
    MotorParams, model_limits,
};

use crate::config::{MotorCommandPolicy, MotorConfig, MotorHealth};
use crate::domain::{BusName, MotorAddress, MotorName};

#[derive(Clone, Debug)]
pub(crate) struct MotorCommandLimits {
    pub(crate) position_min: f64,
    pub(crate) position_max: f64,
    pub(crate) velocity_min: f64,
    pub(crate) velocity_max: f64,
    pub(crate) torque_min: f64,
    pub(crate) torque_max: f64,
    pub(crate) kp_min: f64,
    pub(crate) kp_max: f64,
    pub(crate) kd_min: f64,
    pub(crate) kd_max: f64,
}

/// Validated static definition of one Robstride motor.
#[derive(Clone, Debug)]
pub(crate) struct MotorSpec {
    pub(crate) name: MotorName,
    pub(crate) coordinate: String,
    pub(crate) address: MotorAddress,
    pub(crate) model: String,
    pub(crate) limits: MotorCommandLimits,
    pub(crate) health: MotorHealth,
    pub(crate) command_policy: MotorCommandPolicy,
    pub(crate) fault_damping_kd: f64,
    pub(crate) feedback_position_tolerance: f64,
    pub(crate) params: MotorParams,
}

impl TryFrom<&MotorConfig> for MotorSpec {
    type Error = anyhow::Error;

    fn try_from(config: &MotorConfig) -> Result<Self> {
        if config.name.is_empty() {
            bail!("motors[].name must be non-empty");
        }
        if config.coordinate.is_empty() {
            bail!("motors.{}.coordinate must be non-empty", config.name);
        }
        if config.bus.is_empty() {
            bail!("motors.{}.bus must be non-empty", config.name);
        }
        if config.id > MAX_MOTOR_ID {
            bail!(
                "motors.{}.id must be in the Robstride range 0..={MAX_MOTOR_ID}",
                config.name
            );
        }
        if !matches!(config.direction, -1 | 1) {
            bail!("motors.{}.direction must be -1 or 1", config.name);
        }
        let model = model_limits(&config.model)
            .ok_or_else(|| anyhow::anyhow!("unsupported Robstride model {}", config.model))?;
        validate_limits(config, model)?;
        validate_health(config)?;
        validate_command_policy(config)?;
        let params = MotorParams::for_model(&config.model, config.direction)
            .expect("validated Robstride model and direction");
        let limits = command_limits(config, model);
        validate_fault_damping(config, limits.kd_max)?;
        Ok(Self {
            name: MotorName::from(config.name.as_str()),
            coordinate: config.coordinate.clone(),
            address: MotorAddress {
                bus: BusName::from(config.bus.as_str()),
                id: config.id,
            },
            model: config.model.clone(),
            limits,
            health: config.health.clone(),
            command_policy: config.command_policy.clone(),
            fault_damping_kd: config.fault_damping_kd,
            feedback_position_tolerance: 2.0 * CONTROL_POSITION_SPAN / u16::MAX as f64,
            params,
        })
    }
}

fn command_limits(config: &MotorConfig, model: ModelLimits) -> MotorCommandLimits {
    let (model_velocity_min, model_velocity_max) =
        directed_range(model.velocity_min, model.velocity_max, config.direction);
    let (model_torque_min, model_torque_max) =
        directed_range(model.torque_min, model.torque_max, config.direction);
    let velocity_min = config
        .limits
        .velocity_abs
        .map_or(model_velocity_min, |value| model_velocity_min.max(-value));
    let velocity_max = config
        .limits
        .velocity_abs
        .map_or(model_velocity_max, |value| model_velocity_max.min(value));
    let torque_min = config
        .limits
        .torque_abs
        .map_or(model_torque_min, |value| model_torque_min.max(-value));
    let torque_max = config
        .limits
        .torque_abs
        .map_or(model_torque_max, |value| model_torque_max.min(value));
    MotorCommandLimits {
        position_min: config.limits.position_min,
        position_max: config.limits.position_max,
        velocity_min,
        velocity_max,
        torque_min,
        torque_max,
        kp_min: model.kp_min,
        kp_max: config.limits.kp_max.unwrap_or(model.kp_max),
        kd_min: model.kd_min,
        kd_max: config.limits.kd_max.unwrap_or(model.kd_max),
    }
}

fn directed_range(minimum: f64, maximum: f64, direction: i8) -> (f64, f64) {
    if direction == 1 {
        (minimum, maximum)
    } else {
        (-maximum, -minimum)
    }
}

fn validate_limits(config: &MotorConfig, model: ModelLimits) -> Result<()> {
    let limits = &config.limits;
    if !limits.position_min.is_finite()
        || !limits.position_max.is_finite()
        || limits.position_min >= limits.position_max
        || limits.position_min < CONTROL_POSITION_MIN
        || limits.position_max > CONTROL_POSITION_MAX
    {
        bail!("motors.{}.limits.position range is invalid", config.name);
    }
    for (field, value, allow_zero) in [
        ("velocity_abs", limits.velocity_abs, false),
        ("torque_abs", limits.torque_abs, false),
        ("kp_max", limits.kp_max, true),
        ("kd_max", limits.kd_max, true),
    ] {
        if value
            .is_some_and(|value| !value.is_finite() || value < 0.0 || (!allow_zero && value == 0.0))
        {
            bail!("motors.{}.limits.{field} is invalid", config.name);
        }
    }
    for (field, configured, minimum, maximum) in [
        (
            "velocity_abs",
            limits.velocity_abs,
            model.velocity_min,
            model.velocity_max,
        ),
        (
            "torque_abs",
            limits.torque_abs,
            model.torque_min,
            model.torque_max,
        ),
    ] {
        if let Some(configured) = configured
            && configured > maximum.max(-minimum)
        {
            bail!(
                "motors.{}.limits.{field} {configured} exceeds the maximum absolute {} model capability [{minimum}, {maximum}]",
                config.name,
                config.model
            );
        }
    }
    for (field, configured, maximum) in [
        ("kp_max", limits.kp_max, model.kp_max),
        ("kd_max", limits.kd_max, model.kd_max),
    ] {
        if let Some(configured) = configured
            && configured > maximum
        {
            bail!(
                "motors.{}.limits.{field} {configured} exceeds {} model limit {maximum}",
                config.name,
                config.model
            );
        }
    }
    Ok(())
}

fn validate_health(config: &MotorConfig) -> Result<()> {
    let health = &config.health;
    if !health.safe_position_min.is_finite()
        || !health.safe_position_max.is_finite()
        || !health.temperature_warning_c.is_finite()
        || !health.maximum_temperature_c.is_finite()
        || !health.temperature_recovery_c.is_finite()
        || health.safe_position_min >= health.safe_position_max
        || health.safe_position_min < CONTROL_POSITION_MIN
        || health.safe_position_max > CONTROL_POSITION_MAX
        || health.safe_position_min > config.limits.position_min
        || health.safe_position_max < config.limits.position_max
        || health.temperature_warning_c <= 0.0
        || health.temperature_warning_c >= health.maximum_temperature_c
        || health.maximum_temperature_c <= 0.0
        || health.temperature_recovery_c <= 0.0
        || health.temperature_recovery_c >= health.maximum_temperature_c
        || health.temperature_recovery_duration_ms == 0
        || health.maximum_temperature_c > 200.0
        || health.temperature_recovery_duration_ms > 60_000
    {
        bail!("motors.{}.health contains invalid values", config.name);
    }
    Ok(())
}

fn validate_command_policy(config: &MotorConfig) -> Result<()> {
    let policy = &config.command_policy;
    if !policy.maximum_position_step.is_finite()
        || policy.maximum_position_step <= 0.0
        || policy.maximum_position_step > CONTROL_POSITION_SPAN
    {
        bail!(
            "motors.{}.command_policy.maximum_position_step must be in (0, {CONTROL_POSITION_SPAN}]",
            config.name,
        );
    }
    Ok(())
}

fn validate_fault_damping(config: &MotorConfig, maximum: f64) -> Result<()> {
    if !config.fault_damping_kd.is_finite()
        || config.fault_damping_kd <= 0.0
        || config.fault_damping_kd > maximum
    {
        bail!(
            "motors.{}.fault_damping_kd must be positive and within the motor gain range",
            config.name
        );
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rejects_unknown_motor_model() {
        let config: MotorConfig = serde_yaml::from_str(
            "name: motor_a\ncoordinate: coordinate_a\nbus: can\nid: 1\nmodel: UNKNOWN",
        )
        .unwrap();
        assert!(MotorSpec::try_from(&config).is_err());
    }

    #[test]
    fn rejects_motor_id_outside_protocol_range() {
        let config: MotorConfig = serde_yaml::from_str(
            "name: motor_a\ncoordinate: coordinate_a\nbus: can\nid: 128\nmodel: ROBSTRIDE_03",
        )
        .unwrap();
        assert!(MotorSpec::try_from(&config).is_err());
    }

    #[test]
    fn rejects_temperature_recovery_without_hysteresis() {
        let config: MotorConfig = serde_yaml::from_str(
            "name: motor_a\ncoordinate: coordinate_a\nbus: can\nid: 1\nmodel: ROBSTRIDE_03\nhealth: { maximum_temperature_c: 80.0, temperature_recovery_c: 80.0 }",
        )
        .unwrap();
        assert!(MotorSpec::try_from(&config).is_err());
    }

    #[test]
    fn rejects_warning_at_fault_threshold() {
        let config: MotorConfig = serde_yaml::from_str(
            "name: motor_a\ncoordinate: coordinate_a\nbus: can\nid: 1\nmodel: ROBSTRIDE_03\nhealth: { temperature_warning_c: 80.0, maximum_temperature_c: 80.0 }",
        )
        .unwrap();
        assert!(MotorSpec::try_from(&config).is_err());
    }

    #[test]
    fn rejects_safe_position_narrower_than_command_limits() {
        let config: MotorConfig = serde_yaml::from_str(
            "name: motor_a\ncoordinate: coordinate_a\nbus: can\nid: 1\nmodel: ROBSTRIDE_03\nlimits: { position_min: -1.0, position_max: 1.0 }\nhealth: { safe_position_min: -0.5, safe_position_max: 0.5 }",
        )
        .unwrap();
        assert!(MotorSpec::try_from(&config).is_err());
    }

    #[test]
    fn rejects_safe_position_outside_protocol_range() {
        let config: MotorConfig = serde_yaml::from_str(
            "name: motor_a\ncoordinate: coordinate_a\nbus: can\nid: 1\nmodel: ROBSTRIDE_03\nhealth: { safe_position_min: -13.0, safe_position_max: 13.0 }",
        )
        .unwrap();
        assert!(MotorSpec::try_from(&config).is_err());
    }

    #[test]
    fn damping_response_requires_nonzero_gain() {
        let config: MotorConfig = serde_yaml::from_str(
            "name: motor_a\ncoordinate: coordinate_a\nbus: can\nid: 1\nmodel: ROBSTRIDE_03\nfault_damping_kd: 0.0",
        )
        .unwrap();
        assert!(MotorSpec::try_from(&config).is_err());
    }

    #[test]
    fn rejects_limit_overrides_above_model_capabilities() {
        for (field, value) in [
            ("velocity_abs", 20.1),
            ("torque_abs", 60.1),
            ("kp_max", 5000.1),
            ("kd_max", 100.1),
        ] {
            let yaml = format!(
                "name: motor_a\ncoordinate: coordinate_a\nbus: can\nid: 1\nmodel: ROBSTRIDE_03\nlimits: {{ {field}: {value} }}"
            );
            let config: MotorConfig = serde_yaml::from_str(&yaml).unwrap();
            let error = MotorSpec::try_from(&config).unwrap_err().to_string();
            assert!(
                error.contains("exceeds") && error.contains("ROBSTRIDE_03"),
                "{error}"
            );
        }
    }

    #[test]
    fn optional_absolute_caps_preserve_asymmetric_model_ranges() {
        let mut config: MotorConfig = serde_yaml::from_str(
            "name: motor_a\ncoordinate: coordinate_a\nbus: can\nid: 1\nmodel: ROBSTRIDE_03",
        )
        .unwrap();
        let model = ModelLimits {
            position_min: -12.0,
            position_max: 12.0,
            velocity_min: -10.0,
            velocity_max: 20.0,
            torque_min: -30.0,
            torque_max: 40.0,
            kp_min: 0.0,
            kp_max: 100.0,
            kd_min: 0.0,
            kd_max: 10.0,
            hardware: None,
        };
        let limits = command_limits(&config, model);
        assert_eq!((limits.velocity_min, limits.velocity_max), (-10.0, 20.0));
        assert_eq!((limits.torque_min, limits.torque_max), (-30.0, 40.0));

        config.limits.velocity_abs = Some(15.0);
        config.limits.torque_abs = Some(35.0);
        let limits = command_limits(&config, model);
        assert_eq!((limits.velocity_min, limits.velocity_max), (-10.0, 15.0));
        assert_eq!((limits.torque_min, limits.torque_max), (-30.0, 35.0));
        validate_limits(&config, model).unwrap();

        config.direction = -1;
        let limits = command_limits(&config, model);
        assert_eq!((limits.velocity_min, limits.velocity_max), (-15.0, 10.0));
        assert_eq!((limits.torque_min, limits.torque_max), (-35.0, 30.0));
    }
}
