use anyhow::{Context, Result, bail};
use garde::Validate;
use robstride_protocol::{ModelLimits, MotorParams, model_limits};

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
        config
            .validate()
            .with_context(|| format!("motors.{}", config.name))?;
        if !matches!(config.direction, -1 | 1) {
            bail!("motors.{}.direction must be -1 or 1", config.name);
        }
        let model = model_limits(&config.model)
            .ok_or_else(|| anyhow::anyhow!("unsupported Robstride model {}", config.model))?;
        validate_limits(config, model)?;
        validate_health(config)?;
        let params = MotorParams::for_model(&config.model, config.direction)
            .expect("validated Robstride model and direction");
        let limits = command_limits(config, model);
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
            feedback_position_tolerance: config.health.feedback_position_tolerance,
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
        kp_max: model.kp_max,
        kd_min: model.kd_min,
        kd_max: model.kd_max,
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
    if limits.position_min >= limits.position_max {
        bail!("motors.{}.limits.position range is invalid", config.name);
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
    Ok(())
}

fn validate_health(config: &MotorConfig) -> Result<()> {
    let health = &config.health;
    if health.safe_position_min >= health.safe_position_max
        || health.safe_position_min > config.limits.position_min
        || health.safe_position_max < config.limits.position_max
        || health.temperature_warning_c >= health.maximum_temperature_c
        || health.temperature_recovery_c >= health.maximum_temperature_c
    {
        bail!("motors.{}.health contains invalid values", config.name);
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
    fn rejects_temperature_recovery_without_hysteresis() {
        let config: MotorConfig = serde_yaml::from_str(
            "name: motor_a\ncoordinate: coordinate_a\nbus: can\nid: 1\nmodel: RS_03\nhealth: { maximum_temperature_c: 80.0, temperature_recovery_c: 80.0 }",
        )
        .unwrap();
        assert!(MotorSpec::try_from(&config).is_err());
    }

    #[test]
    fn rejects_warning_at_fault_threshold() {
        let config: MotorConfig = serde_yaml::from_str(
            "name: motor_a\ncoordinate: coordinate_a\nbus: can\nid: 1\nmodel: RS_03\nhealth: { temperature_warning_c: 80.0, maximum_temperature_c: 80.0 }",
        )
        .unwrap();
        assert!(MotorSpec::try_from(&config).is_err());
    }

    #[test]
    fn rejects_safe_position_narrower_than_command_limits() {
        let config: MotorConfig = serde_yaml::from_str(
            "name: motor_a\ncoordinate: coordinate_a\nbus: can\nid: 1\nmodel: RS_03\nlimits: { position_min: -1.0, position_max: 1.0 }\nhealth: { safe_position_min: -0.5, safe_position_max: 0.5 }",
        )
        .unwrap();
        assert!(MotorSpec::try_from(&config).is_err());
    }

    #[test]
    fn accepts_configured_feedback_position_tolerance() {
        let config: MotorConfig = serde_yaml::from_str(
            "name: motor_a\ncoordinate: coordinate_a\nbus: can\nid: 1\nmodel: RS_03\nhealth: { feedback_position_tolerance: 0.4 }",
        )
        .unwrap();
        let spec = MotorSpec::try_from(&config).unwrap();
        assert_eq!(spec.feedback_position_tolerance, 0.4);
    }

    #[test]
    fn accepts_damping_gain_above_protocol_range() {
        let config: MotorConfig = serde_yaml::from_str(
            "name: motor_a\ncoordinate: coordinate_a\nbus: can\nid: 1\nmodel: RS_03\nfault_damping_kd: 200.0",
        )
        .unwrap();
        let spec = MotorSpec::try_from(&config).unwrap();
        assert_eq!(spec.fault_damping_kd, 200.0);
    }

    #[test]
    fn rejects_unsupported_warning_policy_combinations() {
        for policy in ["limit: warn", "position_step: warn"] {
            let yaml = format!(
                "name: motor_a\ncoordinate: coordinate_a\nbus: can\nid: 1\nmodel: RS_03\ncommand_policy: {{ {policy} }}"
            );
            assert!(serde_yaml::from_str::<MotorConfig>(&yaml).is_err());
        }
    }

    #[test]
    fn rejects_limit_overrides_above_model_capabilities() {
        for (field, value) in [("velocity_abs", 20.1), ("torque_abs", 60.1)] {
            let yaml = format!(
                "name: motor_a\ncoordinate: coordinate_a\nbus: can\nid: 1\nmodel: RS_03\nlimits: {{ {field}: {value} }}"
            );
            let config: MotorConfig = serde_yaml::from_str(&yaml).unwrap();
            let error = MotorSpec::try_from(&config).unwrap_err().to_string();
            assert!(
                error.contains("exceeds") && error.contains("RS_03"),
                "{error}"
            );
        }
    }

    #[test]
    fn optional_absolute_caps_preserve_asymmetric_model_ranges() {
        let mut config: MotorConfig = serde_yaml::from_str(
            "name: motor_a\ncoordinate: coordinate_a\nbus: can\nid: 1\nmodel: RS_03",
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
