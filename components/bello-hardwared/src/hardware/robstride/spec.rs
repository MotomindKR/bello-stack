use anyhow::{Result, bail};
use robstride_protocol::model_limits;

use super::MotorParams;
use crate::config::{MotorCommandPolicy, MotorConfig, MotorFaultDamping, MotorHealth, MotorLimits};
use crate::domain::{BusName, MotorAddress, MotorName};

/// Validated static definition of one Robstride motor.
#[derive(Clone, Debug)]
pub(crate) struct MotorSpec {
    pub(crate) name: MotorName,
    pub(crate) joint: String,
    pub(crate) address: MotorAddress,
    pub(crate) model: String,
    pub(crate) limits: MotorLimits,
    pub(crate) health: MotorHealth,
    pub(crate) command_policy: MotorCommandPolicy,
    pub(crate) fault_damping: MotorFaultDamping,
    pub(crate) params: MotorParams,
}

impl TryFrom<&MotorConfig> for MotorSpec {
    type Error = anyhow::Error;

    fn try_from(config: &MotorConfig) -> Result<Self> {
        if config.name.is_empty() {
            bail!("motors[].name must be non-empty");
        }
        if config.joint.is_empty() {
            bail!("motors.{}.joint must be non-empty", config.name);
        }
        if config.bus.is_empty() {
            bail!("motors.{}.bus must be non-empty", config.name);
        }
        if !matches!(config.direction, -1 | 1) {
            bail!("motors.{}.direction must be -1 or 1", config.name);
        }
        validate_limits(config)?;
        validate_health(config)?;
        validate_command_policy(config)?;
        validate_fault_damping(config)?;
        let params = params_for_config(config)?;
        Ok(Self {
            name: MotorName::from(config.name.as_str()),
            joint: config.joint.clone(),
            address: MotorAddress {
                bus: BusName::from(config.bus.as_str()),
                id: config.id,
            },
            model: config.model.clone(),
            limits: config.limits.clone(),
            health: config.health.clone(),
            command_policy: config.command_policy.clone(),
            fault_damping: config.fault_damping.clone(),
            params,
        })
    }
}

fn params_for_config(config: &MotorConfig) -> Result<MotorParams> {
    let model = model_limits(&config.model)
        .ok_or_else(|| anyhow::anyhow!("unsupported Robstride model {}", config.model))?;
    let velocity = config
        .limits
        .velocity_abs
        .unwrap_or(model.velocity_abs)
        .min(model.velocity_abs);
    let torque = config
        .limits
        .torque_abs
        .unwrap_or(model.torque_abs)
        .min(model.torque_abs);
    Ok(MotorParams {
        // The Type-1 MIT position field always uses the protocol span. Robot/joint
        // limits are enforced separately by the command policy before encoding.
        position_min: -12.57,
        position_max: 12.57,
        velocity_min: -velocity,
        velocity_max: velocity,
        torque_min: -torque,
        torque_max: torque,
        kp_min: 0.0,
        kp_max: config
            .limits
            .kp_max
            .unwrap_or(model.kp_max)
            .min(model.kp_max),
        kd_min: 0.0,
        kd_max: config
            .limits
            .kd_max
            .unwrap_or(model.kd_max)
            .min(model.kd_max),
        direction: config.direction,
    })
}

fn validate_limits(config: &MotorConfig) -> Result<()> {
    let limits = &config.limits;
    if !limits.position_min.is_finite()
        || !limits.position_max.is_finite()
        || limits.position_min >= limits.position_max
        || limits.position_min < -12.57
        || limits.position_max > 12.57
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
        || health.safe_position_min > config.limits.position_min
        || health.safe_position_max < config.limits.position_max
        || health.temperature_warning_c <= 0.0
        || health.temperature_warning_c >= health.maximum_temperature_c
        || health.maximum_temperature_c <= 0.0
        || health.temperature_recovery_c <= 0.0
        || health.temperature_recovery_c >= health.maximum_temperature_c
        || health.temperature_recovery_duration_ms == 0
    {
        bail!("motors.{}.health contains invalid values", config.name);
    }
    Ok(())
}

fn validate_command_policy(config: &MotorConfig) -> Result<()> {
    let policy = &config.command_policy;
    if !policy.maximum_position_step.is_finite() || policy.maximum_position_step <= 0.0 {
        bail!(
            "motors.{}.command_policy.maximum_position_step must be positive",
            config.name
        );
    }
    Ok(())
}

fn validate_fault_damping(config: &MotorConfig) -> Result<()> {
    if !config.fault_damping.kd.is_finite() || config.fault_damping.kd < 0.0 {
        bail!(
            "motors.{}.fault_damping.kd must be non-negative",
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
        let config: MotorConfig =
            serde_yaml::from_str("name: motor_a\njoint: joint_a\nbus: can\nid: 1\nmodel: UNKNOWN")
                .unwrap();
        assert!(MotorSpec::try_from(&config).is_err());
    }

    #[test]
    fn rejects_temperature_recovery_without_hysteresis() {
        let config: MotorConfig = serde_yaml::from_str(
            "name: motor_a\njoint: joint_a\nbus: can\nid: 1\nmodel: ROBSTRIDE_03\nhealth: { maximum_temperature_c: 80.0, temperature_recovery_c: 80.0 }",
        )
        .unwrap();
        assert!(MotorSpec::try_from(&config).is_err());
    }

    #[test]
    fn rejects_warning_at_fault_threshold() {
        let config: MotorConfig = serde_yaml::from_str(
            "name: motor_a\njoint: joint_a\nbus: can\nid: 1\nmodel: ROBSTRIDE_03\nhealth: { temperature_warning_c: 80.0, maximum_temperature_c: 80.0 }",
        )
        .unwrap();
        assert!(MotorSpec::try_from(&config).is_err());
    }

    #[test]
    fn rejects_safe_position_narrower_than_command_limits() {
        let config: MotorConfig = serde_yaml::from_str(
            "name: motor_a\njoint: joint_a\nbus: can\nid: 1\nmodel: ROBSTRIDE_03\nlimits: { position_min: -1.0, position_max: 1.0 }\nhealth: { safe_position_min: -0.5, safe_position_max: 0.5 }",
        )
        .unwrap();
        assert!(MotorSpec::try_from(&config).is_err());
    }
}
