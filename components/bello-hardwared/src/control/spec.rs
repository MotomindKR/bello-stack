use std::time::Duration;

use anyhow::{Result, bail};

use crate::config::{CommandTransmission, FaultPolicyConfig, GroupConfig};
use crate::domain::{GroupName, MotorName};
use crate::mechanics::MechanicalEnvelope;

#[derive(Clone, Debug)]
pub(crate) struct GroupSpec {
    pub(crate) name: GroupName,
    pub(crate) motors: Vec<MotorName>,
    pub(crate) control_rate_hz: f64,
    pub(crate) command_refresh_rate_hz: f64,
    pub(crate) command_transmission: CommandTransmission,
    pub(crate) periodic_feedback_during_control: bool,
    pub(crate) command_timeout: Duration,
    pub(crate) feedback_timeout: Duration,
    pub(crate) enable_timeout: Duration,
    pub(crate) enable_retries: u32,
    pub(crate) fault_policy: FaultPolicyConfig,
    pub(crate) mechanical: MechanicalEnvelope,
}

impl GroupSpec {
    pub fn period(&self) -> Duration {
        Duration::from_secs_f64(1.0 / self.control_rate_hz)
    }

    pub fn command_refresh_period(&self) -> Duration {
        Duration::from_secs_f64(1.0 / self.command_refresh_rate_hz)
    }
}

impl TryFrom<&GroupConfig> for GroupSpec {
    type Error = anyhow::Error;

    fn try_from(config: &GroupConfig) -> Result<Self> {
        if config.name.is_empty() {
            bail!("motor_groups[].name must be non-empty");
        }
        if config.motors.is_empty() {
            bail!("motor_groups.{}.motors must not be empty", config.name);
        }
        if !config.parallel_feedback_position_tolerance.is_finite()
            || !(0.0 < config.parallel_feedback_position_tolerance
                && config.parallel_feedback_position_tolerance <= 0.4)
        {
            bail!(
                "motor_groups.{}.parallel_feedback_position_tolerance must be in (0, 0.4]",
                config.name
            );
        }
        if !config.control_rate_hz.is_finite() || !(1.0..=2_000.0).contains(&config.control_rate_hz)
        {
            bail!(
                "motor_groups.{}.control_rate_hz must be in 1..=2000",
                config.name
            );
        }
        let command_refresh_rate_hz = config
            .command_refresh_rate_hz
            .unwrap_or(config.control_rate_hz);
        if !command_refresh_rate_hz.is_finite()
            || !(1.0..=config.control_rate_hz).contains(&command_refresh_rate_hz)
        {
            bail!(
                "motor_groups.{}.command_refresh_rate_hz must be in 1..=control_rate_hz",
                config.name
            );
        }
        if config.command_timeout_ms == 0
            || config.feedback_timeout_ms == 0
            || config.enable_timeout_ms == 0
            || config.enable_retries == 0
            || config.command_timeout_ms > 10_000
            || config.feedback_timeout_ms > 10_000
            || config.enable_timeout_ms > 30_000
            || config.enable_retries > 100
        {
            bail!(
                "motor_groups.{} timeouts or retries are outside supported bounds",
                config.name
            );
        }
        Ok(Self {
            name: GroupName::from(config.name.as_str()),
            motors: config
                .motors
                .iter()
                .map(|name| MotorName::from(name.as_str()))
                .collect(),
            control_rate_hz: config.control_rate_hz,
            command_refresh_rate_hz,
            command_transmission: config.command_transmission,
            periodic_feedback_during_control: config.periodic_feedback_during_control,
            command_timeout: Duration::from_millis(config.command_timeout_ms),
            feedback_timeout: Duration::from_millis(config.feedback_timeout_ms),
            enable_timeout: Duration::from_millis(config.enable_timeout_ms),
            enable_retries: config.enable_retries,
            fault_policy: config.fault_policy.clone(),
            mechanical: MechanicalEnvelope::default(),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rejects_zero_control_rate() {
        let config: GroupConfig =
            serde_yaml::from_str("name: group_a\nmotors: [motor_a]\ncontrol_rate_hz: 0.0").unwrap();
        assert!(GroupSpec::try_from(&config).is_err());
    }

    #[test]
    fn rejects_invalid_parallel_feedback_tolerance() {
        for value in ["0", "-0.1", "0.5", ".nan"] {
            let config: GroupConfig = serde_yaml::from_str(&format!(
                "name: group\nmotors: [a, b]\nparallel_feedback_position_tolerance: {value}"
            ))
            .unwrap();
            assert!(GroupSpec::try_from(&config).is_err());
        }
    }

    #[test]
    fn command_refresh_rate_defaults_to_the_safety_rate() {
        let config: GroupConfig =
            serde_yaml::from_str("name: group_a\nmotors: [motor_a]\ncontrol_rate_hz: 250.0")
                .unwrap();
        let spec = GroupSpec::try_from(&config).unwrap();

        assert_eq!(spec.command_refresh_rate_hz, 250.0);
    }

    #[test]
    fn rejects_command_refresh_rate_above_the_safety_rate() {
        let config: GroupConfig = serde_yaml::from_str(
            "name: group_a\nmotors: [motor_a]\ncontrol_rate_hz: 100.0\ncommand_refresh_rate_hz: 101.0",
        )
        .unwrap();

        assert!(GroupSpec::try_from(&config).is_err());
    }

    #[test]
    fn resolves_cause_specific_responses() {
        let config: GroupConfig = serde_yaml::from_str(
            "name: group_a\nmotors: [motor_a]\nfault_policy: { default: damping, overrides: { temperature: disable } }",
        )
        .unwrap();
        let spec = GroupSpec::try_from(&config).unwrap();
        assert_eq!(
            spec.fault_policy
                .response(crate::config::FaultCause::Temperature),
            crate::config::FaultResponse::Disable
        );
        assert_eq!(
            spec.fault_policy
                .response(crate::config::FaultCause::StaleFeedback),
            crate::config::FaultResponse::Damping
        );
    }
}
