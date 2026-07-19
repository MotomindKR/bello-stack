use std::time::Duration;

use anyhow::{Result, bail};

use crate::config::{GroupConfig, GroupFaultResponses};
use crate::domain::{GroupName, MotorName};

/// Validated, side-effect-free definition of a synchronized motor group.
#[derive(Clone, Debug)]
pub(crate) struct GroupSpec {
    pub(crate) name: GroupName,
    pub(crate) motors: Vec<MotorName>,
    pub(crate) control_rate_hz: f64,
    pub(crate) command_timeout: Duration,
    pub(crate) feedback_timeout: Duration,
    pub(crate) enable_timeout: Duration,
    pub(crate) enable_retries: u32,
    pub(crate) fault_responses: GroupFaultResponses,
    pub(crate) damping_send_count: u32,
    pub(crate) damping_send_period: Duration,
}

impl GroupSpec {
    pub fn period(&self) -> Duration {
        Duration::from_secs_f64(1.0 / self.control_rate_hz)
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
        if !config.control_rate_hz.is_finite() || config.control_rate_hz <= 0.0 {
            bail!(
                "motor_groups.{}.control_rate_hz must be positive",
                config.name
            );
        }
        if config.command_timeout_ms == 0
            || config.feedback_timeout_ms == 0
            || config.enable_timeout_ms == 0
            || config.enable_retries == 0
        {
            bail!(
                "motor_groups.{} timeouts and retries must be positive",
                config.name
            );
        }
        if config.damping_response.send_count == 0
            || !config.damping_response.send_rate_hz.is_finite()
            || config.damping_response.send_rate_hz <= 0.0
        {
            bail!(
                "motor_groups.{}.damping_response must have a positive send_count and send_rate_hz",
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
            command_timeout: Duration::from_millis(config.command_timeout_ms),
            feedback_timeout: Duration::from_millis(config.feedback_timeout_ms),
            enable_timeout: Duration::from_millis(config.enable_timeout_ms),
            enable_retries: config.enable_retries,
            fault_responses: config.fault_responses.clone(),
            damping_send_count: config.damping_response.send_count,
            damping_send_period: Duration::from_secs_f64(
                1.0 / config.damping_response.send_rate_hz,
            ),
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
    fn resolves_cause_specific_responses() {
        let config: GroupConfig = serde_yaml::from_str(
            "name: group_a\nmotors: [motor_a]\nfault_responses: { temperature: disable, stale_feedback: damping, command_timeout: none }",
        )
        .unwrap();
        let spec = GroupSpec::try_from(&config).unwrap();
        assert_eq!(
            spec.fault_responses.temperature,
            crate::config::FaultResponse::Disable
        );
        assert_eq!(
            spec.fault_responses.stale_feedback,
            crate::config::FaultResponse::Damping
        );
        assert_eq!(
            spec.fault_responses.command_timeout,
            crate::config::FaultResponse::None
        );
    }
}
