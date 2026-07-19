use crate::config::{CommandLimitAction, CommandViolationSeverity};
use crate::domain::{ImpedanceCommand, SafetyViolation};
use crate::hardware::robstride::{Feedback, MotorSpec};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum CommandPolicyCheck {
    CommandLimit,
    PositionStep,
}

impl CommandPolicyCheck {
    pub(crate) fn as_str(self) -> &'static str {
        match self {
            Self::CommandLimit => "command_limit",
            Self::PositionStep => "command_position_step_limit",
        }
    }
}

#[derive(Clone, Debug)]
pub(crate) struct CommandPolicyIssue {
    pub(crate) check: CommandPolicyCheck,
    pub(crate) severity: CommandViolationSeverity,
    pub(crate) violation: SafetyViolation,
    pub(crate) detail: String,
    pub(crate) command_modified: bool,
    pub(crate) command_rejected: bool,
}

#[derive(Clone, Debug)]
pub(crate) struct CommandPolicyReport {
    pub(crate) command: ImpedanceCommand,
    pub(crate) issues: Vec<CommandPolicyIssue>,
}

pub(crate) fn check_command(
    mut command: ImpedanceCommand,
    measured_position: Option<f64>,
    motor: &MotorSpec,
) -> CommandPolicyReport {
    let mut issues = Vec::new();
    for (label, value) in [
        ("position", command.position),
        ("velocity", command.velocity),
        ("torque", command.torque),
        ("kp", command.kp),
        ("kd", command.kd),
    ] {
        if !value.is_finite() {
            issues.push(CommandPolicyIssue {
                check: CommandPolicyCheck::CommandLimit,
                severity: CommandViolationSeverity::Fault,
                violation: SafetyViolation::NonFinite { field: label },
                detail: format!("{label} command is not finite"),
                command_modified: false,
                command_rejected: true,
            });
        }
    }
    if !issues.is_empty() {
        return CommandPolicyReport { command, issues };
    }

    let limits = [
        (
            SafetyViolation::PositionLimit,
            &mut command.position,
            motor.limits.position_min,
            motor.limits.position_max,
        ),
        (
            SafetyViolation::VelocityLimit,
            &mut command.velocity,
            motor.params.velocity_min,
            motor.params.velocity_max,
        ),
        (
            SafetyViolation::TorqueLimit,
            &mut command.torque,
            motor.params.torque_min,
            motor.params.torque_max,
        ),
        (
            SafetyViolation::GainLimit,
            &mut command.kp,
            motor.params.kp_min,
            motor.params.kp_max,
        ),
        (
            SafetyViolation::GainLimit,
            &mut command.kd,
            motor.params.kd_min,
            motor.params.kd_max,
        ),
    ];
    for (violation, value, minimum, maximum) in limits {
        if !(minimum..=maximum).contains(value) {
            let original = *value;
            let modified = motor.command_policy.limit_action == CommandLimitAction::Clamp;
            if modified {
                *value = value.clamp(minimum, maximum);
            }
            issues.push(CommandPolicyIssue {
                check: CommandPolicyCheck::CommandLimit,
                severity: motor.command_policy.limit_severity,
                violation,
                detail: format!("command value {original} outside [{minimum}, {maximum}]"),
                command_modified: modified,
                command_rejected: !modified,
            });
        }
    }

    if let Some(measured_position) = measured_position {
        let delta = command.position - measured_position;
        if delta.abs() > motor.command_policy.maximum_position_step {
            let modified = motor.command_policy.position_step_action == CommandLimitAction::Clamp;
            if modified {
                command.position = measured_position
                    + delta.clamp(
                        -motor.command_policy.maximum_position_step,
                        motor.command_policy.maximum_position_step,
                    );
            }
            issues.push(CommandPolicyIssue {
                    check: CommandPolicyCheck::PositionStep,
                    severity: motor.command_policy.position_step_severity,
                    violation: SafetyViolation::PositionStep,
                    detail: format!(
                        "position target step {delta} from measured position {measured_position} exceeds {}",
                        motor.command_policy.maximum_position_step
                    ),
                    command_modified: modified,
                    command_rejected: !modified,
                });
        }
    }

    CommandPolicyReport { command, issues }
}

pub(crate) fn damping_command(feedback: Option<Feedback>, motor: &MotorSpec) -> ImpedanceCommand {
    ImpedanceCommand {
        position: feedback.map_or(0.0, |value| value.position),
        kd: motor.fault_damping.kd,
        ..Default::default()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::ResolvedTopology;

    fn motor() -> MotorSpec {
        ResolvedTopology::load(concat!(env!("CARGO_MANIFEST_DIR"), "/config/bello.yaml"))
            .unwrap()
            .motors()
            .next()
            .unwrap()
            .1
            .spec
            .clone()
    }

    #[test]
    fn warning_limit_can_clamp_and_continue() {
        let mut motor = motor();
        motor.command_policy.limit_action = CommandLimitAction::Clamp;
        motor.command_policy.limit_severity = CommandViolationSeverity::Warning;
        let report = check_command(
            ImpedanceCommand {
                position: motor.limits.position_max + 1.0,
                ..Default::default()
            },
            None,
            &motor,
        );
        assert_eq!(report.command.position, motor.limits.position_max);
        assert!(report.issues[0].command_modified);
        assert!(!report.issues[0].command_rejected);
    }

    #[test]
    fn position_step_is_measured_from_feedback() {
        let mut motor = motor();
        motor.command_policy.maximum_position_step = 0.5;
        let report = check_command(
            ImpedanceCommand {
                position: 1.0,
                ..Default::default()
            },
            Some(0.0),
            &motor,
        );
        assert_eq!(report.issues[0].check, CommandPolicyCheck::PositionStep);
    }
}
