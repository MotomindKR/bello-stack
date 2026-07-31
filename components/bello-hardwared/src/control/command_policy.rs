use crate::config::{CommandBehavior, MotorCommandPolicy};
use crate::domain::{ImpedanceCommand, SafetyViolation};
use crate::hardware::robstride::MotorSpec;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum CommandPolicyCheck {
    CommandLimit,
    PositionStep,
}

impl CommandPolicyCheck {
    pub(crate) fn as_str(self) -> &'static str {
        match self {
            Self::CommandLimit => "command_limit",
            Self::PositionStep => "command_position_step",
        }
    }
}

#[derive(Clone, Debug)]
pub(crate) struct CommandPolicyIssue {
    pub(crate) check: CommandPolicyCheck,
    pub(crate) behavior: CommandBehavior,
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

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) enum PositionReference {
    Measured(f64),
    PreviousTarget(f64),
}

impl PositionReference {
    fn value(self) -> f64 {
        match self {
            Self::Measured(value) | Self::PreviousTarget(value) => value,
        }
    }

    fn description(self) -> &'static str {
        match self {
            Self::Measured(_) => "measured position",
            Self::PreviousTarget(_) => "previous accepted target",
        }
    }
}

pub(crate) fn select_position_reference(
    same_lease_generation: bool,
    previous_target: Option<f64>,
    measured_position: Option<f64>,
) -> Option<PositionReference> {
    if same_lease_generation {
        previous_target.map(PositionReference::PreviousTarget)
    } else {
        measured_position.map(PositionReference::Measured)
    }
}

pub(crate) fn coupled_limit_behavior<'a>(
    policies: impl IntoIterator<Item = &'a MotorCommandPolicy>,
) -> CommandBehavior {
    for policy in policies {
        if policy.limit == CommandBehavior::Fault {
            return CommandBehavior::Fault;
        }
    }
    CommandBehavior::Reject
}

pub(crate) fn check_command(
    mut command: ImpedanceCommand,
    position_reference: Option<PositionReference>,
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
                behavior: CommandBehavior::Fault,
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
            motor.limits.velocity_min,
            motor.limits.velocity_max,
        ),
        (
            SafetyViolation::TorqueLimit,
            &mut command.torque,
            motor.limits.torque_min,
            motor.limits.torque_max,
        ),
    ];
    for (violation, value, minimum, maximum) in limits {
        if !(minimum..=maximum).contains(value) {
            let original = *value;
            let behavior = motor.command_policy.limit;
            let modified = behavior == CommandBehavior::Clamp;
            if modified {
                *value = value.clamp(minimum, maximum);
            }
            issues.push(CommandPolicyIssue {
                check: CommandPolicyCheck::CommandLimit,
                behavior,
                violation,
                detail: format!("command value {original} outside [{minimum}, {maximum}]"),
                command_modified: modified,
                command_rejected: !modified,
            });
        }
    }

    if let Some(reference) = position_reference {
        let reference_position = reference.value();
        let delta = command.position - reference_position;
        if delta.abs() > motor.command_policy.maximum_position_step {
            let behavior = motor.command_policy.position_step;
            let modified = behavior == CommandBehavior::Clamp;
            if modified {
                command.position = reference_position
                    + delta.clamp(
                        -motor.command_policy.maximum_position_step,
                        motor.command_policy.maximum_position_step,
                    );
            }
            issues.push(CommandPolicyIssue {
                check: CommandPolicyCheck::PositionStep,
                behavior,
                violation: SafetyViolation::PositionStep,
                detail: format!(
                    "position target step {delta} from {} {reference_position} exceeds {}",
                    reference.description(),
                    motor.command_policy.maximum_position_step
                ),
                command_modified: modified,
                command_rejected: !modified,
            });
        }
    }

    CommandPolicyReport { command, issues }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::ResolvedTopology;

    fn motor() -> MotorSpec {
        ResolvedTopology::load(concat!(env!("CARGO_MANIFEST_DIR"), "/config/bello.yaml"))
            .unwrap()
            .motors()
            .find(|(_, motor)| motor.spec.name.as_str() == "left_hip_pitch")
            .unwrap()
            .1
            .spec
            .clone()
    }

    #[test]
    fn warning_limit_can_clamp_and_continue() {
        let mut motor = motor();
        motor.command_policy.limit = CommandBehavior::Clamp;
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
    fn feedforward_torque_remains_clamped() {
        let motor = motor();
        let report = check_command(
            ImpedanceCommand {
                torque: motor.limits.torque_max + 1.0,
                ..Default::default()
            },
            None,
            &motor,
        );
        assert_eq!(report.command.torque, motor.limits.torque_max);
        assert_eq!(report.issues[0].violation, SafetyViolation::TorqueLimit);
        assert!(report.issues[0].command_modified);
        assert!(!report.issues[0].command_rejected);
    }

    #[test]
    fn coupled_limit_faults_if_any_affected_motor_requests_fault() {
        let first = MotorCommandPolicy::default();
        let mut second = first.clone();
        assert_eq!(
            coupled_limit_behavior([&first, &second]),
            CommandBehavior::Reject
        );

        second.limit = CommandBehavior::Clamp;
        assert_eq!(
            coupled_limit_behavior([&first, &second]),
            CommandBehavior::Reject
        );

        second.limit = CommandBehavior::Fault;
        assert_eq!(
            coupled_limit_behavior([&first, &second]),
            CommandBehavior::Fault
        );
    }

    #[test]
    fn finite_gains_are_not_limited() {
        let motor = motor();
        let command = ImpedanceCommand {
            kp: motor.limits.kp_max * 2.0,
            kd: motor.limits.kd_max * 2.0,
            ..Default::default()
        };
        let report = check_command(command, None, &motor);
        assert!(report.issues.is_empty());
        assert_eq!(report.command, command);
    }

    #[test]
    fn position_step_reports_its_reference() {
        let mut motor = motor();
        motor.command_policy.maximum_position_step = 0.5;
        let report = check_command(
            ImpedanceCommand {
                position: 1.0,
                ..Default::default()
            },
            Some(PositionReference::Measured(0.0)),
            &motor,
        );
        assert_eq!(report.issues[0].check, CommandPolicyCheck::PositionStep);
        assert!(report.issues[0].detail.contains("measured position"));

        let report = check_command(
            ImpedanceCommand {
                position: 1.0,
                ..Default::default()
            },
            Some(PositionReference::PreviousTarget(0.25)),
            &motor,
        );
        assert_eq!(report.issues[0].check, CommandPolicyCheck::PositionStep);
        assert!(report.issues[0].detail.contains("previous accepted target"));
    }

    #[test]
    fn canonical_target_step_rejects_more_than_two_radians() {
        let motor = motor();
        assert_eq!(motor.command_policy.maximum_position_step, 2.0);
        let report = check_command(
            ImpedanceCommand {
                position: 1.0,
                ..Default::default()
            },
            Some(PositionReference::Measured(-1.001)),
            &motor,
        );
        assert_eq!(report.issues.len(), 1);
        assert_eq!(report.issues[0].check, CommandPolicyCheck::PositionStep);
        assert_eq!(report.issues[0].behavior, CommandBehavior::Reject);
        assert!(report.issues[0].command_rejected);
    }

    #[test]
    fn first_target_uses_feedback_then_later_targets_use_the_last_accepted_target() {
        assert_eq!(
            select_position_reference(false, Some(0.75), Some(0.1)),
            Some(PositionReference::Measured(0.1))
        );
        assert_eq!(
            select_position_reference(true, Some(0.75), Some(0.1)),
            Some(PositionReference::PreviousTarget(0.75))
        );
    }
}
