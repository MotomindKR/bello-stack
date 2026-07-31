use std::collections::{BTreeMap, HashMap, HashSet};

use anyhow::{Context, Result, bail};
use bello_robot_description::{
    CoordinateState, DifferentialTransmission, JointKind, JointLimits, RobotDescription,
};

use crate::domain::{ImpedanceCommand, MotorName, SafetyViolation};
use crate::hardware::robstride::{Feedback, MotorSpec};

#[derive(Clone, Debug)]
struct JointEnvelope {
    name: String,
    lower: f64,
    upper: f64,
    velocity: f64,
}

impl JointEnvelope {
    fn from_description(description: &RobotDescription, name: &str) -> Result<Self> {
        let joint = description
            .joint(name)
            .with_context(|| format!("robot description has no joint {name:?}"))?;
        if !matches!(joint.kind, JointKind::Revolute | JointKind::Prismatic) {
            bail!("controlled coordinate {name:?} resolves to a fixed joint");
        }
        let limits = joint
            .limits
            .as_ref()
            .with_context(|| format!("controlled joint {name:?} has no mechanical limits"))?;
        Ok(Self::new(name, limits))
    }

    fn new(name: &str, limits: &JointLimits) -> Self {
        Self {
            name: name.to_owned(),
            lower: limits.lower,
            upper: limits.upper,
            velocity: limits.velocity,
        }
    }
}

#[derive(Clone, Debug)]
struct DirectEnvelope {
    motor: MotorName,
    joint: JointEnvelope,
}

#[derive(Clone, Debug)]
struct DifferentialEnvelope {
    transmission: DifferentialTransmission,
    motors: [MotorName; 2],
    joints: [JointEnvelope; 2],
    feedback_position_tolerance: [f64; 2],
}

#[derive(Clone, Debug)]
pub(crate) struct MechanicalViolation {
    pub(crate) motor: MotorName,
    pub(crate) affected_motors: Vec<MotorName>,
    pub(crate) violation: SafetyViolation,
    pub(crate) detail: String,
}

#[derive(Clone, Debug, Default)]
pub(crate) struct MechanicalEnvelope {
    direct: Vec<DirectEnvelope>,
    differentials: Vec<DifferentialEnvelope>,
    motor_damping_kd: HashMap<MotorName, f64>,
}

impl MechanicalEnvelope {
    pub(crate) fn build(
        description: &RobotDescription,
        group_motors: &[MotorName],
        motors: &HashMap<MotorName, MotorSpec>,
    ) -> Result<Self> {
        let group_set = group_motors.iter().cloned().collect::<HashSet<_>>();
        let coordinate_motors = motors
            .values()
            .map(|motor| (motor.coordinate.as_str(), motor.name.clone()))
            .collect::<HashMap<_, _>>();
        let mut direct = Vec::new();
        let mut differentials = Vec::new();
        let mut used_transmissions = HashSet::new();

        for motor_name in group_motors {
            let motor = motors
                .get(motor_name)
                .with_context(|| format!("group refers to unknown motor {motor_name}"))?;
            if let Some(transmission) = description.transmission_for_coordinate(&motor.coordinate) {
                if !used_transmissions.insert(transmission.name.as_str()) {
                    continue;
                }
                let transmission_motors = transmission
                    .coordinates
                    .clone()
                    .map(|coordinate| coordinate_motors.get(coordinate.as_str()).cloned());
                let [Some(first), Some(second)] = transmission_motors else {
                    bail!(
                        "differential transmission {:?} must have motors for both coordinates {:?}",
                        transmission.name,
                        transmission.coordinates
                    );
                };
                if !group_set.contains(&first) || !group_set.contains(&second) {
                    bail!(
                        "differential transmission {:?} coordinates must belong to one motor group",
                        transmission.name
                    );
                }
                let coordinate_tolerance = [
                    motors[&first].feedback_position_tolerance,
                    motors[&second].feedback_position_tolerance,
                ];
                let summed_coordinate_tolerance = coordinate_tolerance[0]
                    / transmission.coordinate_reduction[0].abs()
                    + coordinate_tolerance[1] / transmission.coordinate_reduction[1].abs();
                differentials.push(DifferentialEnvelope {
                    transmission: transmission.clone(),
                    motors: [first, second],
                    joints: [
                        JointEnvelope::from_description(description, &transmission.joints[0])?,
                        JointEnvelope::from_description(description, &transmission.joints[1])?,
                    ],
                    feedback_position_tolerance: [
                        summed_coordinate_tolerance / (2.0 * transmission.joint_reduction[0].abs()),
                        summed_coordinate_tolerance / (2.0 * transmission.joint_reduction[1].abs()),
                    ],
                });
            } else {
                direct.push(DirectEnvelope {
                    motor: motor.name.clone(),
                    joint: JointEnvelope::from_description(description, &motor.coordinate)?,
                });
            }
        }
        Ok(Self {
            direct,
            differentials,
            motor_damping_kd: group_motors
                .iter()
                .map(|name| (name.clone(), motors[name].fault_damping_kd))
                .collect(),
        })
    }

    pub(crate) fn validate_targets(
        &self,
        commands: &HashMap<MotorName, ImpedanceCommand>,
    ) -> Result<(), MechanicalViolation> {
        if let Some(motor) = self
            .motor_damping_kd
            .keys()
            .find(|motor| !commands.contains_key(*motor))
        {
            return Err(MechanicalViolation {
                motor: motor.clone(),
                affected_motors: vec![motor.clone()],
                violation: SafetyViolation::PositionLimit,
                detail: "cannot validate an incomplete mechanical command".into(),
            });
        }
        for direct in &self.direct {
            let command = commands[&direct.motor];
            validate_joint_target(&direct.joint, command.position, command.velocity).map_err(
                |(violation, detail)| MechanicalViolation {
                    motor: direct.motor.clone(),
                    affected_motors: vec![direct.motor.clone()],
                    violation,
                    detail,
                },
            )?;
        }
        for differential in &self.differentials {
            let states = differential
                .transmission
                .coordinates
                .iter()
                .zip(&differential.motors)
                .map(|(coordinate, motor)| {
                    let command = commands[motor];
                    (
                        coordinate.clone(),
                        CoordinateState {
                            position: command.position,
                            velocity: command.velocity,
                        },
                    )
                })
                .collect::<BTreeMap<_, _>>();
            let joints = differential
                .transmission
                .resolve(&states)
                .expect("complete validated differential coordinates");
            for ((_, state), joint) in joints.iter().zip(&differential.joints) {
                validate_joint_target(joint, state.position, state.velocity).map_err(
                    |(violation, detail)| MechanicalViolation {
                        motor: differential.motors[0].clone(),
                        affected_motors: differential.motors.to_vec(),
                        violation,
                        detail: format!(
                            "differential {:?}: {detail}",
                            differential.transmission.name
                        ),
                    },
                )?;
            }
        }
        Ok(())
    }

    pub(crate) fn damping_commands(
        &self,
        feedback: &HashMap<MotorName, Feedback>,
    ) -> Result<HashMap<MotorName, ImpedanceCommand>, MechanicalViolation> {
        for motor in self.motor_damping_kd.keys() {
            let measured = feedback.get(motor).ok_or_else(|| MechanicalViolation {
                motor: motor.clone(),
                affected_motors: vec![motor.clone()],
                violation: SafetyViolation::PositionLimit,
                detail: "cannot construct damping without fresh motor feedback".into(),
            })?;
            validate_finite_feedback_position(motor, measured)?;
        }

        Ok(self
            .motor_damping_kd
            .iter()
            .map(|(motor, kd)| {
                let measured = feedback[motor];
                (
                    motor.clone(),
                    ImpedanceCommand {
                        position: measured.position,
                        kd: *kd,
                        ..Default::default()
                    },
                )
            })
            .collect())
    }

    pub(crate) fn validate_feedback_positions(
        &self,
        feedback: &HashMap<MotorName, Feedback>,
    ) -> Result<(), MechanicalViolation> {
        for differential in &self.differentials {
            let Some(states) = differential
                .transmission
                .coordinates
                .iter()
                .zip(&differential.motors)
                .map(|(coordinate, motor)| {
                    feedback.get(motor).map(|measured| {
                        (
                            coordinate.clone(),
                            CoordinateState {
                                position: measured.position,
                                velocity: measured.velocity,
                            },
                        )
                    })
                })
                .collect::<Option<BTreeMap<_, _>>>()
            else {
                continue;
            };
            let joints = differential
                .transmission
                .resolve(&states)
                .expect("complete validated differential coordinates");
            for (((_, state), joint), tolerance) in joints
                .iter()
                .zip(&differential.joints)
                .zip(differential.feedback_position_tolerance)
            {
                if state.position < joint.lower - tolerance
                    || state.position > joint.upper + tolerance
                {
                    return Err(MechanicalViolation {
                        motor: differential.motors[0].clone(),
                        affected_motors: differential.motors.to_vec(),
                        violation: SafetyViolation::PositionLimit,
                        detail: format!(
                            "differential {:?} feedback resolves joint {:?} position {:.4} outside URDF limit [{:.4}, {:.4}] (feedback tolerance {:.6})",
                            differential.transmission.name,
                            joint.name,
                            state.position,
                            joint.lower,
                            joint.upper,
                            tolerance
                        ),
                    });
                }
            }
        }
        Ok(())
    }

    #[cfg(test)]
    pub(crate) fn differential_count(&self) -> usize {
        self.differentials.len()
    }
}

fn validate_finite_feedback_position(
    motor: &MotorName,
    feedback: &Feedback,
) -> Result<(), MechanicalViolation> {
    if !feedback.position.is_finite() {
        return Err(nonfinite_feedback_violation(motor, "feedback position"));
    }
    Ok(())
}

fn nonfinite_feedback_violation(motor: &MotorName, field: &'static str) -> MechanicalViolation {
    MechanicalViolation {
        motor: motor.clone(),
        affected_motors: vec![motor.clone()],
        violation: SafetyViolation::NonFinite { field },
        detail: format!("{field} is not finite"),
    }
}

pub(crate) fn apply_coordinate_limits(
    description: &RobotDescription,
    motor: &mut MotorSpec,
) -> Result<()> {
    let position = description
        .coordinate_position_limits(&motor.coordinate)
        .with_context(|| {
            format!(
                "motor {} coordinate {:?} has no URDF position limits",
                motor.name, motor.coordinate
            )
        })?;
    let velocity = description
        .coordinate_velocity_limit(&motor.coordinate)
        .with_context(|| {
            format!(
                "motor {} coordinate {:?} has no URDF velocity limit",
                motor.name, motor.coordinate
            )
        })?;
    let effort = description
        .coordinate_effort_limit(&motor.coordinate)
        .with_context(|| {
            format!(
                "motor {} coordinate {:?} has no URDF effort limit",
                motor.name, motor.coordinate
            )
        })?;

    motor.limits.position_min = motor.limits.position_min.max(position.0);
    motor.limits.position_max = motor.limits.position_max.min(position.1);
    motor.limits.velocity_min = motor.limits.velocity_min.max(-velocity);
    motor.limits.velocity_max = motor.limits.velocity_max.min(velocity);
    motor.limits.torque_min = motor.limits.torque_min.max(-effort);
    motor.limits.torque_max = motor.limits.torque_max.min(effort);
    motor.health.safe_position_min = motor.health.safe_position_min.max(position.0);
    motor.health.safe_position_max = motor.health.safe_position_max.min(position.1);

    if motor.limits.position_min >= motor.limits.position_max
        || motor.limits.velocity_min >= motor.limits.velocity_max
        || motor.limits.torque_min >= motor.limits.torque_max
        || motor.health.safe_position_min >= motor.health.safe_position_max
        || motor.health.safe_position_min > motor.limits.position_min
        || motor.health.safe_position_max < motor.limits.position_max
    {
        bail!(
            "motor {} configured limits do not overlap the URDF envelope for coordinate {:?}",
            motor.name,
            motor.coordinate
        );
    }
    Ok(())
}

fn validate_joint_target(
    joint: &JointEnvelope,
    position: f64,
    velocity: f64,
) -> Result<(), (SafetyViolation, String)> {
    if !(joint.lower..=joint.upper).contains(&position) {
        return Err((
            SafetyViolation::PositionLimit,
            format!(
                "joint {:?} target {position:.4} outside URDF limit [{:.4}, {:.4}]",
                joint.name, joint.lower, joint.upper
            ),
        ));
    }
    if velocity.abs() > joint.velocity {
        return Err((
            SafetyViolation::VelocityLimit,
            format!(
                "joint {:?} target velocity {velocity:.4} exceeds URDF limit {:.4}",
                joint.name, joint.velocity
            ),
        ));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn damping_preserves_configured_gain_above_protocol_range() {
        let motor = MotorName::from("motor");
        let envelope = MechanicalEnvelope {
            motor_damping_kd: HashMap::from([(motor.clone(), 200.0)]),
            ..MechanicalEnvelope::default()
        };
        let feedback = HashMap::from([(
            motor.clone(),
            Feedback {
                position: 0.5,
                ..Feedback::default()
            },
        )]);

        let commands = envelope.damping_commands(&feedback).unwrap();
        assert_eq!(commands[&motor].position, 0.5);
        assert_eq!(commands[&motor].kd, 200.0);
    }
}
