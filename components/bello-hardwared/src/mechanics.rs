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
    effort: f64,
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
            effort: limits.effort,
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
    motor_torque_limits: HashMap<MotorName, (f64, f64)>,
    motor_damping_kd: HashMap<MotorName, f64>,
    motor_feedback_velocity_limits: HashMap<MotorName, (f64, f64)>,
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
            motor_torque_limits: group_motors
                .iter()
                .map(|name| {
                    let limits = &motors[name].limits;
                    (name.clone(), (limits.torque_min, limits.torque_max))
                })
                .collect(),
            motor_damping_kd: group_motors
                .iter()
                .map(|name| (name.clone(), motors[name].fault_damping_kd))
                .collect(),
            motor_feedback_velocity_limits: group_motors
                .iter()
                .map(|name| {
                    let motor = &motors[name];
                    let limits = if motor.params.direction == 1 {
                        (motor.params.velocity_min, motor.params.velocity_max)
                    } else {
                        (-motor.params.velocity_max, -motor.params.velocity_min)
                    };
                    (name.clone(), limits)
                })
                .collect(),
        })
    }

    pub(crate) fn validate_commands(
        &self,
        commands: &HashMap<MotorName, ImpedanceCommand>,
        feedback: &HashMap<MotorName, Feedback>,
    ) -> Result<(), MechanicalViolation> {
        if let Some(motor) = self
            .motor_torque_limits
            .keys()
            .find(|motor| !commands.contains_key(*motor))
        {
            return Err(MechanicalViolation {
                motor: motor.clone(),
                affected_motors: vec![motor.clone()],
                violation: SafetyViolation::TorqueLimit,
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

        let requested_torques = self
            .motor_torque_limits
            .keys()
            .map(|motor| {
                let command = commands.get(motor).ok_or_else(|| MechanicalViolation {
                    motor: motor.clone(),
                    affected_motors: vec![motor.clone()],
                    violation: SafetyViolation::TorqueLimit,
                    detail: "cannot validate impedance torque without a complete command".into(),
                })?;
                let measured = feedback.get(motor).ok_or_else(|| MechanicalViolation {
                    motor: motor.clone(),
                    affected_motors: vec![motor.clone()],
                    violation: SafetyViolation::TorqueLimit,
                    detail: "cannot validate impedance torque without fresh motor feedback".into(),
                })?;
                validate_finite_feedback(motor, measured)?;
                let requested = command.torque
                    + command.kp * (command.position - measured.position)
                    + command.kd * (command.velocity - measured.velocity);
                if !requested.is_finite() {
                    return Err(nonfinite_feedback_violation(
                        motor,
                        "computed impedance torque",
                    ));
                }
                Ok((motor.clone(), requested))
            })
            .collect::<Result<HashMap<_, _>, _>>()?;
        self.validate_torque_envelope(&requested_torques)
    }

    pub(crate) fn damping_commands(
        &self,
        feedback: &HashMap<MotorName, Feedback>,
    ) -> Result<HashMap<MotorName, ImpedanceCommand>, MechanicalViolation> {
        let mut requested_torques = HashMap::with_capacity(self.motor_torque_limits.len());
        for motor in self.motor_torque_limits.keys() {
            let measured = feedback.get(motor).ok_or_else(|| MechanicalViolation {
                motor: motor.clone(),
                affected_motors: vec![motor.clone()],
                violation: SafetyViolation::TorqueLimit,
                detail: "cannot construct safe damping without fresh motor feedback".into(),
            })?;
            validate_finite_feedback(motor, measured)?;
            let requested = -self.motor_damping_kd[motor] * measured.velocity;
            if !requested.is_finite() {
                return Err(nonfinite_feedback_violation(
                    motor,
                    "computed damping torque",
                ));
            }
            requested_torques.insert(motor.clone(), requested);
        }

        let scale = self.damping_scale()?;
        let scaled_torques = requested_torques
            .iter()
            .map(|(motor, torque)| (motor.clone(), torque * scale))
            .collect();
        self.validate_torque_envelope(&scaled_torques)?;

        Ok(self
            .motor_damping_kd
            .iter()
            .map(|(motor, kd)| {
                let measured = feedback[motor];
                (
                    motor.clone(),
                    ImpedanceCommand {
                        position: measured.position,
                        kd: kd * scale,
                        ..Default::default()
                    },
                )
            })
            .collect())
    }

    fn validate_torque_envelope(
        &self,
        requested_torques: &HashMap<MotorName, f64>,
    ) -> Result<(), MechanicalViolation> {
        for (motor, limits) in &self.motor_torque_limits {
            let requested = requested_torques[motor];
            if !(limits.0..=limits.1).contains(&requested) {
                return Err(MechanicalViolation {
                    motor: motor.clone(),
                    affected_motors: vec![motor.clone()],
                    violation: SafetyViolation::TorqueLimit,
                    detail: format!(
                        "impedance torque request {requested:.4} outside motor limit [{:.4}, {:.4}]",
                        limits.0, limits.1
                    ),
                });
            }
        }
        for direct in &self.direct {
            let torque = requested_torques[&direct.motor];
            if torque.abs() > direct.joint.effort {
                return Err(MechanicalViolation {
                    motor: direct.motor.clone(),
                    affected_motors: vec![direct.motor.clone()],
                    violation: SafetyViolation::TorqueLimit,
                    detail: format!(
                        "joint {:?} impedance torque request {torque:.4} exceeds URDF effort limit {:.4}",
                        direct.joint.name, direct.joint.effort
                    ),
                });
            }
        }
        for differential in &self.differentials {
            let coordinates = differential
                .transmission
                .coordinates
                .iter()
                .zip(&differential.motors)
                .map(|(coordinate, motor)| (coordinate.clone(), requested_torques[motor]))
                .collect::<BTreeMap<_, _>>();
            let joints = differential
                .transmission
                .resolve_torques(&coordinates)
                .expect("complete validated differential torques");
            for ((_, torque), joint) in joints.iter().zip(&differential.joints) {
                if torque.abs() > joint.effort {
                    return Err(MechanicalViolation {
                        motor: differential.motors[0].clone(),
                        affected_motors: differential.motors.to_vec(),
                        violation: SafetyViolation::TorqueLimit,
                        detail: format!(
                            "differential {:?} joint {:?} impedance torque request {torque:.4} exceeds URDF effort limit {:.4}",
                            differential.transmission.name, joint.name, joint.effort
                        ),
                    });
                }
            }
        }
        Ok(())
    }

    fn damping_scale(&self) -> Result<f64, MechanicalViolation> {
        let mut scale = 1.0_f64;
        for (motor, (minimum, maximum)) in &self.motor_torque_limits {
            if *minimum > 0.0 || *maximum < 0.0 {
                return Err(MechanicalViolation {
                    motor: motor.clone(),
                    affected_motors: vec![motor.clone()],
                    violation: SafetyViolation::TorqueLimit,
                    detail: format!(
                        "motor damping requires a torque envelope containing zero, got [{minimum:.4}, {maximum:.4}]"
                    ),
                });
            }
            let (velocity_min, velocity_max) = self.motor_feedback_velocity_limits[motor];
            for torque in [
                -self.motor_damping_kd[motor] * velocity_min,
                -self.motor_damping_kd[motor] * velocity_max,
            ] {
                scale = scale.min(scale_for_interval(torque, *minimum, *maximum));
            }
        }
        for direct in &self.direct {
            let (velocity_min, velocity_max) = self.motor_feedback_velocity_limits[&direct.motor];
            let kd = self.motor_damping_kd[&direct.motor];
            scale = scale.min(scale_for_effort(
                (-kd * velocity_min).abs().max((-kd * velocity_max).abs()),
                direct.joint.effort,
            ));
        }
        for differential in &self.differentials {
            let mut maximum_joint_torque = [0.0_f64; 2];
            for corner in 0..4 {
                let coordinates = differential
                    .transmission
                    .coordinates
                    .iter()
                    .zip(&differential.motors)
                    .enumerate()
                    .map(|(index, (coordinate, motor))| {
                        let limits = self.motor_feedback_velocity_limits[motor];
                        let velocity = if corner & (1 << index) == 0 {
                            limits.0
                        } else {
                            limits.1
                        };
                        (coordinate.clone(), -self.motor_damping_kd[motor] * velocity)
                    })
                    .collect::<BTreeMap<_, _>>();
                for (index, (_, torque)) in differential
                    .transmission
                    .resolve_torques(&coordinates)
                    .expect("complete validated differential torques")
                    .iter()
                    .enumerate()
                {
                    maximum_joint_torque[index] = maximum_joint_torque[index].max(torque.abs());
                }
            }
            for (torque, joint) in maximum_joint_torque.iter().zip(&differential.joints) {
                scale = scale.min(scale_for_effort(*torque, joint.effort));
            }
        }
        if scale < 1.0 {
            scale *= 1.0 - 16.0 * f64::EPSILON;
        }
        Ok(scale)
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

fn validate_finite_feedback(
    motor: &MotorName,
    feedback: &Feedback,
) -> Result<(), MechanicalViolation> {
    for (field, value) in [
        ("feedback position", feedback.position),
        ("feedback velocity", feedback.velocity),
    ] {
        if !value.is_finite() {
            return Err(nonfinite_feedback_violation(motor, field));
        }
    }
    Ok(())
}

fn nonfinite_feedback_violation(motor: &MotorName, field: &'static str) -> MechanicalViolation {
    MechanicalViolation {
        motor: motor.clone(),
        affected_motors: vec![motor.clone()],
        violation: SafetyViolation::NonFinite { field },
        detail: format!("cannot validate mechanical torque: {field} is not finite"),
    }
}

fn scale_for_interval(value: f64, minimum: f64, maximum: f64) -> f64 {
    if value > maximum {
        maximum / value
    } else if value < minimum {
        minimum / value
    } else {
        1.0
    }
}

fn scale_for_effort(value: f64, effort: f64) -> f64 {
    if value.abs() > effort {
        effort / value.abs()
    } else {
        1.0
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
