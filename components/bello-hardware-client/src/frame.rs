use std::sync::Arc;

use bello_hardware_api::MotorCommand;

use crate::inventory::GroupDefinition;
use crate::{Error, MotorGroup};

/// One coordinate's hybrid impedance target, expressed in SI units.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CoordinateTarget {
    position: f64,
    velocity: f64,
    torque_feedforward: f64,
    gains: Option<(f64, f64)>,
}

impl CoordinateTarget {
    /// Creates a position target. Call [`Self::gains`] before adding it to a frame.
    pub fn position(position_radians: f64) -> Self {
        Self {
            position: position_radians,
            velocity: 0.0,
            torque_feedforward: 0.0,
            gains: None,
        }
    }

    /// Creates a pure feed-forward torque target with zero feedback gains.
    pub fn torque(torque_newton_metres: f64) -> Self {
        Self {
            position: 0.0,
            velocity: 0.0,
            torque_feedforward: torque_newton_metres,
            gains: Some((0.0, 0.0)),
        }
    }

    /// Creates zero-velocity damping with no position stiffness.
    pub fn damping(kd: f64) -> Self {
        Self {
            position: 0.0,
            velocity: 0.0,
            torque_feedforward: 0.0,
            gains: Some((0.0, kd)),
        }
    }

    /// Creates a complete hybrid impedance target.
    pub fn hybrid(
        position_radians: f64,
        velocity_radians_per_second: f64,
        torque_feedforward_newton_metres: f64,
        kp: f64,
        kd: f64,
    ) -> Self {
        Self {
            position: position_radians,
            velocity: velocity_radians_per_second,
            torque_feedforward: torque_feedforward_newton_metres,
            gains: Some((kp, kd)),
        }
    }

    pub fn velocity(mut self, velocity_radians_per_second: f64) -> Self {
        self.velocity = velocity_radians_per_second;
        self
    }

    pub fn torque_feedforward(mut self, torque_newton_metres: f64) -> Self {
        self.torque_feedforward = torque_newton_metres;
        self
    }

    pub fn gains(mut self, kp: f64, kd: f64) -> Self {
        self.gains = Some((kp, kd));
        self
    }

    fn validate(self, coordinate: &str) -> Result<ValidatedTarget, Error> {
        let (kp, kd) = self.gains.ok_or_else(|| Error::InvalidTarget {
            coordinate: coordinate.into(),
            detail: "position targets require explicit kp and kd gains".into(),
        })?;
        for (field, value) in [
            ("position", self.position),
            ("velocity", self.velocity),
            ("torque feed-forward", self.torque_feedforward),
            ("kp", kp),
            ("kd", kd),
        ] {
            if !value.is_finite() {
                return Err(Error::InvalidTarget {
                    coordinate: coordinate.into(),
                    detail: format!("{field} must be finite"),
                });
            }
        }
        if kp < 0.0 || kd < 0.0 {
            return Err(Error::InvalidTarget {
                coordinate: coordinate.into(),
                detail: "kp and kd must be non-negative".into(),
            });
        }
        Ok(ValidatedTarget {
            position: self.position,
            velocity: self.velocity,
            torque_feedforward: self.torque_feedforward,
            kp,
            kd,
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct ValidatedTarget {
    position: f64,
    velocity: f64,
    torque_feedforward: f64,
    kp: f64,
    kd: f64,
}

/// Builder for one complete, locally validated motor-group command snapshot.
#[derive(Debug)]
pub struct CommandFrameBuilder {
    group: Arc<GroupDefinition>,
    targets: Vec<Option<ValidatedTarget>>,
}

impl CommandFrameBuilder {
    pub(crate) fn new(group: Arc<GroupDefinition>) -> Self {
        Self {
            targets: vec![None; group.motors.len()],
            group,
        }
    }

    pub fn coordinate(
        mut self,
        coordinate: impl AsRef<str>,
        target: CoordinateTarget,
    ) -> Result<Self, Error> {
        let coordinate = coordinate.as_ref();
        let Some(&index) = self.group.coordinate_indices.get(coordinate) else {
            return Err(Error::UnknownCoordinate {
                group: self.group.name.clone(),
                coordinate: coordinate.into(),
            });
        };
        if self.targets[index].is_some() {
            return Err(Error::DuplicateCoordinate {
                group: self.group.name.clone(),
                coordinate: coordinate.into(),
            });
        }
        self.targets[index] = Some(target.validate(coordinate)?);
        Ok(self)
    }

    pub fn build(self) -> Result<CommandFrame, Error> {
        let missing = self
            .targets
            .iter()
            .enumerate()
            .filter(|(_, target)| target.is_none())
            .map(|(index, _)| self.group.motors[index].coordinate().to_owned())
            .collect::<Vec<_>>();
        if !missing.is_empty() {
            return Err(Error::IncompleteFrame {
                group: self.group.name.clone(),
                missing,
            });
        }
        let commands = self
            .group
            .motors
            .iter()
            .zip(self.targets)
            .map(|(motor, target)| {
                let target = target.expect("missing targets were checked above");
                MotorCommand {
                    motor_name: motor.name().into(),
                    position: target.position,
                    velocity: target.velocity,
                    torque_feedforward: target.torque_feedforward,
                    kp: target.kp,
                    kd: target.kd,
                }
            })
            .collect();
        Ok(CommandFrame {
            group: self.group.name.clone(),
            commands,
        })
    }
}

/// Complete command snapshot for one motor group.
#[derive(Clone, Debug, PartialEq)]
pub struct CommandFrame {
    group: String,
    commands: Vec<MotorCommand>,
}

impl CommandFrame {
    pub fn group(&self) -> &str {
        &self.group
    }

    pub fn len(&self) -> usize {
        self.commands.len()
    }

    pub fn is_empty(&self) -> bool {
        self.commands.is_empty()
    }

    pub(crate) fn into_commands(self, expected_group: &str) -> Result<Vec<MotorCommand>, Error> {
        if self.group != expected_group {
            return Err(Error::FrameGroupMismatch {
                expected: expected_group.into(),
                actual: self.group,
            });
        }
        Ok(self.commands)
    }
}

impl MotorGroup {
    pub fn frame(&self) -> CommandFrameBuilder {
        CommandFrameBuilder::new(Arc::clone(&self.definition))
    }
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeMap;

    use super::*;
    use crate::GroupState;
    use crate::inventory::Motor;

    fn group() -> Arc<GroupDefinition> {
        let motors: Arc<[Motor]> = vec![
            Motor {
                name: "motor_a".into(),
                coordinate: "coordinate_a".into(),
                group: "pair".into(),
                bus: "can0".into(),
                id: 1,
                model: "test".into(),
                direction: 1,
                initial_status: None,
            },
            Motor {
                name: "motor_b".into(),
                coordinate: "coordinate_b".into(),
                group: "pair".into(),
                bus: "can0".into(),
                id: 2,
                model: "test".into(),
                direction: 1,
                initial_status: None,
            },
        ]
        .into();
        Arc::new(GroupDefinition {
            name: "pair".into(),
            control_rate_hz: 250.0,
            initial_state: GroupState::Idle,
            coordinate_indices: motors
                .iter()
                .enumerate()
                .map(|(index, motor)| (motor.coordinate().to_owned(), index))
                .collect::<BTreeMap<_, _>>(),
            motors,
        })
    }

    #[test]
    fn complete_frame_maps_coordinates_to_motor_commands() {
        let group = group();
        let frame = CommandFrameBuilder::new(group)
            .coordinate("coordinate_b", CoordinateTarget::torque(2.0))
            .unwrap()
            .coordinate(
                "coordinate_a",
                CoordinateTarget::position(1.0).gains(20.0, 1.0),
            )
            .unwrap()
            .build()
            .unwrap();
        assert_eq!(frame.commands[0].motor_name, "motor_a");
        assert_eq!(frame.commands[0].position, 1.0);
        assert_eq!(frame.commands[1].motor_name, "motor_b");
        assert_eq!(frame.commands[1].torque_feedforward, 2.0);
    }

    #[test]
    fn incomplete_and_duplicate_frames_fail_locally() {
        let group = group();
        let incomplete = CommandFrameBuilder::new(Arc::clone(&group))
            .coordinate("coordinate_a", CoordinateTarget::damping(1.0))
            .unwrap()
            .build();
        assert!(matches!(incomplete, Err(Error::IncompleteFrame { .. })));

        let duplicate = CommandFrameBuilder::new(group)
            .coordinate("coordinate_a", CoordinateTarget::damping(1.0))
            .unwrap()
            .coordinate("coordinate_a", CoordinateTarget::damping(1.0));
        assert!(matches!(duplicate, Err(Error::DuplicateCoordinate { .. })));
    }
}
