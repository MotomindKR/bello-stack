use bello_hardware_api::MotorCommand;

use crate::inventory::{GroupDefinition, Motor};
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

    fn into_command(self, motor: &Motor) -> Result<MotorCommand, Error> {
        let coordinate = motor.coordinate();
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
        Ok(MotorCommand {
            motor_name: motor.name().into(),
            position: self.position,
            velocity: self.velocity,
            torque_feedforward: self.torque_feedforward,
            kp,
            kd,
        })
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct CommandFrame {
    group: String,
    commands: Vec<MotorCommand>,
}

impl CommandFrame {
    fn new(
        group: &GroupDefinition,
        targets: impl IntoIterator<Item = (impl AsRef<str>, CoordinateTarget)>,
    ) -> Result<Self, Error> {
        let mut commands = vec![None; group.motors.len()];
        for (coordinate, target) in targets {
            let coordinate = coordinate.as_ref();
            let Some(&index) = group.coordinate_indices.get(coordinate) else {
                return Err(Error::UnknownCoordinate {
                    group: group.name.clone(),
                    coordinate: coordinate.into(),
                });
            };
            if commands[index].is_some() {
                return Err(Error::DuplicateCoordinate {
                    group: group.name.clone(),
                    coordinate: coordinate.into(),
                });
            }
            commands[index] = Some(target.into_command(&group.motors[index])?);
        }

        let missing = group
            .motors
            .iter()
            .zip(&commands)
            .filter(|(_, command)| command.is_none())
            .map(|(motor, _)| motor.coordinate().to_owned())
            .collect::<Vec<_>>();
        if !missing.is_empty() {
            return Err(Error::IncompleteFrame {
                group: group.name.clone(),
                missing,
            });
        }

        Ok(Self {
            group: group.name.clone(),
            commands: commands.into_iter().flatten().collect(),
        })
    }

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
    /// Creates a command frame from one target for each coordinate in the group.
    pub fn frame(
        &self,
        targets: impl IntoIterator<Item = (impl AsRef<str>, CoordinateTarget)>,
    ) -> Result<CommandFrame, Error> {
        CommandFrame::new(&self.definition, targets)
    }
}

#[cfg(test)]
mod tests {
    use std::{collections::BTreeMap, sync::Arc};

    use super::*;
    use crate::GroupState;
    use crate::inventory::MotorLimits;

    fn limits() -> MotorLimits {
        MotorLimits {
            position_min: -1.0,
            position_max: 1.0,
            velocity_min: -2.0,
            velocity_max: 2.0,
            torque_min: -3.0,
            torque_max: 3.0,
            kp_min: 0.0,
            kp_max: 100.0,
            kd_min: 0.0,
            kd_max: 10.0,
        }
    }

    fn group() -> GroupDefinition {
        let motors: Arc<[Motor]> = vec![
            Motor {
                name: "motor_a".into(),
                coordinate: "coordinate_a".into(),
                group: "pair".into(),
                bus: "can0".into(),
                id: 1,
                model: "test".into(),
                firmware_version: None,
                direction: 1,
                limits: limits(),
                initial_status: None,
            },
            Motor {
                name: "motor_b".into(),
                coordinate: "coordinate_b".into(),
                group: "pair".into(),
                bus: "can0".into(),
                id: 2,
                model: "test".into(),
                firmware_version: None,
                direction: 1,
                limits: limits(),
                initial_status: None,
            },
        ]
        .into();
        GroupDefinition {
            name: "pair".into(),
            control_rate_hz: 250.0,
            initial_state: crate::EnumValue::Known(GroupState::Idle),
            coordinate_indices: motors
                .iter()
                .enumerate()
                .map(|(index, motor)| (motor.coordinate().to_owned(), index))
                .collect::<BTreeMap<_, _>>(),
            motors,
        }
    }

    #[test]
    fn complete_frame_maps_coordinates_to_motor_commands() {
        let group = group();
        let frame = CommandFrame::new(
            &group,
            [
                ("coordinate_b", CoordinateTarget::torque(2.0)),
                (
                    "coordinate_a",
                    CoordinateTarget::position(1.0).gains(20.0, 1.0),
                ),
            ],
        )
        .unwrap();
        assert_eq!(frame.commands[0].motor_name, "motor_a");
        assert_eq!(frame.commands[0].position, 1.0);
        assert_eq!(frame.commands[1].motor_name, "motor_b");
        assert_eq!(frame.commands[1].torque_feedforward, 2.0);
    }

    #[test]
    fn incomplete_and_duplicate_frames_fail_locally() {
        let group = group();
        let target = CoordinateTarget::damping(1.0);
        let incomplete = CommandFrame::new(&group, [("coordinate_a", target)]);
        assert!(matches!(incomplete, Err(Error::IncompleteFrame { .. })));

        let duplicate =
            CommandFrame::new(&group, [("coordinate_a", target), ("coordinate_a", target)]);
        assert!(matches!(duplicate, Err(Error::DuplicateCoordinate { .. })));
    }
}
