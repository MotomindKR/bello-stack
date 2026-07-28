use std::collections::BTreeMap;
use std::time::Duration;

use nalgebra::{DMatrix, DVector};

use crate::config::{BoundedDefault, ResolvedGroupControlProfile};
use crate::runtime::{CoordinateCommand, DisplaySnapshot};
use crate::{RobotDescription, Transmission};

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum EditorMode {
    Model,
    #[default]
    Actuator,
}

pub struct GroupEditor {
    pub profile: ResolvedGroupControlProfile,
    pub transmission: Transmission,
    pub joint_position_bounds: Vec<BoundedDefault>,
    pub joint_velocity_bounds: Vec<BoundedDefault>,
    pub joint_effort_bounds: Vec<BoundedDefault>,
    pub actuator: BTreeMap<String, CoordinateCommand>,
    pub model_position: Vec<f64>,
    pub model_velocity: Vec<f64>,
    pub model_effort: Vec<f64>,
    seeded: bool,
}

impl GroupEditor {
    pub fn new(
        description: &RobotDescription,
        profile: ResolvedGroupControlProfile,
    ) -> anyhow::Result<Self> {
        let transmission = Transmission::from_description(description, profile.coordinates.keys())?;
        let joint_position_bounds = transmission
            .joint_names()
            .iter()
            .map(|name| {
                let joint = description
                    .joint(name)
                    .ok_or_else(|| anyhow::anyhow!("model joint {name:?} disappeared"))?;
                let limits = joint.limits.as_ref().ok_or_else(|| {
                    anyhow::anyhow!("model joint {name:?} has no URDF limits for safe editing")
                })?;
                Ok(BoundedDefault {
                    min: limits.lower,
                    max: limits.upper,
                    default: 0.0_f64.clamp(limits.lower, limits.upper),
                })
            })
            .collect::<anyhow::Result<Vec<_>>>()?;
        let coordinate_velocity_bounds = transmission
            .coordinate_names()
            .iter()
            .map(|name| profile.coordinates[name].velocity)
            .collect::<Vec<_>>();
        let coordinate_velocity_default = coordinate_velocity_bounds
            .iter()
            .map(|bounds| bounds.default)
            .collect::<Vec<_>>();
        let joint_velocity_default =
            transmission.coordinate_velocity_to_joint(&coordinate_velocity_default)?;
        let joint_velocity_bounds = conservative_joint_bounds(
            transmission.forward_matrix(),
            &joint_velocity_default,
            &coordinate_velocity_bounds,
        )?;
        let coordinate_effort_bounds = transmission
            .coordinate_names()
            .iter()
            .map(|name| profile.coordinates[name].torque_feedforward)
            .collect::<Vec<_>>();
        let coordinate_effort_default = coordinate_effort_bounds
            .iter()
            .map(|bounds| bounds.default)
            .collect::<Vec<_>>();
        let joint_effort_default =
            transmission.coordinate_effort_to_joint(&coordinate_effort_default)?;
        let joint_effort_bounds = conservative_joint_bounds(
            transmission.model_effort_to_coordinate_matrix(),
            &joint_effort_default,
            &coordinate_effort_bounds,
        )?;
        let actuator = profile
            .coordinates
            .iter()
            .map(|(name, bounds)| {
                (
                    name.clone(),
                    CoordinateCommand {
                        position: bounds.position.default,
                        velocity: bounds.velocity.default,
                        torque_feedforward: bounds.torque_feedforward.default,
                        kp: bounds.kp.default,
                        kd: bounds.kd.default,
                    },
                )
            })
            .collect();
        let mut editor = Self {
            profile,
            transmission,
            joint_position_bounds,
            joint_velocity_bounds,
            joint_effort_bounds,
            actuator,
            model_position: Vec::new(),
            model_velocity: Vec::new(),
            model_effort: Vec::new(),
            seeded: false,
        };
        editor.sync_model_from_actuator()?;
        Ok(editor)
    }

    pub fn seed_from_feedback(&mut self, group: &str, snapshot: &DisplaySnapshot) -> bool {
        if self.seeded {
            return true;
        }
        self.apply_feedback(group, snapshot)
    }

    /// Tracks live feedback while the group is not being commanded, so a new
    /// session always presents and starts from the current physical pose.
    pub fn follow_feedback(&mut self, group: &str, snapshot: &DisplaySnapshot) -> bool {
        self.apply_feedback(group, snapshot)
    }

    fn apply_feedback(&mut self, group: &str, snapshot: &DisplaySnapshot) -> bool {
        let maximum_age = Duration::from_millis(self.profile.telemetry_staleness_ms);
        let samples = self
            .transmission
            .coordinate_names()
            .iter()
            .map(|name| snapshot.motors.get(name))
            .collect::<Option<Vec<_>>>();
        let Some(samples) = samples else {
            return false;
        };
        if samples.iter().any(|sample| {
            !sample.feedback_available
                || !sample.feedback_current_connection
                || sample.group != group
                || sample.age > maximum_age
        }) {
            return false;
        }
        for sample in samples {
            let Some(target) = self.actuator.get_mut(&sample.coordinate) else {
                return false;
            };
            let limits = &self.profile.coordinates[&sample.coordinate];
            let Some(position) = limits.position.clamp_feedback_position(sample.position) else {
                return false;
            };
            target.position = position;
            target.velocity = limits.velocity.default;
            target.torque_feedforward = limits.torque_feedforward.default;
            target.kp = limits.kp.default;
            target.kd = limits.kd.default;
        }
        self.seeded = self.sync_model_from_actuator().is_ok();
        self.seeded
    }

    /// Forces the next session to discard prior-session targets and seed from fresh feedback.
    pub fn reset_for_new_session(&mut self) {
        self.seeded = false;
    }

    pub fn sync_model_from_actuator(&mut self) -> anyhow::Result<()> {
        let ordered = self
            .transmission
            .coordinate_names()
            .iter()
            .map(|name| self.actuator[name])
            .collect::<Vec<_>>();
        let mut model_position = self.transmission.coordinate_to_joint(
            &ordered
                .iter()
                .map(|target| target.position)
                .collect::<Vec<_>>(),
        )?;
        let mut position_clamped = false;
        for (position, bounds) in model_position.iter_mut().zip(&self.joint_position_bounds) {
            let bounded = position.clamp(bounds.min, bounds.max);
            position_clamped |= bounded != *position;
            *position = bounded;
        }
        if position_clamped {
            let coordinate_position = self.transmission.joint_to_coordinate(&model_position)?;
            for (index, name) in self.transmission.coordinate_names().iter().enumerate() {
                self.actuator
                    .get_mut(name)
                    .expect("actuator editor contains every transmission coordinate")
                    .position = coordinate_position[index];
            }
        }
        self.model_position = model_position;
        self.model_velocity = self.transmission.coordinate_velocity_to_joint(
            &ordered
                .iter()
                .map(|target| target.velocity)
                .collect::<Vec<_>>(),
        )?;
        self.model_effort = self.transmission.coordinate_effort_to_joint(
            &ordered
                .iter()
                .map(|target| target.torque_feedforward)
                .collect::<Vec<_>>(),
        )?;
        Ok(())
    }

    pub fn sync_actuator_from_model(&mut self) -> anyhow::Result<()> {
        clamp_values(&mut self.model_velocity, &self.joint_velocity_bounds);
        clamp_values(&mut self.model_effort, &self.joint_effort_bounds);
        let position = self
            .transmission
            .joint_to_coordinate(&self.model_position)?;
        let velocity = self
            .transmission
            .joint_velocity_to_coordinate(&self.model_velocity)?;
        let effort = self
            .transmission
            .joint_effort_to_coordinate(&self.model_effort)?;
        let mut clamped = false;
        for (index, name) in self.transmission.coordinate_names().iter().enumerate() {
            let target = self
                .actuator
                .get_mut(name)
                .expect("actuator editor contains every transmission coordinate");
            let limits = self.profile.coordinates[name].position;
            let bounded_position = position[index].clamp(limits.min, limits.max);
            clamped |= bounded_position != position[index];
            target.position = bounded_position;
            target.velocity = velocity[index];
            target.torque_feedforward = effort[index];
            // Gains intentionally remain the reviewed per-actuator YAML values.
        }
        if clamped {
            self.sync_model_from_actuator()?;
        }
        Ok(())
    }
}

fn clamp_values(values: &mut [f64], bounds: &[BoundedDefault]) {
    for (value, bounds) in values.iter_mut().zip(bounds) {
        *value = value.clamp(bounds.min, bounds.max);
    }
}

fn conservative_joint_bounds(
    mapping: &DMatrix<f64>,
    joint_default: &[f64],
    coordinate_bounds: &[BoundedDefault],
) -> anyhow::Result<Vec<BoundedDefault>> {
    if mapping.ncols() != joint_default.len() || mapping.nrows() != coordinate_bounds.len() {
        anyhow::bail!("transmission bounds do not match the mapping dimensions");
    }
    let default_coordinates = mapping * DVector::from_column_slice(joint_default);
    for (index, (value, bounds)) in default_coordinates
        .iter()
        .zip(coordinate_bounds)
        .enumerate()
    {
        if *value < bounds.min || *value > bounds.max {
            anyhow::bail!(
                "transmission default for coordinate {} is outside its live limits",
                index
            );
        }
    }
    let contributors = (0..mapping.nrows())
        .map(|row| {
            (0..mapping.ncols())
                .filter(|column| mapping[(row, *column)] != 0.0)
                .count()
        })
        .collect::<Vec<_>>();
    let mut result = Vec::with_capacity(mapping.ncols());
    for column in 0..mapping.ncols() {
        let mut minimum_delta = f64::NEG_INFINITY;
        let mut maximum_delta = f64::INFINITY;
        for row in 0..mapping.nrows() {
            let coefficient = mapping[(row, column)];
            if coefficient == 0.0 {
                continue;
            }
            let share = contributors[row] as f64;
            let lower_slack = default_coordinates[row] - coordinate_bounds[row].min;
            let upper_slack = coordinate_bounds[row].max - default_coordinates[row];
            let (lower, upper) = if coefficient > 0.0 {
                (
                    -lower_slack / (share * coefficient),
                    upper_slack / (share * coefficient),
                )
            } else {
                (
                    -upper_slack / (share * -coefficient),
                    lower_slack / (share * -coefficient),
                )
            };
            minimum_delta = minimum_delta.max(lower);
            maximum_delta = maximum_delta.min(upper);
        }
        if !minimum_delta.is_finite() || !maximum_delta.is_finite() || minimum_delta > maximum_delta
        {
            anyhow::bail!(
                "model coordinate {} has no conservative live actuator bounds",
                column
            );
        }
        result.push(BoundedDefault {
            min: joint_default[column] + minimum_delta,
            max: joint_default[column] + maximum_delta,
            default: joint_default[column],
        });
    }
    Ok(result)
}

#[cfg(test)]
mod tests {
    use std::fs;

    use crate::config::GroupControlProfile;
    use tempfile::tempdir;

    use super::*;

    fn bounds(default: f64) -> BoundedDefault {
        BoundedDefault {
            min: -100.0,
            max: 100.0,
            default,
        }
    }

    #[test]
    fn model_edits_convert_through_tendons_without_rewriting_actuator_gains() {
        let directory = tempdir().unwrap();
        let path = directory.path().join("ankle.urdf");
        fs::write(
            &path,
            r#"<robot name="ankle">
              <link name="base"/><link name="pitch_link"/><link name="roll_link"/>
              <joint name="pitch" type="revolute">
                <parent link="base"/><child link="pitch_link"/>
                <limit lower="-100" upper="100" effort="100" velocity="100"/>
              </joint>
              <joint name="roll" type="revolute">
                <parent link="pitch_link"/><child link="roll_link"/>
                <limit lower="-100" upper="100" effort="100" velocity="100"/>
              </joint>
              <transmission name="ankle">
                <type>transmission_interface/DifferentialTransmission</type>
                <actuator name="motor_1" role="actuator1"/>
                <actuator name="motor_2" role="actuator2"/>
                <joint name="roll" role="joint1"/>
                <joint name="pitch" role="joint2"><mechanical_reduction>-1</mechanical_reduction></joint>
              </transmission>
            </robot>"#,
        )
        .unwrap();
        let description = RobotDescription::load(&path).unwrap();
        let profile = GroupControlProfile {
            command_rate_hz: 50.0,
            validity_ms: 100,
            telemetry_staleness_ms: 200,
            position_slew_rate: 0.5,
            velocity: bounds(0.0),
            torque_feedforward: bounds(0.0),
            kp: BoundedDefault {
                min: 0.0,
                max: 100.0,
                default: 10.0,
            },
            kd: BoundedDefault {
                min: 0.0,
                max: 100.0,
                default: 1.0,
            },
        }
        .resolve_limits(
            &description,
            [
                (
                    "motor_1".into(),
                    crate::config::CoordinateLimits {
                        position_min: -100.0,
                        position_max: 100.0,
                        velocity_min: -100.0,
                        velocity_max: 100.0,
                        torque_min: -100.0,
                        torque_max: 100.0,
                        kp_min: 0.0,
                        kp_max: 100.0,
                        kd_min: 0.0,
                        kd_max: 100.0,
                    },
                ),
                (
                    "motor_2".into(),
                    crate::config::CoordinateLimits {
                        position_min: -100.0,
                        position_max: 100.0,
                        velocity_min: -100.0,
                        velocity_max: 100.0,
                        torque_min: -100.0,
                        torque_max: 100.0,
                        kp_min: 0.0,
                        kp_max: 100.0,
                        kd_min: 0.0,
                        kd_max: 100.0,
                    },
                ),
            ],
        )
        .unwrap();
        let mut editor = GroupEditor::new(&description, profile).unwrap();
        for first in [
            editor.joint_velocity_bounds[0].min,
            editor.joint_velocity_bounds[0].max,
        ] {
            for second in [
                editor.joint_velocity_bounds[1].min,
                editor.joint_velocity_bounds[1].max,
            ] {
                let coordinates = editor
                    .transmission
                    .joint_velocity_to_coordinate(&[first, second])
                    .unwrap();
                assert!(
                    coordinates
                        .iter()
                        .all(|value| (-100.0..=100.0).contains(value))
                );
            }
        }
        for first in [
            editor.joint_effort_bounds[0].min,
            editor.joint_effort_bounds[0].max,
        ] {
            for second in [
                editor.joint_effort_bounds[1].min,
                editor.joint_effort_bounds[1].max,
            ] {
                let coordinates = editor
                    .transmission
                    .joint_effort_to_coordinate(&[first, second])
                    .unwrap();
                assert!(
                    coordinates
                        .iter()
                        .all(|value| (-100.0..=100.0).contains(value))
                );
            }
        }
        editor.model_position = vec![2.0, 3.0];
        editor.model_velocity = vec![4.0, 5.0];
        editor.model_effort = vec![6.0, 8.0];

        editor.sync_actuator_from_model().unwrap();

        assert_eq!(editor.actuator["motor_1"].position, 1.0);
        assert_eq!(editor.actuator["motor_2"].position, 5.0);
        assert_eq!(editor.actuator["motor_1"].kp, 10.0);
        assert_eq!(editor.actuator["motor_2"].kp, 10.0);
        assert_eq!(editor.actuator["motor_1"].kd, 1.0);
        assert_eq!(editor.actuator["motor_2"].kd, 1.0);
        editor.sync_model_from_actuator().unwrap();
        assert_eq!(editor.model_position, [2.0, 3.0]);

        editor.model_velocity = vec![f64::NEG_INFINITY, f64::INFINITY];
        editor.model_effort = vec![f64::NEG_INFINITY, f64::INFINITY];
        editor.sync_actuator_from_model().unwrap();
        assert!(editor.actuator.values().all(|target| {
            (-100.0..=100.0).contains(&target.velocity)
                && (-100.0..=100.0).contains(&target.torque_feedforward)
        }));

        let feedback = |coordinate: &str, position: f64| crate::runtime::DisplayMotor {
            motor: format!("{coordinate}_motor"),
            coordinate: coordinate.into(),
            group: "ankles".into(),
            bus: "can0".into(),
            id: 1,
            model: "test".into(),
            direction: 1,
            limits: crate::config::CoordinateLimits {
                position_min: -100.0,
                position_max: 100.0,
                velocity_min: -100.0,
                velocity_max: 100.0,
                torque_min: -100.0,
                torque_max: 100.0,
                kp_min: 0.0,
                kp_max: 100.0,
                kd_min: 0.0,
                kd_max: 100.0,
            },
            status: None,
            feedback_available: true,
            feedback_current_connection: true,
            monotonic_ns: Some(1),
            position,
            velocity: 0.0,
            effort: 0.0,
            temperature: 25.0,
            fault_bits: 0,
            mode_status: 0,
            age: Duration::ZERO,
            stale_after: Duration::from_millis(200),
        };
        let mut snapshot = DisplaySnapshot::default();
        snapshot
            .motors
            .insert("motor_1".into(), feedback("motor_1", 0.2));
        snapshot
            .motors
            .insert("motor_2".into(), feedback("motor_2", 0.4));
        assert!(editor.seed_from_feedback("ankles", &snapshot));

        editor.actuator.get_mut("motor_1").unwrap().position = 9.0;
        editor.actuator.get_mut("motor_1").unwrap().velocity = 9.0;
        snapshot.motors.get_mut("motor_1").unwrap().position = 0.3;
        editor.reset_for_new_session();
        assert!(editor.seed_from_feedback("ankles", &snapshot));
        assert_eq!(editor.actuator["motor_1"].position, 0.3);
        assert_eq!(editor.actuator["motor_1"].velocity, 0.0);
        assert_eq!(editor.actuator["motor_1"].kp, 10.0);

        snapshot.motors.get_mut("motor_1").unwrap().position = 100.0005;
        assert!(editor.follow_feedback("ankles", &snapshot));
        assert_eq!(editor.actuator["motor_1"].position, 100.0);

        snapshot.motors.get_mut("motor_1").unwrap().position = 100.002;
        assert!(editor.follow_feedback("ankles", &snapshot));
        assert_eq!(editor.actuator["motor_1"].position, 100.0);

        snapshot.motors.get_mut("motor_1").unwrap().position = f64::NAN;
        assert!(!editor.follow_feedback("ankles", &snapshot));
    }
}
