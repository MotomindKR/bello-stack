use std::collections::BTreeMap;
use std::time::Duration;

use crate::config::{BoundedDefault, ResolvedGroupControlProfile};
use crate::runtime::{CoordinateCommand, DisplaySnapshot};
use crate::{Model, Transmission};

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
    pub actuator: BTreeMap<String, CoordinateCommand>,
    pub model_position: Vec<f64>,
    pub model_velocity: Vec<f64>,
    pub model_effort: Vec<f64>,
    seeded: bool,
}

impl GroupEditor {
    pub fn new(model: &Model, profile: ResolvedGroupControlProfile) -> anyhow::Result<Self> {
        let transmission = Transmission::from_model(model, profile.coordinates.keys())?;
        let joint_position_bounds = transmission
            .joint_names()
            .iter()
            .map(|name| {
                let joint = &model.joints[name];
                let range = joint.range.ok_or_else(|| {
                    anyhow::anyhow!("model joint {name:?} has no MJCF range for safe editing")
                })?;
                Ok(BoundedDefault {
                    min: range[0],
                    max: range[1],
                    default: 0.0_f64.clamp(range[0], range[1]),
                })
            })
            .collect::<anyhow::Result<Vec<_>>>()?;
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
        self.model_position = self.transmission.coordinate_to_joint(
            &ordered
                .iter()
                .map(|target| target.position)
                .collect::<Vec<_>>(),
        )?;
        self.model_velocity = self.transmission.coordinate_to_joint(
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
        let position = self
            .transmission
            .joint_to_coordinate(&self.model_position)?;
        let velocity = self
            .transmission
            .joint_to_coordinate(&self.model_velocity)?;
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

#[cfg(test)]
mod tests {
    use crate::config::GroupControlProfile;

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
        let model = Model::parse(
            r#"<mujoco>
                <worldbody><body name="ankle">
                  <joint name="pitch" range="-100 100"/><joint name="roll" range="-100 100"/>
                </body></worldbody>
                <tendon>
                  <fixed name="motor_1"><joint joint="pitch" coef="-1"/><joint joint="roll" coef="1"/></fixed>
                  <fixed name="motor_2"><joint joint="pitch" coef="1"/><joint joint="roll" coef="1"/></fixed>
                </tendon>
                <actuator>
                  <position name="motor_1_position" tendon="motor_1" ctrlrange="-100 100" user="0"/>
                  <position name="motor_2_position" tendon="motor_2" ctrlrange="-100 100" user="0"/>
                </actuator>
              </mujoco>"#,
            "/tmp/model.xml",
        )
        .unwrap();
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
        .resolve(&model, ["motor_1", "motor_2"])
        .unwrap();
        let mut editor = GroupEditor::new(&model, profile).unwrap();
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

        let feedback = |coordinate: &str, position: f64| crate::runtime::DisplayMotor {
            motor: format!("{coordinate}_motor"),
            coordinate: coordinate.into(),
            group: "ankles".into(),
            bus: "can0".into(),
            id: 1,
            model: "test".into(),
            direction: 1,
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
