use std::time::Duration;

use crate::config::EstimatorConfig;

const ACQUIRE_PROBABILITY: f64 = 0.70;
const RELEASE_PROBABILITY: f64 = 0.08;

#[derive(Clone, Copy, Debug)]
struct ContactProbabilityModel {
    height_full: f64,
    height_zero: f64,
    motion_full: f64,
    motion_zero: f64,
    world_speed_full: f64,
    world_speed_zero: f64,
    rise_time: Duration,
    fall_time: Duration,
}

impl ContactProbabilityModel {
    fn from_config(config: &EstimatorConfig) -> Self {
        Self {
            height_full: config.contact_height_full_m,
            height_zero: config.contact_height_zero_m,
            motion_full: config.contact_motion_full_mps,
            motion_zero: config.contact_motion_zero_mps,
            world_speed_full: config.contact_world_speed_full_mps,
            world_speed_zero: config.contact_world_speed_zero_mps,
            rise_time: config.contact_probability_rise(),
            fall_time: config.contact_probability_fall(),
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
struct ContactState {
    probability: f64,
    active: bool,
}

impl ContactState {
    fn update(
        &mut self,
        target: f64,
        model: &ContactProbabilityModel,
        dt: Duration,
    ) -> ContactUpdate {
        let time_constant = if target >= self.probability {
            model.rise_time
        } else {
            model.fall_time
        };
        self.probability = low_pass(self.probability, target, time_constant, dt).clamp(0., 1.);
        let was_active = self.active;
        self.active = if was_active {
            self.probability > RELEASE_PROBABILITY
        } else {
            self.probability >= ACQUIRE_PROBABILITY
        };
        ContactUpdate {
            probability: self.probability,
            active: self.active,
            changed: was_active != self.active,
            world_speed_reliability: 0.,
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct ContactUpdate {
    pub(crate) probability: f64,
    pub(crate) active: bool,
    pub(crate) changed: bool,
    pub(crate) world_speed_reliability: f64,
}

#[derive(Debug)]
pub(crate) struct ContactManager {
    model: ContactProbabilityModel,
    feet: [ContactState; 2],
}

impl ContactManager {
    pub(crate) fn new(config: &EstimatorConfig) -> Self {
        Self {
            model: ContactProbabilityModel::from_config(config),
            feet: [ContactState::default(); 2],
        }
    }

    pub(crate) fn initialize(
        &mut self,
        heights: [f64; 2],
        motion_speeds: [f64; 2],
    ) -> [ContactUpdate; 2] {
        let minimum_height = heights[0].min(heights[1]);
        std::array::from_fn(|index| {
            let probability =
                self.kinematic_probability(heights[index] - minimum_height, motion_speeds[index]);
            let active = probability >= ACQUIRE_PROBABILITY;
            self.feet[index] = ContactState {
                probability,
                active,
            };
            ContactUpdate {
                probability,
                active,
                changed: active,
                world_speed_reliability: 0.0,
            }
        })
    }

    pub(crate) fn update(
        &mut self,
        heights: [f64; 2],
        motion_speeds: [f64; 2],
        world_speeds: [f64; 2],
        dt: Duration,
    ) -> [ContactUpdate; 2] {
        let minimum_height = heights[0].min(heights[1]);
        let previous_feet = self.feet;
        std::array::from_fn(|index| {
            let other = 1 - index;
            let opposite_world_probability = self.world_speed_probability(world_speeds[other]);
            let world_speed_reliability = if previous_feet[other].active {
                previous_feet[other].probability * opposite_world_probability
            } else {
                0.0
            };
            let kinematic_probability =
                self.kinematic_probability(heights[index] - minimum_height, motion_speeds[index]);
            let world_speed_probability = self.world_speed_probability(world_speeds[index]);
            let world_speed_factor =
                1.0 - world_speed_reliability * (1.0 - world_speed_probability);
            let height_probability = descending_smoothstep(
                heights[index] - minimum_height,
                self.model.height_full,
                self.model.height_zero,
            );
            // A trusted opposite stance foot makes world velocity informative.
            // Body-relative vertical motion alone is not swing evidence in a squat.
            let motion_probability = (1.0 - world_speed_reliability) * kinematic_probability
                + world_speed_reliability * height_probability;
            let target = motion_probability * world_speed_factor;

            ContactUpdate {
                world_speed_reliability,
                ..self.feet[index].update(target, &self.model, dt)
            }
        })
    }

    pub(crate) fn active(&self) -> [bool; 2] {
        self.feet.map(|foot| foot.active)
    }

    pub(crate) fn update_loaded(
        &mut self,
        heights: [f64; 2],
        load_probability: [f64; 2],
        dt: Duration,
    ) -> [ContactUpdate; 2] {
        let minimum = heights[0].min(heights[1]);
        std::array::from_fn(|index| {
            let target = load_probability[index]
                * descending_smoothstep(
                    heights[index] - minimum,
                    self.model.height_full,
                    self.model.height_zero,
                );
            self.feet[index].update(target, &self.model, dt)
        })
    }

    pub(crate) fn probabilities(&self) -> [f64; 2] {
        self.feet.map(|foot| foot.probability)
    }

    #[cfg(test)]
    pub(crate) fn clear(&mut self) {
        self.feet = [ContactState::default(); 2];
    }

    fn kinematic_probability(&self, relative_height: f64, motion_speed: f64) -> f64 {
        descending_smoothstep(
            relative_height,
            self.model.height_full,
            self.model.height_zero,
        ) * descending_smoothstep(motion_speed, self.model.motion_full, self.model.motion_zero)
    }

    fn world_speed_probability(&self, world_speed: f64) -> f64 {
        descending_smoothstep(
            world_speed,
            self.model.world_speed_full,
            self.model.world_speed_zero,
        )
    }
}

pub(crate) fn descending_smoothstep(value: f64, full: f64, zero: f64) -> f64 {
    if value <= full {
        return 1.0;
    }
    if value >= zero {
        return 0.0;
    }
    let fraction = (value - full) / (zero - full);
    1.0 - fraction * fraction * (3.0 - 2.0 * fraction)
}

fn low_pass(previous: f64, target: f64, time_constant: Duration, dt: Duration) -> f64 {
    let alpha = 1.0 - (-dt.as_secs_f64() / time_constant.as_secs_f64()).exp();
    previous + alpha * (target - previous)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn manager() -> ContactManager {
        ContactManager {
            model: ContactProbabilityModel {
                height_full: 0.015,
                height_zero: 0.06,
                motion_full: 0.10,
                motion_zero: 0.60,
                world_speed_full: 0.10,
                world_speed_zero: 0.50,
                rise_time: Duration::from_millis(80),
                fall_time: Duration::from_millis(20),
            },
            feet: [ContactState::default(); 2],
        }
    }

    #[test]
    fn probability_fades_before_releasing_the_anchor() {
        let mut contacts = manager();
        let initialized = contacts.initialize([0.0, 0.0], [0.0, 0.0]);
        assert!(initialized.iter().all(|contact| contact.active));

        let fading = contacts.update(
            [0.0, 0.0],
            [1.0, 1.0],
            [1.0, 1.0],
            Duration::from_millis(20),
        );
        assert!(fading.iter().all(|contact| contact.active));
        assert!(
            fading
                .iter()
                .all(|contact| contact.probability > RELEASE_PROBABILITY)
        );

        let released = contacts.update(
            [0.0, 0.0],
            [1.0, 1.0],
            [1.0, 1.0],
            Duration::from_millis(40),
        );
        assert!(released.iter().all(|contact| !contact.active));
        assert!(released.iter().all(|contact| contact.changed));
    }

    #[test]
    fn low_moving_foot_is_not_anchored_at_initialization() {
        let mut contacts = manager();

        let initialized = contacts.initialize([0.0, 0.01], [0.0, 1.0]);

        assert!(initialized[0].active);
        assert_eq!(initialized[0].probability, 1.0);
        assert!(!initialized[1].active);
        assert_eq!(initialized[1].probability, 0.0);
    }

    #[test]
    fn trusted_opposite_support_uses_world_speed_to_reject_a_swing_foot() {
        let mut contacts = manager();
        contacts.initialize([0.0, 0.0], [0.0, 0.0]);

        let fading = contacts.update(
            [0.0, 0.0],
            [0.0, 0.0],
            [0.0, 1.0],
            Duration::from_millis(20),
        );
        assert_eq!(fading[0].probability, 1.0);
        assert!(fading[1].probability < 0.5);
        assert_eq!(fading[1].world_speed_reliability, 1.0);

        let released = contacts.update(
            [0.0, 0.0],
            [0.0, 0.0],
            [0.0, 1.0],
            Duration::from_millis(40),
        );
        assert!(released[0].active);
        assert!(!released[1].active);
    }

    #[test]
    fn world_speed_does_not_block_recovery_without_stationary_opposite_support() {
        let mut contacts = manager();

        for _ in 0..3 {
            contacts.update(
                [0.0, 0.0],
                [0.0, 0.0],
                [10.0, 10.0],
                Duration::from_millis(80),
            );
        }

        assert_eq!(contacts.active(), [true, true]);
    }

    #[test]
    fn stationary_world_feet_remain_contacts_during_body_vertical_motion() {
        let mut contacts = manager();
        contacts.initialize([0.0, 0.0], [0.0, 0.0]);
        for _ in 0..100 {
            contacts.update(
                [0.0, 0.0],
                [0.8, 0.8],
                [0.0, 0.0],
                Duration::from_millis(10),
            );
        }
        assert_eq!(contacts.probabilities(), [1.0, 1.0]);
        assert_eq!(contacts.active(), [true, true]);
    }

    #[test]
    fn moving_world_feet_cannot_validate_each_other_as_stationary() {
        let mut contacts = manager();
        contacts.initialize([0.0, 0.0], [0.0, 0.0]);
        for _ in 0..10 {
            contacts.update(
                [0.0, 0.0],
                [0.8, 0.8],
                [0.8, 0.8],
                Duration::from_millis(10),
            );
        }
        assert_eq!(contacts.active(), [false, false]);
    }

    #[test]
    fn loaded_contact_still_requires_relative_ground_height() {
        let mut contacts = manager();
        for _ in 0..30 {
            contacts.update_loaded([0., 0.2], [1., 1.], Duration::from_millis(20));
        }
        assert_eq!(contacts.active(), [true, false]);
        for _ in 0..30 {
            contacts.update_loaded([0., 0.], [0., 0.], Duration::from_millis(20));
        }
        assert_eq!(contacts.active(), [false, false]);
    }
}
