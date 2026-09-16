use std::f64::consts::TAU;
use std::time::Duration;

use nalgebra::{UnitQuaternion, Vector3};

const MAXIMUM_FILTER_INTERVAL: Duration = Duration::from_millis(100);

#[derive(Clone, Copy, Debug)]
pub(crate) struct FilteredImu {
    pub orientation: UnitQuaternion<f64>,
    pub angular_velocity: Vector3<f64>,
    pub linear_acceleration: Vector3<f64>,
}

#[derive(Debug)]
pub(crate) struct ImuLowPass {
    orientation_cutoff_hz: f64,
    gyro_cutoff_hz: f64,
    acceleration_cutoff_hz: f64,
    previous_source_ns: Option<u64>,
    filtered: Option<FilteredImu>,
}

impl ImuLowPass {
    pub(crate) fn new(
        orientation_cutoff_hz: f64,
        gyro_cutoff_hz: f64,
        acceleration_cutoff_hz: f64,
    ) -> Self {
        Self {
            orientation_cutoff_hz,
            gyro_cutoff_hz,
            acceleration_cutoff_hz,
            previous_source_ns: None,
            filtered: None,
        }
    }

    pub(crate) fn update(
        &mut self,
        source_ns: u64,
        orientation: UnitQuaternion<f64>,
        angular_velocity: Vector3<f64>,
        linear_acceleration: Vector3<f64>,
    ) -> Option<FilteredImu> {
        if self
            .previous_source_ns
            .is_some_and(|previous| source_ns <= previous)
        {
            return None;
        }
        let interval = self
            .previous_source_ns
            .and_then(|previous| source_ns.checked_sub(previous))
            .map(Duration::from_nanos);
        self.previous_source_ns = Some(source_ns);

        let Some((previous, interval)) = self
            .filtered
            .zip(interval)
            .filter(|(_, interval)| !interval.is_zero() && *interval <= MAXIMUM_FILTER_INTERVAL)
        else {
            let sample = FilteredImu {
                orientation,
                angular_velocity,
                linear_acceleration,
            };
            self.filtered = Some(sample);
            return Some(sample);
        };

        let orientation_alpha = low_pass_alpha(self.orientation_cutoff_hz, interval);
        let gyro_alpha = low_pass_alpha(self.gyro_cutoff_hz, interval);
        let acceleration_alpha = low_pass_alpha(self.acceleration_cutoff_hz, interval);
        let sample = FilteredImu {
            orientation: previous.orientation.slerp(&orientation, orientation_alpha),
            angular_velocity: previous.angular_velocity
                + gyro_alpha * (angular_velocity - previous.angular_velocity),
            linear_acceleration: previous.linear_acceleration
                + acceleration_alpha * (linear_acceleration - previous.linear_acceleration),
        };
        self.filtered = Some(sample);
        Some(sample)
    }
}

fn low_pass_alpha(cutoff_hz: f64, interval: Duration) -> f64 {
    1.0 - (-TAU * cutoff_hz * interval.as_secs_f64()).exp()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn first_sample_initializes_without_delay() {
        let mut filter = ImuLowPass::new(20.0, 30.0, 25.0);
        let orientation = UnitQuaternion::from_euler_angles(0.1, -0.2, 0.3);
        let angular_velocity = Vector3::new(1.0, 2.0, 3.0);
        let linear_acceleration = Vector3::new(4.0, 5.0, 6.0);

        let sample = filter
            .update(1_000, orientation, angular_velocity, linear_acceleration)
            .unwrap();

        assert!(sample.orientation.angle_to(&orientation) < 1.0e-12);
        assert_eq!(sample.angular_velocity, angular_velocity);
        assert_eq!(sample.linear_acceleration, linear_acceleration);
    }

    #[test]
    fn subsequent_samples_are_smoothed_on_the_quaternion_manifold() {
        let mut filter = ImuLowPass::new(20.0, 30.0, 25.0);
        let identity = UnitQuaternion::identity();
        let target = UnitQuaternion::from_euler_angles(0.0, 0.0, 1.0);
        filter
            .update(0, identity, Vector3::zeros(), Vector3::zeros())
            .unwrap();

        let sample = filter
            .update(
                Duration::from_millis(2).as_nanos() as u64,
                target,
                Vector3::new(1.0, 0.0, 0.0),
                Vector3::new(2.0, 0.0, 0.0),
            )
            .unwrap();

        assert!(sample.orientation.angle() > 0.0);
        assert!(sample.orientation.angle() < target.angle());
        assert!(sample.angular_velocity.x > 0.0);
        assert!(sample.angular_velocity.x < 1.0);
        assert!(sample.linear_acceleration.x > 0.0);
        assert!(sample.linear_acceleration.x < 2.0);
        assert!((sample.orientation.quaternion().norm() - 1.0).abs() < 1.0e-12);
    }

    #[test]
    fn a_long_gap_reinitializes_instead_of_smearing_the_outage() {
        let mut filter = ImuLowPass::new(20.0, 30.0, 25.0);
        filter
            .update(
                0,
                UnitQuaternion::identity(),
                Vector3::zeros(),
                Vector3::zeros(),
            )
            .unwrap();
        let orientation = UnitQuaternion::from_euler_angles(0.5, 0.0, 0.0);
        let angular_velocity = Vector3::new(3.0, 0.0, 0.0);
        let linear_acceleration = Vector3::new(4.0, 0.0, 0.0);

        let sample = filter
            .update(
                Duration::from_secs(1).as_nanos() as u64,
                orientation,
                angular_velocity,
                linear_acceleration,
            )
            .unwrap();

        assert!(sample.orientation.angle_to(&orientation) < 1.0e-12);
        assert_eq!(sample.angular_velocity, angular_velocity);
        assert_eq!(sample.linear_acceleration, linear_acceleration);
    }

    #[test]
    fn non_increasing_source_time_is_rejected_without_resetting_filter_state() {
        let mut filter = ImuLowPass::new(20.0, 30.0, 25.0);
        let identity = UnitQuaternion::identity();
        filter
            .update(1_000, identity, Vector3::zeros(), Vector3::zeros())
            .unwrap();

        assert!(
            filter
                .update(
                    999,
                    identity,
                    Vector3::new(10.0, 0.0, 0.0),
                    Vector3::new(10.0, 0.0, 0.0),
                )
                .is_none()
        );

        let sample = filter
            .update(
                2_000,
                identity,
                Vector3::new(1.0, 0.0, 0.0),
                Vector3::new(1.0, 0.0, 0.0),
            )
            .unwrap();
        assert!(sample.angular_velocity.x > 0.0);
        assert!(sample.angular_velocity.x < 1.0);
        assert!(sample.linear_acceleration.x > 0.0);
        assert!(sample.linear_acceleration.x < 1.0);
    }
}
