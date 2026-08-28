use std::f64::consts::TAU;
use std::time::Duration;

use nalgebra::{Quaternion, UnitQuaternion, Vector3};

use super::ImuSample;
use crate::config::ImuFilterConfig;

const MAXIMUM_FILTER_INTERVAL: Duration = Duration::from_millis(100);

pub(super) struct ImuLowPass {
    config: ImuFilterConfig,
    previous_device_timestamp_ms: Option<u64>,
    filtered: Option<ImuSample>,
}

impl ImuLowPass {
    pub(super) fn new(config: ImuFilterConfig) -> Self {
        Self {
            config,
            previous_device_timestamp_ms: None,
            filtered: None,
        }
    }

    pub(super) fn reset(&mut self) {
        self.previous_device_timestamp_ms = None;
        self.filtered = None;
    }

    pub(super) fn apply(&mut self, sample: ImuSample) -> ImuSample {
        let interval = self
            .previous_device_timestamp_ms
            .and_then(|previous| sample.device_timestamp_ms.checked_sub(previous))
            .map(Duration::from_millis);
        self.previous_device_timestamp_ms = Some(sample.device_timestamp_ms);

        let Some((previous, interval)) = self
            .filtered
            .as_ref()
            .zip(interval)
            .filter(|(_, interval)| !interval.is_zero() && *interval <= MAXIMUM_FILTER_INTERVAL)
        else {
            self.filtered = Some(sample.clone());
            return sample;
        };

        let orientation_alpha = alpha(self.config.orientation_cutoff_hz, interval);
        let gyro_alpha = alpha(self.config.gyro_cutoff_hz, interval);
        let acceleration_alpha = alpha(self.config.acceleration_cutoff_hz, interval);
        let orientation = unit_quaternion(previous.orientation_xyzw)
            .slerp(&unit_quaternion(sample.orientation_xyzw), orientation_alpha);
        let angular_velocity = vector(previous.angular_velocity_xyz)
            + gyro_alpha
                * (vector(sample.angular_velocity_xyz) - vector(previous.angular_velocity_xyz));
        let linear_acceleration = vector(previous.linear_acceleration_xyz)
            + acceleration_alpha
                * (vector(sample.linear_acceleration_xyz)
                    - vector(previous.linear_acceleration_xyz));
        let filtered = ImuSample {
            orientation_xyzw: xyzw(&orientation),
            angular_velocity_xyz: angular_velocity.into(),
            linear_acceleration_xyz: linear_acceleration.into(),
            device_timestamp_ms: sample.device_timestamp_ms,
            received: sample.received,
        };
        self.filtered = Some(filtered.clone());
        filtered
    }
}

fn alpha(cutoff_hz: f64, interval: Duration) -> f64 {
    1.0 - (-TAU * cutoff_hz * interval.as_secs_f64()).exp()
}

fn unit_quaternion(xyzw: [f64; 4]) -> UnitQuaternion<f64> {
    UnitQuaternion::new_normalize(Quaternion::new(xyzw[3], xyzw[0], xyzw[1], xyzw[2]))
}

fn xyzw(value: &UnitQuaternion<f64>) -> [f64; 4] {
    let quaternion = value.quaternion();
    [quaternion.i, quaternion.j, quaternion.k, quaternion.w]
}

fn vector(value: [f64; 3]) -> Vector3<f64> {
    Vector3::from(value)
}

#[cfg(test)]
mod tests {
    use std::time::Instant;

    use super::*;

    fn sample(
        device_timestamp_ms: u64,
        orientation_xyzw: [f64; 4],
        angular_velocity_xyz: [f64; 3],
        linear_acceleration_xyz: [f64; 3],
    ) -> ImuSample {
        ImuSample {
            orientation_xyzw,
            angular_velocity_xyz,
            linear_acceleration_xyz,
            device_timestamp_ms,
            received: Instant::now(),
        }
    }

    #[test]
    fn first_sample_passes_through() {
        let mut filter = ImuLowPass::new(ImuFilterConfig::default());
        let input = sample(100, [0.0, 0.0, 0.0, 1.0], [1.0, 2.0, 3.0], [4.0, 5.0, 6.0]);

        let output = filter.apply(input.clone());

        assert_eq!(output.orientation_xyzw, input.orientation_xyzw);
        assert_eq!(output.angular_velocity_xyz, input.angular_velocity_xyz);
        assert_eq!(
            output.linear_acceleration_xyz,
            input.linear_acceleration_xyz
        );
        assert_eq!(output.device_timestamp_ms, input.device_timestamp_ms);
    }

    #[test]
    fn uses_device_interval_for_vector_and_quaternion_filtering() {
        let config = ImuFilterConfig {
            orientation_cutoff_hz: 10.0,
            gyro_cutoff_hz: 10.0,
            acceleration_cutoff_hz: 10.0,
        };
        let mut filter = ImuLowPass::new(config);
        filter.apply(sample(100, [0.0, 0.0, 0.0, 1.0], [0.0; 3], [0.0; 3]));
        let half_turn = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), std::f64::consts::PI);

        let output = filter.apply(sample(
            105,
            xyzw(&half_turn),
            [1.0, 2.0, 3.0],
            [4.0, 5.0, 6.0],
        ));

        let expected_alpha = alpha(10.0, Duration::from_millis(5));
        assert!((output.angular_velocity_xyz[0] - expected_alpha).abs() < 1.0e-12);
        assert!((output.linear_acceleration_xyz[2] - 6.0 * expected_alpha).abs() < 1.0e-12);
        let filtered_orientation = unit_quaternion(output.orientation_xyzw);
        assert!(
            (filtered_orientation.angle() - std::f64::consts::PI * expected_alpha).abs() < 1.0e-12
        );
        assert!((filtered_orientation.quaternion().norm() - 1.0).abs() < 1.0e-12);
    }

    #[test]
    fn non_increasing_timestamp_restarts_filter() {
        let mut filter = ImuLowPass::new(ImuFilterConfig::default());
        filter.apply(sample(100, [0.0, 0.0, 0.0, 1.0], [0.0; 3], [0.0; 3]));
        let input = sample(100, [0.0, 0.0, 0.0, 1.0], [2.0; 3], [3.0; 3]);

        let output = filter.apply(input.clone());

        assert_eq!(output.angular_velocity_xyz, input.angular_velocity_xyz);
        assert_eq!(
            output.linear_acceleration_xyz,
            input.linear_acceleration_xyz
        );
    }

    #[test]
    fn long_gap_restarts_filter() {
        let mut filter = ImuLowPass::new(ImuFilterConfig::default());
        filter.apply(sample(100, [0.0, 0.0, 0.0, 1.0], [0.0; 3], [0.0; 3]));
        let input = sample(201, [0.0, 0.0, 0.0, 1.0], [2.0; 3], [3.0; 3]);

        let output = filter.apply(input.clone());

        assert_eq!(output.angular_velocity_xyz, input.angular_velocity_xyz);
        assert_eq!(
            output.linear_acceleration_xyz,
            input.linear_acceleration_xyz
        );
    }
}
