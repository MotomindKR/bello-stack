use std::time::Duration;

use anyhow::{Context, Result, bail};
use bello_robot_description::RobotDescription;
use nalgebra::{Quaternion, UnitQuaternion, Vector3};

use crate::config::ImuConfig;
use crate::domain::{ImuName, SerialDeviceName};

use super::ImuSample;

pub(super) const POLL_PERIOD: Duration = Duration::from_millis(20);

/// Validated EBIMU configuration. Construction performs no device I/O.
#[derive(Clone, Debug)]
pub(crate) struct ImuSpec {
    pub(crate) name: ImuName,
    pub(crate) serial_device: SerialDeviceName,
    pub(crate) source_frame: String,
    pub(crate) output_frame: String,
    pub(crate) stale_timeout: Duration,
    pub(crate) output_interval: Duration,
    output_from_source: UnitQuaternion<f64>,
}

impl ImuSpec {
    pub(crate) fn resolve(config: &ImuConfig, robot: &RobotDescription) -> Result<Self> {
        if config.name.is_empty() {
            bail!("imus[].name must be non-empty");
        }
        if config.serial_device.is_empty() {
            bail!("imus.{}.serial_device must be non-empty", config.name);
        }
        for (name, value) in [
            ("source_frame", config.source_frame.as_str()),
            ("output_frame", config.output_frame.as_str()),
        ] {
            if value.is_empty() || value.trim() != value {
                bail!(
                    "imus.{}.{name} must be non-empty without surrounding whitespace",
                    config.name
                );
            }
        }
        if !(1..=60_000).contains(&config.stale_timeout_ms) {
            bail!("imus.{}.stale_timeout_ms must be in 1..=60000", config.name);
        }
        if !(1..=1000).contains(&config.output_rate_hz) {
            bail!("imus.{}.output_rate_hz must be in 1..=1000", config.name);
        }
        if 1000 % config.output_rate_hz != 0 {
            bail!(
                "imus.{}.output_rate_hz must divide 1000 exactly because EBIMU configures an integer millisecond interval",
                config.name
            );
        }
        let output_interval = Duration::from_millis(u64::from(1000 / config.output_rate_hz));
        let minimum_stale_timeout = (output_interval * 2).max(POLL_PERIOD);
        let stale_timeout = Duration::from_millis(config.stale_timeout_ms);
        if stale_timeout < minimum_stale_timeout {
            bail!(
                "imus.{}.stale_timeout_ms must be at least {} for output_rate_hz {}",
                config.name,
                minimum_stale_timeout.as_millis(),
                config.output_rate_hz
            );
        }
        let output_from_source = robot
            .fixed_transform(&config.output_frame, &config.source_frame)
            .with_context(|| {
                format!(
                    "imus.{} cannot transform source_frame {:?} to output_frame {:?}",
                    config.name, config.source_frame, config.output_frame
                )
            })?;
        if output_from_source.translation.vector.norm() > 1.0e-9 {
            bail!(
                "imus.{} source_frame {:?} and output_frame {:?} must have coincident origins",
                config.name,
                config.source_frame,
                config.output_frame
            );
        }
        Ok(Self {
            name: ImuName::from(config.name.as_str()),
            serial_device: SerialDeviceName::from(config.serial_device.as_str()),
            source_frame: config.source_frame.clone(),
            output_frame: config.output_frame.clone(),
            stale_timeout,
            output_interval,
            output_from_source: output_from_source.rotation,
        })
    }

    pub(super) fn transform_sample(&self, sample: ImuSample) -> ImuSample {
        let [x, y, z, w] = sample.orientation_xyzw;
        let world_from_source = UnitQuaternion::new_normalize(Quaternion::new(w, x, y, z));
        let world_from_output = world_from_source * self.output_from_source.inverse();
        let orientation = world_from_output.quaternion();
        let angular_velocity = self.output_from_source * Vector3::from(sample.angular_velocity_xyz);
        let linear_acceleration =
            self.output_from_source * Vector3::from(sample.linear_acceleration_xyz);
        ImuSample {
            orientation_xyzw: [orientation.i, orientation.j, orientation.k, orientation.w],
            angular_velocity_xyz: angular_velocity.into(),
            linear_acceleration_xyz: linear_acceleration.into(),
            device_timestamp_ms: sample.device_timestamp_ms,
            received: sample.received,
        }
    }
}

#[cfg(test)]
mod tests {
    use std::path::Path;
    use std::time::Instant;

    use super::*;

    fn mini_robot() -> RobotDescription {
        RobotDescription::load(
            &Path::new(env!("CARGO_MANIFEST_DIR"))
                .join("../../descriptions/bello_mini/urdf/bello_mini.urdf"),
        )
        .unwrap()
    }

    fn valid_config() -> ImuConfig {
        ImuConfig {
            source_frame: "imu_sensor_link".into(),
            output_frame: "imu_link".into(),
            ..Default::default()
        }
    }

    fn resolve(config: &ImuConfig) -> Result<ImuSpec> {
        ImuSpec::resolve(config, &mini_robot())
    }

    #[test]
    fn requires_explicit_source_and_output_frames() {
        let error = resolve(&ImuConfig::default()).unwrap_err().to_string();
        assert!(error.contains("source_frame"), "{error}");
    }

    #[test]
    fn rejects_output_rate_above_device_limit() {
        let config = ImuConfig {
            output_rate_hz: 1001,
            ..valid_config()
        };
        assert!(resolve(&config).is_err());
    }

    #[test]
    fn rejects_output_rate_that_cannot_be_encoded_exactly() {
        let config = ImuConfig {
            output_rate_hz: 333,
            ..valid_config()
        };
        assert!(resolve(&config).is_err());
    }

    #[test]
    fn stale_timeout_covers_at_least_two_sample_periods() {
        let too_short = ImuConfig {
            output_rate_hz: 10,
            stale_timeout_ms: 199,
            ..valid_config()
        };
        assert!(resolve(&too_short).is_err());

        let valid = ImuConfig {
            stale_timeout_ms: 200,
            ..too_short
        };
        assert!(resolve(&valid).is_ok());
    }

    #[test]
    fn rejects_missing_movable_and_noncoincident_frames() {
        let missing = ImuConfig {
            source_frame: "missing_link".into(),
            ..valid_config()
        };
        let error = resolve(&missing).unwrap_err().to_string();
        assert!(error.contains("missing_link"), "{error}");

        let movable = ImuConfig {
            source_frame: "left_hip_pitch_link".into(),
            output_frame: "pelvis_link".into(),
            ..valid_config()
        };
        let error = resolve(&movable).unwrap_err().to_string();
        assert!(error.contains("cannot transform"), "{error}");

        let noncoincident = ImuConfig {
            source_frame: "imu_link".into(),
            output_frame: "pelvis_link".into(),
            ..valid_config()
        };
        let error = resolve(&noncoincident).unwrap_err().to_string();
        assert!(error.contains("coincident origins"), "{error}");
    }

    #[test]
    fn transforms_orientation_and_vectors_into_output_frame() {
        let mut spec = resolve(&valid_config()).unwrap();
        let output_from_source = UnitQuaternion::from_euler_angles(0.31, -0.42, 0.73);
        spec.output_from_source = output_from_source;

        let world_from_output = UnitQuaternion::from_euler_angles(-0.2, 0.4, -0.6);
        let world_from_source = world_from_output * output_from_source;
        let source_orientation = world_from_source.quaternion();
        let angular_velocity_output = Vector3::new(1.2, -2.3, 3.4);
        let acceleration_output = Vector3::new(-4.5, 5.6, -6.7);
        let received = Instant::now();
        let output = spec.transform_sample(ImuSample {
            orientation_xyzw: [
                source_orientation.i,
                source_orientation.j,
                source_orientation.k,
                source_orientation.w,
            ],
            angular_velocity_xyz: (output_from_source.inverse() * angular_velocity_output).into(),
            linear_acceleration_xyz: (output_from_source.inverse() * acceleration_output).into(),
            device_timestamp_ms: 123,
            received,
        });

        let [x, y, z, w] = output.orientation_xyzw;
        let orientation = UnitQuaternion::new_normalize(Quaternion::new(w, x, y, z));
        assert!(orientation.angle_to(&world_from_output) < 1.0e-12);
        assert!(
            (Vector3::from(output.angular_velocity_xyz) - angular_velocity_output).norm() < 1.0e-12
        );
        assert!(
            (Vector3::from(output.linear_acceleration_xyz) - acceleration_output).norm() < 1.0e-12
        );
        assert_eq!(output.device_timestamp_ms, 123);
        assert_eq!(output.received, received);
    }

    #[test]
    fn mini_urdf_turns_upside_down_sensor_sample_upright() {
        let spec = resolve(&valid_config()).unwrap();
        let output = spec.transform_sample(ImuSample {
            orientation_xyzw: [1.0, 0.0, 0.0, 0.0],
            angular_velocity_xyz: [1.0, 2.0, 3.0],
            linear_acceleration_xyz: [-4.0, 5.0, -6.0],
            device_timestamp_ms: 456,
            received: Instant::now(),
        });

        let [x, y, z, w] = output.orientation_xyzw;
        let orientation = UnitQuaternion::new_normalize(Quaternion::new(w, x, y, z));
        assert!(orientation.angle() < 1.0e-12);
        assert!(
            (Vector3::from(output.angular_velocity_xyz) - Vector3::new(1.0, -2.0, -3.0)).norm()
                < 1.0e-12
        );
        assert!(
            (Vector3::from(output.linear_acceleration_xyz) - Vector3::new(-4.0, -5.0, 6.0)).norm()
                < 1.0e-12
        );
    }
}
