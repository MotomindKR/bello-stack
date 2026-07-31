use std::time::Duration;

use anyhow::bail;
use nalgebra::{SMatrix, SVector, UnitQuaternion, Vector3};

use crate::config::EstimatorConfig;
use crate::contact::ContactManager;
use crate::model::FootKinematics;

const POSITION: usize = 0;
const VELOCITY: usize = 3;
const ACCEL_BIAS: usize = 6;
const LEFT_CONTACT: usize = 9;
const RIGHT_CONTACT: usize = 12;
const STATE_SIZE: usize = 15;
const CONTACT_MEASUREMENT_VARIANCE_MIN: f64 = 4.0e-6;
const CONTACT_MEASUREMENT_VARIANCE_MAX: f64 = 1.0e-2;
const CONTACT_PROCESS_VARIANCE_MIN: f64 = 4.0e-6;
const CONTACT_PROCESS_VARIANCE_MAX: f64 = 2.5e-2;
const CONTACT_PROBABILITY_EXPONENT: i32 = 3;
const CONTACT_BIAS_UPDATE_PROBABILITY: f64 = 0.95;
const CONTACT_REFERENCE_INTERVAL_SECONDS: f64 = 0.01;
const CONTACT_MINIMUM_INTERVAL_SECONDS: f64 = 0.002;
const CONTACT_MAXIMUM_INTERVAL_SECONDS: f64 = 0.05;

type StateVector = SVector<f64, STATE_SIZE>;
type Covariance = SMatrix<f64, STATE_SIZE, STATE_SIZE>;
type MeasurementMatrix = SMatrix<f64, 3, STATE_SIZE>;

#[derive(Clone, Copy, Debug)]
pub struct PoseEstimate {
    pub position: Vector3<f64>,
    pub orientation: UnitQuaternion<f64>,
    pub linear_velocity: Vector3<f64>,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct ContactDiagnostics {
    pub active: bool,
    pub changed: bool,
    pub probability: f64,
    pub effective_probability: f64,
    pub height: f64,
    pub relative_height: f64,
    pub world_speed: f64,
    pub kinematic_speed: f64,
    pub contact_motion_speed: f64,
    pub world_speed_reliability: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct ContactCorrection {
    pub pose: PoseEstimate,
    pub contacts: [ContactDiagnostics; 2],
}

#[derive(Clone, Copy, Debug)]
pub struct FilterDiagnostics {
    pub position: Vector3<f64>,
    pub velocity: Vector3<f64>,
    pub accelerometer_bias: Vector3<f64>,
    pub contact_positions: [Vector3<f64>; 2],
    pub active_contacts: [bool; 2],
    pub contact_probabilities: [f64; 2],
    pub effective_contact_probabilities: [f64; 2],
}

/// Translation-only contact-aided linear Kalman filter. Filtered EBIMU
/// orientation deliberately remains outside the state.
#[derive(Debug)]
pub struct ContactFilter {
    state: StateVector,
    covariance: Covariance,
    contacts: ContactManager,
    planar: bool,
    ground_height: f64,
    initialized: bool,
}

impl ContactFilter {
    pub fn new(config: &EstimatorConfig) -> Self {
        Self {
            state: StateVector::zeros(),
            covariance: Covariance::identity(),
            contacts: ContactManager::new(config),
            planar: config.planar,
            ground_height: config.ground_height_m,
            initialized: false,
        }
    }

    pub fn initialized(&self) -> bool {
        self.initialized
    }

    pub fn diagnostics(&self) -> FilterDiagnostics {
        let contact_probabilities = self.contacts.probabilities();
        FilterDiagnostics {
            position: self.position(),
            velocity: self.velocity(),
            accelerometer_bias: self.accelerometer_bias(),
            contact_positions: [self.contact_position(0), self.contact_position(1)],
            active_contacts: self.contacts.active(),
            contact_probabilities,
            effective_contact_probabilities: contact_probabilities
                .map(effective_contact_probability),
        }
    }

    pub fn initialize(
        &mut self,
        orientation: UnitQuaternion<f64>,
        angular_velocity_local: Vector3<f64>,
        feet: [FootKinematics; 2],
    ) -> PoseEstimate {
        self.state.fill(0.0);
        self.covariance.fill(0.0);
        for index in 0..STATE_SIZE {
            self.covariance[(index, index)] = match index {
                POSITION..=2 => 1.0e-3,
                VELOCITY..=5 => 1.0e-2,
                ACCEL_BIAS..=8 => 1.0e-2,
                _ => 1.0e-3,
            };
        }
        let rotation = orientation.to_rotation_matrix();
        let omega_world = rotation * angular_velocity_local;
        let relative = feet.map(|foot| rotation * foot.position);
        let kinematic_velocities = feet.map(|foot| {
            omega_world.cross(&(rotation * foot.position)) + rotation * foot.relative_velocity
        });
        let contact_motion_speeds = kinematic_velocities.map(|velocity| {
            if self.planar {
                velocity.z.abs()
            } else {
                velocity.norm()
            }
        });
        let minimum_height = relative[0].z.min(relative[1].z);
        let ground_height = if self.planar { self.ground_height } else { 0.0 };
        self.state[POSITION + 2] = ground_height - minimum_height;
        let base_position = self.position();
        let contacts = self.contacts.initialize(
            [
                base_position.z + relative[0].z,
                base_position.z + relative[1].z,
            ],
            contact_motion_speeds,
        );
        for (index, relative_position) in relative.into_iter().enumerate() {
            self.set_contact_position(index, base_position + relative_position);
            if self.planar && contacts[index].active {
                self.constrain_contact_to_plane(index);
            }
        }
        self.initialized = true;
        PoseEstimate {
            position: base_position,
            orientation,
            linear_velocity: self.velocity(),
        }
    }

    pub fn propagate(
        &mut self,
        orientation: UnitQuaternion<f64>,
        linear_acceleration_local: Vector3<f64>,
        dt: Duration,
    ) -> anyhow::Result<PoseEstimate> {
        if !self.initialized {
            bail!("contact filter is not initialized");
        }
        let dt = dt.as_secs_f64();
        if !dt.is_finite() || dt <= 0.0 || dt > 0.1 {
            bail!("IMU integration interval must be in (0, 0.1] seconds, got {dt}");
        }
        let rotation = orientation.to_rotation_matrix().into_inner();
        let bias = self.accelerometer_bias();
        let acceleration = rotation * (linear_acceleration_local - bias);
        let position = self.position();
        let velocity = self.velocity();
        self.set_position(position + velocity * dt + acceleration * (0.5 * dt * dt));
        self.set_velocity(velocity + acceleration * dt);

        let mut transition = Covariance::identity();
        transition
            .fixed_view_mut::<3, 3>(POSITION, VELOCITY)
            .copy_from(&(SMatrix::<f64, 3, 3>::identity() * dt));
        transition
            .fixed_view_mut::<3, 3>(POSITION, ACCEL_BIAS)
            .copy_from(&(-rotation * (0.5 * dt * dt)));
        transition
            .fixed_view_mut::<3, 3>(VELOCITY, ACCEL_BIAS)
            .copy_from(&(-rotation * dt));
        let mut process = Covariance::zeros();
        process
            .fixed_view_mut::<3, 3>(POSITION, POSITION)
            .copy_from(&(SMatrix::<f64, 3, 3>::identity() * (1.0e-4 * dt)));
        process
            .fixed_view_mut::<3, 3>(VELOCITY, VELOCITY)
            .copy_from(&(SMatrix::<f64, 3, 3>::identity() * (1.0e-2 * dt)));
        process
            .fixed_view_mut::<3, 3>(ACCEL_BIAS, ACCEL_BIAS)
            .copy_from(&(SMatrix::<f64, 3, 3>::identity() * (2.5e-5 * dt)));
        let contact_probabilities = self.contacts.probabilities();
        for (index, offset) in [LEFT_CONTACT, RIGHT_CONTACT].into_iter().enumerate() {
            let variance = confidence_scaled_variance(
                effective_contact_probability(contact_probabilities[index]),
                CONTACT_PROCESS_VARIANCE_MIN,
                CONTACT_PROCESS_VARIANCE_MAX,
            );
            process
                .fixed_view_mut::<3, 3>(offset, offset)
                .copy_from(&(SMatrix::<f64, 3, 3>::identity() * (variance * dt)));
        }
        self.covariance = transition * self.covariance * transition.transpose() + process;
        self.symmetrize();
        Ok(PoseEstimate {
            position: self.position(),
            orientation,
            linear_velocity: self.velocity(),
        })
    }

    pub fn correct_contacts(
        &mut self,
        orientation: UnitQuaternion<f64>,
        angular_velocity_local: Vector3<f64>,
        feet: [FootKinematics; 2],
        update_interval: Duration,
    ) -> anyhow::Result<ContactCorrection> {
        if !self.initialized {
            let pose = self.initialize(orientation, angular_velocity_local, feet);
            return Ok(ContactCorrection {
                pose,
                contacts: [ContactDiagnostics::default(); 2],
            });
        }
        let rotation = orientation.to_rotation_matrix();
        let omega_world = rotation * angular_velocity_local;
        let base_position = self.position();
        let base_velocity = self.velocity();
        let relative = feet.map(|foot| rotation * foot.position);
        let kinematic_velocity = feet.map(|foot| {
            omega_world.cross(&(rotation * foot.position)) + rotation * foot.relative_velocity
        });
        let world_velocity = kinematic_velocity.map(|velocity| base_velocity + velocity);
        let heights = [
            base_position.z + relative[0].z,
            base_position.z + relative[1].z,
        ];
        let speeds = [world_velocity[0].norm(), world_velocity[1].norm()];
        let kinematic_speeds = [kinematic_velocity[0].norm(), kinematic_velocity[1].norm()];
        let contact_motion_speeds = kinematic_velocity.map(|velocity| {
            if self.planar {
                velocity.z.abs()
            } else {
                velocity.norm()
            }
        });
        let contacts =
            self.contacts
                .update(heights, contact_motion_speeds, speeds, update_interval);
        let minimum_height = heights[0].min(heights[1]);
        let mut diagnostics = [ContactDiagnostics::default(); 2];
        for index in 0..2 {
            diagnostics[index].probability = contacts[index].probability;
            diagnostics[index].effective_probability =
                effective_contact_probability(contacts[index].probability);
            diagnostics[index].height = heights[index];
            diagnostics[index].relative_height = heights[index] - minimum_height;
            diagnostics[index].world_speed = speeds[index];
            diagnostics[index].kinematic_speed = kinematic_speeds[index];
            diagnostics[index].contact_motion_speed = contact_motion_speeds[index];
            diagnostics[index].world_speed_reliability = contacts[index].world_speed_reliability;
            if contacts[index].active && contacts[index].changed {
                self.set_contact_position(index, self.position() + relative[index]);
                self.reset_contact_covariance(index);
            }
            if contacts[index].active {
                if self.planar {
                    self.constrain_contact_to_plane(index);
                }
                self.correct_contact(
                    index,
                    relative[index],
                    contacts[index].probability,
                    update_interval,
                )?;
            }
            diagnostics[index].active = contacts[index].active;
            diagnostics[index].changed = contacts[index].changed;
        }
        Ok(ContactCorrection {
            pose: PoseEstimate {
                position: self.position(),
                orientation,
                linear_velocity: self.velocity(),
            },
            contacts: diagnostics,
        })
    }

    fn correct_contact(
        &mut self,
        index: usize,
        relative: Vector3<f64>,
        probability: f64,
        update_interval: Duration,
    ) -> anyhow::Result<()> {
        let offset = contact_offset(index);
        let mut observation = MeasurementMatrix::zeros();
        observation
            .fixed_view_mut::<3, 3>(0, POSITION)
            .copy_from(&SMatrix::<f64, 3, 3>::identity());
        observation
            .fixed_view_mut::<3, 3>(0, offset)
            .copy_from(&-SMatrix::<f64, 3, 3>::identity());
        let measurement = -relative;
        let innovation = measurement - observation * self.state;
        let variance = contact_measurement_variance(probability, update_interval);
        let noise = SMatrix::<f64, 3, 3>::identity() * variance;
        let innovation_covariance = observation * self.covariance * observation.transpose() + noise;
        let inverse = innovation_covariance
            .try_inverse()
            .ok_or_else(|| anyhow::anyhow!("contact innovation covariance is singular"))?;
        let mut gain = self.covariance * observation.transpose() * inverse;
        if probability < CONTACT_BIAS_UPDATE_PROBABILITY {
            gain.fixed_rows_mut::<3>(ACCEL_BIAS).fill(0.0);
        }
        self.state += gain * innovation;
        let identity = Covariance::identity();
        let residual = identity - gain * observation;
        self.covariance =
            residual * self.covariance * residual.transpose() + gain * noise * gain.transpose();
        self.symmetrize();
        Ok(())
    }

    fn reset_contact_covariance(&mut self, index: usize) {
        let offset = contact_offset(index);
        for row in 0..STATE_SIZE {
            for column in offset..offset + 3 {
                self.covariance[(row, column)] = 0.0;
                self.covariance[(column, row)] = 0.0;
            }
        }
        for axis in 0..3 {
            self.covariance[(offset + axis, offset + axis)] = 1.0e-3;
        }
    }

    fn constrain_contact_to_plane(&mut self, index: usize) {
        let vertical = contact_offset(index) + 2;
        self.state[vertical] = self.ground_height;
        for axis in 0..STATE_SIZE {
            self.covariance[(vertical, axis)] = 0.0;
            self.covariance[(axis, vertical)] = 0.0;
        }
    }

    fn symmetrize(&mut self) {
        self.covariance = (self.covariance + self.covariance.transpose()) * 0.5;
    }

    fn position(&self) -> Vector3<f64> {
        self.state.fixed_rows::<3>(POSITION).into_owned()
    }

    fn set_position(&mut self, value: Vector3<f64>) {
        self.state.fixed_rows_mut::<3>(POSITION).copy_from(&value);
    }

    fn velocity(&self) -> Vector3<f64> {
        self.state.fixed_rows::<3>(VELOCITY).into_owned()
    }

    fn set_velocity(&mut self, value: Vector3<f64>) {
        self.state.fixed_rows_mut::<3>(VELOCITY).copy_from(&value);
    }

    fn accelerometer_bias(&self) -> Vector3<f64> {
        self.state.fixed_rows::<3>(ACCEL_BIAS).into_owned()
    }

    fn contact_position(&self, index: usize) -> Vector3<f64> {
        self.state
            .fixed_rows::<3>(contact_offset(index))
            .into_owned()
    }

    fn set_contact_position(&mut self, index: usize, value: Vector3<f64>) {
        self.state
            .fixed_rows_mut::<3>(contact_offset(index))
            .copy_from(&value);
    }
}

fn confidence_scaled_variance(probability: f64, minimum: f64, maximum: f64) -> f64 {
    minimum + (1.0 - probability.clamp(0.0, 1.0)) * (maximum - minimum)
}

fn effective_contact_probability(probability: f64) -> f64 {
    probability
        .clamp(0.0, 1.0)
        .powi(CONTACT_PROBABILITY_EXPONENT)
}

fn contact_measurement_variance(probability: f64, update_interval: Duration) -> f64 {
    let confidence_variance = confidence_scaled_variance(
        effective_contact_probability(probability),
        CONTACT_MEASUREMENT_VARIANCE_MIN,
        CONTACT_MEASUREMENT_VARIANCE_MAX,
    );
    let interval = update_interval.as_secs_f64().clamp(
        CONTACT_MINIMUM_INTERVAL_SECONDS,
        CONTACT_MAXIMUM_INTERVAL_SECONDS,
    );
    confidence_variance * CONTACT_REFERENCE_INTERVAL_SECONDS / interval
}

fn contact_offset(index: usize) -> usize {
    if index == 0 {
        LEFT_CONTACT
    } else {
        RIGHT_CONTACT
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn config() -> EstimatorConfig {
        EstimatorConfig {
            urdf: "unused".into(),
            hardware_endpoint: "http://127.0.0.1:50051".into(),
            listen: "127.0.0.1:50053".parse().unwrap(),
            imu: "imu".into(),
            reference_frame: "odom".into(),
            tracked_frame: "pelvis".into(),
            planar: false,
            ground_height_m: 0.0,
            left_foot: "left".into(),
            right_foot: "right".into(),
            imu_stale_ms: 20,
            motor_stale_ms: 50,
            orientation_lpf_cutoff_hz: 20.0,
            gyro_lpf_cutoff_hz: 30.0,
            acceleration_lpf_cutoff_hz: 25.0,
            contact_height_full_m: 0.015,
            contact_height_zero_m: 0.06,
            contact_motion_full_mps: 0.10,
            contact_motion_zero_mps: 0.60,
            contact_world_speed_full_mps: 0.10,
            contact_world_speed_zero_mps: 0.50,
            contact_probability_rise_ms: 80,
            contact_probability_fall_ms: 20,
        }
    }

    fn feet() -> [FootKinematics; 2] {
        [
            FootKinematics {
                position: Vector3::new(0.0, 0.1, -0.8),
                relative_velocity: Vector3::zeros(),
            },
            FootKinematics {
                position: Vector3::new(0.0, -0.1, -0.8),
                relative_velocity: Vector3::zeros(),
            },
        ]
    }

    #[test]
    fn stationary_robot_initializes_upright_on_ground() {
        let mut filter = ContactFilter::new(&config());
        let pose = filter.initialize(UnitQuaternion::identity(), Vector3::zeros(), feet());
        assert!((pose.position.z - 0.8).abs() < 1.0e-12);
        assert_eq!(filter.contacts.active(), [true, true]);
        assert_eq!(filter.contacts.probabilities(), [1.0, 1.0]);
    }

    #[test]
    fn planar_initialization_uses_the_configured_ground_height() {
        let mut config = config();
        config.planar = true;
        config.ground_height_m = 0.2;
        let mut filter = ContactFilter::new(&config);

        let pose = filter.initialize(UnitQuaternion::identity(), Vector3::zeros(), feet());

        assert!((pose.position.z - 1.0).abs() < 1.0e-12);
        assert_eq!(filter.contacts.active(), [true, true]);
    }

    #[test]
    fn planar_primary_motion_score_ignores_horizontal_kinematic_motion() {
        let mut config = config();
        config.planar = true;
        let mut filter = ContactFilter::new(&config);
        let mut moving_feet = feet();
        for foot in &mut moving_feet {
            foot.relative_velocity = Vector3::new(2.0, 0.0, 0.0);
        }

        filter.initialize(UnitQuaternion::identity(), Vector3::zeros(), moving_feet);

        assert_eq!(filter.contacts.active(), [true, true]);
        assert_eq!(filter.contacts.probabilities(), [1.0, 1.0]);
    }

    #[test]
    fn confidence_scales_measurement_and_anchor_process_variance() {
        assert_eq!(
            confidence_scaled_variance(
                1.0,
                CONTACT_MEASUREMENT_VARIANCE_MIN,
                CONTACT_MEASUREMENT_VARIANCE_MAX,
            ),
            CONTACT_MEASUREMENT_VARIANCE_MIN
        );
        assert_eq!(
            confidence_scaled_variance(
                0.0,
                CONTACT_PROCESS_VARIANCE_MIN,
                CONTACT_PROCESS_VARIANCE_MAX,
            ),
            CONTACT_PROCESS_VARIANCE_MAX
        );
        let half = confidence_scaled_variance(0.5, 2.0, 10.0);
        assert_eq!(half, 6.0);
    }

    #[test]
    fn contact_probability_is_cubed_before_filter_weighting() {
        assert_eq!(effective_contact_probability(0.5), 0.125);
        assert_eq!(effective_contact_probability(1.0), 1.0);
        assert_eq!(effective_contact_probability(-1.0), 0.0);
    }

    #[test]
    fn contact_information_is_normalized_by_update_interval() {
        let reference = contact_measurement_variance(0.8, Duration::from_secs_f64(0.01));
        let twice_as_fast = contact_measurement_variance(0.8, Duration::from_secs_f64(0.005));
        let half_as_fast = contact_measurement_variance(0.8, Duration::from_secs_f64(0.02));

        assert!((twice_as_fast - 2.0 * reference).abs() < 1.0e-12);
        assert!((half_as_fast - 0.5 * reference).abs() < 1.0e-12);
    }

    #[test]
    fn uncertain_contact_cannot_update_accelerometer_bias() {
        fn corrected_bias(probability: f64) -> Vector3<f64> {
            let mut filter = ContactFilter::new(&config());
            filter.initialize(UnitQuaternion::identity(), Vector3::zeros(), feet());
            filter
                .propagate(
                    UnitQuaternion::identity(),
                    Vector3::zeros(),
                    Duration::from_millis(100),
                )
                .unwrap();
            filter
                .correct_contact(
                    0,
                    Vector3::new(1.0, 0.1, -0.8),
                    probability,
                    Duration::from_millis(10),
                )
                .unwrap();
            filter.accelerometer_bias()
        }

        assert_eq!(corrected_bias(0.9), Vector3::zeros());
        assert!(corrected_bias(1.0).norm() > 1.0e-6);
    }

    #[test]
    fn gravity_compensated_zero_acceleration_does_not_fall() {
        let mut filter = ContactFilter::new(&config());
        filter.initialize(UnitQuaternion::identity(), Vector3::zeros(), feet());
        for _ in 0..100 {
            filter
                .propagate(
                    UnitQuaternion::identity(),
                    Vector3::zeros(),
                    Duration::from_millis(2),
                )
                .unwrap();
            filter
                .correct_contacts(
                    UnitQuaternion::identity(),
                    Vector3::zeros(),
                    feet(),
                    Duration::from_millis(10),
                )
                .unwrap();
        }
        assert!((filter.position().z - 0.8).abs() < 1.0e-8);
        assert!(filter.velocity().norm() < 1.0e-8);
    }

    #[test]
    fn planar_recontact_uses_relative_height_and_restores_the_ground() {
        fn recontact_after_vertical_error(planar: bool) -> ContactFilter {
            let mut config = config();
            config.planar = planar;
            let mut filter = ContactFilter::new(&config);
            filter.initialize(UnitQuaternion::identity(), Vector3::zeros(), feet());
            filter.state[POSITION + 2] = 0.4;
            filter.contacts.clear();
            for _ in 0..20 {
                filter
                    .correct_contacts(
                        UnitQuaternion::identity(),
                        Vector3::zeros(),
                        feet(),
                        Duration::from_millis(40),
                    )
                    .unwrap();
            }
            filter
        }

        let unconstrained = recontact_after_vertical_error(false);
        assert!((unconstrained.position().z - 0.4).abs() < 1.0e-12);

        let planar = recontact_after_vertical_error(true);
        assert!((planar.position().z - 0.8).abs() < 2.0e-3);
        assert_eq!(planar.state[LEFT_CONTACT + 2], 0.0);
        assert_eq!(planar.state[RIGHT_CONTACT + 2], 0.0);
    }

    #[test]
    fn planar_recontact_is_not_blocked_by_drifting_base_velocity() {
        let mut config = config();
        config.planar = true;
        let mut filter = ContactFilter::new(&config);
        filter.initialize(UnitQuaternion::identity(), Vector3::zeros(), feet());
        filter.set_velocity(Vector3::new(1.0, -2.0, 0.5));
        filter.contacts.clear();

        for _ in 0..3 {
            filter
                .correct_contacts(
                    UnitQuaternion::identity(),
                    Vector3::zeros(),
                    feet(),
                    Duration::from_millis(config.contact_probability_rise_ms),
                )
                .unwrap();
        }

        assert_eq!(filter.contacts.active(), [true, true]);
    }
}
