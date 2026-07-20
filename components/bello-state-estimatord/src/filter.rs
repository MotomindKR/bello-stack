use std::time::Duration;

use anyhow::bail;
use nalgebra::{SMatrix, SVector, UnitQuaternion, Vector3};

use crate::config::EstimatorConfig;
use crate::model::FootKinematics;

const POSITION: usize = 0;
const VELOCITY: usize = 3;
const ACCEL_BIAS: usize = 6;
const LEFT_CONTACT: usize = 9;
const RIGHT_CONTACT: usize = 12;
const STATE_SIZE: usize = 15;

type StateVector = SVector<f64, STATE_SIZE>;
type Covariance = SMatrix<f64, STATE_SIZE, STATE_SIZE>;
type MeasurementMatrix = SMatrix<f64, 3, STATE_SIZE>;

#[derive(Clone, Copy, Debug)]
pub struct PoseEstimate {
    pub position: Vector3<f64>,
    pub orientation: UnitQuaternion<f64>,
}

#[derive(Clone, Copy, Debug)]
struct ContactThresholds {
    entry_speed: f64,
    exit_speed: f64,
    entry_height: f64,
    exit_height: f64,
    dwell: Duration,
}

#[derive(Clone, Copy, Debug, Default)]
struct ContactLatch {
    active: bool,
    qualifying_for: Duration,
}

#[derive(Debug)]
struct ContactManager {
    thresholds: ContactThresholds,
    feet: [ContactLatch; 2],
}

impl ContactManager {
    fn new(config: &EstimatorConfig) -> Self {
        Self {
            thresholds: ContactThresholds {
                entry_speed: config.contact_entry_speed_mps,
                exit_speed: config.contact_exit_speed_mps,
                entry_height: config.contact_entry_height_m,
                exit_height: config.contact_exit_height_m,
                dwell: config.contact_dwell(),
            },
            feet: [ContactLatch::default(); 2],
        }
    }

    fn initialize(&mut self, heights: [f64; 2], speeds: [f64; 2]) -> [bool; 2] {
        let minimum = heights[0].min(heights[1]);
        for index in 0..2 {
            self.feet[index].active = heights[index] - minimum <= self.thresholds.entry_height
                && speeds[index] <= self.thresholds.entry_speed;
            self.feet[index].qualifying_for = Duration::ZERO;
        }
        [self.feet[0].active, self.feet[1].active]
    }

    fn update(&mut self, heights: [f64; 2], speeds: [f64; 2], dt: Duration) -> [bool; 2] {
        let minimum = heights[0].min(heights[1]);
        for index in 0..2 {
            let relative_height = heights[index] - minimum;
            let latch = &mut self.feet[index];
            if latch.active {
                if relative_height > self.thresholds.exit_height
                    || speeds[index] > self.thresholds.exit_speed
                {
                    latch.active = false;
                    latch.qualifying_for = Duration::ZERO;
                }
            } else if relative_height <= self.thresholds.entry_height
                && speeds[index] <= self.thresholds.entry_speed
            {
                latch.qualifying_for = latch.qualifying_for.saturating_add(dt);
                if latch.qualifying_for >= self.thresholds.dwell {
                    latch.active = true;
                    latch.qualifying_for = Duration::ZERO;
                }
            } else {
                latch.qualifying_for = Duration::ZERO;
            }
        }
        [self.feet[0].active, self.feet[1].active]
    }
}

/// Translation-only contact-aided linear Kalman filter. Orientation is supplied
/// by the EBIMU and deliberately remains outside the state.
#[derive(Debug)]
pub struct ContactFilter {
    state: StateVector,
    covariance: Covariance,
    contacts: ContactManager,
    initialized: bool,
    active_contacts: [bool; 2],
}

impl ContactFilter {
    pub fn new(config: &EstimatorConfig) -> Self {
        Self {
            state: StateVector::zeros(),
            covariance: Covariance::identity(),
            contacts: ContactManager::new(config),
            initialized: false,
            active_contacts: [false; 2],
        }
    }

    pub fn initialized(&self) -> bool {
        self.initialized
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
        let velocities = feet.map(|foot| {
            (omega_world.cross(&(rotation * foot.position)) + rotation * foot.relative_velocity)
                .norm()
        });
        let minimum_height = relative[0].z.min(relative[1].z);
        self.state[POSITION + 2] = -minimum_height;
        let base_position = self.position();
        self.active_contacts = self
            .contacts
            .initialize([relative[0].z, relative[1].z], velocities);
        for (index, relative_position) in relative.into_iter().enumerate() {
            self.set_contact_position(index, base_position + relative_position);
        }
        self.initialized = true;
        PoseEstimate {
            position: base_position,
            orientation,
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
        for offset in [LEFT_CONTACT, RIGHT_CONTACT] {
            process
                .fixed_view_mut::<3, 3>(offset, offset)
                .copy_from(&(SMatrix::<f64, 3, 3>::identity() * (4.0e-6 * dt)));
        }
        self.covariance = transition * self.covariance * transition.transpose() + process;
        self.symmetrize();
        Ok(PoseEstimate {
            position: self.position(),
            orientation,
        })
    }

    pub fn correct_contacts(
        &mut self,
        orientation: UnitQuaternion<f64>,
        angular_velocity_local: Vector3<f64>,
        feet: [FootKinematics; 2],
        update_interval: Duration,
    ) -> anyhow::Result<PoseEstimate> {
        if !self.initialized {
            return Ok(self.initialize(orientation, angular_velocity_local, feet));
        }
        let rotation = orientation.to_rotation_matrix();
        let omega_world = rotation * angular_velocity_local;
        let base_position = self.position();
        let base_velocity = self.velocity();
        let relative = feet.map(|foot| rotation * foot.position);
        let world_velocity = feet.map(|foot| {
            base_velocity
                + omega_world.cross(&(rotation * foot.position))
                + rotation * foot.relative_velocity
        });
        let heights = [
            base_position.z + relative[0].z,
            base_position.z + relative[1].z,
        ];
        let speeds = [world_velocity[0].norm(), world_velocity[1].norm()];
        let previous = self.active_contacts;
        self.active_contacts = self.contacts.update(heights, speeds, update_interval);
        for index in 0..2 {
            if self.active_contacts[index] && !previous[index] {
                self.set_contact_position(index, self.position() + relative[index]);
                self.reset_contact_covariance(index);
            }
            if self.active_contacts[index] {
                self.correct_contact(index, relative[index])?;
            }
        }
        Ok(PoseEstimate {
            position: self.position(),
            orientation,
        })
    }

    fn correct_contact(&mut self, index: usize, relative: Vector3<f64>) -> anyhow::Result<()> {
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
        let noise = SMatrix::<f64, 3, 3>::identity() * 4.0e-6;
        let innovation_covariance = observation * self.covariance * observation.transpose() + noise;
        let inverse = innovation_covariance
            .try_inverse()
            .ok_or_else(|| anyhow::anyhow!("contact innovation covariance is singular"))?;
        let gain = self.covariance * observation.transpose() * inverse;
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

    fn set_contact_position(&mut self, index: usize, value: Vector3<f64>) {
        self.state
            .fixed_rows_mut::<3>(contact_offset(index))
            .copy_from(&value);
    }
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
            listen: "127.0.0.1:50052".parse().unwrap(),
            imu: "imu".into(),
            reference_frame: "odom".into(),
            tracked_frame: "pelvis".into(),
            left_foot: "left".into(),
            right_foot: "right".into(),
            imu_stale_ms: 20,
            motor_stale_ms: 50,
            contact_entry_speed_mps: 0.15,
            contact_exit_speed_mps: 0.30,
            contact_entry_height_m: 0.03,
            contact_exit_height_m: 0.06,
            contact_dwell_ms: 40,
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
        assert_eq!(filter.active_contacts, [true, true]);
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
}
