use nalgebra::{Isometry3, UnitQuaternion, Vector3};

pub use bello_robot_description::{CoordinateState, FootKinematics, KinematicModel};

#[derive(Clone, Copy, Debug)]
pub struct TrackedFrameState {
    pub position: Vector3<f64>,
    pub orientation: UnitQuaternion<f64>,
    pub linear_velocity: Vector3<f64>,
    pub angular_velocity_local: Vector3<f64>,
}

/// Rigid transform between the pose consumers request and the physical IMU.
#[derive(Clone, Copy, Debug)]
pub struct ImuFrameTransform {
    tracked_from_imu: Isometry3<f64>,
}

impl ImuFrameTransform {
    pub fn new(tracked_from_imu: Isometry3<f64>) -> Self {
        Self { tracked_from_imu }
    }

    /// Express tracked-frame foot kinematics relative to the IMU origin.
    pub fn feet_in_imu(&self, feet: [FootKinematics; 2]) -> [FootKinematics; 2] {
        let imu_from_tracked = self.tracked_from_imu.inverse();
        feet.map(|foot| FootKinematics {
            position: imu_from_tracked
                .transform_point(&foot.position.into())
                .coords,
            relative_velocity: imu_from_tracked.rotation * foot.relative_velocity,
        })
    }

    /// Convert an IMU-origin estimate to the configured tracked-frame origin.
    pub fn tracked_state(
        &self,
        imu_position: Vector3<f64>,
        world_from_imu: UnitQuaternion<f64>,
        imu_linear_velocity: Vector3<f64>,
        angular_velocity_imu: Vector3<f64>,
    ) -> TrackedFrameState {
        let imu_from_tracked = self.tracked_from_imu.inverse();
        let imu_to_tracked_world = world_from_imu * imu_from_tracked.translation.vector;
        let angular_velocity_world = world_from_imu * angular_velocity_imu;
        TrackedFrameState {
            position: imu_position + imu_to_tracked_world,
            orientation: world_from_imu * imu_from_tracked.rotation,
            linear_velocity: imu_linear_velocity
                + angular_velocity_world.cross(&imu_to_tracked_world),
            angular_velocity_local: self.tracked_from_imu.rotation * angular_velocity_imu,
        }
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::{Translation3, UnitQuaternion};

    use super::*;

    fn transform() -> ImuFrameTransform {
        ImuFrameTransform::new(Isometry3::from_parts(
            Translation3::new(-0.01, 0.0, -0.23),
            UnitQuaternion::from_euler_angles(0.0, 0.0, -std::f64::consts::FRAC_PI_2),
        ))
    }

    #[test]
    fn foot_kinematics_are_expressed_about_the_imu() {
        let transform = transform();
        let feet = transform.feet_in_imu([
            FootKinematics {
                position: Vector3::new(0.2, 0.1, -0.5),
                relative_velocity: Vector3::new(1.0, 2.0, 3.0),
            },
            FootKinematics {
                position: Vector3::zeros(),
                relative_velocity: Vector3::zeros(),
            },
        ]);
        let expected_position = transform
            .tracked_from_imu
            .inverse_transform_point(&Vector3::new(0.2, 0.1, -0.5).into())
            .coords;
        assert!((feet[0].position - expected_position).norm() < 1.0e-12);
        assert!(
            (transform.tracked_from_imu.rotation * feet[0].relative_velocity
                - Vector3::new(1.0, 2.0, 3.0))
            .norm()
                < 1.0e-12
        );
    }

    #[test]
    fn imu_pose_and_twist_are_shifted_to_the_tracked_origin() {
        let transform = transform();
        let world_from_tracked = UnitQuaternion::from_euler_angles(0.2, -0.3, 0.4);
        let world_from_imu = world_from_tracked * transform.tracked_from_imu.rotation;
        let angular_velocity_tracked = Vector3::new(0.7, -0.2, 0.4);
        let angular_velocity_imu =
            transform.tracked_from_imu.rotation.inverse() * angular_velocity_tracked;
        let tracked = transform.tracked_state(
            Vector3::new(1.0, 2.0, 3.0),
            world_from_imu,
            Vector3::new(0.3, -0.4, 0.5),
            angular_velocity_imu,
        );

        assert!(tracked.orientation.angle_to(&world_from_tracked) < 1.0e-12);
        assert!((tracked.angular_velocity_local - angular_velocity_tracked).norm() < 1.0e-12);
        let imu_from_tracked = transform.tracked_from_imu.inverse();
        let offset_world = world_from_imu * imu_from_tracked.translation.vector;
        assert!((tracked.position - (Vector3::new(1.0, 2.0, 3.0) + offset_world)).norm() < 1.0e-12);
        let omega_world = world_from_imu * angular_velocity_imu;
        assert!(
            (tracked.linear_velocity
                - (Vector3::new(0.3, -0.4, 0.5) + omega_world.cross(&offset_world)))
            .norm()
                < 1.0e-12
        );
    }
}
