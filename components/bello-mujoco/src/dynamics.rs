//! Model-only dynamics for measured-state observers. Never steps the simulation
//! or reads its contact solution, actuator commands, or simulated acceleration.
use crate::Simulation;
use anyhow::{Result, ensure};
use nalgebra::{DMatrix, DVector};

pub struct Dynamics {
    pub mass: DMatrix<f64>,
    pub bias_minus_passive: DVector<f64>,
    pub motor_moment: DMatrix<f64>,
    pub foot_wrench: DMatrix<f64>,
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn observer_dynamics_preserves_native_tendon_virtual_work_without_stepping() {
        let mut sim = crate::tests::simulation();
        let before = sim.data().time;
        let dynamics = sim.dynamics(&[0.2, 0.1], &[0.4, -0.2], [1, 2]).unwrap();
        assert_eq!(sim.data().time, before);
        assert!((&dynamics.mass - dynamics.mass.transpose()).norm() < 1e-12);
        assert!(dynamics.mass.clone().cholesky().is_some());
        let joint_torque = dynamics.motor_moment.transpose() * DVector::from_vec(vec![2., 1.]);
        assert_eq!(joint_torque.as_slice(), &[3., 1.]);
        assert!(dynamics.foot_wrench.iter().all(|x| x.is_finite()));
        assert!(sim.dynamics(&[0.], &[0., 0.], [1, 2]).is_err());
        assert!(sim.dynamics(&[0., 0.], &[f64::NAN, 0.], [1, 2]).is_err());
    }
}

impl Simulation {
    pub fn dynamics(&mut self, qpos: &[f64], qvel: &[f64], feet: [usize; 2]) -> Result<Dynamics> {
        let nv = self.model().nv as usize;
        let nu = self.model().nu as usize;
        ensure!(
            qpos.len() == self.model().nq as usize && qvel.len() == nv,
            "dynamics state dimensions do not match model"
        );
        ensure!(
            qpos.iter().chain(qvel).all(|x| x.is_finite()),
            "nonfinite dynamics state"
        );
        ensure!(
            feet.iter().all(|i| *i < self.model().nbody as usize),
            "invalid foot body"
        );
        self.data.qpos_mut().copy_from_slice(qpos);
        self.data.qvel_mut().copy_from_slice(qvel);
        self.data.fwd_position();
        self.data.fwd_velocity();
        let mut mass = vec![0.; nv * nv];
        self.data.full_m(&mut mass)?;
        let mut motor_moment = DMatrix::zeros(nu, nv);
        // MuJoCo stores transmission moments sparsely, including fixed tendons.
        for motor in 0..nu {
            unsafe {
                let data = self.data();
                let start = *data.moment_rowadr.add(motor) as usize;
                let count = *data.moment_rownnz.add(motor) as usize;
                for entry in start..start + count {
                    motor_moment[(motor, *data.moment_colind.add(entry) as usize)] =
                        *data.actuator_moment.add(entry);
                }
            }
        }
        let mut foot_wrench = DMatrix::zeros(nv, 12);
        for (foot, body) in feet.into_iter().enumerate() {
            let (position, rotation) = self.data.jac_body(true, true, body);
            for row in 0..nv {
                for axis in 0..3 {
                    foot_wrench[(row, foot * 6 + axis)] = position[axis * nv + row];
                    // Unknown moment divided by 0.1 m gives all wrench columns force units.
                    foot_wrench[(row, foot * 6 + axis + 3)] = 0.1 * rotation[axis * nv + row];
                }
            }
        }
        Ok(Dynamics {
            mass: DMatrix::from_row_slice(nv, nv, &mass),
            bias_minus_passive: DVector::from_iterator(
                nv,
                self.data
                    .qfrc_bias()
                    .iter()
                    .zip(self.data.qfrc_passive())
                    .map(|(bias, passive)| bias - passive),
            ),
            motor_moment,
            foot_wrench,
        })
    }
}
