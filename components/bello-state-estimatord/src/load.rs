//! Causal momentum observer using aligned measured motor feedback and the
//! contact filter's *predicted* root state, never truth or corrected current pose.
use crate::model::TrackedFrameState;
use anyhow::{Context, Result, ensure};
use bello_mujoco::{
    Simulation, ffi,
    model::{self, ModelConfig},
};
use bello_robot_description::{CoordinateState, JointKind, RobotDescription};
use nalgebra::{DMatrix, DVector};
use serde::Deserialize;
use std::collections::BTreeMap;
use std::path::PathBuf;
use tokio::sync::{mpsc, oneshot};

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LoadConfig {
    pub model: PathBuf,
    pub cutoff_hz: f64,
    pub velocity_cutoff_hz: f64,
    pub zero_weight_fraction: f64,
    pub full_weight_fraction: f64,
}

impl LoadConfig {
    pub fn validate(&self) -> Result<()> {
        ensure!(
            self.model.is_file(),
            "load observer model does not exist: {}",
            self.model.display()
        );
        ensure!(
            [
                self.cutoff_hz,
                self.velocity_cutoff_hz,
                self.full_weight_fraction
            ]
            .iter()
            .all(|v| v.is_finite() && *v > 0.),
            "invalid load observer parameters"
        );
        ensure!(
            self.zero_weight_fraction.is_finite()
                && self.zero_weight_fraction >= 0.
                && self.zero_weight_fraction < self.full_weight_fraction,
            "load fractions must satisfy 0 <= zero < full"
        );
        Ok(())
    }
}

pub(crate) fn check(
    config: LoadConfig,
    urdf: PathBuf,
    root: String,
    feet: [String; 2],
) -> Result<()> {
    Observer::new(config, urdf, root, feet).map(|_| ())
}

pub(crate) struct Sample {
    pub time_ns: u64,
    pub reset: bool,
    pub root: TrackedFrameState,
    pub coordinates: BTreeMap<String, CoordinateState>,
    pub torques: BTreeMap<String, f64>,
}

#[derive(Debug)]
pub(crate) struct Support {
    pub force_z: [f64; 2],
    pub probability: [f64; 2],
}

// MuJoCo model/data never cross threads. Only bounded requests and owned results do.
pub(crate) struct LoadObserver {
    pub coordinates: Vec<String>,
    requests: mpsc::Sender<(Sample, oneshot::Sender<Result<Support>>)>,
}

impl LoadObserver {
    pub async fn start(
        config: LoadConfig,
        urdf: PathBuf,
        root: String,
        feet: [String; 2],
    ) -> Result<Self> {
        let (requests, mut incoming) =
            mpsc::channel::<(Sample, oneshot::Sender<Result<Support>>)>(1);
        let (ready, initialized) = oneshot::channel();
        std::thread::Builder::new()
            .name("contact-dynamics".into())
            .spawn(move || {
                let mut dynamics = match Observer::new(config, urdf, root, feet) {
                    Ok(value) => value,
                    Err(error) => {
                        let _ = ready.send(Err(error));
                        return;
                    }
                };
                if ready
                    .send(Ok(dynamics.motors.keys().cloned().collect()))
                    .is_err()
                {
                    return;
                }
                while let Some((sample, response)) = incoming.blocking_recv() {
                    let _ = response.send(dynamics.update(sample));
                }
            })?;
        Ok(Self {
            coordinates: initialized
                .await
                .context("load observer initialization stopped")??,
            requests,
        })
    }

    pub async fn update(&self, sample: Sample) -> Result<Support> {
        let (response, result) = oneshot::channel();
        self.requests
            .send((sample, response))
            .await
            .context("load observer stopped")?;
        result.await.context("load observer response lost")?
    }
}

struct Previous {
    time_ns: u64,
    mass: DMatrix<f64>,
    velocity: DVector<f64>,
    momentum: DVector<f64>,
    internal: DVector<f64>,
}

impl Previous {
    fn external_force(
        &self,
        mass: &DMatrix<f64>,
        momentum: &DVector<f64>,
        residual: &DVector<f64>,
        gain: f64,
        dt: f64,
    ) -> DVector<f64> {
        let mdotv = (mass - &self.mass) * &self.velocity / dt;
        (momentum - &self.momentum - dt * (&self.internal + mdotv) + residual / gain)
            * (gain / (1. + gain * dt))
    }
}

fn wrench_fit(system: DMatrix<f64>, residual: DVector<f64>) -> Result<DVector<f64>> {
    let svd = system.svd(true, true);
    let gain = svd.singular_values.map(|s| s / (s * s + 0.001_f64.powi(2)));
    let wrench = svd.v_t.context("wrench SVD missing V")?.transpose()
        * gain.component_mul(&(svd.u.context("wrench SVD missing U")?.transpose() * residual));
    ensure!(
        wrench.iter().all(|v| v.is_finite()),
        "nonfinite load estimate"
    );
    Ok(wrench)
}

struct Observer {
    simulation: Simulation,
    description: RobotDescription,
    config: LoadConfig,
    motors: BTreeMap<String, usize>,
    joints: Vec<(String, usize, usize)>,
    feet: [usize; 2],
    weight: f64,
    previous: Option<Previous>,
    residual: DVector<f64>,
}

impl Observer {
    fn new(config: LoadConfig, urdf: PathBuf, root: String, feet: [String; 2]) -> Result<Self> {
        config.validate()?;
        let mut model_config = ModelConfig::load(&config.model)?;
        model_config.torso_guide = None;
        let robot = model::build(&model_config, None)?;
        let model_urdf = model_config
            .hardware
            .parent()
            .context("hardware directory missing")?
            .join(&robot.hardware.robot_description)
            .canonicalize()?;
        ensure!(
            model_urdf == urdf.canonicalize()?,
            "observer and kinematic URDF must be identical"
        );
        ensure!(
            robot.root_body == root,
            "load observer tracked frame must be the model root"
        );
        let description = RobotDescription::load(&urdf)?;
        let simulation = robot.simulation;
        let root_id = simulation.id(ffi::mjtObj::mjOBJ_JOINT, "root_free")? as usize;
        unsafe {
            ensure!(
                *simulation.model().jnt_qposadr.add(root_id) == 0
                    && *simulation.model().jnt_dofadr.add(root_id) == 0,
                "observer requires root-first free-joint layout"
            );
        }
        let mut joints = Vec::new();
        for joint in description.joints().filter(|j| j.kind != JointKind::Fixed) {
            let id = simulation.id(ffi::mjtObj::mjOBJ_JOINT, &joint.name)? as usize;
            unsafe {
                joints.push((
                    joint.name.clone(),
                    *simulation.model().jnt_qposadr.add(id) as usize,
                    *simulation.model().jnt_dofadr.add(id) as usize,
                ));
            }
        }
        ensure!(
            joints.len() + 6 == simulation.model().nv as usize,
            "unexpected observer joint layout"
        );
        let feet = [
            simulation.id(ffi::mjtObj::mjOBJ_BODY, &feet[0])? as usize,
            simulation.id(ffi::mjtObj::mjOBJ_BODY, &feet[1])? as usize,
        ];
        let weight = unsafe {
            std::slice::from_raw_parts(
                simulation.model().body_mass,
                simulation.model().nbody as usize,
            )
            .iter()
            .sum::<f64>()
                * 9.81
        };
        ensure!(
            weight.is_finite() && weight > 0.,
            "invalid observer body weight"
        );
        let nv = simulation.model().nv as usize;
        Ok(Self {
            simulation,
            description,
            config,
            joints,
            feet,
            weight,
            motors: robot
                .coordinates
                .into_iter()
                .map(|(name, value)| (name, value.actuator))
                .collect(),
            previous: None,
            residual: DVector::zeros(nv),
        })
    }

    fn update(&mut self, sample: Sample) -> Result<Support> {
        let dt = self
            .previous
            .as_ref()
            .and_then(|old| sample.time_ns.checked_sub(old.time_ns))
            .map(|ns| ns as f64 * 1e-9)
            .filter(|dt| *dt > 0. && *dt <= 0.1);
        if sample.reset || dt.is_none() {
            self.previous = None;
            self.residual.fill(0.);
        }
        let nv = self.simulation.model().nv as usize;
        let mut qpos = vec![0.; self.simulation.model().nq as usize];
        let mut velocity = DVector::zeros(nv);
        qpos[..3].copy_from_slice(sample.root.position.as_slice());
        let q = sample.root.orientation.quaternion();
        qpos[3..7].copy_from_slice(&[q.w, q.i, q.j, q.k]);
        velocity.as_mut_slice()[..3].copy_from_slice(sample.root.linear_velocity.as_slice());
        velocity.as_mut_slice()[3..6]
            .copy_from_slice(sample.root.angular_velocity_local.as_slice());
        let mut joints = sample.coordinates.clone();
        for transmission in self.description.transmissions() {
            joints.extend(
                transmission
                    .resolve(&sample.coordinates)?
                    .map(|(name, state)| {
                        (
                            name,
                            CoordinateState {
                                position: state.position,
                                velocity: state.velocity,
                            },
                        )
                    }),
            );
        }
        for (name, position, speed) in &self.joints {
            let value = joints
                .get(name)
                .with_context(|| format!("missing measured joint {name}"))?;
            qpos[*position] = value.position;
            velocity[*speed] = value.velocity;
            if let Some(old) = &self.previous {
                let alpha = 1.
                    - (-std::f64::consts::TAU * self.config.velocity_cutoff_hz * dt.unwrap()).exp();
                velocity[*speed] =
                    old.velocity[*speed] + alpha * (velocity[*speed] - old.velocity[*speed]);
            }
        }
        let mut torque = DVector::zeros(self.simulation.model().nu as usize);
        for (name, index) in &self.motors {
            torque[*index] = *sample
                .torques
                .get(name)
                .with_context(|| format!("missing measured torque {name}"))?;
        }
        ensure!(
            torque.iter().all(|x| x.is_finite()),
            "nonfinite measured torque"
        );
        let dynamics = self
            .simulation
            .dynamics(&qpos, velocity.as_slice(), self.feet)?;
        let momentum = &dynamics.mass * &velocity;
        let internal = dynamics.motor_moment.transpose() * torque - dynamics.bias_minus_passive;
        if let Some(old) = &self.previous {
            let dt = dt.unwrap();
            let gain = std::f64::consts::TAU * self.config.cutoff_hz;
            self.residual = old.external_force(&dynamics.mass, &momentum, &self.residual, gain, dt);
        }
        let system = DMatrix::from_fn(self.joints.len(), 12, |row, column| {
            dynamics.foot_wrench[(self.joints[row].2, column)]
        });
        let residual = DVector::from_iterator(
            self.joints.len(),
            self.joints.iter().map(|(_, _, row)| self.residual[*row]),
        );
        let wrench = wrench_fit(system, residual)?;
        let force_z = [wrench[2], wrench[8]];
        let probability = force_z.map(|force| {
            1. - crate::contact::descending_smoothstep(
                force,
                self.config.zero_weight_fraction * self.weight,
                self.config.full_weight_fraction * self.weight,
            )
        });
        self.previous = Some(Previous {
            time_ns: sample.time_ns,
            mass: dynamics.mass,
            velocity,
            momentum,
            internal,
        });
        Ok(Support {
            force_z,
            probability,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn momentum_observer_converges_to_unmeasured_external_force() {
        let mut previous = Previous {
            time_ns: 0,
            mass: DMatrix::identity(1, 1) * 2.,
            velocity: DVector::zeros(1),
            momentum: DVector::zeros(1),
            internal: DVector::from_element(1, 3.),
        };
        let mut residual = DVector::zeros(1);
        for _ in 0..100 {
            let momentum = &previous.momentum + DVector::from_element(1, (3. + 5.) * 0.01);
            residual = previous.external_force(&previous.mass, &momentum, &residual, 60., 0.01);
            previous.momentum = momentum;
            previous.velocity = &previous.momentum / 2.;
        }
        assert!((residual[0] - 5.).abs() < 1e-10);
    }

    #[test]
    fn changing_mass_is_not_mistaken_for_external_load() {
        let previous = Previous {
            time_ns: 0,
            mass: DMatrix::identity(2, 2),
            velocity: DVector::from_vec(vec![2., 3.]),
            momentum: DVector::from_vec(vec![2., 3.]),
            internal: DVector::zeros(2),
        };
        let mass = DMatrix::identity(2, 2) * 1.1;
        let momentum = &mass * &previous.velocity;
        let residual = previous.external_force(&mass, &momentum, &DVector::zeros(2), 60., 0.01);
        assert!(residual.norm() < 1e-10);
    }

    #[test]
    fn rank_deficient_wrench_system_remains_finite_and_unobservable_load_is_zero() {
        let mut system = DMatrix::zeros(20, 12);
        system[(0, 2)] = 1.;
        system[(1, 8)] = 1.;
        let mut residual = DVector::zeros(20);
        residual[0] = 20.;
        residual[1] = 30.;
        let wrench = wrench_fit(system, residual).unwrap();
        assert!((wrench[2] - 20.).abs() < 1e-4);
        assert!((wrench[8] - 30.).abs() < 1e-4);
        assert_eq!(wrench[0], 0.);
        assert_eq!(
            wrench_fit(DMatrix::zeros(20, 12), DVector::from_element(20, 3.))
                .unwrap()
                .norm(),
            0.
        );
    }
}
