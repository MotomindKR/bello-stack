use crate::Simulation;
use anyhow::{Context, Result, ensure};
use mujoco_rs::prelude::*;
use mujoco_rs::wrappers::mj_editing::MjtLimited;
use nalgebra::{Quaternion, UnitQuaternion, Vector3};
use serde::Deserialize;

const BODY: &str = "bello_emulator_torso_support";
const JOINT: &str = "bello_emulator_torso_slider";
const EQUALITY: &str = "bello_emulator_torso_hold";

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Config {
    pub body: String,
    pub drop_limit_m: f64,
}

pub fn add(spec: &mut MjSpec, config: &Config) -> Result<()> {
    ensure!(
        config.drop_limit_m.is_finite() && config.drop_limit_m > 0.,
        "invalid torso drop limit"
    );
    spec.body(&config.body)
        .context("missing torso guide body")?;
    let support = spec.world_body_mut().add_body();
    support.set_name(BODY)?;
    support.set_explicitinertial(true);
    support.set_mass(0.1);
    support.with_inertia([1e-4; 3]).with_gravcomp(1.);
    support
        .add_joint()
        .with_name(JOINT)
        .with_type(MjtJoint::mjJNT_SLIDE)
        .with_axis([0., 0., 1.])
        .with_limited(MjtLimited::mjLIMITED_TRUE)
        .with_range([-config.drop_limit_m, 0.])
        .with_solref_limit([0.01, 1.])
        .with_solimp_limit([0.99, 0.9999, 0.001, 0.5, 2.]);
    let equality = spec.add_equality();
    equality.set_name(EQUALITY)?;
    equality.set_type(MjtEq::mjEQ_WELD);
    equality.set_objtype(MjtObj::mjOBJ_BODY);
    equality.set_name1(&config.body);
    equality.set_name2(BODY);
    equality.set_active(false);
    equality
        .with_solref([0.01, 1.])
        .with_solimp([0.99, 0.9999, 0.001, 0.5, 2.]);
    Ok(())
}

pub struct Guide {
    equality: usize,
    torso: usize,
    support: usize,
    pub active: bool,
}

impl Guide {
    pub fn bind(sim: &mut Simulation, config: &Config) -> Result<Self> {
        let mut guide = Self {
            equality: sim.id(MjtObj::mjOBJ_EQUALITY, EQUALITY)? as usize,
            torso: sim.id(MjtObj::mjOBJ_BODY, &config.body)? as usize,
            support: sim.id(MjtObj::mjOBJ_BODY, BODY)? as usize,
            active: false,
        };
        guide.initialize(sim)?;
        Ok(guide)
    }

    pub fn initialize(&mut self, sim: &mut Simulation) -> Result<()> {
        self.release(sim);
        sim.forward();
        let a = sim.body_state(self.torso)?;
        let b = sim.body_state(self.support)?;
        let rotation =
            |q: [f64; 4]| UnitQuaternion::new_normalize(Quaternion::new(q[0], q[1], q[2], q[3]));
        let qa = rotation(a.orientation_wxyz);
        let position =
            qa.inverse_transform_vector(&(Vector3::from(b.position) - Vector3::from(a.position)));
        let quaternion = qa.inverse() * rotation(b.orientation_wxyz);
        // Only the weld's numeric target changes; topology and allocated data stay fixed.
        unsafe {
            sim.data.model_mut().eq_data_mut()[self.equality] = [
                0.,
                0.,
                0.,
                position.x,
                position.y,
                position.z,
                quaternion.w,
                quaternion.i,
                quaternion.j,
                quaternion.k,
                1.,
            ];
        }
        sim.data.eq_active_mut()[self.equality] = true;
        self.active = true;
        sim.forward();
        Ok(())
    }

    pub fn release(&mut self, sim: &mut Simulation) {
        sim.data.eq_active_mut()[self.equality] = false;
        self.active = false;
    }
}
