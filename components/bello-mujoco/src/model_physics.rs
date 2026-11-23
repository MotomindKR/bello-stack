use anyhow::{Context, Result, ensure};
use bello_robot_description::RobotDescription;
use mujoco_rs::prelude::*;
use nalgebra::Point3;
use serde::Deserialize;
use std::collections::{BTreeMap, BTreeSet};

#[derive(Debug, Default, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct PhysicsConfig {
    pub allow_mesh_collisions: bool,
    pub additional_inertias: Vec<AdditionalInertia>,
    pub joint_dynamics: BTreeMap<String, JointDynamics>,
    pub collision_exclusions: Vec<[String; 2]>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AdditionalInertia {
    pub frame: String,
    pub mass_kg: f64,
    pub center_m: [f64; 3],
    pub diagonal_kg_m2: [f64; 3],
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct JointDynamics {
    pub damping: f64,
    pub armature: f64,
    #[serde(default)]
    pub limit_margin: f64,
    pub limit_time_constant: Option<f64>,
}

pub(crate) fn apply_physics(spec: &mut MjSpec, config: &PhysicsConfig) -> Result<f64> {
    let mut frames = BTreeSet::new();
    let mut mass = 0.;
    for inertia in &config.additional_inertias {
        ensure!(
            frames.insert(&inertia.frame),
            "duplicate additional inertia"
        );
        let diagonal = inertia.diagonal_kg_m2;
        ensure!(
            inertia.mass_kg.is_finite()
                && inertia.mass_kg > 0.
                && inertia.center_m.iter().all(|v| v.is_finite())
                && diagonal.iter().all(|v| v.is_finite() && *v > 0.)
                && (0..3).all(|i| diagonal[i] <= diagonal[(i + 1) % 3] + diagonal[(i + 2) % 3]),
            "invalid additional inertia for {}",
            inertia.frame
        );
        let body = spec
            .body_mut(&inertia.frame)
            .context("additional inertia frame missing")?;
        ensure!(
            body.mass() == 0.,
            "additional inertia cannot replace existing inertia"
        );
        body.set_explicitinertial(true);
        body.set_mass(inertia.mass_kg);
        body.with_ipos(inertia.center_m)
            .with_iquat([1., 0., 0., 0.])
            .with_inertia(diagonal);
        mass += inertia.mass_kg;
    }
    for (name, dynamics) in &config.joint_dynamics {
        ensure!(
            [dynamics.damping, dynamics.armature, dynamics.limit_margin]
                .iter()
                .all(|v| v.is_finite() && *v >= 0.),
            "invalid joint dynamics for {name}"
        );
        let joint = spec
            .joint_mut(name)
            .context("joint dynamics names missing joint")?;
        if dynamics.limit_margin > 0. {
            let range = joint.range();
            ensure!(
                2. * dynamics.limit_margin < range[1] - range[0],
                "joint limit margin is too large for {name}"
            );
        }
        if let Some(seconds) = dynamics.limit_time_constant {
            ensure!(
                seconds.is_finite() && seconds > 0.,
                "invalid joint limit time constant"
            );
            joint.with_solref_limit([seconds, 1.]);
        }
        joint
            .with_damping([dynamics.damping, 0., 0.])
            .with_armature(dynamics.armature)
            .with_margin(dynamics.limit_margin);
    }
    for [a, b] in &config.collision_exclusions {
        ensure!(
            a != b && spec.body(a).is_some() && spec.body(b).is_some(),
            "invalid collision exclusion"
        );
        let exclude = spec.add_exclude();
        exclude.set_bodyname1(a);
        exclude.set_bodyname2(b);
    }
    Ok(mass)
}

pub(crate) fn add_slider_cranks(
    spec: &mut MjSpec,
    description: &RobotDescription,
    initial: &BTreeMap<String, f64>,
) -> Result<()> {
    for linkage in description.slider_cranks() {
        for side in 0..2 {
            let rod = &description.joint(&linkage.rods[side]).unwrap().child;
            let slider = &description.joint(&linkage.sliders[side]).unwrap().child;
            let base = description.root_link();
            // Both anchors lie in the hinge plane; translating them equally along
            // the parallel hinge axes gives the same pin constraint.
            let rod_pin = Point3::new(linkage.rod_length, 0., 0.);
            let root_from_rod = description.link_transform(base, rod, initial)?;
            let root_from_slider = description.link_transform(base, slider, initial)?;
            let slider_pin = root_from_slider.inverse() * root_from_rod * rod_pin;
            let limits = description
                .joint(&linkage.driver)
                .unwrap()
                .limits
                .as_ref()
                .context("slider-crank driver needs limits")?;
            for q in [limits.lower, limits.upper] {
                let mut pose = initial.clone();
                pose.insert(linkage.driver.clone(), q);
                let a = description.link_transform(base, rod, &pose)? * rod_pin;
                let b = description.link_transform(base, slider, &pose)? * slider_pin;
                ensure!(
                    (a - b).norm() < 1e-7,
                    "URDF frames disagree with slider-crank {}",
                    linkage.name
                );
            }
            let rod_site = format!("{}_rod_{side}", linkage.name);
            let slider_site = format!("{}_slider_{side}", linkage.name);
            spec.body_mut(rod)
                .context("missing rod body")?
                .add_site()
                .with_name(&rod_site)
                .with_pos(rod_pin.coords.into())
                .with_size([0.001; 3]);
            spec.body_mut(slider)
                .context("missing slider body")?
                .add_site()
                .with_name(&slider_site)
                .with_pos(slider_pin.coords.into())
                .with_size([0.001; 3]);
            let equality = spec.add_equality();
            equality.set_name(&format!("{}_closure_{side}", linkage.name))?;
            equality.set_type(MjtEq::mjEQ_CONNECT);
            equality.set_objtype(MjtObj::mjOBJ_SITE);
            equality.set_name1(&rod_site);
            equality.set_name2(&slider_site);
            equality
                .with_solref([0.004, 1.])
                .with_solimp([0.999, 0.9999, 0.001, 0.5, 2.]);
        }
    }
    Ok(())
}
