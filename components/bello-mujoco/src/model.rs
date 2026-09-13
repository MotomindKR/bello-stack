//! Robot-owned native model construction. No policy or deployment artifact inputs.

use std::collections::{BTreeMap, BTreeSet};
use std::path::{Path, PathBuf};

use anyhow::{Context, Result, ensure};
use bello_robot_description::{JointKind, RobotDescription};
use mujoco_rs::prelude::*;
use mujoco_rs::wrappers::mj_editing::MjtLimited;
use robstride_protocol::MotorParams;
use serde::Deserialize;

use crate::Simulation;

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ModelConfig {
    pub hardware: PathBuf,
    pub timestep_seconds: f64,
    pub initial_root_position: [f64; 3],
    pub initial_root_quaternion: [f64; 4],
    pub initial_joint_positions: BTreeMap<String, f64>,
    pub neighbor_exclusion_depth: usize,
    #[serde(default)]
    pub payloads: Vec<Payload>,
    #[serde(default)]
    pub torso_guide: Option<crate::guide::Config>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Payload {
    pub frame: String,
    pub mass_kg: f64,
    pub radius_m: f64,
}

fn add_payloads(spec: &mut MjSpec, payloads: &[Payload]) -> Result<f64> {
    let mut frames = BTreeSet::new();
    let mut mass = 0.;
    for payload in payloads {
        ensure!(frames.insert(&payload.frame), "duplicate payload frame");
        ensure!(
            payload.mass_kg.is_finite()
                && payload.mass_kg > 0.
                && payload.radius_m.is_finite()
                && payload.radius_m > 0.,
            "invalid payload mass/radius"
        );
        let body = spec
            .body_mut(&payload.frame)
            .context("payload frame is missing")?;
        ensure!(body.mass() == 0., "payload frame must be inertial-free");
        let inertia = 0.4 * payload.mass_kg * payload.radius_m.powi(2);
        ensure!(
            inertia.is_finite() && inertia > 0.,
            "invalid payload inertia"
        );
        body.set_explicitinertial(true);
        body.set_mass(payload.mass_kg);
        body.with_ipos([0.; 3])
            .with_iquat([1., 0., 0., 0.])
            .with_inertia([inertia; 3]);
        mass += payload.mass_kg;
    }
    Ok(mass)
}

impl ModelConfig {
    pub fn load(path: &Path) -> Result<Self> {
        let mut config: Self = serde_yaml::from_str(&std::fs::read_to_string(path)?)?;
        config.hardware = path
            .parent()
            .context("config needs a parent directory")?
            .join(&config.hardware)
            .canonicalize()?;
        Ok(config)
    }
}

// This reads identity/geometry only; daemon health and command policy remain daemon-owned.
#[derive(Debug, Deserialize)]
pub struct Hardware {
    pub robot_description: PathBuf,
    pub logical_buses: Vec<Bus>,
    pub motors: Vec<Motor>,
    pub imus: Vec<Imu>,
}

#[derive(Debug, Deserialize)]
pub struct Bus {
    pub name: String,
    pub host_id: u8,
}

#[derive(Debug, Deserialize)]
pub struct Motor {
    pub name: String,
    pub coordinate: String,
    pub bus: String,
    pub id: u8,
    pub model: String,
    pub direction: i8,
}

#[derive(Debug, Deserialize)]
pub struct Imu {
    pub name: String,
    pub source_frame: String,
    pub output_frame: String,
}

#[derive(Clone, Copy, Debug)]
pub struct Coordinate {
    pub actuator: usize,
    // q_motor = native tendon length + this constant, derived from URDF offsets.
    pub offset: f64,
}

pub struct RobotModel {
    pub simulation: Simulation,
    pub hardware: Hardware,
    pub coordinates: BTreeMap<String, Coordinate>,
    pub root_body: String,
    pub guide: Option<crate::guide::Guide>,
}

pub fn build(config: &ModelConfig, export: Option<&Path>) -> Result<RobotModel> {
    Simulation::check_version()?;
    ensure!(
        config.timestep_seconds.is_finite() && config.timestep_seconds > 0.,
        "invalid timestep"
    );
    ensure!(
        config.initial_root_position.iter().all(|x| x.is_finite()),
        "invalid root position"
    );
    let norm: f64 = config.initial_root_quaternion.iter().map(|x| x * x).sum();
    ensure!(
        norm.is_finite() && (norm - 1.).abs() < 1e-8,
        "root quaternion must be normalized"
    );
    let hardware: Hardware = serde_yaml::from_str(&std::fs::read_to_string(&config.hardware)?)?;
    let urdf = config
        .hardware
        .parent()
        .context("hardware file needs a directory")?
        .join(&hardware.robot_description)
        .canonicalize()?;
    let description = RobotDescription::load(&urdf)?;
    let moving: BTreeSet<_> = description
        .joints()
        .filter(|j| j.kind != JointKind::Fixed)
        .map(|j| j.name.as_str())
        .collect();
    ensure!(
        moving
            == config
                .initial_joint_positions
                .keys()
                .map(String::as_str)
                .collect(),
        "initial_joint_positions must cover every movable URDF joint exactly"
    );
    for (name, position) in &config.initial_joint_positions {
        let limits = description
            .joint(name)
            .unwrap()
            .limits
            .as_ref()
            .context("joint has no limits")?;
        ensure!(
            position.is_finite() && *position >= limits.lower && *position <= limits.upper,
            "initial position for {name} is outside URDF limits"
        );
    }
    let buses: BTreeSet<_> = hardware
        .logical_buses
        .iter()
        .map(|b| b.name.as_str())
        .collect();
    ensure!(
        buses.len() == hardware.logical_buses.len(),
        "duplicate logical bus"
    );
    let mut addresses = BTreeSet::new();
    let mut names = BTreeSet::new();
    let mut coordinate_names = BTreeSet::new();
    for motor in &hardware.motors {
        ensure!(
            buses.contains(motor.bus.as_str()),
            "unknown motor bus {}",
            motor.bus
        );
        ensure!(
            addresses.insert((&motor.bus, motor.id))
                && names.insert(&motor.name)
                && coordinate_names.insert(motor.coordinate.as_str()),
            "duplicate motor identity"
        );
        ensure!(
            motor.id > 0 && motor.id <= robstride_protocol::MAX_MOTOR_ID,
            "invalid CAN id"
        );
        ensure!(
            MotorParams::for_model(&motor.model, motor.direction).is_some(),
            "invalid motor model/direction"
        );
    }
    let mut expected_coordinates = moving.clone();
    for transmission in description.transmissions() {
        for joint in &transmission.joints {
            expected_coordinates.remove(joint.as_str());
        }
        expected_coordinates.extend(transmission.coordinates.iter().map(String::as_str));
    }
    ensure!(
        coordinate_names == expected_coordinates,
        "motor coordinates must match the URDF transmissions exactly"
    );
    let mut spec = MjSpec::from_xml(&urdf)?;
    spec.compiler_mut().set_fusestatic(false);
    spec.option_mut().timestep = config.timestep_seconds;
    spec.option_mut().gravity = [0., 0., -9.81];
    spec.option_mut().integrator = MjtIntegrator::mjINT_IMPLICITFAST as i32;
    // Keep source link frames and inertias; only the world mounting becomes free.
    let root = spec
        .body_mut(description.root_link())
        .context("native URDF root body is missing")?;
    root.add_joint()
        .with_name("root_free")
        .with_type(MjtJoint::mjJNT_FREE);
    root.with_pos(config.initial_root_position)
        .with_quat(config.initial_root_quaternion);

    let xml = std::fs::read_to_string(&urdf)?;
    let document = roxmltree::Document::parse(&xml)?;
    let mut source_mass = 0.;
    for link in document
        .root_element()
        .children()
        .filter(|n| n.has_tag_name("link"))
    {
        let name = link.attribute("name").context("unnamed URDF link")?;
        if let Some(inertial) = link.children().find(|n| n.has_tag_name("inertial")) {
            source_mass += inertial
                .children()
                .find(|n| n.has_tag_name("mass"))
                .and_then(|n| n.attribute("value"))
                .context("missing URDF mass")?
                .parse::<f64>()?;
        } else {
            ensure!(
                description
                    .joints()
                    .any(|j| j.child == name && j.kind == JointKind::Fixed),
                "dynamic link {name} is missing explicit inertia"
            );
            let body = spec.body_mut(name).context("missing fixed URDF frame")?;
            body.set_mass(0.);
            for geom in body.geom_iter_mut(false) {
                geom.set_mass(0.);
                geom.set_density(0.);
            }
        }
    }
    let payload_mass = add_payloads(&mut spec, &config.payloads)?;
    while let Some(geom) = spec
        .geom_iter_mut()
        .find(|g| g.contype() == 0 && g.conaffinity() == 0)
    {
        // The iterator is dropped and this is the only live handle to the element.
        unsafe {
            geom.delete()?;
        }
    }
    ensure!(
        spec.geom_iter().all(|g| g.type_() != MjtGeom::mjGEOM_MESH),
        "robot collisions must use primitives"
    );
    while let Some(mesh) = spec.mesh_iter_mut().next() {
        // All mesh geoms were removed; there are no other live handles.
        unsafe {
            mesh.delete()?;
        }
    }
    add_neighbor_exclusions(&mut spec, &description, config.neighbor_exclusion_depth)?;
    spec.world_body_mut()
        .add_geom()
        .with_name("floor")
        .with_type(MjtGeom::mjGEOM_PLANE)
        .with_size([5., 5., 0.1])
        .with_friction([1.0, 0.005, 0.0001]);
    let mut offsets = BTreeMap::new();
    let mut covered = BTreeSet::new();
    for motor in &hardware.motors {
        let (target, kind, offset) =
            if let Some(t) = description.transmission_for_coordinate(&motor.coordinate) {
                let index = t
                    .coordinates
                    .iter()
                    .position(|c| c == &motor.coordinate)
                    .unwrap();
                let coefficients = [
                    t.coordinate_reduction[index] * t.joint_reduction[0],
                    (if index == 0 { 1. } else { -1. })
                        * t.coordinate_reduction[index]
                        * t.joint_reduction[1],
                ];
                let tendon = spec.add_tendon();
                tendon.set_name(&motor.coordinate)?;
                for (joint, coefficient) in t.joints.iter().zip(coefficients) {
                    tendon.wrap_joint(joint, coefficient);
                    covered.insert(joint.as_str());
                }
                (
                    motor.coordinate.as_str(),
                    MjtTrn::mjTRN_TENDON,
                    -coefficients
                        .iter()
                        .zip(t.joint_offset)
                        .map(|(c, q)| c * q)
                        .sum::<f64>(),
                )
            } else {
                ensure!(
                    moving.contains(motor.coordinate.as_str()),
                    "unknown motor coordinate {}",
                    motor.coordinate
                );
                ensure!(
                    covered.insert(motor.coordinate.as_str()),
                    "duplicate joint actuation"
                );
                (motor.coordinate.as_str(), MjtTrn::mjTRN_JOINT, 0.)
            };
        offsets.insert(motor.coordinate.clone(), offset);
        let params = MotorParams::for_model(&motor.model, motor.direction).unwrap();
        let actuator = spec.add_actuator();
        actuator.set_name(&motor.coordinate)?;
        actuator.set_to_motor();
        actuator.set_trntype(kind);
        actuator.set_target(target);
        actuator.with_forcerange([params.torque_min, params.torque_max]);
        actuator.set_forcelimited(MjtLimited::mjLIMITED_TRUE);
        actuator.set_ctrllimited(MjtLimited::mjLIMITED_FALSE);
    }
    ensure!(
        covered == moving,
        "motors do not cover every movable URDF joint"
    );
    for t in description.transmissions() {
        ensure!(
            t.coordinates
                .iter()
                .all(|c| coordinate_names.contains(c.as_str())),
            "incomplete differential transmission"
        );
    }
    let mut imu_names = BTreeSet::new();
    for imu in &hardware.imus {
        ensure!(imu_names.insert(&imu.name), "duplicate IMU name");
        description.fixed_transform(&imu.output_frame, &imu.source_frame)?;
        let site = format!("{}_site", imu.name);
        spec.body_mut(&imu.source_frame)
            .context("missing IMU source body")?
            .add_site()
            .with_name(&site)
            .with_size([0.001; 3]);
        for (suffix, kind) in [
            ("gyro", MjtSensor::mjSENS_GYRO),
            ("accelerometer", MjtSensor::mjSENS_ACCELEROMETER),
            ("orientation", MjtSensor::mjSENS_FRAMEQUAT),
        ] {
            let sensor = spec.add_sensor();
            sensor.set_name(&format!("{}_{suffix}", imu.name))?;
            sensor.set_type(kind);
            sensor.set_objtype(MjtObj::mjOBJ_SITE)?;
            sensor.set_objname(&site);
        }
    }
    let model = spec.compile()?;
    ensure!(
        (model.body_mass().iter().sum::<f64>() - source_mass - payload_mass).abs() < 1e-9,
        "native compilation changed URDF plus payload mass"
    );
    let mut qpos = model.qpos0().to_vec();
    for (name, position) in &config.initial_joint_positions {
        let id = model
            .name_to_id(MjtObj::mjOBJ_JOINT, name)
            .context("compiled joint missing")?;
        qpos[model.jnt_qposadr()[id] as usize] = *position;
    }
    if let Some(guide) = &config.torso_guide {
        crate::guide::add(&mut spec, guide)?;
        qpos.push(0.);
    }
    let key = spec.add_key();
    key.set_name("home")?;
    key.set_qpos(&qpos);
    let mut simulation = Simulation::from_model(spec.compile()?)?;
    simulation.reset_home()?;
    validate_home(&simulation)?;
    let guide = config
        .torso_guide
        .as_ref()
        .map(|c| crate::guide::Guide::bind(&mut simulation, c))
        .transpose()?;
    let coordinates = offsets
        .into_iter()
        .map(|(name, offset)| {
            let actuator = simulation.id(MjtObj::mjOBJ_ACTUATOR, &name)? as usize;
            Ok((name, Coordinate { actuator, offset }))
        })
        .collect::<Result<_>>()?;
    if let Some(path) = export {
        export_physics(&spec, &simulation, path)?;
    }
    Ok(RobotModel {
        simulation,
        hardware,
        coordinates,
        root_body: description.root_link().into(),
        guide,
    })
}

fn export_physics(spec: &MjSpec, simulation: &Simulation, path: &Path) -> Result<()> {
    spec.save_xml(path)?;
    // MuJoCo's writer rounds inertias; restore full precision using structured XML.
    let mut xml = xmltree::Element::parse(std::fs::File::open(path)?)?;
    xml.get_mut_child("compiler")
        .context("missing compiler")?
        .attributes
        .insert("inertiafromgeom".into(), "false".into());
    fn visit(element: &mut xmltree::Element, model: &MjModel) -> Result<()> {
        if element.name == "body" {
            let name = element.attributes.get("name").context("unnamed body")?;
            let id = model
                .name_to_id(MjtObj::mjOBJ_BODY, name)
                .context("exported body missing")?;
            if let Some(inertial) = element.get_mut_child("inertial") {
                inertial
                    .attributes
                    .insert("mass".into(), model.body_mass()[id].to_string());
                for (key, values) in [
                    ("pos", model.body_ipos()[id].as_slice()),
                    ("quat", model.body_iquat()[id].as_slice()),
                    ("diaginertia", model.body_inertia()[id].as_slice()),
                ] {
                    inertial.attributes.insert(
                        key.into(),
                        values
                            .iter()
                            .map(ToString::to_string)
                            .collect::<Vec<_>>()
                            .join(" "),
                    );
                }
            }
        }
        for child in &mut element.children {
            if let xmltree::XMLNode::Element(child) = child {
                visit(child, model)?;
            }
        }
        Ok(())
    }
    visit(&mut xml, simulation.data.model())?;
    if let Some(equalities) = xml.get_mut_child("equality") {
        for child in &mut equalities.children {
            let xmltree::XMLNode::Element(weld) = child else {
                continue;
            };
            if weld.name != "weld" {
                continue;
            }
            let name = weld.attributes.get("name").context("unnamed weld")?;
            let id = simulation.id(MjtObj::mjOBJ_EQUALITY, name)? as usize;
            let values = simulation.data.model().eq_data()[id];
            weld.attributes.insert(
                "relpose".into(),
                values[3..10]
                    .iter()
                    .map(ToString::to_string)
                    .collect::<Vec<_>>()
                    .join(" "),
            );
            weld.attributes
                .insert("active".into(), simulation.data.eq_active()[id].to_string());
        }
    }
    xml.write_with_config(
        std::fs::File::create(path)?,
        xmltree::EmitterConfig::new().perform_indent(true),
    )?;
    let reloaded = MjModel::from_xml(path)?;
    ensure!(
        (reloaded.body_mass().iter().sum::<f64>()
            - simulation.data.model().body_mass().iter().sum::<f64>())
        .abs()
            < 1e-12,
        "exported model changed physical mass"
    );
    Ok(())
}

fn add_neighbor_exclusions(
    spec: &mut MjSpec,
    description: &RobotDescription,
    depth: usize,
) -> Result<()> {
    let mut graph: BTreeMap<&str, BTreeSet<&str>> = description
        .links()
        .map(|l| (l.name.as_str(), BTreeSet::new()))
        .collect();
    for joint in description.joints() {
        graph
            .get_mut(joint.parent.as_str())
            .unwrap()
            .insert(&joint.child);
        graph
            .get_mut(joint.child.as_str())
            .unwrap()
            .insert(&joint.parent);
    }
    for source in graph.keys() {
        let mut visited = BTreeSet::from([*source]);
        let mut frontier = visited.clone();
        for _ in 0..depth {
            let next: BTreeSet<_> = frontier
                .iter()
                .flat_map(|n| graph[n].iter().copied())
                .filter(|n| !visited.contains(n))
                .collect();
            for target in &next {
                if source < target {
                    let exclude = spec.add_exclude();
                    exclude.set_bodyname1(source);
                    exclude.set_bodyname2(target);
                }
            }
            visited.extend(&next);
            frontier = next;
        }
    }
    Ok(())
}

fn validate_home(simulation: &Simulation) -> Result<()> {
    let model = simulation.data.model();
    for contact in simulation.data.contact() {
        if contact.dist < -1e-6
            && contact
                .geom
                .iter()
                .all(|&g| g >= 0 && model.geom_bodyid()[g as usize] != 0)
        {
            anyhow::bail!(
                "initial pose self-collides: {:?} / {:?} ({:.6} m)",
                model.id_to_name(MjtObj::mjOBJ_GEOM, contact.geom[0] as usize),
                model.id_to_name(MjtObj::mjOBJ_GEOM, contact.geom[1] as usize),
                -contact.dist
            );
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixture() -> (tempfile::TempDir, ModelConfig) {
        let dir = tempfile::tempdir().unwrap();
        let mut xml = String::from("<robot name='fixture'>");
        for name in ["base", "first", "second"] {
            xml.push_str(&format!("<link name='{name}'><inertial><mass value='1.123456789123'/><inertia ixx='0.0123456789123' iyy='0.02' izz='0.025' ixy='0' ixz='0' iyz='0'/></inertial><collision><geometry><sphere radius='0.02'/></geometry></collision></link>"));
        }
        xml.push_str(r#"
          <link name="imu"/>
          <joint name="imu_fixed" type="fixed"><parent link="base"/><child link="imu"/><origin xyz="0.1 0 0.2" rpy="0 0 1.5707963267948966"/></joint>
          <joint name="roll" type="revolute"><parent link="base"/><child link="first"/><origin xyz="0 0 -0.1"/><axis xyz="1 0 0"/><limit lower="-2" upper="2" effort="100" velocity="10"/></joint>
          <joint name="pitch" type="revolute"><parent link="first"/><child link="second"/><origin xyz="0 0 -0.1"/><axis xyz="0 1 0"/><limit lower="-2" upper="2" effort="100" velocity="10"/></joint>
          <transmission name="ankle"><type>transmission_interface/DifferentialTransmission</type>
            <actuator name="b" role="actuator2"><mechanicalReduction>-4</mechanicalReduction></actuator>
            <actuator name="a" role="actuator1"><mechanicalReduction>2</mechanicalReduction></actuator>
            <joint name="pitch" role="joint2"><mechanicalReduction>5</mechanicalReduction><offset>-0.02</offset></joint>
            <joint name="roll" role="joint1"><mechanicalReduction>-3</mechanicalReduction><offset>0.01</offset></joint>
          </transmission></robot>"#);
        std::fs::write(dir.path().join("robot.urdf"), xml).unwrap();
        let hardware = dir.path().join("hardware.yaml");
        std::fs::write(
            &hardware,
            r#"
robot_description: robot.urdf
logical_buses: [{name: lower, host_id: 253}]
motors:
  - {name: second_motor, coordinate: b, bus: lower, id: 2, model: ROBSTRIDE_03, direction: -1}
  - {name: first_motor, coordinate: a, bus: lower, id: 1, model: ROBSTRIDE_03, direction: 1}
imus: [{name: imu, source_frame: imu, output_frame: base}]
"#,
        )
        .unwrap();
        (
            dir,
            ModelConfig {
                hardware,
                timestep_seconds: 0.001,
                initial_root_position: [0., 0., 1.],
                initial_root_quaternion: [1., 0., 0., 0.],
                initial_joint_positions: BTreeMap::from([
                    ("roll".into(), 0.2),
                    ("pitch".into(), -0.1),
                ]),
                neighbor_exclusion_depth: 2,
                payloads: Vec::new(),
                torso_guide: None,
            },
        )
    }

    #[test]
    fn payload_adds_mass_at_its_frame_without_replacing_robot_inertia() {
        let (dir, mut config) = fixture();
        config.payloads.push(Payload {
            frame: "imu".into(),
            mass_kg: 0.7,
            radius_m: 0.04,
        });
        let xml = dir.path().join("payload.xml");
        let robot = build(&config, Some(&xml)).unwrap();
        let model = robot.simulation.data.model();
        let frame = model.name_to_id(MjtObj::mjOBJ_BODY, "imu").unwrap();
        assert!((model.body_mass()[frame] - 0.7).abs() < 1e-12);
        assert_eq!(model.body_ipos()[frame], [0.; 3]);
        for value in model.body_inertia()[frame] {
            assert!((value - 0.4 * 0.7 * 0.04_f64.powi(2)).abs() < 1e-12);
        }
        let reloaded = MjModel::from_xml(&xml).unwrap();
        assert_eq!(reloaded.body_mass()[frame], model.body_mass()[frame]);
        config.payloads[0].frame = "base".into();
        assert!(
            build(&config, None)
                .err()
                .unwrap()
                .to_string()
                .contains("inertial-free")
        );
        config.payloads[0].frame = "missing".into();
        assert!(build(&config, None).is_err());
        config.payloads[0].mass_kg = -1.;
        assert!(build(&config, None).is_err());
    }

    #[test]
    fn native_differential_preserves_offsets_virtual_work_and_sensor_frames() {
        let (_dir, config) = fixture();
        let mut robot = build(&config, None).unwrap();
        let roll = robot.simulation.id(MjtObj::mjOBJ_JOINT, "roll").unwrap() as usize;
        let pitch = robot.simulation.id(MjtObj::mjOBJ_JOINT, "pitch").unwrap() as usize;
        let r = robot.simulation.data.model().jnt_dofadr()[roll] as usize;
        let p = robot.simulation.data.model().jnt_dofadr()[pitch] as usize;
        robot.simulation.data.qvel_mut()[r] = 0.3;
        robot.simulation.data.qvel_mut()[p] = -0.2;
        for (name, coefficients, torque) in [("a", [-6., 10.], 2.), ("b", [12., 20.], -1.)] {
            let c = &robot.coordinates[name];
            robot
                .simulation
                .set_impedance(
                    c.actuator,
                    robstride_protocol::ControlCommand {
                        torque,
                        ..Default::default()
                    },
                    60.,
                )
                .unwrap();
            robot.simulation.forward();
            let (q, v, _) = robot.simulation.actuator_state(c.actuator).unwrap();
            let expected = coefficients[0] * (0.2 - 0.01) + coefficients[1] * (-0.1 + 0.02);
            assert!((q + c.offset - expected).abs() < 1e-12);
            assert!((v - (coefficients[0] * 0.3 - coefficients[1] * 0.2)).abs() < 1e-12);
        }
        robot.simulation.forward();
        let force = robot.simulation.data.qfrc_actuator();
        assert!((force[r] - (-6. * 2. - 12.)).abs() < 1e-12);
        assert!((force[p] - (10. * 2. - 20.)).abs() < 1e-12);
        let orientation = robot.simulation.sensor::<4>("imu_orientation").unwrap();
        assert!((orientation[0] - std::f64::consts::FRAC_1_SQRT_2).abs() < 1e-12);
        assert!((orientation[3] - std::f64::consts::FRAC_1_SQRT_2).abs() < 1e-12);
    }

    #[test]
    fn exported_inertias_keep_source_precision() {
        let (dir, config) = fixture();
        let path = dir.path().join("physics.xml");
        let robot = build(&config, Some(&path)).unwrap();
        let reloaded = MjModel::from_xml(&path).unwrap();
        for name in ["base", "first", "second"] {
            let a = robot.simulation.data.model();
            let i = a.name_to_id(MjtObj::mjOBJ_BODY, name).unwrap();
            let j = reloaded.name_to_id(MjtObj::mjOBJ_BODY, name).unwrap();
            assert_eq!(a.body_mass()[i], reloaded.body_mass()[j]);
            for (x, y) in a.body_inertia()[i].iter().zip(reloaded.body_inertia()[j]) {
                assert!((x - y).abs() < 1e-14);
            }
        }
    }

    #[test]
    fn torso_guide_limits_drop_releases_and_reinitializes() {
        let (dir, mut config) = fixture();
        config.torso_guide = Some(crate::guide::Config {
            body: "base".into(),
            drop_limit_m: 0.05,
        });
        let export = dir.path().join("guided.xml");
        let mut robot = build(&config, Some(&export)).unwrap();
        let reloaded = MjModel::from_xml(&export).unwrap();
        assert!(reloaded.eq_active0()[0]);
        let body = robot.simulation.id(MjtObj::mjOBJ_BODY, "base").unwrap() as usize;
        for _ in 0..1000 {
            robot.simulation.step();
        }
        robot.simulation.forward();
        let pose = robot.simulation.body_state(body).unwrap();
        assert!(pose.position[0].abs() < 0.001 && pose.position[1].abs() < 0.001);
        assert!(pose.position[2] > 0.94 && pose.position[2] < 0.96);
        let guide = robot.guide.as_mut().unwrap();
        guide.release(&mut robot.simulation);
        for _ in 0..100 {
            robot.simulation.step();
        }
        robot.simulation.forward();
        assert!(robot.simulation.body_state(body).unwrap().position[2] < pose.position[2] - 0.03);
        robot.simulation.reset_home().unwrap();
        guide.initialize(&mut robot.simulation).unwrap();
        assert!((robot.simulation.body_state(body).unwrap().position[2] - 1.).abs() < 1e-12);
    }

    #[test]
    fn invalid_initial_pose_and_incomplete_ownership_fail_before_runtime() {
        let (_dir, mut config) = fixture();
        config.initial_joint_positions.insert("roll".into(), 3.);
        assert!(
            build(&config, None)
                .err()
                .unwrap()
                .to_string()
                .contains("outside URDF limits")
        );
        config.initial_joint_positions.remove("roll");
        assert!(
            build(&config, None)
                .err()
                .unwrap()
                .to_string()
                .contains("cover every movable")
        );
    }
}
