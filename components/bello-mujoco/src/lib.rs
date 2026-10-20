//! Headless MuJoCo 3.12.0 physics through mujoco-rs.

use std::ffi::CString;
use std::marker::PhantomData;
use std::path::Path;
use std::rc::Rc;

use anyhow::{Context, Result, ensure};
use mujoco_rs::prelude::{MjData, MjModel, MjtObj};
use robstride_protocol::ControlCommand;

pub mod ebimu;
pub mod guide;
pub mod model;
mod model_physics;
pub mod pose;
pub mod power_board;
pub mod robstride;
pub mod runtime;
mod transport;
pub mod video;

pub use mujoco_rs::mujoco_c as ffi;

pub struct Simulation {
    data: MjData<Box<MjModel>>,
    // Model/data access stays on one simulation thread.
    _thread: PhantomData<Rc<()>>,
}

#[derive(Debug)]
pub struct BodyState {
    pub position: [f64; 3],
    pub orientation_wxyz: [f64; 4],
    pub linear_velocity_world: [f64; 3],
    pub angular_velocity_local: [f64; 3],
}

impl Simulation {
    pub fn load(path: &Path) -> Result<Self> {
        Self::check_version()?;
        CString::new(path.to_str().context("model path is not UTF-8")?)?;
        Self::from_model(MjModel::from_xml(path)?)
    }

    /// Compile self-contained model XML without Python or file extraction.
    pub fn from_xml(xml: &[u8]) -> Result<Self> {
        Self::check_version()?;
        Self::from_model(MjModel::from_xml_string(std::str::from_utf8(xml)?)?)
    }

    fn check_version() -> Result<()> {
        ensure!(
            unsafe { ffi::mj_version() } == ffi::mjVERSION_HEADER as i32,
            "loaded MuJoCo library must match the pinned 3.12.0 bindings"
        );
        Ok(())
    }

    fn from_model(model: MjModel) -> Result<Self> {
        Ok(Self {
            data: MjData::try_new(Box::new(model))?,
            _thread: PhantomData,
        })
    }

    pub fn model(&self) -> &ffi::mjModel {
        self.data.model().ffi()
    }
    pub fn data(&self) -> &ffi::mjData {
        self.data.ffi()
    }
    pub fn step(&mut self) {
        self.data.step();
    }
    pub fn forward(&mut self) {
        self.data.forward();
    }

    pub fn reset_home(&mut self) -> Result<()> {
        let id = self.id(MjtObj::mjOBJ_KEY, "home")?;
        self.data.reset_keyframe(id as usize)?;
        self.forward();
        Ok(())
    }

    pub fn id(&self, kind: MjtObj, name: &str) -> Result<i32> {
        CString::new(name)?;
        self.data
            .model()
            .name_to_id(kind, name)
            .map(|id| id as i32)
            .with_context(|| format!("model object is missing: {name}"))
    }

    pub fn actuator_state(&self, id: usize) -> Result<(f64, f64, f64)> {
        ensure!(id < self.model().nu as usize, "actuator index out of range");
        let data = self.data();
        Ok(unsafe {
            (
                *data.actuator_length.add(id),
                *data.actuator_velocity.add(id),
                *data.actuator_force.add(id),
            )
        })
    }

    pub fn body_state(&self, id: usize) -> Result<BodyState> {
        ensure!(id < self.model().nbody as usize, "body index out of range");
        // XBODY matches xpos/xquat, not the offset/rotated inertial frame.
        let world = self.data.object_velocity(MjtObj::mjOBJ_XBODY, id, false);
        let local = self.data.object_velocity(MjtObj::mjOBJ_XBODY, id, true);
        unsafe {
            Ok(BodyState {
                position: std::array::from_fn(|axis| *self.data().xpos.add(3 * id + axis)),
                orientation_wxyz: std::array::from_fn(|axis| *self.data().xquat.add(4 * id + axis)),
                linear_velocity_world: [world[3], world[4], world[5]],
                angular_velocity_local: [local[0], local[1], local[2]],
            })
        }
    }

    pub fn sensor<const N: usize>(&self, name: &str) -> Result<[f64; N]> {
        let id = self.id(MjtObj::mjOBJ_SENSOR, name)? as usize;
        unsafe {
            ensure!(
                *self.model().sensor_dim.add(id) as usize == N,
                "sensor {name} dimension mismatch"
            );
            let address = *self.model().sensor_adr.add(id) as usize;
            let values = std::array::from_fn(|axis| *self.data().sensordata.add(address + axis));
            ensure!(
                values.iter().all(|value| value.is_finite()),
                "nonfinite sensor {name}"
            );
            Ok(values)
        }
    }

    pub fn set_impedance(
        &mut self,
        id: usize,
        command: ControlCommand,
        torque_limit: f64,
    ) -> Result<()> {
        let ControlCommand {
            position,
            velocity,
            kp,
            kd,
            torque,
            ..
        } = command;
        ensure!(id < self.model().nu as usize, "actuator index out of range");
        ensure!(
            [position, velocity, kp, kd, torque, torque_limit]
                .iter()
                .all(|x| x.is_finite())
                && kp >= 0.
                && kd >= 0.
                && torque_limit > 0.,
            "invalid motor command"
        );
        unsafe {
            // Only existing servo parameters change; model topology and sizes stay fixed.
            let model = self.data.model_mut().ffi_mut();
            // Native affine servos retain MuJoCo's implicit damping treatment.
            *model.actuator_gaintype.add(id) = ffi::mjtGain::mjGAIN_FIXED as i32;
            *model.actuator_biastype.add(id) = ffi::mjtBias::mjBIAS_AFFINE as i32;
            *model.actuator_gainprm.add(id * ffi::mjNGAIN as usize) = kp;
            *model.actuator_biasprm.add(id * ffi::mjNBIAS as usize) = torque + kd * velocity;
            *model.actuator_biasprm.add(id * ffi::mjNBIAS as usize + 1) = -kp;
            *model.actuator_biasprm.add(id * ffi::mjNBIAS as usize + 2) = -kd;
            *model.actuator_ctrllimited.add(id) = false;
            *model.actuator_forcelimited.add(id) = true;
            *model.actuator_forcerange.add(id * 2) = -torque_limit;
            *model.actuator_forcerange.add(id * 2 + 1) = torque_limit;
            self.data.ctrl_mut()[id] = position;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;

    pub(crate) fn simulation() -> Simulation {
        let mut file = tempfile::NamedTempFile::new().unwrap();
        file.write_all(br#"<mujoco>
          <option gravity="0 0 0" integrator="implicitfast"/>
          <worldbody><body><joint name="pitch" axis="0 1 0"/><geom size=".1"/>
            <body pos="0 0 .2"><joint name="roll" axis="1 0 0"/><geom size=".1"/></body>
          </body></worldbody>
          <tendon><fixed name="motor_a"><joint joint="pitch" coef="1"/><joint joint="roll" coef="1"/></fixed>
            <fixed name="motor_b"><joint joint="pitch" coef="1"/><joint joint="roll" coef="-1"/></fixed></tendon>
          <actuator><position name="a" tendon="motor_a" kp="1"/><position name="b" tendon="motor_b" kp="1"/></actuator>
          <keyframe><key name="home" qpos=".2 .1" ctrl=".3 .1"/></keyframe>
        </mujoco>"#).unwrap();
        let mut simulation = Simulation::load(file.path()).unwrap();
        simulation.reset_home().unwrap();
        simulation
    }

    #[test]
    fn affine_servo_matches_split_mit_actuators_through_saturation() {
        let mut affine = Simulation::from_xml(
            br#"<mujoco>
          <option gravity="0 0 0" timestep=".0025" integrator="implicitfast"/>
          <worldbody><body><joint name="q" axis="0 1 0"/><geom size=".1"/></body></worldbody>
          <actuator><general joint="q"/></actuator>
        </mujoco>"#,
        )
        .unwrap();
        let mut split = Simulation::from_xml(
            br#"<mujoco>
          <option gravity="0 0 0" timestep=".0025" integrator="implicitfast"/>
          <worldbody><body><joint name="q" axis="0 1 0" actuatorfrcrange="-2 2"/>
          <geom size=".1"/></body></worldbody>
          <actuator><position joint="q" kp="12"/><velocity joint="q" kv=".4"/>
          <motor joint="q"/></actuator>
        </mujoco>"#,
        )
        .unwrap();
        for step in 0..1000 {
            let phase = step as f64 * 0.013;
            let position = phase.sin();
            let velocity = phase.cos() * 0.2;
            let torque = (phase * 0.7).sin() * 3.;
            affine
                .set_impedance(
                    0,
                    ControlCommand {
                        position,
                        velocity,
                        torque,
                        kp: 12.,
                        kd: 0.4,
                        ..Default::default()
                    },
                    2.,
                )
                .unwrap();
            split
                .data
                .ctrl_mut()
                .copy_from_slice(&[position, velocity, torque]);
            affine.step();
            split.step();
            assert!((affine.data.qpos()[0] - split.data.qpos()[0]).abs() < 1e-10);
            assert!((affine.data.qvel()[0] - split.data.qvel()[0]).abs() < 1e-10);
        }
    }

    #[test]
    fn native_tendons_report_motor_coordinates() {
        let simulation = simulation();
        assert!((simulation.actuator_state(0).unwrap().0 - 0.3).abs() < 1e-12);
        assert!((simulation.actuator_state(1).unwrap().0 - 0.1).abs() < 1e-12);
    }

    #[test]
    fn tendon_velocity_and_force_follow_the_same_transmission() {
        let mut simulation = simulation();
        simulation.data.qvel_mut().copy_from_slice(&[0.4, -0.2]);
        for (id, torque) in [(0, 2.), (1, 1.)] {
            simulation
                .set_impedance(
                    id,
                    ControlCommand {
                        torque,
                        ..Default::default()
                    },
                    3.,
                )
                .unwrap();
        }
        simulation.forward();
        assert!((simulation.actuator_state(0).unwrap().1 - 0.2).abs() < 1e-12);
        assert!((simulation.actuator_state(1).unwrap().1 - 0.6).abs() < 1e-12);
        // q_motor = J q_joint, hence joint torque = J^T motor torque.
        assert_eq!(simulation.data.qfrc_actuator(), &[3., 1.]);
    }

    #[test]
    fn native_urdf_can_be_compiled_and_exported_as_mjcf() {
        use mujoco_rs::prelude::MjSpec;
        let directory = tempfile::tempdir().unwrap();
        let mut urdf = tempfile::NamedTempFile::new_in(&directory).unwrap();
        urdf.write_all(
            br#"<robot name="synthetic">
          <link name="base"/>
          <link name="pendulum">
            <inertial><origin xyz="0 0 -.2"/><mass value="1"/>
              <inertia ixx=".1" iyy=".1" izz=".1" ixy="0" ixz="0" iyz="0"/>
            </inertial>
          </link>
          <joint name="hinge" type="revolute">
            <parent link="base"/><child link="pendulum"/><axis xyz="0 1 0"/>
            <limit lower="-1" upper="1" effort="3" velocity="2"/>
          </joint>
        </robot>"#,
        )
        .unwrap();
        let mut spec = MjSpec::from_xml(urdf.path()).unwrap();
        let model = spec.compile().unwrap();
        let output = directory.path().join("model.xml");
        spec.save_xml(&output).unwrap();
        let from_urdf = Simulation::load(urdf.path()).unwrap();
        let mut from_mjcf = Simulation::load(&output).unwrap();
        assert_eq!(model.nv(), 1);
        assert_eq!(from_urdf.model().nv, from_mjcf.model().nv);
        assert_eq!(from_urdf.model().nu, 0);
        from_mjcf.step();
        assert!(from_mjcf.data().time > 0.);
    }

    #[test]
    fn xml_bytes_load_without_python_or_extraction() {
        let mut simulation = Simulation::from_xml(
            br#"<mujoco>
          <worldbody><body name="body"><joint/>
            <geom type="box" size=".1 .2 .3"/>
          </body></worldbody>
          <keyframe><key name="home" qpos=".25"/></keyframe>
        </mujoco>"#,
        )
        .unwrap();
        simulation.reset_home().unwrap();
        assert_eq!(unsafe { *simulation.data().qpos }, 0.25);
        simulation.step();
        assert!(simulation.data().time > 0.);
        assert!(Simulation::from_xml(b"broken xml").is_err());
        assert!(
            Simulation::from_xml(b"<mujoco><worldbody><body><joint/></body></worldbody></mujoco>")
                .is_err()
        );
    }

    #[test]
    fn link_pose_velocity_and_sensor_frames_ignore_inertial_offset() {
        let mut file = tempfile::NamedTempFile::new().unwrap();
        file.write_all(br#"<mujoco>
          <compiler alignfree="false"/>
          <worldbody><body name="link" pos="0 0 1"><freejoint/>
            <inertial pos=".4 0 0" quat=".7071067811865476 .7071067811865476 0 0"
              mass="1" diaginertia=".1 .2 .3"/>
            <site name="imu"/>
          </body></worldbody>
          <sensor><gyro name="gyro" site="imu"/><framequat name="orientation" objtype="site" objname="imu"/></sensor>
          <keyframe><key name="home" qvel="1 2 3 0 0 2"/></keyframe>
        </mujoco>"#).unwrap();
        let mut sim = Simulation::load(file.path()).unwrap();
        sim.reset_home().unwrap();
        let state = sim
            .body_state(sim.id(MjtObj::mjOBJ_BODY, "link").unwrap() as usize)
            .unwrap();
        for (actual, expected) in [
            (state.position, [0., 0., 1.]),
            (state.linear_velocity_world, [1., 2., 3.]),
            (state.angular_velocity_local, [0., 0., 2.]),
        ] {
            for (a, b) in actual.into_iter().zip(expected) {
                assert!((a - b).abs() < 1e-12);
            }
        }
        assert_eq!(sim.sensor::<3>("gyro").unwrap(), [0., 0., 2.]);
        assert_eq!(
            sim.sensor::<4>("orientation").unwrap(),
            state.orientation_wxyz
        );
        assert!(sim.sensor::<4>("gyro").is_err());
        assert!(sim.sensor::<3>("absent").is_err());
    }

    #[test]
    fn impedance_retains_native_force_saturation() {
        let mut simulation = simulation();
        simulation
            .set_impedance(
                0,
                ControlCommand {
                    position: 10.,
                    kp: 100.,
                    kd: 2.,
                    ..Default::default()
                },
                3.,
            )
            .unwrap();
        simulation.forward();
        assert!((simulation.actuator_state(0).unwrap().2 - 3.).abs() < 1e-12);
        for _ in 0..20 {
            simulation.step();
        }
        assert!(simulation.actuator_state(0).unwrap().2.abs() <= 3.);
        assert!(
            simulation
                .set_impedance(
                    0,
                    ControlCommand {
                        position: f64::NAN,
                        kp: 1.,
                        kd: 1.,
                        ..Default::default()
                    },
                    3.
                )
                .is_err()
        );
    }
}
