//! Minimal ownership wrappers over the installed MuJoCo C API.
//! Bindings are generated from that installation's headers, not a second physics version.

use std::ffi::{CStr, CString};
use std::marker::PhantomData;
use std::path::Path;
use std::ptr::NonNull;
use std::rc::Rc;

use anyhow::{Context, Result, ensure};
use robstride_protocol::ControlCommand;

pub mod robstride;

#[allow(
    non_camel_case_types,
    non_snake_case,
    non_upper_case_globals,
    dead_code,
    clippy::approx_constant
)]
pub mod ffi {
    include!(concat!(env!("OUT_DIR"), "/bindings.rs"));
}

pub struct Simulation {
    model: NonNull<ffi::mjModel>,
    data: NonNull<ffi::mjData>,
    // Model/data access stays on one simulation thread.
    _thread: PhantomData<Rc<()>>,
}

impl Simulation {
    pub fn load(path: &Path) -> Result<Self> {
        ensure!(
            unsafe { ffi::mj_version() } == ffi::mjVERSION_HEADER as i32,
            "loaded MuJoCo library does not match the build headers"
        );
        let filename = CString::new(path.to_str().context("model path is not UTF-8")?)?;
        let mut error = [0_i8; 2048];
        let model = unsafe {
            ffi::mj_loadXML(
                filename.as_ptr(),
                std::ptr::null(),
                error.as_mut_ptr(),
                error.len() as i32,
            )
        };
        let model = NonNull::new(model).with_context(|| {
            format!(
                "MuJoCo load: {}",
                unsafe { CStr::from_ptr(error.as_ptr()) }.to_string_lossy()
            )
        })?;
        let Some(data) = NonNull::new(unsafe { ffi::mj_makeData(model.as_ptr()) }) else {
            unsafe {
                ffi::mj_deleteModel(model.as_ptr());
            }
            anyhow::bail!("MuJoCo data allocation failed");
        };
        Ok(Self {
            model,
            data,
            _thread: PhantomData,
        })
    }

    pub fn model(&self) -> &ffi::mjModel {
        unsafe { self.model.as_ref() }
    }
    pub fn data(&self) -> &ffi::mjData {
        unsafe { self.data.as_ref() }
    }
    pub fn step(&mut self) {
        unsafe {
            ffi::mj_step(self.model.as_ptr(), self.data.as_ptr());
        }
    }
    pub fn forward(&mut self) {
        unsafe {
            ffi::mj_forward(self.model.as_ptr(), self.data.as_ptr());
        }
    }

    pub fn reset_home(&mut self) -> Result<()> {
        let id = self.id(ffi::mjtObj__mjOBJ_KEY as i32, "home")?;
        unsafe {
            ffi::mj_resetDataKeyframe(self.model.as_ptr(), self.data.as_ptr(), id);
        }
        self.forward();
        Ok(())
    }

    pub fn id(&self, kind: i32, name: &str) -> Result<i32> {
        let name = CString::new(name)?;
        let id = unsafe { ffi::mj_name2id(self.model.as_ptr(), kind, name.as_ptr()) };
        ensure!(id >= 0, "model object is missing: {:?}", name);
        Ok(id)
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
            let model = self.model.as_mut();
            // Native affine servos retain MuJoCo's implicit damping treatment.
            *model.actuator_gaintype.add(id) = ffi::mjtGain__mjGAIN_FIXED as i32;
            *model.actuator_biastype.add(id) = ffi::mjtBias__mjBIAS_AFFINE as i32;
            *model.actuator_gainprm.add(id * ffi::mjNGAIN as usize) = kp;
            *model.actuator_biasprm.add(id * ffi::mjNBIAS as usize) = torque + kd * velocity;
            *model.actuator_biasprm.add(id * ffi::mjNBIAS as usize + 1) = -kp;
            *model.actuator_biasprm.add(id * ffi::mjNBIAS as usize + 2) = -kd;
            *model.actuator_ctrllimited.add(id) = false;
            *model.actuator_forcelimited.add(id) = true;
            *model.actuator_forcerange.add(id * 2) = -torque_limit;
            *model.actuator_forcerange.add(id * 2 + 1) = torque_limit;
            *self.data.as_mut().ctrl.add(id) = position;
        }
        Ok(())
    }
}

impl Drop for Simulation {
    fn drop(&mut self) {
        unsafe {
            ffi::mj_deleteData(self.data.as_ptr());
            ffi::mj_deleteModel(self.model.as_ptr());
        }
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
    fn native_tendons_report_motor_coordinates() {
        let simulation = simulation();
        assert!((simulation.actuator_state(0).unwrap().0 - 0.3).abs() < 1e-12);
        assert!((simulation.actuator_state(1).unwrap().0 - 0.1).abs() < 1e-12);
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
