//! Native physics behind the existing RobStride protocol engine.

use std::collections::{BTreeMap, BTreeSet};
use std::time::Duration;

use anyhow::{Context, Result, ensure};
use robstride_emulator::config::Config;
use robstride_emulator::emulator::{Emulator, MotorState, ScheduledFrame};
use robstride_protocol::{Frame, MotorParams};

use crate::{Simulation, ffi};

struct Binding {
    motor: u8,
    actuator: usize,
    torque_limit: f64,
}

struct Bus {
    protocol: Emulator,
    bindings: Vec<Binding>,
}

pub struct RobstrideSimulation {
    pub simulation: Simulation,
    buses: BTreeMap<String, Bus>,
}

impl RobstrideSimulation {
    /// Coordinates map CAN IDs to already-compiled MuJoCo actuator names.
    /// The caller derives that mapping from the canonical hardware description.
    pub fn new(
        simulation: Simulation,
        configs: Vec<(Config, BTreeMap<u8, String>)>,
    ) -> Result<Self> {
        let mut buses = BTreeMap::new();
        let mut assigned = BTreeSet::new();
        for (config, coordinates) in configs {
            config.validate()?;
            ensure!(
                coordinates.len() == config.actuators.len(),
                "incomplete bus coordinate mapping"
            );
            let mut bindings = Vec::new();
            for motor in &config.actuators {
                let name = coordinates
                    .get(&motor.id)
                    .context("motor coordinate is missing")?;
                let actuator = simulation.id(ffi::mjtObj::mjOBJ_ACTUATOR, name)? as usize;
                ensure!(
                    assigned.insert(actuator),
                    "MuJoCo actuator {name} is mapped more than once"
                );
                let params = MotorParams::for_model(&motor.model, motor.direction)
                    .context("unsupported motor")?;
                ensure!(
                    -params.torque_min == params.torque_max,
                    "native servo requires symmetric motor torque limits"
                );
                bindings.push(Binding {
                    motor: motor.id,
                    actuator,
                    torque_limit: params.torque_max,
                });
            }
            let name = config.interface.clone();
            ensure!(
                buses
                    .insert(
                        name,
                        Bus {
                            protocol: Emulator::new(config)?,
                            bindings
                        }
                    )
                    .is_none(),
                "duplicate CAN interface"
            );
        }
        ensure!(
            assigned.len() == simulation.model().nu as usize,
            "every MuJoCo actuator must have exactly one motor binding"
        );
        Ok(Self { simulation, buses })
    }

    pub fn handle(&mut self, bus: &str, frame: Frame) -> Result<()> {
        self.buses
            .get_mut(bus)
            .context("unknown CAN interface")?
            .protocol
            .handle(frame);
        Ok(())
    }

    pub fn step(&mut self) -> Result<Vec<(String, ScheduledFrame)>> {
        for bus in self.buses.values() {
            for binding in &bus.bindings {
                let command = bus.protocol.command(binding.motor)?.unwrap_or_default();
                self.simulation
                    .set_impedance(binding.actuator, command, binding.torque_limit)?;
            }
        }
        self.simulation.step();
        self.simulation.forward();
        let elapsed = Duration::from_secs_f64(self.simulation.model().opt.timestep);
        let mut feedback = Vec::new();
        for (name, bus) in &mut self.buses {
            for binding in &bus.bindings {
                let (position, velocity, torque) =
                    self.simulation.actuator_state(binding.actuator)?;
                bus.protocol.set_motor_state(
                    binding.motor,
                    MotorState {
                        position,
                        velocity,
                        torque,
                    },
                )?;
            }
            feedback.extend(
                bus.protocol
                    .advance_feedback(elapsed)
                    .into_iter()
                    .map(|frame| (name.clone(), frame)),
            );
        }
        Ok(feedback)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use robstride_emulator::config::{ActuatorConfig, DynamicsConfig};
    use robstride_protocol::{
        ControlCommand, active_report_frame, control_frame, enable_frame, param_write_u8_frame,
        parse_feedback, stop_frame,
    };

    fn config() -> Config {
        Config {
            interface: "vcan-test".into(),
            host_id: 0xfd,
            feedback_rate_hz: 1000.,
            dynamics: DynamicsConfig::default(),
            events: vec![],
            actuators: (1..=2)
                .map(|id| ActuatorConfig {
                    id,
                    model: "ROBSTRIDE_03".into(),
                    direction: if id == 1 { -1 } else { 1 },
                    initial_position: 0.,
                    initial_temperature_c: 25.,
                })
                .collect(),
        }
    }

    #[test]
    fn native_physics_drives_wire_feedback_and_stop_removes_torque() {
        let mapping = BTreeMap::from([(1, "a".into()), (2, "b".into())]);
        let mut runtime =
            RobstrideSimulation::new(crate::tests::simulation(), vec![(config(), mapping)])
                .unwrap();
        let params = MotorParams::for_model("ROBSTRIDE_03", -1).unwrap();
        for frame in [
            active_report_frame(0xfd, 1, true),
            param_write_u8_frame(0xfd, 1, 0x7005, 0),
            enable_frame(0xfd, 1),
            control_frame(
                params,
                ControlCommand {
                    motor_id: 1,
                    position: 10.,
                    kp: 100.,
                    ..Default::default()
                },
            ),
        ] {
            runtime.handle("vcan-test", frame).unwrap();
        }
        let frames = runtime.step().unwrap();
        let wire = parse_feedback(&frames[0].1.frame, params).unwrap();
        let native = runtime.simulation.actuator_state(0).unwrap();
        assert!((wire.position - native.0).abs() < 0.001);
        assert!((wire.torque - native.2).abs() < 0.01);
        assert_eq!(wire.mode_status, 2);
        assert!(native.2.abs() <= params.torque_max);
        runtime
            .handle("vcan-test", stop_frame(0xfd, 1, false))
            .unwrap();
        runtime.step().unwrap();
        assert_eq!(runtime.simulation.actuator_state(0).unwrap().2, 0.);
    }

    #[test]
    fn duplicate_actuator_ownership_fails_before_stepping() {
        let mapping = BTreeMap::from([(1, "a".into()), (2, "a".into())]);
        assert!(
            RobstrideSimulation::new(crate::tests::simulation(), vec![(config(), mapping)])
                .is_err()
        );
    }
}
