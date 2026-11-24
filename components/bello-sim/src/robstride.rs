use std::collections::{BTreeMap, BTreeSet};
use std::time::Duration;

use anyhow::{Context, Result, ensure};
use robstride_emulator::config::Config;
use robstride_emulator::emulator::{Emulator, MotorState, ScheduledFrame};
use robstride_protocol::{Frame, MotorParams};

use crate::{Simulation, model::Coordinate};

struct Binding {
    motor: u8,
    actuator: usize,
    torque_limit: f64,
    offset: f64,
}

struct Bus {
    protocol: Emulator,
    bindings: Vec<Binding>,
}

pub struct RobstrideSimulation {
    pub simulation: Simulation,
    buses: BTreeMap<String, Bus>,
    gains: [f64; 2],
    startup_positions: Option<Vec<f64>>,
}

impl RobstrideSimulation {
    /// Coordinates map CAN IDs to compiled actuators and URDF zero offsets.
    pub fn new(
        simulation: Simulation,
        configs: Vec<(Config, BTreeMap<u8, Coordinate>)>,
    ) -> Result<Self> {
        let mut buses = BTreeMap::new();
        let mut assigned = BTreeSet::new();
        for (mut config, coordinates) in configs {
            config.validate()?;
            ensure!(
                coordinates.len() == config.actuators.len(),
                "incomplete bus coordinate mapping"
            );
            let mut bindings = Vec::new();
            for motor in &mut config.actuators {
                let coordinate = coordinates
                    .get(&motor.id)
                    .context("motor coordinate is missing")?;
                let actuator = coordinate.actuator;
                ensure!(coordinate.offset.is_finite(), "invalid motor zero offset");
                motor.initial_position = simulation.actuator_state(actuator)?.0 + coordinate.offset;
                ensure!(
                    assigned.insert(actuator),
                    "MuJoCo actuator {actuator} is mapped more than once"
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
                    torque_limit: unsafe {
                        let model = simulation.model();
                        if *model.actuator_forcelimited.add(actuator) {
                            params
                                .torque_max
                                .min((*model.actuator_forcerange.add(2 * actuator)).abs())
                                .min((*model.actuator_forcerange.add(2 * actuator + 1)).abs())
                        } else {
                            params.torque_max
                        }
                    },
                    offset: coordinate.offset,
                });
            }
            let name = config.interface.to_string();
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
        Ok(Self {
            simulation,
            buses,
            gains: [1., 1.],
            startup_positions: None,
        })
    }

    pub fn set_gain_scales(&mut self, kp: f64, kd: f64) -> Result<()> {
        ensure!(
            [kp, kd].iter().all(|v| v.is_finite() && *v >= 0.),
            "invalid motor gain scale"
        );
        self.gains = [kp, kd];
        Ok(())
    }

    pub fn handle(&mut self, bus: &str, frame: Frame) -> Result<()> {
        self.buses
            .get_mut(bus)
            .context("unknown CAN interface")?
            .protocol
            .handle(frame);
        Ok(())
    }

    /// One-shot kinematic fixture. Feedback and protocol clocks continue while held.
    /// Stop/fault/disable after release never reengages this support.
    pub fn hold_for_startup(&mut self) {
        self.startup_positions = Some(self.simulation.data.qpos().to_vec());
        self.simulation.data.qvel_mut().fill(0.);
        self.simulation.forward();
    }

    pub fn step(&mut self) -> Result<Vec<(String, ScheduledFrame)>> {
        let mut all_commanded = true;
        for bus in self.buses.values() {
            for binding in &bus.bindings {
                let active = bus.protocol.command(binding.motor)?;
                all_commanded &= active.is_some();
                let mut command = active.unwrap_or_default();
                command.position -= binding.offset;
                command.kp *= self.gains[0];
                command.kd *= self.gains[1];
                self.simulation
                    .set_impedance(binding.actuator, command, binding.torque_limit)?;
            }
        }
        if all_commanded && self.startup_positions.take().is_some() {
            eprintln!("Startup support released; every motor has an active command.");
        }
        self.simulation.step();
        if let Some(positions) = &self.startup_positions {
            self.simulation.data.qpos_mut().copy_from_slice(positions);
            self.simulation.data.qvel_mut().fill(0.);
            self.simulation.data.qacc_warmstart_mut().fill(0.);
        }
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
                        position: position + binding.offset,
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

    /// Reset physics and disarm protocol state before accepting fresh operator control.
    pub fn reset_home(&mut self) -> Result<()> {
        self.simulation.reset_home()?;
        for bus in self.buses.values_mut() {
            bus.protocol.disarm();
            for binding in &bus.bindings {
                self.simulation.set_impedance(
                    binding.actuator,
                    Default::default(),
                    binding.torque_limit,
                )?;
                let (position, velocity, torque) =
                    self.simulation.actuator_state(binding.actuator)?;
                bus.protocol.set_motor_state(
                    binding.motor,
                    MotorState {
                        position: position + binding.offset,
                        velocity,
                        torque,
                    },
                )?;
            }
        }
        self.simulation.forward();
        Ok(())
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
            interface: robstride_emulator::config::CanInterface::Socketcan("vcan-test".into()),
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
        let mapping = BTreeMap::from([
            (
                1,
                Coordinate {
                    actuator: 0,
                    offset: 0.2,
                },
            ),
            (
                2,
                Coordinate {
                    actuator: 1,
                    offset: -0.3,
                },
            ),
        ]);
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
        assert!((wire.position - native.0 - 0.2).abs() < 0.001);
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
    fn uncommanded_physics_keeps_advancing_with_feedback() {
        let mapping = BTreeMap::from([
            (
                1,
                Coordinate {
                    actuator: 0,
                    offset: 0.,
                },
            ),
            (
                2,
                Coordinate {
                    actuator: 1,
                    offset: 0.,
                },
            ),
        ]);
        let mut runtime =
            RobstrideSimulation::new(crate::tests::simulation(), vec![(config(), mapping)])
                .unwrap();
        runtime
            .handle("vcan-test", active_report_frame(0xfd, 1, true))
            .unwrap();
        runtime.simulation.data.qvel_mut().fill(0.5);
        let initial = runtime.simulation.data.qpos().to_vec();
        for _ in 0..100 {
            assert!(!runtime.step().unwrap().is_empty());
        }
        assert!(runtime.simulation.data().time > 0.);
        assert_ne!(runtime.simulation.data.qpos(), initial);
        let time = runtime.simulation.data().time;
        runtime
            .handle("vcan-test", stop_frame(0xfd, 1, false))
            .unwrap();
        runtime.step().unwrap();
        assert!(runtime.simulation.data().time > time);
        assert!(runtime.simulation.data.qvel().iter().any(|v| v.abs() > 0.));
    }

    #[test]
    fn startup_support_waits_for_all_buses_then_never_reengages() {
        let buses = (1..=2)
            .map(|motor| {
                let mut bus = config();
                bus.interface =
                    robstride_emulator::config::CanInterface::Socketcan(format!("test{motor}"));
                bus.actuators.retain(|actuator| actuator.id == motor);
                let mapping = BTreeMap::from([(
                    motor,
                    Coordinate {
                        actuator: usize::from(motor - 1),
                        offset: 0.,
                    },
                )]);
                (bus, mapping)
            })
            .collect();
        let mut runtime = RobstrideSimulation::new(crate::tests::simulation(), buses).unwrap();
        runtime.hold_for_startup();
        let initial = runtime.simulation.data.qpos().to_vec();
        for motor in 1..=2 {
            let bus = format!("test{motor}");
            for frame in [
                active_report_frame(0xfd, motor, true),
                param_write_u8_frame(0xfd, motor, 0x7005, 0),
                enable_frame(0xfd, motor),
            ] {
                runtime.handle(&bus, frame).unwrap();
            }
        }
        let command = |motor, direction| {
            control_frame(
                MotorParams::for_model("ROBSTRIDE_03", direction).unwrap(),
                ControlCommand {
                    motor_id: motor,
                    position: 0.5,
                    kp: 100.,
                    ..Default::default()
                },
            )
        };
        runtime.handle("test1", command(1, -1)).unwrap();
        for _ in 0..100 {
            assert!(!runtime.step().unwrap().is_empty());
            assert_eq!(runtime.simulation.data.qpos(), initial);
            assert!(runtime.simulation.data.qvel().iter().all(|v| *v == 0.));
        }
        assert!(runtime.simulation.data().time > 0.);
        runtime.handle("test2", command(2, 1)).unwrap();
        runtime.step().unwrap();
        assert!(runtime.startup_positions.is_none());
        assert_ne!(runtime.simulation.data.qpos(), initial);
        for motor in 1..=2 {
            runtime
                .handle(&format!("test{motor}"), stop_frame(0xfd, motor, false))
                .unwrap();
        }
        runtime.simulation.data.qvel_mut().fill(0.5);
        let released = runtime.simulation.data.qpos().to_vec();
        runtime.step().unwrap();
        assert!(runtime.startup_positions.is_none());
        assert_ne!(runtime.simulation.data.qpos(), released);
        runtime.reset_home().unwrap();
        assert!(runtime.startup_positions.is_none());
    }

    #[test]
    fn duplicate_actuator_ownership_fails_before_stepping() {
        let mapping = BTreeMap::from([
            (
                1,
                Coordinate {
                    actuator: 0,
                    offset: 0.,
                },
            ),
            (
                2,
                Coordinate {
                    actuator: 0,
                    offset: 0.,
                },
            ),
        ]);
        assert!(
            RobstrideSimulation::new(crate::tests::simulation(), vec![(config(), mapping)])
                .is_err()
        );
    }

    #[test]
    fn servo_updates_keep_native_effort_limits() {
        let mut simulation = crate::tests::simulation();
        simulation
            .set_impedance(0, Default::default(), 1.0)
            .unwrap();
        let mapping = BTreeMap::from([
            (
                1,
                Coordinate {
                    actuator: 0,
                    offset: 0.0,
                },
            ),
            (
                2,
                Coordinate {
                    actuator: 1,
                    offset: 0.0,
                },
            ),
        ]);
        let mut runtime = RobstrideSimulation::new(simulation, vec![(config(), mapping)]).unwrap();
        let params = MotorParams::for_model("ROBSTRIDE_03", -1).unwrap();
        for frame in [
            param_write_u8_frame(0xfd, 1, 0x7005, 0),
            enable_frame(0xfd, 1),
            control_frame(
                params,
                ControlCommand {
                    motor_id: 1,
                    torque: 10.0,
                    ..Default::default()
                },
            ),
        ] {
            runtime.handle("vcan-test", frame).unwrap();
        }
        runtime.step().unwrap();
        let effort = runtime.simulation.actuator_state(0).unwrap().2;
        assert!((effort - 1.0).abs() < 1e-12);
    }
}
