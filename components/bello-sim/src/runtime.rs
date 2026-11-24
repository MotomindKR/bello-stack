//! Single-threaded physics with bounded transport queues; async work owns no model.
use crate::{ebimu, guide, model, pose, robstride::RobstrideSimulation, transport::CanEndpoint};
use anyhow::{Context, Result, ensure};
use nalgebra::{UnitQuaternion, Vector3};
use nix::poll::{PollFd, PollFlags, PollTimeout, poll};
use rand::{SeedableRng, rngs::StdRng};
use rand_distr::{Distribution, StandardNormal};
use robstride_emulator::config::{
    ActuatorConfig, CanInterface, Config as BusConfig, DynamicsConfig, ScheduledEvent,
};
use robstride_protocol::{Frame, Request, frame_target, is_control_frame, parse_request};
use serde::Deserialize;
use std::collections::{BTreeMap, BTreeSet, VecDeque};
use std::io::IsTerminal;
use std::net::SocketAddr;
use std::os::fd::{AsFd, AsRawFd};
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};
use tokio::sync::watch;

const MAX_QUEUE: usize = 8192;

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Config {
    pub model: PathBuf,
    pub bus_interfaces: BTreeMap<String, CanInterface>,
    pub feedback_rate_hz: f64,
    pub temperature_c: f64,
    pub command_latency_seconds: f64,
    pub motor_kp_scale: f64,
    pub motor_kd_scale: f64,
    /// Simulation fixture: hold the initial pose until every motor is commanded once.
    #[serde(default)]
    pub startup_hold_until_commanded: bool,
    pub imu: Option<ImuConfig>,
    pub power_board: Option<crate::power_board::Config>,
    pub pose: PoseConfig,
    pub torso_guide: Option<guide::Config>,
    pub fault_events: BTreeMap<String, Vec<ScheduledEvent>>,
    pub video: Option<crate::video::Config>,
    pub cameras: Option<PathBuf>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ImuConfig {
    pub name: String,
    #[serde(flatten)]
    pub endpoint: crate::uart::Endpoint,
    pub rate_hz: f64,
    pub orientation_std_radians: f64,
    pub acceleration_std_mps2: f64,
    pub seed: u64,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PoseConfig {
    pub listen: SocketAddr,
    pub reference_frame: String,
    pub tracked_frame: String,
    pub rate_hz: f64,
}

impl Config {
    pub fn load(path: &Path) -> Result<Self> {
        let mut config: Self = serde_yaml::from_str(&std::fs::read_to_string(path)?)?;
        config.model = path
            .parent()
            .context("config has no parent")?
            .join(&config.model)
            .canonicalize()?;
        if let Some(cameras) = &mut config.cameras {
            *cameras = path
                .parent()
                .context("config has no parent")?
                .join(&*cameras)
                .canonicalize()?;
        }
        if let Some(power) = &config.power_board {
            power.validate()?;
            if let Some(imu) = &config.imu {
                ensure!(
                    power.endpoint.path() != imu.endpoint.path(),
                    "IMU and power UART must be distinct"
                );
            }
        }
        ensure!(
            config.command_latency_seconds.is_finite() && config.command_latency_seconds >= 0.,
            "invalid latency"
        );
        if let Some(imu) = &config.imu {
            for value in [imu.orientation_std_radians, imu.acceleration_std_mps2] {
                ensure!(value.is_finite() && value >= 0., "invalid IMU noise");
            }
        }
        ensure!(
            config.command_latency_seconds <= 1.,
            "command latency exceeds one second"
        );
        ensure!(
            config.pose.rate_hz.is_finite() && (1. ..=1000.).contains(&config.pose.rate_hz),
            "invalid pose rate"
        );
        ensure!(
            !config.pose.reference_frame.is_empty(),
            "empty pose reference frame"
        );
        let mut devices = BTreeSet::new();
        let mut ports = BTreeSet::new();
        for interface in config.bus_interfaces.values() {
            interface.validate()?;
            match interface {
                CanInterface::Socketcan(name) => {
                    ensure!(devices.insert(name), "CAN interfaces must be distinct")
                }
                CanInterface::Sim(bus) => {
                    for port in [bus.daemon_port, bus.simulator_port] {
                        ensure!(
                            ports.insert(port),
                            "simulated CAN buses share UDP port {port}"
                        );
                    }
                }
            }
        }
        for gain in [config.motor_kp_scale, config.motor_kd_scale] {
            ensure!(gain.is_finite() && gain >= 0., "invalid motor gain scale");
        }
        ensure!(
            config
                .fault_events
                .keys()
                .all(|name| config.bus_interfaces.contains_key(name)),
            "unknown fault event bus"
        );
        Ok(config)
    }
}

#[derive(Default)]
struct Queues {
    commands: VecDeque<(Instant, String, Frame)>,
    feedback: BTreeMap<(Instant, u64), (String, Frame)>,
    sequence: u64,
}

impl Queues {
    fn receive(
        &mut self,
        physics: &mut RobstrideSimulation,
        bus: &str,
        frame: Frame,
        now: Instant,
        delay: Duration,
    ) -> Result<()> {
        if is_control_frame(&frame) {
            ensure!(
                self.commands.len() < MAX_QUEUE,
                "command latency queue overflow"
            );
            self.commands.push_back((now + delay, bus.into(), frame));
        } else {
            // Lifecycle changes invalidate delayed targets from a previous motor state.
            if matches!(
                parse_request(&frame),
                Some(Request::Stop { .. } | Request::Enable { .. } | Request::ParamWriteU8 { .. })
            ) {
                self.commands
                    .retain(|(_, b, f)| b != bus || frame_target(f) != frame_target(&frame));
            }
            physics.handle(bus, frame)?;
        }
        Ok(())
    }

    fn apply_due(&mut self, physics: &mut RobstrideSimulation, now: Instant) -> Result<()> {
        while self.commands.front().is_some_and(|(at, _, _)| *at <= now) {
            let (_, bus, frame) = self.commands.pop_front().unwrap();
            physics.handle(&bus, frame)?;
        }
        Ok(())
    }

    fn feedback(
        &mut self,
        bus: String,
        frame: robstride_emulator::emulator::ScheduledFrame,
        now: Instant,
    ) -> Result<()> {
        ensure!(
            self.feedback.len() < MAX_QUEUE,
            "feedback delay queue overflow"
        );
        self.feedback
            .insert((now + frame.delay, self.sequence), (bus, frame.frame));
        self.sequence = self
            .sequence
            .checked_add(1)
            .context("feedback sequence overflow")?;
        Ok(())
    }
}

pub fn run(config: Config, duration: Option<Duration>) -> Result<()> {
    let mut model_config = model::ModelConfig::load(&config.model)?;
    ensure!(
        model_config.torso_guide.is_none(),
        "configure the runtime torso guide in only the emulator profile"
    );
    model_config.torso_guide = config.torso_guide;
    let robot = model::build(&model_config, None)?;
    let mut cameras = config
        .cameras
        .map(|path| {
            crate::cameras::Capture::start(
                crate::cameras::Config::load(&path)?,
                &model_config,
                &robot.simulation,
            )
        })
        .transpose()?;
    let mut video = config
        .video
        .map(|config| crate::video::Capture::new(config, &robot.simulation))
        .transpose()?;
    ensure!(
        config
            .bus_interfaces
            .keys()
            .map(String::as_str)
            .collect::<BTreeSet<_>>()
            == robot
                .hardware
                .logical_buses
                .iter()
                .map(|b| b.name.as_str())
                .collect(),
        "interface mapping must cover every logical bus exactly"
    );
    match &config.imu {
        Some(imu) => ensure!(
            robot.hardware.imus.len() == 1 && robot.hardware.imus[0].name == imu.name,
            "runtime IMU must match the single canonical IMU"
        ),
        None => ensure!(
            robot.hardware.imus.is_empty(),
            "canonical IMU requires an emulator endpoint"
        ),
    }
    let pose_body = robot
        .simulation
        .id(crate::ffi::mjtObj::mjOBJ_BODY, &config.pose.tracked_frame)?
        as usize;
    if let Some(power) = &config.power_board {
        ensure!(
            robot
                .hardware
                .power_boards
                .iter()
                .any(|b| b.name == power.name && b.profile == cog_protocol::Profile::Agx),
            "emulated power board must name a canonical AGX board"
        );
    }
    let mut buses = Vec::new();
    let mut endpoints = BTreeMap::new();
    for bus in &robot.hardware.logical_buses {
        let interface = &config.bus_interfaces[&bus.name];
        let mut coordinates = BTreeMap::new();
        let mut actuators = Vec::new();
        for motor in robot.hardware.motors.iter().filter(|m| m.bus == bus.name) {
            coordinates.insert(motor.id, robot.coordinates[&motor.coordinate]);
            actuators.push(ActuatorConfig {
                id: motor.id,
                model: motor.model.clone(),
                direction: motor.direction,
                initial_position: 0.,
                initial_temperature_c: config.temperature_c,
            });
        }
        buses.push((
            BusConfig {
                interface: interface.clone(),
                host_id: bus.host_id,
                feedback_rate_hz: config.feedback_rate_hz,
                dynamics: DynamicsConfig::default(),
                actuators,
                events: config
                    .fault_events
                    .get(&bus.name)
                    .cloned()
                    .unwrap_or_default(),
            },
            coordinates,
        ));
        endpoints.insert(interface.to_string(), CanEndpoint::open(interface)?);
    }
    let mut physics = RobstrideSimulation::new(robot.simulation, buses)?;
    physics.set_gain_scales(config.motor_kp_scale, config.motor_kd_scale)?;
    if config.startup_hold_until_commanded {
        physics.hold_for_startup();
        eprintln!("Startup support active; waiting for commands to every motor.");
    }
    let mut guide = robot.guide;
    let runtime = tokio::runtime::Builder::new_multi_thread()
        .worker_threads(2)
        .enable_all()
        .build()?;
    let epoch = Instant::now();
    let (pose_tx, pose_rx) = watch::channel(pose::sample(
        physics.simulation.body_state(pose_body)?,
        &config.pose.reference_frame,
        &config.pose.tracked_frame,
        0,
        epoch,
    ));
    let (stop_tx, stop_rx) = watch::channel(false);
    let listener = std::net::TcpListener::bind(config.pose.listen)?;
    listener.set_nonblocking(true)?;
    let listener = {
        let _entered = runtime.enter();
        tokio::net::TcpListener::from_std(listener)?
    };
    let mut stopped = stop_rx.clone();
    let server = runtime.spawn(async move {
        tonic::transport::Server::builder()
            .add_service(pose::Service::new(pose_rx, stopped.clone()))
            .serve_with_incoming_shutdown(
                tokio_stream::wrappers::TcpListenerStream::new(listener),
                async move {
                    if !*stopped.borrow() {
                        let _ = stopped.changed().await;
                    }
                },
            )
            .await
    });
    let (mut interrupt, mut terminate) = {
        let _entered = runtime.enter();
        use tokio::signal::unix::{SignalKind, signal};
        (
            signal(SignalKind::interrupt())?,
            signal(SignalKind::terminate())?,
        )
    };
    let signal_stop = stop_tx.clone();
    let signals = runtime.spawn(async move {
        tokio::select! { _ = interrupt.recv() => {}, _ = terminate.recv() => {} }
        signal_stop.send_replace(true);
    });
    let mut serial = config
        .imu
        .as_ref()
        .map(|imu| ebimu::Serial::open(&imu.endpoint, imu.rate_hz, config.temperature_c))
        .transpose()?;
    let mut power_serial = config
        .power_board
        .map(crate::power_board::Serial::open)
        .transpose()?;
    let result = (|| -> Result<()> {
        let timestep = Duration::from_secs_f64(physics.simulation.model().opt.timestep);
        let pose_period = Duration::from_secs_f64(1. / config.pose.rate_hz);
        let delay = Duration::from_secs_f64(config.command_latency_seconds);
        let mut random = StdRng::seed_from_u64(config.imu.as_ref().map_or(0, |imu| imu.seed));
        let mut deadline = Instant::now();
        let mut next_pose = deadline;
        let mut sequence = 0;
        let mut queues = Queues::default();
        let stdin = std::io::stdin();
        let controls = if guide.is_some() {
            "Enter releases/resets the torso guide; Ctrl-C stops."
        } else {
            "Ctrl-C stops."
        };
        eprintln!(
            "Native emulator online at {}. {controls}",
            config.pose.listen
        );
        while !*stop_rx.borrow() && duration.is_none_or(|limit| epoch.elapsed() < limit) {
            ensure!(!server.is_finished(), "pose service exited unexpectedly");
            ensure!(
                !signals.is_finished() || *stop_rx.borrow(),
                "signal handler failed"
            );
            if stdin.is_terminal() {
                let mut fds = [PollFd::new(stdin.as_fd(), PollFlags::POLLIN)];
                if poll(&mut fds, PollTimeout::ZERO)? > 0
                    && fds[0]
                        .revents()
                        .is_some_and(|f| f.contains(PollFlags::POLLIN))
                {
                    let mut byte = [0];
                    if nix::unistd::read(stdin.as_raw_fd(), &mut byte)? == 1
                        && byte[0] == b'\n'
                        && let Some(guide) = &mut guide
                    {
                        if guide.active {
                            guide.release(&mut physics.simulation);
                            eprintln!("Torso guide released.");
                        } else {
                            physics.reset_home()?;
                            guide.initialize(&mut physics.simulation)?;
                            queues = Queues::default();
                            deadline = Instant::now();
                            eprintln!("Torso guide reset; motors disarmed.");
                        }
                    }
                }
            }
            let now = Instant::now();
            for (bus, endpoint) in &endpoints {
                for _ in 0..256 {
                    let Some(frame) = endpoint.read()? else {
                        break;
                    };
                    queues.receive(&mut physics, bus, frame, now, delay)?;
                }
            }
            queues.apply_due(&mut physics, now)?;
            for (bus, frame) in physics.step()? {
                queues.feedback(bus, frame, now)?;
            }
            while queues
                .feedback
                .first_key_value()
                .is_some_and(|((at, _), _)| *at <= now)
            {
                let (_, (bus, frame)) = queues.feedback.pop_first().unwrap();
                endpoints[&bus].write(frame)?;
            }
            if let (Some(imu), Some(serial)) = (&config.imu, &mut serial) {
                let mut sample = ebimu::Sample::read(&physics.simulation, &imu.name)?;
                let mut gaussian = || -> f64 { StandardNormal.sample(&mut random) };
                sample.orientation *= UnitQuaternion::from_scaled_axis(
                    Vector3::new(gaussian(), gaussian(), gaussian()) * imu.orientation_std_radians,
                );
                sample.specific_force +=
                    Vector3::new(gaussian(), gaussian(), gaussian()) * imu.acceleration_std_mps2;
                serial.update(&sample)?;
            }
            if let Some(power) = &mut power_serial {
                power.update(epoch.elapsed())?;
            }
            if let Some(video) = &mut video {
                video.sample(
                    &physics.simulation,
                    guide.as_ref().is_none_or(|g| !g.active),
                )?;
            }
            if let Some(cameras) = &mut cameras {
                cameras.sample(&physics.simulation);
            }
            if now >= next_pose {
                sequence += 1;
                pose_tx.send_replace(pose::sample(
                    physics.simulation.body_state(pose_body)?,
                    &config.pose.reference_frame,
                    &config.pose.tracked_frame,
                    sequence,
                    epoch,
                ));
                next_pose = now + pose_period;
            }
            deadline += timestep;
            ensure!(
                Instant::now().saturating_duration_since(deadline) < Duration::from_millis(100),
                "simulation fell over 100 ms behind wall clock; refusing stale feedback"
            );
            std::thread::sleep(deadline.saturating_duration_since(Instant::now()));
        }
        Ok(())
    })();
    stop_tx.send_replace(true);
    signals.abort();
    let mut server = server;
    let service_result =
        runtime.block_on(async { tokio::time::timeout(Duration::from_secs(1), &mut server).await });
    if service_result.is_err() {
        server.abort();
    }
    runtime.shutdown_timeout(Duration::from_secs(1));
    drop(endpoints);
    drop(serial);
    drop(power_serial);
    let video_result = video
        .map(|video| video.finish(&model_config, &physics.simulation))
        .transpose();
    if let Err(error) = &video_result {
        eprintln!("Video failed: {error:#}");
    }
    result?;
    service_result.context("pose service did not shut down")???;
    video_result?;
    eprintln!("Native emulator stopped; all endpoints closed.");
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::Coordinate;
    use robstride_protocol::{
        ControlCommand, MotorParams, control_frame, enable_frame, param_write_u8_frame, stop_frame,
    };

    fn physics() -> RobstrideSimulation {
        let config = BusConfig {
            interface: CanInterface::Socketcan("test".into()),
            host_id: 253,
            feedback_rate_hz: 100.,
            dynamics: DynamicsConfig::default(),
            events: vec![],
            actuators: (1..=2)
                .map(|id| ActuatorConfig {
                    id,
                    model: "ROBSTRIDE_03".into(),
                    direction: 1,
                    initial_position: 0.,
                    initial_temperature_c: 25.,
                })
                .collect(),
        };
        RobstrideSimulation::new(
            crate::tests::simulation(),
            vec![(
                config,
                BTreeMap::from([
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
                ]),
            )],
        )
        .unwrap()
    }

    #[test]
    fn delayed_commands_cannot_survive_stop_reenable_or_reset() {
        let mut physics = physics();
        let mut queues = Queues::default();
        let now = Instant::now();
        let delay = Duration::from_millis(10);
        let command = control_frame(
            MotorParams::for_model("ROBSTRIDE_03", 1).unwrap(),
            ControlCommand {
                motor_id: 1,
                torque: 2.,
                ..Default::default()
            },
        );
        for frame in [
            param_write_u8_frame(253, 1, 0x7005, 0),
            enable_frame(253, 1),
            command,
        ] {
            queues
                .receive(&mut physics, "test", frame, now, delay)
                .unwrap();
        }
        queues.apply_due(&mut physics, now).unwrap();
        physics.step().unwrap();
        assert_eq!(physics.simulation.actuator_state(0).unwrap().2, 0.);
        for frame in [stop_frame(253, 1, false), enable_frame(253, 1)] {
            queues
                .receive(&mut physics, "test", frame, now, delay)
                .unwrap();
        }
        queues.apply_due(&mut physics, now + delay).unwrap();
        physics.step().unwrap();
        assert_eq!(physics.simulation.actuator_state(0).unwrap().2, 0.);
        queues
            .receive(&mut physics, "test", command, now, delay)
            .unwrap();
        queues.apply_due(&mut physics, now + delay).unwrap();
        physics.step().unwrap();
        assert!(physics.simulation.actuator_state(0).unwrap().2 > 1.9);
        physics.reset_home().unwrap();
        physics.step().unwrap();
        assert_eq!(physics.simulation.actuator_state(0).unwrap().2, 0.);
    }
}
