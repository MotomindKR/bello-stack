mod support;

use std::fs;
use std::net::{SocketAddr, TcpListener};
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc as std_mpsc;
use std::thread;
use std::time::Duration;

use anyhow::{Context, Result, anyhow, bail};
use bello_hardware_api::hardware_service_client::HardwareServiceClient;
use bello_hardware_api::watch_telemetry_response::Payload;
use bello_hardware_api::{
    AcquireLeaseRequest, CommandResult, EnableGroupRequest, ListHardwareRequest, MotorCommand,
    RenewLeaseRequest, StartControlRequest, StopControlRequest, StreamCommandsRequest,
    WatchTelemetryRequest,
};
use bello_visualizer::runtime::{ControlStatus, DisplaySnapshot, RuntimeHandle};
use bello_visualizer::{RobotDescription, Transmission, VisualizerConfig};
use robstride_protocol::{
    ControlCommand, Feedback, Frame, MotorParams, control_frame, feedback_frame, parse_feedback,
};
use socketcan::{CanFrame, CanSocket, EmbeddedFrame, ExtendedId, Frame as _, Socket};
use support::process::ManagedProcess;
use tokio::sync::mpsc;
use tokio::time::{Instant, sleep, timeout};
use tokio_stream::wrappers::ReceiverStream;

#[test]
fn generated_runtime_configs_are_valid() -> Result<()> {
    let directory = tempfile::tempdir()?;
    let hardwared_config = directory.path().join("hardwared.yaml");
    let emulator_config = directory.path().join("emulator.yaml");
    support::config::write_hardwared(&hardwared_config, "vcan_test", "127.0.0.1:50052".parse()?)?;
    fs::write(&emulator_config, support::config::emulator("vcan_test"))?;

    bello_hardwared::check(&hardwared_config)?;
    robstride_emulator::config::Config::load(&emulator_config)?;

    let description = RobotDescription::load(
        &Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/vcan.urdf"),
    )?;
    let mapping = Transmission::from_description(&description, ["e2e_joint_1", "e2e_joint_2"])?;
    assert_eq!(mapping.joint_to_coordinate(&[0.1, -0.2])?, [0.1, -0.2]);
    Ok(())
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
#[ignore = "requires a host vcan interface"]
async fn control_fault_and_recovery_over_vcan() -> Result<()> {
    let interface = selected_interface()?;
    require_vcan(&interface)?;
    let listen = available_loopback_address()?;
    let directory = tempfile::tempdir().context("creating e2e configuration directory")?;
    let hardwared_config = directory.path().join("hardwared.yaml");
    let emulator_config = directory.path().join("emulator.yaml");
    support::config::write_hardwared(&hardwared_config, &interface, listen)?;
    fs::write(&emulator_config, support::config::emulator(&interface))?;

    let mut emulator = ManagedProcess::spawn(
        "robstride-emulator",
        env!("CARGO_BIN_EXE_robstride-emulator-e2e-fixture"),
        [&emulator_config],
    )?;
    let mut hardwared = ManagedProcess::spawn(
        "bello-hardwared",
        env!("CARGO_BIN_EXE_hardwared-e2e-fixture"),
        [&hardwared_config],
    )?;
    let endpoint = format!("http://{listen}");
    let client = support::scenario::connect(
        &endpoint,
        Duration::from_secs(3),
        &mut hardwared,
        &mut emulator,
    )
    .await?;
    support::scenario::run(
        client,
        &endpoint,
        Duration::from_secs(8),
        &mut hardwared,
        &mut emulator,
    )
    .await
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
#[ignore = "requires a host vcan interface"]
async fn visualizer_runtime_commands_at_profile_rate_without_rendering() -> Result<()> {
    const GROUP: &str = "e2e_group";
    const COMMAND_RATE_HZ: f64 = 40.0;
    const MEASUREMENT: Duration = Duration::from_millis(1_500);

    let interface = selected_interface()?;
    require_vcan(&interface)?;
    let listen = available_loopback_address()?;
    let directory = tempfile::tempdir().context("creating visualizer e2e directory")?;
    let hardwared_config = directory.path().join("hardwared.yaml");
    let emulator_config = directory.path().join("emulator.yaml");
    let visualizer_config = directory.path().join("visualizer.yaml");
    let urdf = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/vcan.urdf");
    support::config::write_hardwared(&hardwared_config, &interface, listen)?;
    fs::write(&emulator_config, support::config::emulator(&interface))?;
    fs::write(
        &visualizer_config,
        visualizer_yaml(&urdf, &format!("http://{listen}"), COMMAND_RATE_HZ),
    )?;

    let config = VisualizerConfig::load(&visualizer_config)?;
    let description = RobotDescription::load(&config.urdf)?;
    let mapping = Transmission::from_description(&description, ["e2e_joint_1", "e2e_joint_2"])?;
    assert_eq!(mapping.coordinate_names(), ["e2e_joint_1", "e2e_joint_2"]);

    let mut emulator = ManagedProcess::spawn(
        "robstride-emulator",
        env!("CARGO_BIN_EXE_robstride-emulator-e2e-fixture"),
        [&emulator_config],
    )?;
    let mut hardwared = ManagedProcess::spawn(
        "bello-hardwared",
        env!("CARGO_BIN_EXE_hardwared-e2e-fixture"),
        [&hardwared_config],
    )?;
    let endpoint = format!("http://{listen}");
    let readiness_client = support::scenario::connect(
        &endpoint,
        Duration::from_secs(3),
        &mut hardwared,
        &mut emulator,
    )
    .await?;

    // RuntimeHandle is the complete headless I/O/control half of the visualizer. No
    // Kiss3d window, egui context, render callback, or calls to `latest` are needed
    // while its command loop runs on the background Tokio runtime.
    let runtime = RuntimeHandle::spawn(config, description)?;
    // Hardwared requests active reports at monitor-only startup; no lease,
    // enable, or command is needed to make feedback available to the visualizer.
    drop(readiness_client);
    wait_for_visualizer(
        &runtime,
        Duration::from_secs(5),
        &mut hardwared,
        &mut emulator,
        |snapshot| {
            snapshot.hardware_connected
                && ["e2e_joint_1", "e2e_joint_2"]
                    .into_iter()
                    .all(|coordinate| {
                        snapshot.motors.get(coordinate).is_some_and(|motor| {
                            motor.feedback_available
                                && motor.feedback_current_connection
                                && motor.age <= Duration::from_millis(500)
                        })
                    })
        },
    )
    .await?;
    runtime
        .start_groups(vec![GROUP.into()])
        .map_err(anyhow::Error::msg)?;
    let before = wait_for_visualizer(
        &runtime,
        Duration::from_secs(5),
        &mut hardwared,
        &mut emulator,
        |snapshot| {
            matches!(snapshot.control.get(GROUP), Some(ControlStatus::Running))
                && snapshot.groups.get(GROUP).is_some_and(|group| {
                    group.state == bello_hardware_client::GroupState::Running
                        && group.command_count > 0
                })
        },
    )
    .await?;
    let before_count = before.groups[GROUP].command_count;

    let started = Instant::now();
    sleep(MEASUREMENT).await;
    let elapsed = started.elapsed();
    hardwared.ensure_running()?;
    emulator.ensure_running()?;
    let after = runtime.latest();
    let group = after
        .groups
        .get(GROUP)
        .ok_or_else(|| anyhow!("visualizer snapshot lost group {GROUP:?}"))?;
    if !matches!(after.control.get(GROUP), Some(ControlStatus::Running))
        || group.state != bello_hardware_client::GroupState::Running
    {
        bail!(
            "visualizer control did not remain running: control={:?}, daemon={:?}, fault={}",
            after.control.get(GROUP),
            group.state,
            group.fault_detail
        );
    }

    let commands = group.command_count.saturating_sub(before_count);
    let expected = COMMAND_RATE_HZ * elapsed.as_secs_f64();
    let minimum = (expected * 0.75).floor() as u64;
    let maximum = (expected * 1.25).ceil() as u64 + 2;
    if !(minimum..=maximum).contains(&commands) {
        bail!(
            "visualizer accepted {commands} complete frames in {elapsed:?}; expected about {expected:.1} at {COMMAND_RATE_HZ:.1} Hz (allowed {minimum}..={maximum})"
        );
    }
    if group.control_tick <= before.groups[GROUP].control_tick {
        bail!("daemon motor-control ticks did not advance while visualizer commanded");
    }

    runtime.stop_all().map_err(anyhow::Error::msg)?;
    let stopped = wait_for_visualizer(
        &runtime,
        Duration::from_secs(3),
        &mut hardwared,
        &mut emulator,
        |snapshot| matches!(snapshot.control.get(GROUP), Some(ControlStatus::Monitoring)),
    )
    .await?;
    assert_ne!(
        stopped.groups[GROUP].state,
        bello_hardware_client::GroupState::Running
    );
    if !runtime.shutdown(Duration::from_secs(3)) {
        bail!("visualizer runtime did not complete bounded shutdown");
    }
    println!(
        "headless visualizer accepted {commands} frames in {:.3} s ({:.1} Hz, configured {:.1} Hz)",
        elapsed.as_secs_f64(),
        commands as f64 / elapsed.as_secs_f64(),
        COMMAND_RATE_HZ
    );
    Ok(())
}

fn visualizer_yaml(urdf: &Path, endpoint: &str, command_rate_hz: f64) -> String {
    format!(
        r#"urdf: {urdf:?}
hardware:
  endpoint: {endpoint}
root:
  source: fixed
refresh_hz: 100
control_profiles:
  e2e_group:
    command_rate_hz: {command_rate_hz}
    validity_ms: 200
    telemetry_staleness_ms: 500
"#,
        urdf = urdf.display(),
    )
}

async fn wait_for_visualizer(
    runtime: &RuntimeHandle,
    duration: Duration,
    hardwared: &mut ManagedProcess,
    emulator: &mut ManagedProcess,
    predicate: impl Fn(&DisplaySnapshot) -> bool,
) -> Result<DisplaySnapshot> {
    let deadline = Instant::now() + duration;
    loop {
        hardwared.ensure_running()?;
        emulator.ensure_running()?;
        let snapshot = runtime.latest();
        if predicate(&snapshot) {
            return Ok(snapshot);
        }
        if Instant::now() >= deadline {
            bail!(
                "visualizer state did not satisfy condition in {duration:?}: connected={}, hardware_warning={:?}, motors={:?}, groups={:?}, events={:?}, control={:?}",
                snapshot.hardware_connected,
                snapshot.hardware_warning,
                snapshot.motors,
                snapshot.groups,
                snapshot.events,
                snapshot.control
            );
        }
        sleep(Duration::from_millis(20)).await;
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
#[ignore = "requires a host vcan interface and is a performance measurement"]
async fn grpc_can_latency_distribution() -> Result<()> {
    const GROUP: &str = "e2e_group";
    const MOTOR: &str = "e2e_motor_1";
    const HOST_ID: u8 = 253;
    const MOTOR_ID: u8 = 1;

    let interface = selected_interface()?;
    require_vcan(&interface)?;
    let listen = available_loopback_address()?;
    let directory = tempfile::tempdir().context("creating latency configuration directory")?;
    let hardwared_config = directory.path().join("hardwared.yaml");
    let emulator_config = directory.path().join("emulator.yaml");
    let control_rate_hz = std::env::var("BELLO_LATENCY_CONTROL_RATE_HZ")
        .ok()
        .and_then(|value| value.parse().ok())
        .unwrap_or(250.0);
    support::config::write_hardwared_with_control_rate(
        &hardwared_config,
        &interface,
        listen,
        control_rate_hz,
    )?;
    fs::write(&emulator_config, support::config::emulator(&interface))?;

    let mut emulator = ManagedProcess::spawn(
        "robstride-emulator",
        env!("CARGO_BIN_EXE_robstride-emulator-e2e-fixture"),
        [&emulator_config],
    )?;
    let mut hardwared = ManagedProcess::spawn(
        "bello-hardwared",
        env!("CARGO_BIN_EXE_hardwared-e2e-fixture"),
        [&hardwared_config],
    )?;
    let endpoint = format!("http://{listen}");
    let hardware = support::scenario::connect(
        &endpoint,
        Duration::from_secs(3),
        &mut hardwared,
        &mut emulator,
    )
    .await?;
    let mut client = hardware.grpc_client();

    let lease = client
        .acquire_lease(AcquireLeaseRequest {
            group: GROUP.into(),
            client_name: "latency-probe".into(),
            requested_duration_ms: 10_000,
        })
        .await?
        .into_inner();
    client
        .enable_group(EnableGroupRequest {
            group: GROUP.into(),
            lease_token: lease.token.clone(),
        })
        .await?;
    wait_for_online(&mut client, Duration::from_secs(3)).await?;

    let (command_tx, command_rx) = mpsc::channel(1);
    let mut responses = client
        .stream_commands(ReceiverStream::new(command_rx))
        .await?
        .into_inner();
    command_tx
        .send(command(&lease.token, 1, 0.0))
        .await
        .context("submitting initial command")?;
    require_accepted(&mut responses, 1).await?;
    client
        .start_control(StartControlRequest {
            group: GROUP.into(),
            lease_token: lease.token.clone(),
        })
        .await?;

    let samples = std::env::var("BELLO_LATENCY_SAMPLES")
        .ok()
        .and_then(|value| value.parse().ok())
        .unwrap_or(500_usize);
    let params = MotorParams::for_model("ROBSTRIDE_03", 1).expect("known motor model");
    let mut monitor = CanMonitor::start(&interface)?;
    let mut outbound = Vec::with_capacity(samples);
    let mut random = 0x9e37_79b9_u32;
    let hardwared_cpu_start = process_cpu_sample(hardwared.pid()?)?;
    let emulator_cpu_start = process_cpu_sample(emulator.pid()?)?;
    for index in 0..samples {
        if index != 0 && index % 100 == 0 {
            client
                .renew_lease(RenewLeaseRequest {
                    token: lease.token.clone(),
                    requested_duration_ms: 10_000,
                })
                .await?;
        }
        random ^= random << 13;
        random ^= random >> 17;
        random ^= random << 5;
        sleep(Duration::from_micros((random % 4_000) as u64)).await;

        let sequence = index as u64 + 2;
        let position = -0.4 + index as f64 * 0.0015;
        let expected = control_frame(
            params,
            ControlCommand {
                motor_id: MOTOR_ID,
                position,
                velocity: 0.0,
                torque: 0.0,
                kp: 20.0,
                kd: 1.0,
            },
        );
        let started = Instant::now();
        command_tx
            .send(command(&lease.token, sequence, position))
            .await
            .context("submitting measured command")?;
        let observed = monitor
            .wait_for(expected, Duration::from_millis(100))
            .with_context(|| format!("observing outbound sample {index}"))?;
        outbound.push(observed.saturating_duration_since(started));
        require_accepted(&mut responses, sequence).await?;
    }
    let hardwared_cpu =
        process_cpu_percent(hardwared_cpu_start, process_cpu_sample(hardwared.pid()?)?)?;
    let emulator_cpu =
        process_cpu_percent(emulator_cpu_start, process_cpu_sample(emulator.pid()?)?)?;

    client
        .stop_control(StopControlRequest {
            group: GROUP.into(),
            lease_token: lease.token,
        })
        .await?;

    let mut telemetry = client
        .watch_telemetry(WatchTelemetryRequest {
            groups: vec![GROUP.into()],
            include_motors: true,
            include_imu: false,
            include_health: false,
            imus: Vec::new(),
            include_command_policy_events: false,
        })
        .await?
        .into_inner();
    let injection = CanSocket::open(&interface)
        .with_context(|| format!("opening {interface} for feedback injection"))?;
    let warmup = feedback_frame(
        HOST_ID,
        Feedback {
            motor_id: MOTOR_ID,
            mode_status: 2,
            fault_bits: 0,
            position: -9.0,
            velocity: 0.0,
            torque: 0.0,
            temperature: 30.0,
        },
        params,
    );
    let warmup_position = parse_feedback(&warmup, params)
        .expect("generated feedback must decode")
        .position;
    let warmup_started = Instant::now();
    write_frame(&injection, warmup)?;
    wait_for_motor_position(&mut telemetry, MOTOR, warmup_position).await?;
    let telemetry_stream_warmup = warmup_started.elapsed();

    let mut inbound = Vec::with_capacity(samples);
    for index in 0..samples {
        let position = -8.0 + index as f64 * 0.0015;
        let frame = feedback_frame(
            HOST_ID,
            Feedback {
                motor_id: MOTOR_ID,
                mode_status: 2,
                fault_bits: 0,
                position,
                velocity: 0.0,
                torque: 0.0,
                temperature: 30.0,
            },
            params,
        );
        let expected_position = parse_feedback(&frame, params)
            .expect("generated feedback must decode")
            .position;
        let started = Instant::now();
        write_frame(&injection, frame)?;
        wait_for_motor_position(&mut telemetry, MOTOR, expected_position).await?;
        inbound.push(Instant::now().saturating_duration_since(started));
    }

    println!("motor-group control rate: {control_rate_hz:.1} Hz");
    println!("outbound-phase CPU: hardwared={hardwared_cpu:.2}% emulator={emulator_cpu:.2}%");
    println!(
        "gRPC command -> observed vCAN control frame: {}",
        summarize(&outbound)
    );
    println!(
        "injected vCAN feedback -> gRPC telemetry: {}",
        summarize(&inbound)
    );
    println!(
        "first feedback after opening telemetry stream (excluded): {:.1} us",
        telemetry_stream_warmup.as_secs_f64() * 1_000_000.0
    );
    Ok(())
}

fn command(token: &str, sequence: u64, position: f64) -> StreamCommandsRequest {
    StreamCommandsRequest {
        group: "e2e_group".into(),
        lease_token: token.into(),
        sequence,
        valid_for_ms: 200,
        commands: ["e2e_motor_1", "e2e_motor_2"]
            .into_iter()
            .map(|motor_name| MotorCommand {
                motor_name: motor_name.into(),
                position,
                velocity: 0.0,
                torque_feedforward: 0.0,
                kp: 20.0,
                kd: 1.0,
            })
            .collect(),
    }
}

async fn require_accepted(
    responses: &mut tonic::Streaming<bello_hardware_api::StreamCommandsResponse>,
    sequence: u64,
) -> Result<()> {
    let response = timeout(Duration::from_secs(1), responses.message())
        .await
        .context("waiting for command acknowledgement")??
        .ok_or_else(|| anyhow!("command response stream closed"))?;
    if response.sequence != sequence || response.result != CommandResult::Accepted as i32 {
        bail!(
            "command {sequence} was not accepted: sequence={}, result={}, detail={}",
            response.sequence,
            response.result,
            response.detail
        );
    }
    Ok(())
}

async fn wait_for_motor_position(
    telemetry: &mut tonic::Streaming<bello_hardware_api::WatchTelemetryResponse>,
    motor_name: &str,
    expected_position: f64,
) -> Result<()> {
    loop {
        let event = timeout(Duration::from_millis(100), telemetry.message())
            .await
            .context("waiting for injected feedback telemetry")??
            .ok_or_else(|| anyhow!("telemetry stream closed"))?;
        let Some(Payload::Motor(motor)) = event.payload else {
            continue;
        };
        if motor.motor_name == motor_name && (motor.position - expected_position).abs() < 1e-9 {
            return Ok(());
        }
    }
}

async fn wait_for_online(
    client: &mut HardwareServiceClient<tonic::transport::Channel>,
    duration: Duration,
) -> Result<()> {
    let deadline = Instant::now() + duration;
    loop {
        let inventory = client
            .list_hardware(ListHardwareRequest {})
            .await?
            .into_inner();
        if inventory.motors.iter().all(|motor| {
            motor.status.as_ref().is_some_and(|status| {
                status.state == bello_hardware_api::DeviceState::Online as i32
            })
        }) {
            return Ok(());
        }
        if Instant::now() >= deadline {
            bail!("motors did not become online before latency measurement");
        }
        sleep(Duration::from_millis(10)).await;
    }
}

struct ObservedFrame {
    frame: Frame,
    observed: Instant,
}

struct CanMonitor {
    frames: std_mpsc::Receiver<ObservedFrame>,
    stop: Arc<AtomicBool>,
    join: Option<thread::JoinHandle<Result<()>>>,
}

impl CanMonitor {
    fn start(interface: &str) -> Result<Self> {
        let socket = CanSocket::open(interface)
            .with_context(|| format!("opening {interface} for control-frame observation"))?;
        socket.set_read_timeout(Duration::from_millis(20))?;
        let (tx, frames) = std_mpsc::channel();
        let stop = Arc::new(AtomicBool::new(false));
        let thread_stop = stop.clone();
        let join = thread::Builder::new()
            .name("latency-can-monitor".into())
            .spawn(move || {
                while !thread_stop.load(Ordering::Acquire) {
                    match socket.read_frame() {
                        Ok(frame) => {
                            let Ok(data) = frame.data().try_into() else {
                                continue;
                            };
                            if tx
                                .send(ObservedFrame {
                                    frame: Frame {
                                        id: frame.raw_id(),
                                        data,
                                    },
                                    observed: Instant::now(),
                                })
                                .is_err()
                            {
                                break;
                            }
                        }
                        Err(error)
                            if matches!(
                                error.kind(),
                                std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut
                            ) => {}
                        Err(error) => return Err(error).context("reading observed CAN frame"),
                    }
                }
                Ok(())
            })?;
        Ok(Self {
            frames,
            stop,
            join: Some(join),
        })
    }

    fn wait_for(&mut self, expected: Frame, duration: Duration) -> Result<Instant> {
        let deadline = Instant::now() + duration;
        loop {
            let remaining = deadline.saturating_duration_since(Instant::now());
            let observed = self
                .frames
                .recv_timeout(remaining)
                .context("waiting for matching CAN control frame")?;
            if observed.frame == expected {
                return Ok(observed.observed);
            }
        }
    }
}

impl Drop for CanMonitor {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Release);
        if let Some(join) = self.join.take() {
            let _ = join.join();
        }
    }
}

fn write_frame(socket: &CanSocket, frame: Frame) -> Result<()> {
    let id = ExtendedId::new(frame.id).ok_or_else(|| anyhow!("invalid CAN id"))?;
    let frame = CanFrame::new(id, &frame.data).ok_or_else(|| anyhow!("invalid CAN frame"))?;
    socket.write_frame(&frame).context("injecting CAN frame")
}

fn summarize(samples: &[Duration]) -> String {
    let mut values = samples
        .iter()
        .map(|sample| sample.as_secs_f64() * 1_000_000.0)
        .collect::<Vec<_>>();
    values.sort_by(f64::total_cmp);
    let mean = values.iter().sum::<f64>() / values.len() as f64;
    let variance = values
        .iter()
        .map(|value| (value - mean).powi(2))
        .sum::<f64>()
        / values.len() as f64;
    let percentile = |fraction: f64| values[((values.len() - 1) as f64 * fraction) as usize];
    format!(
        "n={} min={:.1} us mean={:.1} us p50={:.1} us p95={:.1} us p99={:.1} us max={:.1} us jitter_stddev={:.1} us",
        values.len(),
        values[0],
        mean,
        percentile(0.50),
        percentile(0.95),
        percentile(0.99),
        values[values.len() - 1],
        variance.sqrt(),
    )
}

#[derive(Clone, Copy)]
struct ProcessCpuSample {
    ticks: u64,
    observed: std::time::Instant,
}

fn process_cpu_sample(pid: u32) -> Result<ProcessCpuSample> {
    let stat = fs::read_to_string(format!("/proc/{pid}/stat"))?;
    let fields = stat
        .rsplit_once(')')
        .ok_or_else(|| anyhow!("invalid /proc/{pid}/stat"))?
        .1
        .split_whitespace()
        .collect::<Vec<_>>();
    let user_ticks = fields
        .get(11)
        .ok_or_else(|| anyhow!("missing user CPU time for pid {pid}"))?
        .parse::<u64>()?;
    let system_ticks = fields
        .get(12)
        .ok_or_else(|| anyhow!("missing system CPU time for pid {pid}"))?
        .parse::<u64>()?;
    Ok(ProcessCpuSample {
        ticks: user_ticks + system_ticks,
        observed: std::time::Instant::now(),
    })
}

fn process_cpu_percent(start: ProcessCpuSample, end: ProcessCpuSample) -> Result<f64> {
    // Linux exposes process CPU time in USER_HZ ticks; sysconf provides USER_HZ
    // for the running host rather than assuming the usual value of 100.
    let ticks_per_second = nix::unistd::sysconf(nix::unistd::SysconfVar::CLK_TCK)?
        .filter(|ticks| *ticks > 0)
        .ok_or_else(|| anyhow!("could not determine Linux process clock tick rate"))?;
    let cpu_seconds = end.ticks.saturating_sub(start.ticks) as f64 / ticks_per_second as f64;
    let wall_seconds = end
        .observed
        .saturating_duration_since(start.observed)
        .as_secs_f64();
    Ok(cpu_seconds / wall_seconds * 100.0)
}

fn selected_interface() -> Result<String> {
    let interface = std::env::var("BELLO_E2E_CAN_INTERFACE").unwrap_or_else(|_| "vcan0".into());
    if interface.is_empty()
        || interface.len() > 15
        || !interface
            .bytes()
            .all(|byte| byte.is_ascii_alphanumeric() || matches!(byte, b'_' | b'-' | b'.'))
    {
        bail!("BELLO_E2E_CAN_INTERFACE must be a valid Linux interface name");
    }
    Ok(interface)
}

fn require_vcan(interface: &str) -> Result<()> {
    let path = PathBuf::from("/sys/class/net").join(interface);
    if !path.is_dir() {
        bail!(
            "SocketCAN interface {interface} does not exist; create and bring up a vcan interface first"
        );
    }
    let hardware_type = read_trimmed(path.join("type"))?;
    if hardware_type != "280" {
        bail!("interface {interface} is not a CAN interface (ARPHRD type {hardware_type})");
    }
    let flags = read_trimmed(path.join("flags"))?;
    let flags = u32::from_str_radix(flags.trim_start_matches("0x"), 16)
        .with_context(|| format!("parsing flags for interface {interface}"))?;
    if flags & 1 == 0 {
        bail!("SocketCAN interface {interface} is not up");
    }
    Ok(())
}

fn read_trimmed(path: impl AsRef<Path>) -> Result<String> {
    let path = path.as_ref();
    Ok(fs::read_to_string(path)
        .with_context(|| format!("reading {}", path.display()))?
        .trim()
        .to_owned())
}

fn available_loopback_address() -> Result<SocketAddr> {
    let listener = TcpListener::bind("127.0.0.1:0").context("reserving a loopback gRPC port")?;
    let address = listener.local_addr()?;
    drop(listener);
    Ok(address)
}
