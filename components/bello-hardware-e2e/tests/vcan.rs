#![cfg(target_os = "linux")]

#[allow(dead_code)]
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
    AcquireLeaseRequest, EnableGroupRequest, ListHardwareRequest, MotorCommand, RenewLeaseRequest,
    StartControlRequest, StopControlRequest, StreamCommandsRequest, WatchTelemetryRequest,
};
use robstride_protocol::{
    ControlCommand, Feedback, Frame, MotorParams, control_frame, feedback_frame, parse_feedback,
};
use socketcan::{CanFrame, CanSocket, EmbeddedFrame, ExtendedId, Frame as _, Socket};
use support::process::ManagedProcess;
use tokio::sync::mpsc;
use tokio::time::{Instant, sleep, timeout};
use tokio_stream::wrappers::ReceiverStream;

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
#[ignore = "requires an isolated host vcan interface"]
async fn periodic_300hz_transmission_preserves_expiry_and_reporting() -> Result<()> {
    use bello_hardware_api::{DeviceState, GetStateFrameRequest, GroupCause, GroupState};
    use bello_hardware_client::raw::{CommandStreamOptions, CommandValidity, LeaseDuration};

    let interface = selected_interface()?;
    require_vcan(&interface)?;
    let directory = tempfile::tempdir()?;
    let hardware_config = directory.path().join("hardwared.yaml");
    let emulator_config = directory.path().join("emulator.yaml");
    let address = available_loopback_address()?;
    support::config::write_hardwared_with_control_rate(
        &hardware_config,
        &interface,
        address,
        300.0,
    )?;
    let description = directory.path().join("e2e-hardware.yaml");
    let mut identity: serde_yaml::Value = serde_yaml::from_str(&fs::read_to_string(&description)?)?;
    let group = &mut identity["motor_groups"][0];
    group["command_refresh_rate_hz"] = serde_yaml::to_value(300.0)?;
    group["command_transmission"] = serde_yaml::to_value("periodic")?;
    group["periodic_feedback_during_control"] = serde_yaml::to_value(false)?;
    fs::write(&description, serde_yaml::to_string(&identity)?)?;
    let mut model: serde_yaml::Value =
        serde_yaml::from_str(&support::config::emulator(&interface))?;
    model["feedback_rate_hz"] = serde_yaml::to_value(100.0)?;
    fs::write(&emulator_config, serde_yaml::to_string(&model)?)?;

    let socket = CanSocket::open(&interface)?;
    socket.set_read_timeout(Duration::from_millis(10))?;
    let recorded = Arc::new(std::sync::Mutex::new(Vec::new()));
    let recording = recorded.clone();
    let stop = Arc::new(AtomicBool::new(false));
    let stopping = stop.clone();
    let worker = thread::spawn(move || {
        while !stopping.load(Ordering::Acquire) {
            if let Ok(frame) = socket.read_frame() {
                let Ok(data) = frame.data().try_into() else {
                    continue;
                };
                recording.lock().unwrap().push((
                    std::time::Instant::now(),
                    Frame {
                        id: frame.raw_id(),
                        data,
                    },
                ));
            }
        }
    });
    struct CaptureGuard(Arc<AtomicBool>, Option<thread::JoinHandle<()>>);
    impl Drop for CaptureGuard {
        fn drop(&mut self) {
            self.0.store(true, Ordering::Release);
            if let Some(worker) = self.1.take() {
                let _ = worker.join();
            }
        }
    }
    let _capture = CaptureGuard(stop, Some(worker));
    let mut emulator = ManagedProcess::spawn(
        "emulator",
        env!("CARGO_BIN_EXE_robstride-emulator-e2e-fixture"),
        [&emulator_config],
    )?;
    let mut daemon = ManagedProcess::spawn(
        "hardwared",
        env!("CARGO_BIN_EXE_hardwared-e2e-fixture"),
        [&hardware_config],
    )?;
    let client = support::scenario::connect(
        &format!("http://{address}"),
        Duration::from_secs(5),
        &mut daemon,
        &mut emulator,
    )
    .await?;
    timeout(Duration::from_secs(5), async {
        loop {
            let inventory = client.inventory().await?;
            if inventory.motors.iter().all(|m| {
                m.status
                    .as_ref()
                    .is_some_and(|s| s.state == DeviceState::Online as i32)
            }) {
                break;
            }
            sleep(Duration::from_millis(10)).await;
        }
        Ok::<(), anyhow::Error>(())
    })
    .await??;
    let commands = || {
        ["e2e_motor_1", "e2e_motor_2"]
            .into_iter()
            .map(|name| MotorCommand {
                motor_name: name.into(),
                position: 0.0,
                velocity: 0.0,
                torque_feedforward: 0.0,
                kp: 20.0,
                kd: 1.0,
            })
            .collect::<Vec<_>>()
    };
    let validity = CommandValidity::For(Duration::from_millis(80));
    let mut lease = client
        .acquire_lease(
            "e2e_group",
            "300hz-test",
            LeaseDuration::For(Duration::from_secs(10)),
        )
        .await?;
    lease.enable().await?;
    lease.start(validity, commands()).await?;
    let (mut sender, mut replies) = lease
        .open_command_stream(CommandStreamOptions::default())
        .await?
        .split();
    let mut timer = tokio::time::interval(Duration::from_millis(20));
    timer.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    let started = std::time::Instant::now();
    for _ in 0..50 {
        timer.tick().await;
        sender.send(validity, commands()).await?;
        timeout(Duration::from_secs(1), replies.message())
            .await??
            .context("missing command acknowledgement")?
            .require_accepted()?;
    }
    let ended = std::time::Instant::now();
    let elapsed = ended.duration_since(started).as_secs_f64();
    let inventory = client.inventory().await?;
    assert_eq!(inventory.groups[0].state, GroupState::Running as i32);
    assert!(
        inventory
            .motors
            .iter()
            .all(|m| m.status.as_ref().unwrap().state == DeviceState::Online as i32)
    );
    {
        let traffic = recorded.lock().unwrap();
        for motor in [1u32, 2] {
            let sends = traffic
                .iter()
                .filter(|(at, frame)| {
                    *at >= started
                        && *at <= ended
                        && (frame.id >> 24) & 31 == 1
                        && frame.id & 255 == motor
                })
                .count();
            let rate = sends as f64 / elapsed;
            assert!(
                (270.0..=330.0).contains(&rate),
                "motor {motor}: {rate:.1} Hz"
            );
            println!("motor {motor}: {rate:.1} CAN commands/s with 50 Hz policy updates");
        }
        assert!(
            traffic
                .iter()
                .any(|(_, f)| (f.id >> 24) & 31 == 24 && f.data[6] == 0)
        );
        assert!(
            !traffic.iter().any(|(at, f)| *at >= started
                && *at <= ended
                && (f.id >> 24) & 31 == 24
                && f.data[6] == 1),
            "monitoring re-enabled periodic reports during control"
        );
    }
    sleep(Duration::from_millis(180)).await;
    let frame = client
        .grpc_client()
        .get_state_frame(GetStateFrameRequest {
            groups: vec!["e2e_group".into()],
            imus: vec![],
        })
        .await?
        .into_inner();
    assert_eq!(frame.groups[0].state, GroupState::Damped as i32);
    assert_eq!(frame.groups[0].cause, GroupCause::CommandTimeout as i32);
    assert!(frame.motors.iter().all(|m| m.sample_age_ns < 30_000_000));
    let quiet = std::time::Instant::now();
    sleep(Duration::from_millis(60)).await;
    assert!(
        !recorded
            .lock()
            .unwrap()
            .iter()
            .any(|(at, f)| *at >= quiet && (f.id >> 24) & 31 == 1),
        "stale target continued to transmit after expiry"
    );
    assert!(
        recorded
            .lock()
            .unwrap()
            .iter()
            .any(|(at, f)| *at > ended && (f.id >> 24) & 31 == 24 && f.data[6] == 1),
        "periodic reporting was not restored after expiry"
    );
    drop(sender);
    drop(replies);
    let long_validity = CommandValidity::For(Duration::from_millis(400));
    lease.start(long_validity, commands()).await?;
    sleep(Duration::from_millis(20)).await;
    lease.stop().await?;
    let stopped = std::time::Instant::now();
    sleep(Duration::from_millis(30)).await;
    assert!(
        !recorded
            .lock()
            .unwrap()
            .iter()
            .any(|(at, f)| *at >= stopped && (f.id >> 24) & 31 == 1),
        "periodic targets resumed after StopControl"
    );
    lease.start(long_validity, commands()).await?;
    sleep(Duration::from_millis(20)).await;
    lease.release().await?;
    let released = std::time::Instant::now();
    sleep(Duration::from_millis(30)).await;
    assert!(
        !recorded
            .lock()
            .unwrap()
            .iter()
            .any(|(at, f)| *at >= released && (f.id >> 24) & 31 == 1),
        "periodic targets resumed after lease release"
    );
    let state = client
        .grpc_client()
        .get_state_frame(GetStateFrameRequest {
            groups: vec!["e2e_group".into()],
            imus: vec![],
        })
        .await?
        .into_inner();
    assert_eq!(state.groups[0].state, GroupState::Damped as i32);
    assert_eq!(state.groups[0].cause, GroupCause::LeaseLoss as i32);

    let mut lease = client
        .acquire_lease(
            "e2e_group",
            "stall-test",
            LeaseDuration::For(Duration::from_secs(10)),
        )
        .await?;
    lease.start(long_validity, commands()).await?;
    sleep(Duration::from_millis(20)).await;
    daemon.suspend()?;
    sleep(Duration::from_millis(25)).await;
    daemon.resume()?;
    timeout(Duration::from_secs(2), async {
        loop {
            let inventory = client
                .grpc_client()
                .get_state_frame(GetStateFrameRequest {
                    groups: vec!["e2e_group".into()],
                    imus: vec![],
                })
                .await?
                .into_inner();
            if inventory.groups[0].state == GroupState::Fault as i32 {
                assert_eq!(inventory.groups[0].cause, GroupCause::Transmission as i32);
                assert!(
                    inventory.groups[0]
                        .detail
                        .contains("periodic control stalled")
                );
                break;
            }
            sleep(Duration::from_millis(5)).await;
        }
        Ok::<(), anyhow::Error>(())
    })
    .await??;
    println!("Stop, lease release, expiry, and injected 25 ms TX stall applied safety responses");
    lease.recover().await?;
    lease.release().await?;
    daemon.stop();
    emulator.stop();
    Ok(())
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

    let initial = latency_command(&lease.token, 1, 0.0);
    client
        .start_control(StartControlRequest {
            group: GROUP.into(),
            lease_token: lease.token.clone(),
            valid_for_ms: initial.valid_for_ms,
            commands: initial.commands,
        })
        .await?;
    let (command_tx, command_rx) = mpsc::channel(1);
    let mut responses = client
        .stream_commands(ReceiverStream::new(command_rx))
        .await?
        .into_inner();

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
        let position = latency_triangle(index + 1);
        let expected = control_frame(
            params,
            ControlCommand {
                motor_id: MOTOR_ID,
                position,
                velocity: 0.0,
                torque: 0.0,
                kp: 0.0,
                kd: 1.0,
            },
        );
        let started = Instant::now();
        command_tx
            .send(latency_command(&lease.token, sequence, position))
            .await
            .context("submitting measured command")?;
        let observed = monitor
            .wait_for(expected, Duration::from_millis(100))
            .with_context(|| format!("observing outbound sample {index}"))?;
        outbound.push(observed.saturating_duration_since(started));
        support::scenario::require_accepted(&mut responses, sequence).await?;
    }
    let hardwared_cpu =
        process_cpu_percent(hardwared_cpu_start, process_cpu_sample(hardwared.pid()?)?)?;
    let emulator_cpu =
        process_cpu_percent(emulator_cpu_start, process_cpu_sample(emulator.pid()?)?)?;
    let immediate_delivery_budget = Duration::from_secs_f64(0.75 / control_rate_hz);
    let outbound_p95 = percentile(&outbound, 0.95);
    if outbound_p95 >= immediate_delivery_budget {
        bail!(
            "command-triggered CAN delivery p95 {outbound_p95:?} exceeded budget {immediate_delivery_budget:?}"
        );
    }

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
            include_power_boards: false,
            power_boards: vec![],
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
            position: -0.9,
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
        let position = -0.8 + latency_triangle(index);
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

fn latency_triangle(index: usize) -> f64 {
    let phase = index % 800;
    phase.min(800 - phase) as f64 * 0.0015
}

fn latency_command(token: &str, sequence: u64, position: f64) -> StreamCommandsRequest {
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
                kp: 0.0,
                kd: 1.0,
            })
            .collect(),
    }
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

fn percentile(samples: &[Duration], fraction: f64) -> Duration {
    let mut values = samples.to_vec();
    values.sort();
    values[((values.len() - 1) as f64 * fraction) as usize]
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
