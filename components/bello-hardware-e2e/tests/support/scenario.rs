use std::time::Duration;

use anyhow::{Context, Result, anyhow, bail};
use bello_hardware_api::{
    self as proto, AcquireLeaseResponse, DeviceState, GetStateFrameRequest, GroupCause, GroupState,
    MotorCommand, WatchTelemetryRequest, hardware_service_client::HardwareServiceClient,
    watch_telemetry_response::Payload,
};
use bello_hardware_client::{CommandFrame, ControlOptions, CoordinateTarget, Robot};
use robstride_protocol::{Feedback, Frame, MotorParams, feedback_frame};
use tokio::sync::mpsc;
use tokio::task::JoinHandle;
use tokio::time::{Instant, sleep, timeout};
use tokio_stream::wrappers::ReceiverStream;
use tonic::transport::{Channel, Endpoint};

use super::{process::ManagedProcess, python};

const GROUP: &str = "e2e_group";
const MOTORS: [&str; 2] = ["e2e_motor_1", "e2e_motor_2"];
const PROCESS_POLL: Duration = Duration::from_millis(50);

pub type HardwareClient = HardwareServiceClient<Channel>;

pub async fn connect(
    endpoint: &str,
    deadline: Duration,
    hardwared: &mut ManagedProcess,
    emulator: &mut ManagedProcess,
) -> Result<HardwareClient> {
    let endpoint =
        Endpoint::from_shared(endpoint.to_owned())?.connect_timeout(Duration::from_secs(5));
    let deadline = Instant::now() + deadline;
    loop {
        hardwared.ensure_running()?;
        emulator.ensure_running()?;
        let ready = async {
            let client = HardwareServiceClient::new(endpoint.connect().await?);
            inventory(&client).await?;
            anyhow::Ok(client)
        };
        match ready.await {
            Ok(client) => return Ok(client),
            Err(error) if Instant::now() < deadline => {
                let _ = error;
                sleep(Duration::from_millis(20)).await;
            }
            Err(error) => return Err(error).context("connecting to hardwared gRPC service"),
        }
    }
}

pub async fn queued_feedback_keeps_its_age(
    endpoint: &str,
    daemon: &mut ManagedProcess,
    emulator: &mut ManagedProcess,
    send_feedback: impl FnOnce([Frame; 2]) -> Result<()>,
) -> Result<()> {
    let deadline = Duration::from_secs(5);
    let mut client = connect(endpoint, deadline, daemon, emulator).await?;
    wait_for_motors_online(&client, deadline, daemon, emulator).await?;
    let lease = acquire(&client, GROUP, "queued-feedback", 10_000).await?;
    require_state(enable(&client, &lease).await?, GroupState::Damped)?;

    let before_pause = Instant::now();
    let captured_before_pause = client
        .get_state_frame(GetStateFrameRequest::default())
        .await?
        .into_inner()
        .captured_monotonic_ns;
    daemon.suspend()?;
    emulator.stop();
    send_feedback([1, 2].map(|motor_id| {
        feedback_frame(
            253,
            Feedback {
                motor_id,
                mode_status: 2,
                fault_bits: 0,
                position: 0.25,
                velocity: 0.0,
                torque: 0.0,
                temperature: 25.0,
            },
            MotorParams::for_model("RS_03", 1).unwrap(),
        )
    }))?;
    let latest_possible_send_ns = captured_before_pause + before_pause.elapsed().as_nanos() as u64;
    let feedback_timeout = Duration::from_millis(super::config::FEEDBACK_TIMEOUT_MS);
    sleep(feedback_timeout * 3).await;
    daemon.resume()?;

    timeout(deadline, async {
        loop {
            let frame = client
                .get_state_frame(GetStateFrameRequest::default())
                .await?
                .into_inner();
            if frame.motors.len() == 2 && frame.motors.iter().all(|m| m.position.abs() > 0.2) {
                for motor in &frame.motors {
                    assert!(
                        Duration::from_nanos(motor.sample_age_ns) >= feedback_timeout,
                        "queued feedback appeared fresh: {} age {:?}, timeout {feedback_timeout:?}",
                        motor.motor_name,
                        Duration::from_nanos(motor.sample_age_ns),
                    );
                    assert!(
                        motor.sample_monotonic_ns <= latest_possible_send_ns,
                        "queued feedback timestamp was later than the last send"
                    );
                }
                if frame.groups[0].state == GroupState::Fault as i32 {
                    assert_eq!(frame.groups[0].cause, GroupCause::StaleFeedback as i32);
                    let ages: Vec<_> = frame
                        .motors
                        .iter()
                        .map(|motor| Duration::from_nanos(motor.sample_age_ns))
                        .collect();
                    println!("Queued motor sample ages {ages:?}, group fault StaleFeedback");
                    return Ok(());
                }
            }
            sleep(Duration::from_millis(2)).await;
        }
    })
    .await
    .context("waiting for queued samples and the stale-feedback watchdog")?
}

pub async fn run(
    mut client: HardwareClient,
    endpoint: &str,
    timeout_duration: Duration,
    hardwared: &mut ManagedProcess,
    emulator: &mut ManagedProcess,
) -> Result<()> {
    let mut telemetry = client
        .watch_telemetry(WatchTelemetryRequest {
            groups: vec![GROUP.into()],
            include_motors: true,
            include_health: true,
            include_command_policy_events: true,
            ..Default::default()
        })
        .await?
        .into_inner();

    wait_for_motors_online(&client, timeout_duration, hardwared, emulator).await?;
    run_ergonomic_smoke(endpoint).await?;
    run_python_smoke(endpoint).await?;

    let lease = acquire(&client, GROUP, "bello-hardware-e2e", 10000).await?;

    require_state(enable(&client, &lease).await?, GroupState::Damped)?;
    wait_for_motors_online(&client, timeout_duration, hardwared, emulator).await?;

    require_state(
        start_control(&client, &lease, 200, command()).await?,
        GroupState::Running,
    )?;
    let commands = CommandStream::start(&client, &lease).await?;

    // Suspending the process preserves actuator state while making the test,
    // rather than emulator startup timing, own the feedback-loss phase.
    emulator.suspend()?;
    let feedback_fault = wait_for_fault(
        &mut telemetry,
        timeout_duration,
        GroupState::Fault,
        GroupCause::StaleFeedback,
        hardwared,
        Some(emulator),
    )
    .await?;
    drop(commands);

    emulator.resume()?;
    wait_for_motors_online(&client, timeout_duration, hardwared, emulator).await?;
    require_group_state(&client, GroupState::Fault).await?;

    client
        .renew_lease(proto::RenewLeaseRequest {
            token: lease.token.clone(),
            requested_duration_ms: 10_000,
        })
        .await?;

    require_state(recover(&client, &lease).await?, GroupState::Idle)?;
    require_state(enable(&client, &lease).await?, GroupState::Damped)?;

    require_state(
        start_control(&client, &lease, 200, command()).await?,
        GroupState::Running,
    )?;
    let timeout_commands = CommandStream::start(&client, &lease).await?;
    drop(timeout_commands);
    let timeout_fault = wait_for_fault(
        &mut telemetry,
        timeout_duration,
        GroupState::Damped,
        GroupCause::CommandTimeout,
        hardwared,
        Some(emulator),
    )
    .await?;

    require_state(
        start_control(&client, &lease, 200, command()).await?,
        GroupState::Running,
    )?;
    require_state(stop(&client, &lease).await?, GroupState::Damped)?;
    require_state(enable(&client, &lease).await?, GroupState::Damped)?;
    require_state(disable(&client, &lease).await?, GroupState::Idle)?;
    require_state(
        client
            .release_lease(proto::ReleaseLeaseRequest {
                token: lease.token.clone(),
            })
            .await?
            .into_inner()
            .state(),
        GroupState::Idle,
    )?;

    let expiring = acquire(&client, GROUP, "bello-hardware-e2e-expiry", 300).await?;
    require_state(enable(&client, &expiring).await?, GroupState::Damped)?;
    require_state(
        start_control(&client, &expiring, 200, command()).await?,
        GroupState::Running,
    )?;
    let lease_commands = CommandStream::start(&client, &expiring).await?;
    let lease_fault = wait_for_fault(
        &mut telemetry,
        timeout_duration,
        GroupState::Damped,
        GroupCause::LeaseLoss,
        hardwared,
        Some(emulator),
    )
    .await?;

    hardwared.ensure_running()?;
    emulator.ensure_running()?;
    drop(lease_commands);
    hardwared.interrupt()?;
    wait_for_graceful_shutdown(&mut telemetry, timeout_duration).await?;
    hardwared.stop();
    println!("control and recovery passed: {feedback_fault}; {timeout_fault}; {lease_fault}");
    Ok(())
}

async fn run_python_smoke(endpoint: &str) -> Result<()> {
    let endpoint = endpoint.to_owned();
    tokio::task::spawn_blocking(move || {
        let output = python::command()?
            .arg(python::sdk_package().join("tests/control_smoke.py"))
            .arg(endpoint)
            .output()
            .context("running the Python gRPC smoke client")?;
        if !output.status.success() {
            bail!(
                "Python gRPC smoke client failed with {}: {}",
                output.status,
                String::from_utf8_lossy(&output.stderr).trim()
            );
        }
        Ok(())
    })
    .await
    .context("joining the Python gRPC smoke client")?
}

async fn run_ergonomic_smoke(endpoint: &str) -> Result<()> {
    let robot = Robot::connect(endpoint).await?;
    let group = robot.group(GROUP)?;
    let mut control = group
        .control(
            ControlOptions::new("bello-hardware-e2e-ergonomic")
                .lease_duration(Duration::from_secs(10))
                .command_validity(Duration::from_millis(200)),
        )
        .await?;
    control.start(ergonomic_frame(&group, 0.0)?).await?;
    control.send(ergonomic_frame(&group, 0.01)?).await?;
    let ack = control.try_submit(ergonomic_frame(&group, 0.0)?)?;
    ack.await?;
    control.shutdown().await?;
    Ok(())
}

fn ergonomic_frame(
    group: &bello_hardware_client::MotorGroup,
    position: f64,
) -> Result<CommandFrame> {
    Ok(group.frame([
        (
            "e2e_joint_1",
            CoordinateTarget::position(position).gains(20.0, 1.0),
        ),
        (
            "e2e_joint_2",
            CoordinateTarget::position(position).gains(20.0, 1.0),
        ),
    ])?)
}

struct CommandStream(JoinHandle<()>);

impl CommandStream {
    async fn start(client: &HardwareClient, lease: &AcquireLeaseResponse) -> Result<Self> {
        let (tx, rx) = mpsc::channel(1);
        let mut replies = client
            .clone()
            .stream_commands(ReceiverStream::new(rx))
            .await?
            .into_inner();
        let mut request = proto::StreamCommandsRequest {
            group: lease.group.clone(),
            lease_token: lease.token.clone(),
            sequence: 1,
            valid_for_ms: 200,
            commands: command(),
        };
        tx.send(request.clone()).await?;
        require_accepted(&mut replies, request.sequence).await?;
        Ok(Self(tokio::spawn(async move {
            let mut ticks = tokio::time::interval(Duration::from_millis(20));
            loop {
                ticks.tick().await;
                request.sequence += 1;
                if tx.send(request.clone()).await.is_err()
                    || require_accepted(&mut replies, request.sequence)
                        .await
                        .is_err()
                {
                    break;
                }
            }
        })))
    }
}

impl Drop for CommandStream {
    fn drop(&mut self) {
        self.0.abort();
    }
}

pub async fn acquire(
    client: &HardwareClient,
    group: &str,
    owner: &str,
    duration_ms: u64,
) -> Result<AcquireLeaseResponse> {
    Ok(client
        .clone()
        .acquire_lease(proto::AcquireLeaseRequest {
            group: group.into(),
            client_name: owner.into(),
            requested_duration_ms: duration_ms,
        })
        .await?
        .into_inner())
}

pub async fn inventory(client: &HardwareClient) -> Result<proto::ListHardwareResponse> {
    Ok(client
        .clone()
        .list_hardware(bello_hardware_api::ListHardwareRequest {})
        .await?
        .into_inner())
}

pub async fn enable(client: &HardwareClient, lease: &AcquireLeaseResponse) -> Result<GroupState> {
    Ok(client
        .clone()
        .enable_group(proto::EnableGroupRequest {
            group: lease.group.clone(),
            lease_token: lease.token.clone(),
        })
        .await?
        .into_inner()
        .state())
}

pub async fn recover(client: &HardwareClient, lease: &AcquireLeaseResponse) -> Result<GroupState> {
    Ok(client
        .clone()
        .recover_group(proto::RecoverGroupRequest {
            group: lease.group.clone(),
            lease_token: lease.token.clone(),
        })
        .await?
        .into_inner()
        .state())
}

pub async fn disable(client: &HardwareClient, lease: &AcquireLeaseResponse) -> Result<GroupState> {
    Ok(client
        .clone()
        .disable_group(proto::DisableGroupRequest {
            group: lease.group.clone(),
            lease_token: lease.token.clone(),
        })
        .await?
        .into_inner()
        .state())
}

pub async fn stop(client: &HardwareClient, lease: &AcquireLeaseResponse) -> Result<GroupState> {
    Ok(client
        .clone()
        .stop_control(proto::StopControlRequest {
            group: lease.group.clone(),
            lease_token: lease.token.clone(),
        })
        .await?
        .into_inner()
        .state())
}

pub async fn start_control(
    client: &HardwareClient,
    lease: &AcquireLeaseResponse,
    valid_for_ms: u64,
    commands: Vec<MotorCommand>,
) -> Result<GroupState> {
    Ok(client
        .clone()
        .start_control(proto::StartControlRequest {
            group: lease.group.clone(),
            lease_token: lease.token.clone(),
            valid_for_ms,
            commands,
        })
        .await?
        .into_inner()
        .state())
}

fn command() -> Vec<MotorCommand> {
    MOTORS
        .into_iter()
        .map(|motor_name| MotorCommand {
            motor_name: motor_name.into(),
            position: 0.0,
            velocity: 0.0,
            torque_feedforward: 0.0,
            kp: 20.0,
            kd: 1.0,
        })
        .collect()
}

async fn wait_for_fault(
    telemetry: &mut tonic::Streaming<proto::WatchTelemetryResponse>,
    duration: Duration,
    expected_state: GroupState,
    expected: GroupCause,
    hardwared: &mut ManagedProcess,
    mut emulator: Option<&mut ManagedProcess>,
) -> Result<String> {
    let deadline = Instant::now() + duration;
    loop {
        let now = Instant::now();
        if now >= deadline {
            bail!("timed out waiting for {expected_state:?} with cause {expected:?}");
        }
        match timeout((deadline - now).min(PROCESS_POLL), telemetry.message()).await {
            Ok(result) => {
                let event =
                    result?.ok_or_else(|| anyhow!("telemetry stream closed before group state"))?;
                if let Some(Payload::Group(group)) = event.payload
                    && group.group == GROUP
                {
                    if group.state == expected_state as i32 {
                        if group.cause == expected as i32 {
                            return Ok(group.detail);
                        }
                        if expected_state != GroupState::Fault {
                            continue;
                        }
                        bail!(
                            "expected {expected_state:?} with cause {expected:?}, observed cause {}: {}",
                            group.cause,
                            group.detail
                        );
                    }
                    if group.state == GroupState::Fault as i32 {
                        bail!(
                            "expected {expected_state:?} with cause {expected:?}, observed Fault cause {}: {}",
                            group.cause,
                            group.detail
                        );
                    }
                }
            }
            Err(_) => {
                hardwared.ensure_running()?;
                if let Some(emulator) = emulator.as_deref_mut() {
                    emulator.ensure_running()?;
                }
            }
        }
    }
}

async fn wait_for_graceful_shutdown(
    telemetry: &mut tonic::Streaming<proto::WatchTelemetryResponse>,
    duration: Duration,
) -> Result<()> {
    let deadline = Instant::now() + duration;
    let mut saw_group_shutdown = false;
    loop {
        let remaining = deadline.saturating_duration_since(Instant::now());
        if remaining.is_zero() {
            bail!("timed out waiting for final group telemetry and stream closure");
        }
        match timeout(remaining, telemetry.message()).await?? {
            Some(event) => {
                if let Some(Payload::Group(group)) = event.payload
                    && group.group == GROUP
                    && group.state == GroupState::Shutdown as i32
                {
                    saw_group_shutdown = true;
                }
            }
            None if saw_group_shutdown => return Ok(()),
            None => bail!("telemetry closed before the final group shutdown event"),
        }
    }
}

pub async fn wait_for_motors_online(
    client: &HardwareClient,
    duration: Duration,
    hardwared: &mut ManagedProcess,
    emulator: &mut ManagedProcess,
) -> Result<()> {
    let deadline = Instant::now() + duration;
    loop {
        hardwared.ensure_running()?;
        emulator.ensure_running()?;
        if motors_online(client).await? {
            return Ok(());
        }
        if Instant::now() >= deadline {
            bail!("motors did not become online before the deadline");
        }
        sleep(Duration::from_millis(20)).await;
    }
}

async fn motors_online(client: &HardwareClient) -> Result<bool> {
    let inventory = inventory(client).await?;
    Ok(MOTORS.iter().all(|expected| {
        inventory.motors.iter().any(|motor| {
            motor.name == *expected
                && motor
                    .status
                    .as_ref()
                    .is_some_and(|status| status.state == DeviceState::Online as i32)
        })
    }))
}

async fn require_group_state(client: &HardwareClient, expected: GroupState) -> Result<()> {
    let inventory = inventory(client).await?;
    let group = inventory
        .groups
        .iter()
        .find(|group| group.name == GROUP)
        .ok_or_else(|| anyhow!("e2e group is missing from inventory"))?;
    if group.state != expected as i32 {
        bail!("expected state {expected:?}, got {}", group.state);
    }
    Ok(())
}

fn require_state(actual: GroupState, expected: GroupState) -> Result<()> {
    if actual != expected {
        bail!("expected state {expected:?}, got {actual:?}");
    }
    Ok(())
}

pub async fn require_accepted(
    responses: &mut tonic::Streaming<bello_hardware_api::StreamCommandsResponse>,
    sequence: u64,
) -> Result<()> {
    let response = timeout(Duration::from_secs(1), responses.message())
        .await
        .context("waiting for command acknowledgement")??
        .ok_or_else(|| anyhow!("command response stream closed"))?;
    if response.sequence != sequence
        || response.result != bello_hardware_api::CommandResult::Accepted as i32
    {
        bail!(
            "command {sequence} was not accepted: sequence={}, result={}, detail={}",
            response.sequence,
            response.result,
            response.detail
        );
    }
    Ok(())
}
