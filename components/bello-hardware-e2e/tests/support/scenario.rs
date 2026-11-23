use std::{env, path::PathBuf, process::Command, time::Duration};

use anyhow::{Context, Result, anyhow, bail};
use bello_hardware_client::raw::proto::watch_telemetry_response::Payload;
use bello_hardware_client::raw::proto::{DeviceState, GroupCause, GroupState, MotorCommand};
use bello_hardware_client::raw::{
    CommandStreamOptions, CommandValidity, EnumValue, GroupLease, HardwareClient, LeaseDuration,
    TelemetryFilter, TelemetryStream,
};
use bello_hardware_client::{CommandFrame, ControlOptions, CoordinateTarget, Robot};
use tokio::task::JoinHandle;
use tokio::time::{Instant, sleep, timeout};

use super::process::ManagedProcess;

const GROUP: &str = "e2e_group";
const MOTORS: [&str; 2] = ["e2e_motor_1", "e2e_motor_2"];
const PROCESS_POLL: Duration = Duration::from_millis(50);

pub async fn connect(
    endpoint: &str,
    deadline: Duration,
    hardwared: &mut ManagedProcess,
    emulator: &mut ManagedProcess,
) -> Result<HardwareClient> {
    let deadline = Instant::now() + deadline;
    loop {
        hardwared.ensure_running()?;
        emulator.ensure_running()?;
        match HardwareClient::connect(endpoint.to_owned()).await {
            Ok(client) => return Ok(client),
            Err(error) if Instant::now() < deadline => {
                let _ = error;
                sleep(Duration::from_millis(20)).await;
            }
            Err(error) => return Err(error).context("connecting to hardwared gRPC service"),
        }
    }
}

pub async fn run(
    client: HardwareClient,
    endpoint: &str,
    timeout_duration: Duration,
    hardwared: &mut ManagedProcess,
    emulator: &mut ManagedProcess,
) -> Result<()> {
    let mut telemetry = client
        .watch(
            TelemetryFilter::default()
                .selected_groups([GROUP])
                .include_motors(true)
                .include_health(true)
                .include_command_policy_events(true),
        )
        .await?;

    wait_for_motors_online(&client, timeout_duration, hardwared, emulator).await?;
    run_ergonomic_smoke(&client).await?;
    run_python_smoke(endpoint).await?;

    let mut lease = client
        .acquire_lease(
            GROUP,
            "bello-hardware-e2e",
            LeaseDuration::For(Duration::from_secs(10)),
        )
        .await?;

    require_state(lease.enable().await?, GroupState::Damped)?;
    wait_for_motors_online(&client, timeout_duration, hardwared, emulator).await?;

    require_state(
        lease
            .start(CommandValidity::For(Duration::from_millis(200)), command())
            .await?,
        GroupState::Running,
    )?;
    let commands = CommandStream::start(&lease).await?;

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

    lease
        .renew(LeaseDuration::For(Duration::from_secs(10)))
        .await?;

    require_state(lease.recover().await?, GroupState::Idle)?;
    require_state(lease.enable().await?, GroupState::Damped)?;

    require_state(
        lease
            .start(CommandValidity::For(Duration::from_millis(200)), command())
            .await?,
        GroupState::Running,
    )?;
    let timeout_commands = CommandStream::start(&lease).await?;
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
        lease
            .start(CommandValidity::For(Duration::from_millis(200)), command())
            .await?,
        GroupState::Running,
    )?;
    require_state(lease.stop().await?, GroupState::Damped)?;
    require_state(lease.enable().await?, GroupState::Damped)?;
    require_state(lease.disable().await?, GroupState::Idle)?;
    require_state(lease.release().await?, GroupState::Idle)?;

    let expiring = client
        .acquire_lease(
            GROUP,
            "bello-hardware-e2e-expiry",
            LeaseDuration::For(Duration::from_millis(300)),
        )
        .await?;
    require_state(expiring.enable().await?, GroupState::Damped)?;
    require_state(
        expiring
            .start(CommandValidity::For(Duration::from_millis(200)), command())
            .await?,
        GroupState::Running,
    )?;
    let lease_commands = CommandStream::start(&expiring).await?;
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
    println!("vcan integration passed: {feedback_fault}; {timeout_fault}; {lease_fault}");
    Ok(())
}

async fn run_python_smoke(endpoint: &str) -> Result<()> {
    let endpoint = endpoint.to_owned();
    tokio::task::spawn_blocking(move || {
        let manifest = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        let package = manifest
            .parent()
            .expect("E2E component must have a components parent")
            .join("bello-hardware-python");
        let output = Command::new("python3")
            .arg(package.join("tests/vcan_smoke.py"))
            .arg(endpoint)
            .env("PYTHONPATH", package.join("src"))
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

pub(crate) async fn run_ergonomic_smoke(client: &HardwareClient) -> Result<()> {
    let robot = Robot::from_hardware_client(client.clone()).await?;
    let group = robot.group(GROUP)?;
    let mut control = group
        .control_with(
            ControlOptions::new("bello-hardware-e2e-ergonomic")
                .lease_duration(Duration::from_secs(10))
                .command_validity(Duration::from_millis(200)),
        )
        .await?;
    control.start(ergonomic_frame(&group, 0.0)?).await?;
    control.send(ergonomic_frame(&group, 0.01)?).await?;
    control.shutdown().await?;
    Ok(())
}

fn ergonomic_frame(
    group: &bello_hardware_client::MotorGroup,
    position: f64,
) -> Result<CommandFrame> {
    Ok(group
        .frame()
        .coordinate(
            "e2e_joint_1",
            CoordinateTarget::position(position).gains(20.0, 1.0),
        )?
        .coordinate(
            "e2e_joint_2",
            CoordinateTarget::position(position).gains(20.0, 1.0),
        )?
        .build()?)
}

struct CommandStream {
    sender: JoinHandle<()>,
    receiver: JoinHandle<()>,
}

impl CommandStream {
    async fn start(lease: &GroupLease) -> Result<Self> {
        let (mut commands, mut responses) = lease
            .open_command_stream(CommandStreamOptions::default())
            .await?
            .split();
        commands
            .send(CommandValidity::For(Duration::from_millis(200)), command())
            .await?;
        let acknowledgement = timeout(Duration::from_secs(1), responses.message())
            .await
            .context("waiting for initial command acknowledgement")??
            .ok_or_else(|| anyhow!("command response stream closed"))?;
        acknowledgement.require_accepted()?;

        let sender = tokio::spawn(async move {
            let mut timer = tokio::time::interval(Duration::from_millis(20));
            loop {
                timer.tick().await;
                if commands
                    .send(CommandValidity::For(Duration::from_millis(200)), command())
                    .await
                    .is_err()
                {
                    break;
                }
            }
        });
        let receiver =
            tokio::spawn(async move { while let Ok(Some(_)) = responses.message().await {} });
        Ok(Self { sender, receiver })
    }
}

impl Drop for CommandStream {
    fn drop(&mut self) {
        self.sender.abort();
        self.receiver.abort();
    }
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
    telemetry: &mut TelemetryStream,
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
                if let Payload::Group(group) = event.payload
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
    telemetry: &mut TelemetryStream,
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
                if let Payload::Group(group) = event.payload
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
    let inventory = client.inventory().await?;
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
    let inventory = client.inventory().await?;
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

fn require_state(actual: EnumValue<GroupState>, expected: GroupState) -> Result<()> {
    if actual != EnumValue::Known(expected) {
        bail!("expected state {expected:?}, got {actual:?}");
    }
    Ok(())
}
