use std::time::Duration;

use anyhow::{Context, Result, anyhow, bail};
use bello_hardwared::proto::hardware_service_client::HardwareServiceClient;
use bello_hardwared::proto::watch_telemetry_response::Payload;
use bello_hardwared::proto::{
    AcquireLeaseRequest, CommandResult, DeviceState, DisableGroupRequest, EnableGroupRequest,
    FaultCause, GroupState, ListHardwareRequest, MotorCommand, RecoverGroupRequest,
    ReleaseLeaseRequest, RenewLeaseRequest, StartControlRequest, StreamCommandsRequest,
    WatchTelemetryRequest,
};
use tokio::sync::mpsc;
use tokio::task::JoinHandle;
use tokio::time::{Instant, sleep, timeout};
use tokio_stream::wrappers::ReceiverStream;
use tonic::Streaming;
use tonic::transport::Channel;

use super::process::ManagedProcess;

const GROUP: &str = "e2e_group";
const MOTORS: [&str; 2] = ["e2e_motor_1", "e2e_motor_2"];
const PROCESS_POLL: Duration = Duration::from_millis(50);

type Client = HardwareServiceClient<Channel>;

pub async fn connect(
    endpoint: &str,
    deadline: Duration,
    hardwared: &mut ManagedProcess,
    emulator: &mut ManagedProcess,
) -> Result<Client> {
    let deadline = Instant::now() + deadline;
    loop {
        hardwared.ensure_running()?;
        emulator.ensure_running()?;
        match HardwareServiceClient::connect(endpoint.to_owned()).await {
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
    mut client: Client,
    timeout_duration: Duration,
    hardwared: &mut ManagedProcess,
    emulator: &mut ManagedProcess,
) -> Result<()> {
    let mut telemetry = client
        .watch_telemetry(WatchTelemetryRequest {
            groups: vec![GROUP.into()],
            include_motors: true,
            include_imu: false,
            include_health: true,
            imus: Vec::new(),
            include_command_policy_events: true,
        })
        .await?
        .into_inner();

    let lease = client
        .acquire_lease(AcquireLeaseRequest {
            group: GROUP.into(),
            client_name: "bello-hardware-e2e".into(),
            requested_duration_ms: 10_000,
        })
        .await?
        .into_inner();
    if lease.token.is_empty() {
        bail!("lease acquisition returned an empty token");
    }
    let token = lease.token;

    let enabled = client
        .enable_group(EnableGroupRequest {
            group: GROUP.into(),
            lease_token: token.clone(),
        })
        .await?
        .into_inner();
    require_state(enabled.state, GroupState::Enabled)?;
    wait_for_motors_online(&mut client, timeout_duration, hardwared, emulator).await?;

    let commands = CommandStream::start(client.clone(), token.clone()).await?;
    let started = client
        .start_control(StartControlRequest {
            group: GROUP.into(),
            lease_token: token.clone(),
        })
        .await?
        .into_inner();
    require_state(started.state, GroupState::Running)?;

    // Suspending the process preserves actuator state while making the test,
    // rather than emulator startup timing, own the feedback-loss phase.
    emulator.suspend()?;
    let feedback_fault = wait_for_fault(
        &mut telemetry,
        timeout_duration,
        FaultCause::StaleFeedback,
        hardwared,
        Some(emulator),
    )
    .await?;
    drop(commands);

    emulator.resume()?;
    wait_for_motors_online(&mut client, timeout_duration, hardwared, emulator).await?;
    require_group_state(&mut client, GroupState::Fault).await?;

    client
        .renew_lease(RenewLeaseRequest {
            token: token.clone(),
            requested_duration_ms: 10_000,
        })
        .await?;

    let recovered = client
        .recover_group(RecoverGroupRequest {
            group: GROUP.into(),
            lease_token: token.clone(),
        })
        .await?
        .into_inner();
    require_state(recovered.state, GroupState::Idle)?;

    let reenabled = client
        .enable_group(EnableGroupRequest {
            group: GROUP.into(),
            lease_token: token.clone(),
        })
        .await?
        .into_inner();
    require_state(reenabled.state, GroupState::Enabled)?;

    let timeout_commands = CommandStream::start(client.clone(), token.clone()).await?;
    let restarted = client
        .start_control(StartControlRequest {
            group: GROUP.into(),
            lease_token: token.clone(),
        })
        .await?
        .into_inner();
    require_state(restarted.state, GroupState::Running)?;
    drop(timeout_commands);
    let timeout_fault = wait_for_fault(
        &mut telemetry,
        timeout_duration,
        FaultCause::CommandTimeout,
        hardwared,
        Some(emulator),
    )
    .await?;

    let recovered = client
        .recover_group(RecoverGroupRequest {
            group: GROUP.into(),
            lease_token: token.clone(),
        })
        .await?
        .into_inner();
    require_state(recovered.state, GroupState::Idle)?;
    let enabled = client
        .enable_group(EnableGroupRequest {
            group: GROUP.into(),
            lease_token: token.clone(),
        })
        .await?
        .into_inner();
    require_state(enabled.state, GroupState::Enabled)?;

    let disabled = client
        .disable_group(DisableGroupRequest {
            group: GROUP.into(),
            lease_token: token.clone(),
        })
        .await?
        .into_inner();
    require_state(disabled.state, GroupState::Idle)?;

    let released = client
        .release_lease(ReleaseLeaseRequest { token })
        .await?
        .into_inner();
    require_state(released.state, GroupState::Idle)?;

    let expiring = client
        .acquire_lease(AcquireLeaseRequest {
            group: GROUP.into(),
            client_name: "bello-hardware-e2e-expiry".into(),
            requested_duration_ms: 300,
        })
        .await?
        .into_inner();
    let enabled = client
        .enable_group(EnableGroupRequest {
            group: GROUP.into(),
            lease_token: expiring.token.clone(),
        })
        .await?
        .into_inner();
    require_state(enabled.state, GroupState::Enabled)?;
    let _lease_commands = CommandStream::start(client.clone(), expiring.token.clone()).await?;
    let started = client
        .start_control(StartControlRequest {
            group: GROUP.into(),
            lease_token: expiring.token,
        })
        .await?
        .into_inner();
    require_state(started.state, GroupState::Running)?;
    let lease_fault = wait_for_fault(
        &mut telemetry,
        timeout_duration,
        FaultCause::LeaseLoss,
        hardwared,
        Some(emulator),
    )
    .await?;

    hardwared.ensure_running()?;
    emulator.ensure_running()?;
    println!("vcan integration passed: {feedback_fault}; {timeout_fault}; {lease_fault}");
    Ok(())
}

struct CommandStream {
    sender: JoinHandle<()>,
    receiver: JoinHandle<()>,
}

impl CommandStream {
    async fn start(mut client: Client, token: String) -> Result<Self> {
        let (tx, rx) = mpsc::channel(8);
        let mut responses = client
            .stream_commands(ReceiverStream::new(rx))
            .await?
            .into_inner();
        tx.send(command(&token, 1)).await?;
        let acknowledgement = timeout(Duration::from_secs(1), responses.message())
            .await
            .context("waiting for initial command acknowledgement")??
            .ok_or_else(|| anyhow!("command response stream closed"))?;
        if acknowledgement.result != CommandResult::Accepted as i32 {
            bail!("initial command rejected: {}", acknowledgement.detail);
        }

        let sender = tokio::spawn(async move {
            let mut sequence = 2;
            let mut timer = tokio::time::interval(Duration::from_millis(20));
            loop {
                timer.tick().await;
                if tx.send(command(&token, sequence)).await.is_err() {
                    break;
                }
                sequence += 1;
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

fn command(token: &str, sequence: u64) -> StreamCommandsRequest {
    StreamCommandsRequest {
        group: GROUP.into(),
        lease_token: token.into(),
        sequence,
        valid_for_ms: 200,
        commands: MOTORS
            .into_iter()
            .map(|motor_name| MotorCommand {
                motor_name: motor_name.into(),
                position: 0.0,
                velocity: 0.0,
                torque_feedforward: 0.0,
                kp: 20.0,
                kd: 1.0,
            })
            .collect(),
    }
}

async fn wait_for_fault(
    telemetry: &mut Streaming<bello_hardwared::proto::WatchTelemetryResponse>,
    duration: Duration,
    expected: FaultCause,
    hardwared: &mut ManagedProcess,
    mut emulator: Option<&mut ManagedProcess>,
) -> Result<String> {
    let deadline = Instant::now() + duration;
    loop {
        let now = Instant::now();
        if now >= deadline {
            bail!("timed out waiting for {expected:?} group fault");
        }
        match timeout((deadline - now).min(PROCESS_POLL), telemetry.message()).await {
            Ok(result) => {
                let event =
                    result?.ok_or_else(|| anyhow!("telemetry stream closed before group fault"))?;
                if let Some(Payload::Group(group)) = event.payload
                    && group.group == GROUP
                    && group.state == GroupState::Fault as i32
                    && group.fault_cause == expected as i32
                {
                    return Ok(group.fault_detail);
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

async fn wait_for_motors_online(
    client: &mut Client,
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

async fn motors_online(client: &mut Client) -> Result<bool> {
    let inventory = client
        .list_hardware(ListHardwareRequest {})
        .await?
        .into_inner();
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

async fn require_group_state(client: &mut Client, expected: GroupState) -> Result<()> {
    let inventory = client
        .list_hardware(ListHardwareRequest {})
        .await?
        .into_inner();
    let group = inventory
        .groups
        .iter()
        .find(|group| group.name == GROUP)
        .ok_or_else(|| anyhow!("e2e group is missing from inventory"))?;
    require_state(group.state, expected)
}

fn require_state(actual: i32, expected: GroupState) -> Result<()> {
    if actual != expected as i32 {
        bail!("expected state {expected:?}, got {actual}");
    }
    Ok(())
}
