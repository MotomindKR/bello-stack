use std::time::Duration;

use anyhow::{Context, Result, anyhow, bail};
use bello_hardwared::proto::hardware_service_client::HardwareServiceClient;
use bello_hardwared::proto::watch_telemetry_response::Payload;
use bello_hardwared::proto::{
    AcquireLeaseRequest, DeviceState, DisableGroupRequest, EnableGroupRequest, ListHardwareRequest,
    MotorCommand, RecoverGroupRequest, ReleaseLeaseRequest, StartControlRequest,
    StreamCommandsRequest, WatchTelemetryRequest,
};
use tokio::sync::mpsc;
use tokio::task::JoinHandle;
use tokio::time::{Instant, sleep, timeout, timeout_at};
use tokio_stream::wrappers::ReceiverStream;
use tonic::Streaming;
use tonic::transport::Channel;

const GROUP: &str = "e2e_group";
const MOTORS: [&str; 2] = ["e2e_motor_1", "e2e_motor_2"];

type Client = HardwareServiceClient<Channel>;

pub async fn connect(endpoint: &str, deadline: Duration) -> Result<Client> {
    let deadline = Instant::now() + deadline;
    loop {
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

pub async fn run(mut client: Client, timeout_duration: Duration) -> Result<()> {
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
    require_transition(enabled.success, &enabled.state, "Enabled", &enabled.message)?;
    require_motors_online(&mut client).await?;

    let command_stream = CommandStream::start(client.clone(), token.clone()).await?;
    let started = client
        .start_control(StartControlRequest {
            group: GROUP.into(),
            lease_token: token.clone(),
        })
        .await?
        .into_inner();
    require_transition(started.success, &started.state, "Running", &started.message)?;

    let fault_reason = wait_for_fault(&mut telemetry, timeout_duration).await?;
    if !fault_reason.contains("unavailable") && !fault_reason.contains("feedback") {
        bail!("unexpected feedback-loss fault reason: {fault_reason}");
    }
    drop(command_stream);

    wait_for_motors_online(&mut client, timeout_duration).await?;
    require_group_state(&mut client, "Fault").await?;

    let recovered = client
        .recover_group(RecoverGroupRequest {
            group: GROUP.into(),
            lease_token: token.clone(),
        })
        .await?
        .into_inner();
    require_transition(
        recovered.success,
        &recovered.state,
        "Idle",
        &recovered.message,
    )?;

    let reenabled = client
        .enable_group(EnableGroupRequest {
            group: GROUP.into(),
            lease_token: token.clone(),
        })
        .await?
        .into_inner();
    require_transition(
        reenabled.success,
        &reenabled.state,
        "Enabled",
        &reenabled.message,
    )?;

    let disabled = client
        .disable_group(DisableGroupRequest {
            group: GROUP.into(),
            lease_token: token.clone(),
        })
        .await?
        .into_inner();
    require_transition(disabled.success, &disabled.state, "Idle", &disabled.message)?;

    let released = client
        .release_lease(ReleaseLeaseRequest { token })
        .await?
        .into_inner();
    if !released.success {
        bail!("lease release failed: {}", released.message);
    }
    println!("vcan end-to-end scenario passed: {fault_reason}");
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
        if !acknowledgement.accepted {
            bail!("initial command rejected: {}", acknowledgement.message);
        }

        let sender_token = token;
        let sender = tokio::spawn(async move {
            let mut sequence = 2;
            let mut timer = tokio::time::interval(Duration::from_millis(20));
            loop {
                timer.tick().await;
                if tx.send(command(&sender_token, sequence)).await.is_err() {
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
) -> Result<String> {
    let deadline = Instant::now() + duration;
    loop {
        let event = timeout_at(deadline, telemetry.message())
            .await
            .context("waiting for feedback-loss group fault")??
            .ok_or_else(|| anyhow!("telemetry stream closed before group fault"))?;
        if let Some(Payload::Group(group)) = event.payload
            && group.group == GROUP
            && group.state == "Fault"
        {
            return Ok(group.fault_reason);
        }
    }
}

async fn wait_for_motors_online(client: &mut Client, duration: Duration) -> Result<()> {
    let deadline = Instant::now() + duration;
    loop {
        if motors_online(client).await? {
            return Ok(());
        }
        if Instant::now() >= deadline {
            bail!("motors did not recover after emulator feedback resumed");
        }
        sleep(Duration::from_millis(20)).await;
    }
}

async fn require_motors_online(client: &mut Client) -> Result<()> {
    if !motors_online(client).await? {
        bail!("not every e2e motor is online after group enable");
    }
    Ok(())
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

async fn require_group_state(client: &mut Client, expected: &str) -> Result<()> {
    let inventory = client
        .list_hardware(ListHardwareRequest {})
        .await?
        .into_inner();
    let group = inventory
        .groups
        .iter()
        .find(|group| group.name == GROUP)
        .ok_or_else(|| anyhow!("e2e group is missing from inventory"))?;
    if group.state != expected {
        bail!("expected group state {expected}, got {}", group.state);
    }
    Ok(())
}

fn require_transition(success: bool, actual: &str, expected: &str, message: &str) -> Result<()> {
    if !success || actual != expected {
        bail!("expected transition to {expected}, got {actual}: {message}");
    }
    Ok(())
}
