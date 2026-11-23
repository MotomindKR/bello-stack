use std::collections::HashMap;
use std::net::TcpListener;
use std::time::Duration;

use anyhow::{Context, Result, bail};
use bello_hardware_api::hardware_service_client::HardwareServiceClient;
use bello_hardware_api::watch_telemetry_response::Payload;
use bello_hardware_api::{
    AcquireLeaseRequest, CommandResult, EnableGroupRequest, ListHardwareRequest, MotorCommand,
    StartControlRequest, StopControlRequest, StreamCommandsRequest, WatchTelemetryRequest,
};
use tokio::sync::mpsc;
use tokio::time::{Instant, sleep, timeout};
use tokio_stream::wrappers::ReceiverStream;

use super::support::{self, process::ManagedProcess};

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn canonical_lower_body_enforces_coupled_ankle_limits_over_two_buses() -> Result<()> {
    const GROUP: &str = "lower_body";

    let [can0, can1] = super::sim::available_ports()?;
    let listen = TcpListener::bind(("127.0.0.1", 0))?.local_addr()?;
    let directory = tempfile::tempdir().context("creating canonical e2e directory")?;
    let hardwared_config = directory.path().join("hardwared.yaml");
    let can0_config = directory.path().join("can0-emulator.yaml");
    let can1_config = directory.path().join("can1-emulator.yaml");
    let fixture = support::config::write_canonical_lower_body(
        &hardwared_config,
        [(&can0_config, &can0), (&can1_config, &can1)],
        listen,
    )?;

    let mut emulator_can0 = ManagedProcess::spawn(
        "robstride-emulator-can0",
        env!("CARGO_BIN_EXE_robstride-emulator-e2e-fixture"),
        [&can0_config],
    )?;
    let mut emulator_can1 = ManagedProcess::spawn(
        "robstride-emulator-can1",
        env!("CARGO_BIN_EXE_robstride-emulator-e2e-fixture"),
        [&can1_config],
    )?;
    let mut hardwared = ManagedProcess::spawn(
        "bello-hardwared",
        env!("CARGO_BIN_EXE_hardwared-e2e-fixture"),
        [&hardwared_config],
    )?;
    let endpoint = format!("http://{listen}");
    let mut client = connect_canonical(
        &endpoint,
        Duration::from_secs(5),
        &mut hardwared,
        [&mut emulator_can0, &mut emulator_can1],
    )
    .await?;
    wait_for_named_motors_online(&mut client, &fixture.motor_names, Duration::from_secs(5)).await?;

    let lease = client
        .acquire_lease(AcquireLeaseRequest {
            group: GROUP.into(),
            client_name: "canonical-lower-body-e2e".into(),
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
    wait_for_named_motors_online(&mut client, &fixture.motor_names, Duration::from_secs(5)).await?;

    let mut telemetry = client
        .watch_telemetry(WatchTelemetryRequest {
            groups: vec![GROUP.into()],
            include_motors: true,
            include_imu: false,
            include_health: false,
            imus: Vec::new(),
            include_command_policy_events: true,
            include_power_boards: false,
            power_boards: vec![],
        })
        .await?
        .into_inner();
    let initial_positions =
        wait_for_motor_positions(&mut telemetry, &fixture.motor_names, Duration::from_secs(1))
            .await?;
    let initial = canonical_lower_body_command(
        &lease.token,
        1,
        &fixture.motor_names,
        &initial_positions,
        0.0,
    );
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

    for (sequence, ankle_position) in [(2, 0.04), (3, 0.08)] {
        command_tx
            .send(canonical_lower_body_command(
                &lease.token,
                sequence,
                &fixture.motor_names,
                &initial_positions,
                ankle_position,
            ))
            .await?;
        support::scenario::require_accepted(&mut responses, sequence).await?;
    }
    command_tx
        .send(canonical_lower_body_command(
            &lease.token,
            4,
            &fixture.motor_names,
            &initial_positions,
            0.20,
        ))
        .await?;
    let rejected = timeout(Duration::from_secs(1), responses.message())
        .await
        .context("waiting for coupled-limit command acknowledgement")??
        .context("command response stream closed")?;
    if rejected.sequence != 4 || rejected.result != CommandResult::Unsafe as i32 {
        bail!(
            "coupled ankle violation was not rejected as unsafe: sequence={}, result={}, detail={}",
            rejected.sequence,
            rejected.result,
            rejected.detail
        );
    }
    wait_for_command_policy_detail(
        &mut telemetry,
        "left_ankle_roll_joint",
        Duration::from_secs(1),
    )
    .await?;

    let inventory = client
        .list_hardware(ListHardwareRequest {})
        .await?
        .into_inner();
    let lower = inventory
        .groups
        .iter()
        .find(|group| group.name == GROUP)
        .context("canonical lower_body group is missing")?;
    if lower.state != bello_hardware_api::GroupState::Running as i32 {
        bail!(
            "coupled command rejection changed group state to {}",
            lower.state
        );
    }
    client
        .stop_control(StopControlRequest {
            group: GROUP.into(),
            lease_token: lease.token,
        })
        .await?;
    hardwared.ensure_running()?;
    emulator_can0.ensure_running()?;
    emulator_can1.ensure_running()?;
    Ok(())
}
fn canonical_lower_body_command(
    token: &str,
    sequence: u64,
    motor_names: &[String],
    initial_positions: &HashMap<String, f64>,
    left_ankle_offset: f64,
) -> StreamCommandsRequest {
    StreamCommandsRequest {
        group: "lower_body".into(),
        lease_token: token.into(),
        sequence,
        valid_for_ms: 500,
        commands: motor_names
            .iter()
            .map(|motor_name| MotorCommand {
                motor_name: motor_name.clone(),
                position: initial_positions[motor_name]
                    + matches!(motor_name.as_str(), "left_ankle_1" | "left_ankle_2")
                        .then_some(left_ankle_offset)
                        .unwrap_or(0.0),
                velocity: 0.0,
                torque_feedforward: 0.0,
                kp: 0.0,
                kd: 0.0,
            })
            .collect(),
    }
}

async fn wait_for_motor_positions(
    telemetry: &mut tonic::Streaming<bello_hardware_api::WatchTelemetryResponse>,
    motor_names: &[String],
    duration: Duration,
) -> Result<HashMap<String, f64>> {
    let deadline = Instant::now() + duration;
    let mut positions = HashMap::with_capacity(motor_names.len());
    while positions.len() < motor_names.len() {
        let remaining = deadline.saturating_duration_since(Instant::now());
        if remaining.is_zero() {
            bail!("timed out waiting for complete initial motor feedback");
        }
        let event = timeout(remaining, telemetry.message())
            .await
            .context("waiting for initial motor feedback")??
            .context("telemetry stream closed")?;
        if let Some(Payload::Motor(motor)) = event.payload
            && motor_names.contains(&motor.motor_name)
        {
            positions.insert(motor.motor_name, motor.position);
        }
    }
    Ok(positions)
}

async fn connect_canonical(
    endpoint: &str,
    duration: Duration,
    hardwared: &mut ManagedProcess,
    mut emulators: [&mut ManagedProcess; 2],
) -> Result<HardwareServiceClient<tonic::transport::Channel>> {
    let deadline = Instant::now() + duration;
    loop {
        hardwared.ensure_running()?;
        for emulator in &mut emulators {
            emulator.ensure_running()?;
        }
        match HardwareServiceClient::connect(endpoint.to_owned()).await {
            Ok(client) => return Ok(client),
            Err(error) if Instant::now() < deadline => {
                let _ = error;
                sleep(Duration::from_millis(20)).await;
            }
            Err(error) => return Err(error).context("connecting to canonical hardwared fixture"),
        }
    }
}

async fn wait_for_named_motors_online(
    client: &mut HardwareServiceClient<tonic::transport::Channel>,
    motor_names: &[String],
    duration: Duration,
) -> Result<()> {
    let deadline = Instant::now() + duration;
    loop {
        let inventory = client
            .list_hardware(ListHardwareRequest {})
            .await?
            .into_inner();
        if motor_names.iter().all(|expected| {
            inventory.motors.iter().any(|motor| {
                motor.name == *expected
                    && motor.status.as_ref().is_some_and(|status| {
                        status.state == bello_hardware_api::DeviceState::Online as i32
                    })
            })
        }) {
            return Ok(());
        }
        if Instant::now() >= deadline {
            let unavailable = inventory
                .motors
                .iter()
                .filter(|motor| motor_names.contains(&motor.name))
                .filter(|motor| {
                    motor.status.as_ref().is_none_or(|status| {
                        status.state != bello_hardware_api::DeviceState::Online as i32
                    })
                })
                .map(|motor| {
                    let status = motor.status.as_ref();
                    format!(
                        "{}={} ({})",
                        motor.name,
                        status.map_or(0, |value| value.state),
                        status.map_or("missing status", |value| value.detail.as_str())
                    )
                })
                .collect::<Vec<_>>()
                .join(", ");
            bail!("canonical lower-body motors did not become online: {unavailable}");
        }
        sleep(Duration::from_millis(20)).await;
    }
}

async fn wait_for_command_policy_detail(
    telemetry: &mut tonic::Streaming<bello_hardware_api::WatchTelemetryResponse>,
    expected: &str,
    duration: Duration,
) -> Result<()> {
    let deadline = Instant::now() + duration;
    loop {
        let remaining = deadline.saturating_duration_since(Instant::now());
        let event = timeout(remaining, telemetry.message())
            .await
            .context("waiting for coupled-limit telemetry")??
            .context("telemetry stream closed")?;
        if let Some(Payload::CommandPolicy(policy)) = event.payload
            && policy.command_rejected
            && policy.detail.contains(expected)
        {
            return Ok(());
        }
    }
}
