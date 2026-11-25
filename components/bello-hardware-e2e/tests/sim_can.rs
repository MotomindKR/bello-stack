#![cfg(unix)]

#[allow(dead_code)]
mod support;

#[path = "support/sim.rs"]
mod sim;

#[path = "sim_can/lower_body.rs"]
mod lower_body;

#[path = "sim_can/configuration.rs"]
mod configuration;

use std::fs;
use std::net::TcpListener;
use std::time::Duration;

use anyhow::{Context, Result};
use bello_hardware_api::{GetStateFrameRequest, GetStateFrameResponse, GroupState, MotorCommand};
use bello_hardware_client::raw::{CommandValidity, EnumValue, HardwareClient, LeaseDuration};
use bello_sim_can::SimBus;
use support::process::ManagedProcess;
use tokio::time::{sleep, timeout};

#[tokio::test]
async fn queued_sim_feedback_keeps_its_age() -> Result<()> {
    let [ports] = sim::available_ports()?;
    let bus: SimBus = serde_yaml::from_str(&format!("{{ {ports} }}"))?;
    let (_directory, endpoint, mut daemon, mut emulator) = spawn_pair_on_ports(&ports)?;
    support::scenario::queued_feedback_keeps_its_age(
        &endpoint,
        &mut daemon,
        &mut emulator,
        |frames| {
            let sender = bus.open_simulator()?;
            for frame in frames {
                sender.send(frame)?;
            }
            Ok(())
        },
    )
    .await
}

#[tokio::test]
async fn simulated_can_delivers_motion_and_faults_when_emulator_stops() -> Result<()> {
    let (_directory, endpoint, mut daemon, mut emulator) = spawn_pair()?;
    let deadline = Duration::from_secs(5);
    let client =
        support::scenario::connect(&endpoint, deadline, &mut daemon, &mut emulator).await?;
    support::scenario::wait_for_motors_online(&client, deadline, &mut daemon, &mut emulator)
        .await?;
    let group = "e2e_group";
    let lease = client
        .acquire_lease(
            group,
            "sim-can-test",
            LeaseDuration::For(Duration::from_secs(30)),
        )
        .await?;
    lease.enable().await?;
    let commands = [("e2e_motor_1", 0.1), ("e2e_motor_2", -0.1)]
        .into_iter()
        .map(|(motor, position)| MotorCommand {
            motor_name: motor.into(),
            position,
            kp: 20.0,
            kd: 1.0,
            ..Default::default()
        })
        .collect();
    lease
        .start(CommandValidity::For(Duration::from_millis(200)), commands)
        .await?;
    wait_for_frame(&client, |frame| {
        frame
            .motors
            .iter()
            .any(|m| m.motor_name == "e2e_motor_1" && m.position > 0.01)
            && frame
                .motors
                .iter()
                .any(|m| m.motor_name == "e2e_motor_2" && m.position < -0.01)
    })
    .await
    .context("waiting for commanded motor motion")?;
    emulator.stop();
    timeout(deadline, async {
        loop {
            if client.inventory().await?.groups[0].state == GroupState::Fault as i32 {
                return anyhow::Ok(());
            }
            sleep(Duration::from_millis(10)).await;
        }
    })
    .await
    .context("losing the simulator did not fault the active group")??;
    let before_restart = client
        .grpc_client()
        .get_state_frame(GetStateFrameRequest::default())
        .await?
        .into_inner()
        .captured_monotonic_ns;
    emulator.restart()?;
    wait_for_frame(&client, |frame| {
        frame.motors.len() == 2
            && frame
                .motors
                .iter()
                .all(|m| m.sample_monotonic_ns > before_restart)
    })
    .await
    .context("waiting for fresh feedback from the restarted emulator")?;
    assert_eq!(
        client.inventory().await?.groups[0].state,
        GroupState::Fault as i32
    );
    assert_eq!(lease.recover().await?, EnumValue::Known(GroupState::Idle));
    Ok(())
}

async fn wait_for_frame(
    client: &HardwareClient,
    ready: impl Fn(&GetStateFrameResponse) -> bool,
) -> Result<()> {
    timeout(Duration::from_secs(5), async {
        loop {
            match client
                .grpc_client()
                .get_state_frame(GetStateFrameRequest::default())
                .await
            {
                Ok(frame) if ready(frame.get_ref()) => return anyhow::Ok(()),
                Ok(_) => (),
                Err(status) if status.code() == tonic::Code::FailedPrecondition => (),
                Err(status) => return Err(status.into()),
            }
            sleep(Duration::from_millis(10)).await;
        }
    })
    .await?
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn control_fault_and_recovery() -> Result<()> {
    let (_directory, endpoint, mut daemon, mut emulator) = spawn_pair()?;
    let deadline = Duration::from_secs(8);
    let client =
        support::scenario::connect(&endpoint, deadline, &mut daemon, &mut emulator).await?;
    support::scenario::run(client, &endpoint, deadline, &mut daemon, &mut emulator).await
}

fn spawn_pair() -> Result<(tempfile::TempDir, String, ManagedProcess, ManagedProcess)> {
    let [ports] = sim::available_ports()?;
    spawn_pair_on_ports(&ports)
}

fn spawn_pair_on_ports(
    ports: &str,
) -> Result<(tempfile::TempDir, String, ManagedProcess, ManagedProcess)> {
    let directory = tempfile::tempdir()?;
    let deployment = directory.path().join("hardwared.yaml");
    let emulator_config = directory.path().join("emulator.yaml");
    let grpc = TcpListener::bind(("127.0.0.1", 0))?.local_addr()?;
    support::config::write_hardwared_with_grpc(
        &deployment,
        &format!("{{ type: sim, {ports} }}"),
        &format!("  listen: [{{ type: tcp, address: {grpc} }}]\n  maximum_lease_ms: 30000"),
        250.0,
    )?;
    fs::write(
        &emulator_config,
        support::config::emulator(&format!("{{ {ports} }}")),
    )?;
    let daemon = ManagedProcess::spawn(
        "hardwared",
        env!("CARGO_BIN_EXE_hardwared-e2e-fixture"),
        [&deployment],
    )?;
    let emulator = ManagedProcess::spawn(
        "emulator",
        env!("CARGO_BIN_EXE_robstride-emulator-e2e-fixture"),
        [&emulator_config],
    )?;
    Ok((directory, format!("http://{grpc}"), daemon, emulator))
}
