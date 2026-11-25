use super::*;
use bello_hardware_api::motor_configuration_service_client::MotorConfigurationServiceClient;
use bello_hardware_api::{
    CancelMotorConfigurationRequest, CommitMotorConfigurationRequest, GetMotorConfigurationRequest,
    GetMotorConfigurationResponse, MotorConfigurationKind as Kind,
    MotorConfigurationPhase as Phase, PrepareMotorConfigurationRequest,
};
use tonic::transport::Channel;

async fn wait_phase(
    api: &mut MotorConfigurationServiceClient<Channel>,
    id: &str,
    wanted: Phase,
) -> Result<GetMotorConfigurationResponse> {
    timeout(Duration::from_secs(15), async {
        loop {
            let job = api
                .get_motor_configuration(GetMotorConfigurationRequest { id: id.into() })
                .await?
                .into_inner();
            if job.phase() == wanted {
                return Ok(job);
            }
            anyhow::ensure!(
                matches!(
                    job.phase(),
                    Phase::Preparing | Phase::Ready | Phase::Writing
                ),
                "unexpected configuration result {job:?}"
            );
            sleep(Duration::from_millis(10)).await;
        }
    })
    .await?
}

async fn wait_for_versions(client: &HardwareClient, target: &str) -> Result<()> {
    timeout(Duration::from_secs(8), async {
        loop {
            let inventory = client.inventory().await?;
            let version = |name| {
                inventory
                    .motors
                    .iter()
                    .find(|motor| motor.name == name)
                    .and_then(|motor| motor.firmware_version.as_deref())
            };
            if version("e2e_motor_1") == Some(target) && version("e2e_motor_2") == Some("0.3.0.1") {
                return anyhow::Ok(());
            }
            sleep(Duration::from_millis(20)).await;
        }
    })
    .await?
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn re_zero_and_update_with_another_group_on_the_bus() -> Result<()> {
    let [ports] = sim::available_ports()?;
    let directory = tempfile::tempdir()?;
    let deployment = directory.path().join("hardwared.yaml");
    let emulator_config = directory.path().join("emulator.yaml");
    let grpc = TcpListener::bind(("127.0.0.1", 0))?.local_addr()?;
    support::config::write_hardwared_with_grpc(
        &deployment,
        &format!("{{ type: sim, {ports} }}"),
        &format!("  listen: [{{ type: tcp, address: {grpc} }}]"),
        100.0,
    )?;
    let description = directory.path().join("e2e-hardware.yaml");
    let mut yaml: serde_yaml::Value = serde_yaml::from_reader(fs::File::open(&description)?)?;
    let groups = yaml["motor_groups"].as_sequence_mut().unwrap();
    let mut peer = groups[0].clone();
    groups[0]["motors"] = serde_yaml::to_value(["e2e_motor_1"])?;
    peer["name"] = "peer".into();
    peer["motors"] = serde_yaml::to_value(["e2e_motor_2"])?;
    groups.push(peer);
    fs::write(description, serde_yaml::to_string(&yaml)?)?;
    let mut yaml: serde_yaml::Value = serde_yaml::from_reader(fs::File::open(&deployment)?)?;
    yaml["enabled_groups"] = serde_yaml::to_value(["e2e_group", "peer"])?;
    fs::write(&deployment, serde_yaml::to_string(&yaml)?)?;
    let mut yaml: serde_yaml::Value =
        serde_yaml::from_str(&support::config::emulator(&format!("{{ {ports} }}")))?;
    yaml["actuators"][0]["initial_position"] = 0.4.into();
    fs::write(&emulator_config, serde_yaml::to_string(&yaml)?)?;

    let mut emulator = ManagedProcess::spawn(
        "emulator",
        env!("CARGO_BIN_EXE_robstride-emulator-e2e-fixture"),
        [&emulator_config],
    )?;
    let mut daemon = ManagedProcess::spawn(
        "hardwared",
        env!("CARGO_BIN_EXE_hardwared-e2e-fixture"),
        [&deployment],
    )?;
    let endpoint = format!("http://{grpc}");
    let deadline = Duration::from_secs(8);
    let client =
        support::scenario::connect(&endpoint, deadline, &mut daemon, &mut emulator).await?;
    support::scenario::wait_for_motors_online(&client, deadline, &mut daemon, &mut emulator)
        .await?;
    let mut api = MotorConfigurationServiceClient::connect(endpoint.clone()).await?;
    let request = |kind: Kind| PrepareMotorConfigurationRequest {
        motor: "e2e_motor_1".into(),
        kind: kind as i32,
    };
    let position_is = |frame: &GetStateFrameResponse, position: f64| {
        frame
            .motors
            .iter()
            .any(|m| m.motor_name == "e2e_motor_1" && (m.position - position).abs() < 0.02)
    };
    wait_for_frame(&client, |f| position_is(f, 0.4)).await?;
    wait_for_versions(&client, "0.3.0.1").await?;
    let mut peer_lease = client
        .acquire_lease(
            "peer",
            "operator",
            LeaseDuration::For(Duration::from_secs(5)),
        )
        .await?;
    assert_eq!(
        api.prepare_motor_configuration(request(Kind::Zero))
            .await
            .unwrap_err()
            .code(),
        tonic::Code::FailedPrecondition
    );
    peer_lease.release().await?;
    let mut uuid = Vec::new();
    for (kind, cancel) in [
        (Kind::Zero, true),
        (Kind::Zero, false),
        (Kind::Firmware, false),
        (Kind::Zero, true),
    ] {
        let id = api
            .prepare_motor_configuration(request(kind))
            .await?
            .into_inner()
            .id;
        let reviewed = wait_phase(&mut api, &id, Phase::Ready).await?;
        assert!(
            client
                .inventory()
                .await?
                .groups
                .iter()
                .all(|g| g.state == GroupState::Configuring as i32)
        );
        assert!(
            client
                .acquire_lease("peer", "other", LeaseDuration::For(Duration::from_secs(1)))
                .await
                .is_err()
        );
        if uuid.is_empty() {
            uuid.clone_from(&reviewed.motor_uuid);
        }
        assert_eq!(reviewed.motor_uuid, uuid);
        if cancel {
            api.cancel_motor_configuration(CancelMotorConfigurationRequest { id: id.clone() })
                .await?;
            wait_phase(&mut api, &id, Phase::Cancelled).await?;
            if reviewed.current_version == "0.3.0.1" {
                wait_for_frame(&client, |f| position_is(f, 0.4)).await?;
            } else {
                assert_eq!(reviewed.current_version, "0.3.0.2");
            }
        } else {
            api.commit_motor_configuration(CommitMotorConfigurationRequest { id: id.clone() })
                .await?;
            drop(api);
            api = MotorConfigurationServiceClient::connect(endpoint.clone()).await?;
            api.commit_motor_configuration(CommitMotorConfigurationRequest { id: id.clone() })
                .await?;
            let job = wait_phase(&mut api, &id, Phase::Succeeded).await?;
            if kind == Kind::Firmware {
                wait_for_versions(&client, "0.3.0.2").await?;
                assert_eq!(job.current_version, "0.3.0.1");
                assert_eq!(job.target_version.as_deref(), Some("0.3.0.2"));
            }
            wait_for_frame(&client, |f| position_is(f, 0.0)).await?;
        }
        assert!(
            client
                .inventory()
                .await?
                .groups
                .iter()
                .all(|g| g.state == GroupState::Idle as i32)
        );
    }
    Ok(())
}
