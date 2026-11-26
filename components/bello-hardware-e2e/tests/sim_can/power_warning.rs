use super::*;
use bello_hardware_api::{AcquireLeaseRequest, GroupCause};
use serialport::SerialPort;
use std::io::Write;

#[tokio::test]
async fn cog_overcurrent_damps_running_motors_and_revokes_control() -> Result<()> {
    let (mut board, uart) = serialport::TTYPort::pair()?;
    let port = uart.name().context("PTY has no name")?;
    drop(uart);
    let [ports] = sim::available_ports()?;
    let (_directory, endpoint, mut daemon, mut emulator) =
        spawn_pair_with_config(&ports, &ports, |dir| {
            let mut description = fs::OpenOptions::new()
                .append(true)
                .open(dir.join("e2e-hardware.yaml"))?;
            writeln!(
                description,
                "
serial_devices:
  - {{name: uart, baud_rate: 921600}}
power_boards:
  - {{name: agx, profile: agx, serial_device: uart, sync_period_ms: 0, stale_timeout_ms: 3000}}"
            )?;
            writeln!(
                fs::OpenOptions::new()
                    .append(true)
                    .open(dir.join("hardwared.yaml"))?,
                "\nserial_ports: {{uart: {port}}}\nenabled_power_boards: [agx]"
            )?;
            Ok(())
        })?;
    let deadline = Duration::from_secs(5);
    let client =
        support::scenario::connect(&endpoint, deadline, &mut daemon, &mut emulator).await?;
    support::scenario::wait_for_motors_online(&client, deadline, &mut daemon, &mut emulator)
        .await?;
    let group = "e2e_group";
    let lease = client
        .acquire_lease(group, "operator", LeaseDuration::For(deadline))
        .await?;
    lease.enable().await?;
    let commands = ["e2e_motor_1", "e2e_motor_2"].map(|name| MotorCommand {
        motor_name: name.into(),
        position: 0.05,
        kp: 10.,
        kd: 1.,
        ..Default::default()
    });
    assert_eq!(
        lease
            .start(
                CommandValidity::For(Duration::from_millis(200)),
                commands.into()
            )
            .await?,
        EnumValue::Known(GroupState::Running)
    );

    let iwc = |warning: u32| {
        let payload = [0, warning, 0].map(u32::to_le_bytes).concat();
        cog_protocol::encode(cog_protocol::Profile::Agx, 3, 0, &payload)
    };
    board.write_all(&iwc(1 << 4)?)?;
    wait_for_frame(&client, |frame| {
        frame.groups.iter().any(|g| {
            g.group == group
                && g.state == GroupState::Damped as i32
                && g.cause == GroupCause::Stop as i32
        })
    })
    .await?;
    let mut api = client.grpc_client();
    let takeover = AcquireLeaseRequest {
        group: group.into(),
        client_name: "replacement".into(),
        requested_duration_ms: 5000,
    };
    assert_eq!(
        api.acquire_lease(takeover.clone())
            .await
            .unwrap_err()
            .code(),
        tonic::Code::Unavailable
    );
    board.write_all(&iwc(0)?)?;
    timeout(deadline, async {
        loop {
            match api.acquire_lease(takeover.clone()).await {
                Ok(_) => return anyhow::Ok(()),
                Err(e) if e.code() == tonic::Code::Unavailable => {
                    sleep(Duration::from_millis(10)).await
                }
                Err(e) => return Err(e.into()),
            }
        }
    })
    .await??;
    assert!(lease.renew(LeaseDuration::For(deadline)).await.is_err());
    Ok(())
}
