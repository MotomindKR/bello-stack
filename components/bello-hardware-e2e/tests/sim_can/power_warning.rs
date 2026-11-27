use super::*;
use bello_hardware_api::{AcquireLeaseRequest, GroupCause, RenewLeaseRequest};
use std::io::Write;

#[tokio::test]
async fn cog_overcurrent_damps_running_motors_and_revokes_control() -> Result<()> {
    let pty = nix::pty::openpty(None, None)?;
    let port = nix::unistd::ttyname(&pty.slave)?;
    let mut board = fs::File::from(pty.master);
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
                "\nsimulation: true\nserial_ports: {{uart: {}}}\nenabled_power_boards: [agx]",
                port.display()
            )?;
            Ok(())
        })?;
    let deadline = Duration::from_secs(5);
    let mut client =
        support::scenario::connect(&endpoint, deadline, &mut daemon, &mut emulator).await?;
    support::scenario::wait_for_motors_online(&client, deadline, &mut daemon, &mut emulator)
        .await?;
    let group = "e2e_group";
    let lease = acquire(&client, group, "operator", 5000).await?;
    enable(&client, &lease).await?;
    let commands = ["e2e_motor_1", "e2e_motor_2"].map(|name| MotorCommand {
        motor_name: name.into(),
        position: 0.05,
        kp: 10.,
        kd: 1.,
        ..Default::default()
    });
    assert_eq!(
        start_control(&client, &lease, 200, commands.into()).await?,
        GroupState::Running
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
    let takeover = AcquireLeaseRequest {
        group: group.into(),
        client_name: "replacement".into(),
        requested_duration_ms: 5000,
    };
    assert_eq!(
        client
            .acquire_lease(takeover.clone())
            .await
            .unwrap_err()
            .code(),
        tonic::Code::Unavailable
    );
    board.write_all(&iwc(0)?)?;
    timeout(deadline, async {
        loop {
            match client.acquire_lease(takeover.clone()).await {
                Ok(_) => return anyhow::Ok(()),
                Err(e) if e.code() == tonic::Code::Unavailable => {
                    sleep(Duration::from_millis(10)).await
                }
                Err(e) => return Err(e.into()),
            }
        }
    })
    .await??;
    let expired = client
        .renew_lease(RenewLeaseRequest {
            token: lease.token,
            requested_duration_ms: 5000,
        })
        .await
        .unwrap_err();
    assert_eq!(expired.code(), tonic::Code::PermissionDenied);
    Ok(())
}
