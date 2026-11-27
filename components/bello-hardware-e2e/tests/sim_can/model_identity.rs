use super::*;
use std::sync::Arc;
use std::sync::atomic::{AtomicU8, AtomicUsize, Ordering};

use bello_hardware_api::{DeviceState, ListHardwareResponse};
use robstride_protocol::{Request, feedback_motor_id, firmware_version_reply, parse_request};

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn whole_group_waits_for_matching_motor_models_after_an_unplug() -> Result<()> {
    let [host, peer] = sim::available_ports()?;
    let host_socket = serde_yaml::from_str::<SimBus>(&format!("{{ {host} }}"))?.open_simulator()?;
    let peer_socket = serde_yaml::from_str::<SimBus>(&format!("{{ {peer} }}"))?.open_daemon()?;
    host_socket.set_nonblocking(true)?;
    peer_socket.set_nonblocking(true)?;
    let unplugged = 255;
    let no_reply = 254;
    let family = Arc::new(AtomicU8::new(no_reply));
    let enables = Arc::new(AtomicUsize::new(0));
    let reply_family = family.clone();
    let count = enables.clone();
    let bridge = tokio::spawn(async move {
        loop {
            while let Ok(frame) = host_socket.recv() {
                if matches!(parse_request(&frame), Some(Request::Enable { .. })) {
                    count.fetch_add(1, Ordering::Relaxed);
                }
                peer_socket.send(frame).unwrap();
            }
            while let Ok(mut frame) = peer_socket.recv() {
                if feedback_motor_id(&frame) == Some(2) {
                    let family = reply_family.load(Ordering::Relaxed);
                    if family == unplugged {
                        continue;
                    }
                    if firmware_version_reply(&frame).is_some() {
                        if family == no_reply {
                            continue;
                        }
                        frame.data[4] = family;
                    }
                }
                host_socket.send(frame).unwrap();
            }
            sleep(Duration::from_millis(1)).await;
        }
    });

    let (_directory, endpoint, mut daemon, mut emulator) =
        spawn_pair_with_transports(&host, &peer)?;
    let client = support::scenario::connect(
        &endpoint,
        Duration::from_secs(5),
        &mut daemon,
        &mut emulator,
    )
    .await?;
    let lease = acquire(&client, "e2e_group", "model-check", 30000).await?;
    let wait_inventory =
        async |expected: Option<&str>, detail: &str| -> Result<ListHardwareResponse> {
            timeout(Duration::from_secs(8), async {
                loop {
                    let inventory = inventory(&client).await?;
                    if inventory.motors.iter().any(|m| {
                        m.address.as_ref().unwrap().id == 1
                            && m.firmware_version.as_deref() == Some("0.3.0.1")
                    }) && inventory.motors.iter().any(|m| {
                        m.address.as_ref().unwrap().id == 2
                            && m.firmware_version.as_deref() == expected
                            && m.status.as_ref().unwrap().detail.contains(detail)
                    }) {
                        return anyhow::Ok(inventory);
                    }
                    sleep(Duration::from_millis(10)).await;
                }
            })
            .await?
        };
    for (response, expected, detail) in [
        (no_reply, None, "waiting for firmware identification"),
        (
            6,
            Some("0.6.0.1"),
            "does not identify configured model RS_03",
        ),
    ] {
        family.store(response, Ordering::Relaxed);
        let current = wait_inventory(expected, detail).await?;
        assert_ne!(
            current.motors[1].status.as_ref().unwrap().state,
            DeviceState::Online as i32
        );
        let error = enable(&client, &lease).await.unwrap_err().to_string();
        assert!(
            error.contains("e2e_motor_2") && error.contains("RS_03"),
            "{error}"
        );
        assert_eq!(
            inventory(&client).await?.groups[0].state,
            GroupState::Idle as i32
        );
        assert_eq!(enables.load(Ordering::Relaxed), 0);
    }
    family.store(unplugged, Ordering::Relaxed);
    wait_inventory(None, "").await?;
    family.store(3, Ordering::Relaxed);
    wait_inventory(Some("0.3.0.1"), "healthy motor feedback").await?;
    assert_eq!(enable(&client, &lease).await?, GroupState::Damped);
    disable(&client, &lease).await?;
    let before_unplug = enables.load(Ordering::Relaxed);
    assert!(before_unplug >= 2);

    family.store(unplugged, Ordering::Relaxed);
    wait_inventory(None, "").await?;
    family.store(no_reply, Ordering::Relaxed);
    wait_inventory(None, "waiting for firmware identification").await?;
    assert!(enable(&client, &lease).await.is_err());
    assert_eq!(enables.load(Ordering::Relaxed), before_unplug);
    bridge.abort();
    Ok(())
}
