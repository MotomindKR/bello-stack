use super::*;
use crate::config::{Config, ReconnectConfig, ResolvedTopology, SerialDeviceConfig};
use crate::hardware::serial::SerialManager;
use serialport::SerialPort;
use std::io::{Read, Write};

fn fixture() -> (
    serialport::TTYPort,
    SerialManager,
    SerialHandle,
    PowerBoardSpec,
    Telemetry,
) {
    let (mut master, slave) = serialport::TTYPort::pair().unwrap();
    master.set_timeout(Duration::from_millis(50)).unwrap();
    let path = slave.name().unwrap();
    drop(slave);
    let mut raw = Config::load_raw(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/config/bello-mini.yaml"
    ))
    .unwrap();
    raw.imus.clear();
    raw.serial_devices = vec![SerialDeviceConfig {
        name: "power_serial".into(),
        port: path,
        baud_rate: 921600,
        timeout_ms: 50,
        reconnect: ReconnectConfig::default(),
    }];
    raw.power_boards = vec![PowerBoardConfig {
        name: "board".into(),
        serial_device: "power_serial".into(),
        profile: Profile::Agx,
        sync_period_ms: Some(100),
        emergency_stop: EmergencyStopMode::Hardware,
        stale_timeout_ms: 500,
    }];
    let topology = ResolvedTopology::resolve(raw).unwrap();
    let spec = topology.power_boards().next().unwrap().1.clone();
    let telemetry = Telemetry::new(4096);
    let manager = SerialManager::start(&topology, telemetry.clone()).unwrap();
    let handle = manager.handle(&spec.serial_device).unwrap();
    (master, manager, handle, spec, telemetry)
}

fn await_snapshot(
    rx: &mut tokio::sync::broadcast::Receiver<crate::domain::TelemetryEvent>,
    predicate: impl Fn(&PowerTelemetry) -> bool,
) -> PowerTelemetry {
    let deadline = Instant::now() + Duration::from_secs(5);
    loop {
        while let Ok(event) = rx.try_recv() {
            if let TelemetryPayload::PowerBoard(board) = event.payload
                && predicate(&board)
            {
                return board;
            }
        }
        assert!(Instant::now() < deadline, "power snapshot timed out");
        thread::sleep(Duration::from_millis(10));
    }
}

fn read_until(master: &mut serialport::TTYPort, needle: &[u8]) -> Vec<u8> {
    let deadline = Instant::now() + Duration::from_secs(3);
    let mut received = Vec::new();
    while !received.windows(needle.len()).any(|w| w == needle) {
        let mut bytes = [0; 256];
        if let Ok(n) = master.read(&mut bytes) {
            received.extend_from_slice(&bytes[..n]);
        }
        assert!(Instant::now() < deadline, "missing UART command {needle:?}");
    }
    received
}

fn send_iwc(master: &mut serialport::TTYPort, info: u32) {
    let data = [info, 0, 0]
        .into_iter()
        .flat_map(u32::to_le_bytes)
        .collect::<Vec<_>>();
    master
        .write_all(&protocol::encode(Profile::Agx, 3, 0, &data).unwrap())
        .unwrap();
}

#[test]
fn emergency_handoff_waits_for_release_preserves_edges_and_restores_on_shutdown() {
    let (mut master, _manager, handle, mut spec, telemetry) = fixture();
    spec.emergency_stop = EmergencyStopMode::Damping;
    let (tx, rx) = std::sync::mpsc::channel();
    let worker = PowerBoardWorker::start(
        spec,
        handle,
        telemetry,
        Some(Arc::new(move |blocked| {
            let _ = tx.send(blocked);
        })),
    )
    .unwrap();
    assert!(rx.recv_timeout(Duration::from_secs(1)).unwrap());
    let setup = read_until(&mut master, b"sync\r\n");
    assert!(setup.starts_with(b"iwc emergency stop_category 0\r\niwc emergency latch 0\r\n"));
    send_iwc(&mut master, 3);
    let pressed = read_until(&mut master, b"sync\r\n");
    let host_mode = b"iwc emergency stop_category 1\r\n";
    assert!(!pressed.windows(host_mode.len()).any(|w| w == host_mode));
    assert!(rx.try_recv().is_err());
    send_iwc(&mut master, 0);
    read_until(&mut master, b"iwc emergency stop_category 1\r\n");
    assert!(!rx.recv_timeout(Duration::from_secs(1)).unwrap());
    send_iwc(&mut master, 3);
    send_iwc(&mut master, 0);
    assert!(rx.recv_timeout(Duration::from_secs(1)).unwrap());
    assert!(!rx.recv_timeout(Duration::from_secs(1)).unwrap());
    drop(worker);
    assert!(rx.recv_timeout(Duration::from_secs(1)).unwrap());
    let shutdown = read_until(&mut master, b"iwc emergency latch 1\r\n");
    let hardware_mode = b"iwc emergency stop_category 0\r\n";
    assert!(
        shutdown
            .windows(hardware_mode.len())
            .any(|w| w == hardware_mode)
    );
}

#[test]
fn fresh_other_packets_cannot_hide_stale_emergency_status() {
    let (mut master, _manager, handle, mut spec, telemetry) = fixture();
    spec.emergency_stop = EmergencyStopMode::Damping;
    let (tx, rx) = std::sync::mpsc::channel();
    let worker = PowerBoardWorker::start(
        spec,
        handle,
        telemetry,
        Some(Arc::new(move |blocked| {
            let _ = tx.send(blocked);
        })),
    )
    .unwrap();
    assert!(rx.recv_timeout(Duration::from_secs(1)).unwrap());
    read_until(&mut master, b"sync\r\n");
    send_iwc(&mut master, 0);
    read_until(&mut master, b"iwc emergency stop_category 1\r\n");
    assert!(!rx.recv_timeout(Duration::from_secs(1)).unwrap());
    for _ in 0..8 {
        let tick = protocol::encode(Profile::Agx, 8, 0, &1u32.to_le_bytes()).unwrap();
        if master.write_all(&tick).is_err() {
            break;
        }
        thread::sleep(Duration::from_millis(100));
    }
    assert!(rx.recv_timeout(Duration::from_secs(1)).unwrap());
    read_until(&mut master, b"iwc emergency stop_category 0\r\n");
    assert!(rx.try_recv().is_err());
    drop(worker);
}

#[test]
fn burst_publishes_every_sample_and_intermediate_status_transition() {
    let (mut master, _manager, handle, spec, telemetry) = fixture();
    let mut rx = telemetry.subscribe();
    let _worker = PowerBoardWorker::start(spec, handle, telemetry, None).unwrap();
    await_snapshot(&mut rx, |b| b.generation > 0);

    let battery: Vec<u8> = [48.5f32, -1.25, 73.0]
        .into_iter()
        .flat_map(f32::to_le_bytes)
        .collect();
    let iwc = |info: u32| {
        [info, 0, 0]
            .into_iter()
            .flat_map(u32::to_le_bytes)
            .collect::<Vec<_>>()
    };
    let packets = [
        (1, 10, battery.clone()),
        (1, 11, battery),
        (2, 20, 1u32.to_le_bytes().to_vec()),
        (2, 21, 0u32.to_le_bytes().to_vec()),
        (3, 30, iwc(1)),
        (3, 31, iwc(0)),
    ];
    let burst: Vec<u8> = packets
        .iter()
        .flat_map(|(kind, sequence, data)| {
            protocol::encode(Profile::Agx, *kind, *sequence, data).unwrap()
        })
        .collect();
    master.write_all(&burst).unwrap();

    for (kind, sequence, _) in packets {
        let snapshot = await_snapshot(&mut rx, |b| {
            b.samples.iter().any(|s| s.sequence == sequence)
        });
        let sample = snapshot
            .samples
            .iter()
            .find(|s| s.sequence == sequence)
            .unwrap();
        match (kind, sequence, &sample.reading) {
            (1, _, Reading::Battery { voltage, .. }) => assert_eq!(*voltage, 48.5),
            (2, 20, Reading::Gpi { bits, .. }) => assert_eq!(*bits, 1),
            (2, 21, Reading::Gpi { bits, .. }) => assert_eq!(*bits, 0),
            (3, 30, Reading::Iwc { info, .. }) => assert_eq!(*info, 1),
            (3, 31, Reading::Iwc { info, .. }) => assert_eq!(*info, 0),
            other => panic!("unexpected reading: {other:?}"),
        }
    }
}

#[test]
fn serial_transport_delivers_heartbeat_samples_corruption_and_reconnect() {
    let (mut master, _manager, handle, spec, telemetry) = fixture();
    let mut rx = telemetry.subscribe();
    let worker = PowerBoardWorker::start(spec, handle.clone(), telemetry, None).unwrap();
    let deadline = Instant::now() + Duration::from_secs(3);
    let mut received = Vec::new();
    while !received.windows(6).any(|w| w == b"sync\r\n") {
        let mut buf = [0; 128];
        if let Ok(n) = master.read(&mut buf) {
            received.extend_from_slice(&buf[..n]);
        }
        assert!(Instant::now() < deadline, "no initial AGX sync");
    }
    let data: Vec<u8> = [48.5f32, -1.25, 73.0]
        .into_iter()
        .flat_map(f32::to_le_bytes)
        .collect();
    let bytes = protocol::encode(Profile::Agx, 1, 7, &data).unwrap();
    master.write_all(&bytes[..4]).unwrap();
    thread::sleep(Duration::from_millis(30));
    master.write_all(&bytes[4..]).unwrap();
    let first = await_snapshot(&mut rx, |b| b.samples.len() == 1);
    assert!(matches!(
        first.samples[0].reading,
        Reading::Battery {
            soc_percent: Some(73.0),
            ..
        }
    ));
    master.write_all(&[2, 42, 0]).unwrap();
    let broken = await_snapshot(&mut rx, |b| b.invalid_frames > 0);
    assert_eq!(broken.samples.len(), 1);
    for _ in 0..8 {
        master
            .write_all(&protocol::encode(Profile::Agx, 8, 7, &123u32.to_le_bytes()).unwrap())
            .unwrap();
        thread::sleep(Duration::from_millis(100));
    }
    let partial = await_snapshot(&mut rx, |b| b.samples.len() == 2);
    assert!(
        partial
            .samples
            .iter()
            .find(|s| matches!(s.reading, Reading::Battery { .. }))
            .unwrap()
            .received
            .elapsed()
            > Duration::from_millis(500)
    );
    handle.request_reconnect("test reconnect");
    let reset = await_snapshot(&mut rx, |b| {
        b.generation != first.generation && b.samples.is_empty()
    });
    assert!(reset.samples.is_empty());
    drop(worker);
}
