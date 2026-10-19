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
        stale_timeout_ms: 500,
    }];
    let topology = ResolvedTopology::resolve(raw).unwrap();
    let spec = topology.power_boards().next().unwrap().1.clone();
    let telemetry = Telemetry::new(4096);
    let serial_specs = topology.serial_devices().map(|(_, spec)| spec.clone());
    // macOS pseudo-terminals reject the IOSSIOSPEED ioctl used for physical
    // serial ports. A zero baud rate tells serialport to skip that ioctl for
    // this test fixture; real configurations still require a positive rate.
    #[cfg(target_os = "macos")]
    let serial_specs = serial_specs.map(|mut spec| {
        spec.baud_rate = 0;
        spec
    });
    let manager = SerialManager::start(serial_specs, telemetry.clone()).unwrap();
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

#[test]
fn serial_transport_delivers_heartbeat_samples_corruption_and_reconnect() {
    let (mut master, _manager, handle, spec, telemetry) = fixture();
    let mut rx = telemetry.subscribe();
    let worker = PowerBoardWorker::start(spec, handle.clone(), telemetry).unwrap();
    let deadline = Instant::now() + Duration::from_secs(3);
    let mut received = Vec::new();
    while !received.windows(6).any(|w| w == b"sync\r\n") {
        let mut buf = [0; 128];
        if let Ok(n) = master.read(&mut buf) {
            received.extend_from_slice(&buf[..n]);
        }
        assert!(
            Instant::now() < deadline,
            "no initial AGX sync; serial status: {:?}",
            handle.status()
        );
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
    // Fresh ticks do not refresh the battery timestamp.
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
    // Worker shutdown is bounded even with no further incoming data.
}
