#![cfg(target_os = "linux")]

//! Opt-in transport behavior check; no robot-specific configuration snapshots.
use bello_mujoco::{
    model::{Hardware, ModelConfig},
    runtime::Config,
};
use bello_pose_api::{WatchPoseRequest, pose_service_client::PoseServiceClient};
use robstride_protocol::{Frame, MotorParams, active_report_frame, parse_feedback};
use socketcan::{CanFrame, CanSocket, EmbeddedFrame, ExtendedId, Frame as _, Socket};
use std::fs::File;
use std::io::{Read, Write};
use std::os::fd::AsRawFd;
use std::process::{Child, Command};
use std::time::{Duration, Instant};

struct Process(Child);
impl Drop for Process {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}

#[tokio::test]
#[ignore = "requires explicitly selected isolated virtual CAN interfaces and an emulator profile"]
async fn native_process_serves_can_serial_pose_and_exits_on_sigterm() {
    let profile = std::path::PathBuf::from(std::env::var("BELLO_TEST_EMULATOR_PROFILE").unwrap());
    let interfaces: Vec<String> = std::env::var("BELLO_TEST_CAN_INTERFACES")
        .unwrap()
        .split(',')
        .map(str::to_owned)
        .collect();
    let config = Config::load(&profile).unwrap();
    assert_eq!(interfaces.len(), config.bus_interfaces.len());
    for interface in &interfaces {
        assert!(
            std::path::Path::new("/sys/devices/virtual/net")
                .join(interface)
                .is_dir()
        );
    }
    let model = ModelConfig::load(&config.model).unwrap();
    let hardware: Hardware = serde_yaml::from_reader(File::open(&model.hardware).unwrap()).unwrap();
    let first_bus = config.bus_interfaces.keys().next().unwrap();
    let bus = hardware
        .logical_buses
        .iter()
        .find(|b| &b.name == first_bus)
        .unwrap();
    let motor = hardware
        .motors
        .iter()
        .find(|m| &m.bus == first_bus)
        .unwrap();
    let params = MotorParams::for_model(&motor.model, motor.direction).unwrap();
    let socket = CanSocket::open(&interfaces[0]).unwrap();
    socket.set_nonblocking(true).unwrap();
    let pty = nix::pty::openpty(None, None).unwrap();
    let device = nix::unistd::ttyname(&pty.slave).unwrap();
    let power_pty = nix::pty::openpty(None, None).unwrap();
    let power_device = nix::unistd::ttyname(&power_pty.slave).unwrap();
    let mut power_peer = File::from(power_pty.master);
    nix::fcntl::fcntl(
        power_peer.as_raw_fd(),
        nix::fcntl::FcntlArg::F_SETFL(nix::fcntl::OFlag::O_NONBLOCK),
    )
    .unwrap();
    let mut serial = File::from(pty.master);
    nix::fcntl::fcntl(
        serial.as_raw_fd(),
        nix::fcntl::FcntlArg::F_SETFL(nix::fcntl::OFlag::O_NONBLOCK),
    )
    .unwrap();
    let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
    let address = listener.local_addr().unwrap();
    drop(listener);
    let mut yaml: serde_yaml::Value =
        serde_yaml::from_reader(File::open(&profile).unwrap()).unwrap();
    yaml["model"] = config.model.to_string_lossy().into_owned().into();
    for (bus, interface) in config.bus_interfaces.keys().zip(&interfaces) {
        yaml["bus_interfaces"][bus.as_str()] = interface.clone().into();
    }
    if config.imu.is_some() {
        yaml["imu"]["device"] = device.to_string_lossy().into_owned().into();
    }
    if config.power_board.is_some() {
        yaml["power_board"]["device"] = power_device.to_string_lossy().into_owned().into();
    }
    yaml["pose"]["listen"] = address.to_string().into();
    let recording = tempfile::tempdir().unwrap();
    let video_path = recording.path().join("emulator.mp4");
    yaml["video"] = serde_yaml::to_value(serde_json::json!({
        "output": video_path, "width": 320, "height": 240, "fps": 20,
        "max_seconds": 10., "render_timeout_seconds": 15.,
        "camera": {"body": config.pose.tracked_frame, "distance_m": 3.,
            "azimuth_degrees": 135., "elevation_degrees": -14.,
            "lookat_offset_m": [0., 0., 0.]}
    }))
    .unwrap();
    let mut file = tempfile::NamedTempFile::new().unwrap();
    serde_yaml::to_writer(&mut file, &yaml).unwrap();
    let operator = nix::pty::openpty(None, None).unwrap();
    let mut operator_input = File::from(operator.master);
    let mut process_log = tempfile::tempfile().unwrap();
    let mut process = Process(
        Command::new(env!("CARGO_BIN_EXE_bello-mujoco"))
            .args(["run", "--config"])
            .arg(file.path())
            .stdin(File::from(operator.slave))
            .stderr(process_log.try_clone().unwrap())
            .spawn()
            .unwrap(),
    );
    let start = Instant::now();
    let mut client = loop {
        assert!(
            start.elapsed() < Duration::from_secs(5),
            "pose startup timed out"
        );
        if let Ok(client) = PoseServiceClient::connect(format!("http://{address}")).await {
            break client;
        }
        tokio::time::sleep(Duration::from_millis(20)).await;
    };
    let mut poses = client
        .watch_pose(WatchPoseRequest {
            reference_frame: config.pose.reference_frame,
            tracked_frame: config.pose.tracked_frame,
        })
        .await
        .unwrap()
        .into_inner();
    serial
        .write_all(b"<soc1><sof2><sog1><soa2><som0><sot0><sots1><sor5>")
        .unwrap();
    let request = active_report_frame(bus.host_id, motor.id, true);
    socket
        .write_frame(&CanFrame::new(ExtendedId::new(request.id).unwrap(), &request.data).unwrap())
        .unwrap();
    let mut feedback_count = 0;
    let mut power_parser = cog_protocol::Framer::default();
    let mut power_kinds = std::collections::BTreeSet::new();
    power_peer.write_all(b"sync\r\n").unwrap();
    let mut serial_bytes = Vec::new();
    let deadline = Instant::now() + Duration::from_secs(1);
    while Instant::now() < deadline {
        let mut bytes = [0; 4096];
        match power_peer.read(&mut bytes) {
            Ok(n) => {
                for &byte in &bytes[..n] {
                    if let Some(frame) = power_parser.push(byte, cog_protocol::Profile::Agx) {
                        power_kinds.insert(frame.unwrap().kind);
                    }
                }
            }
            Err(e) => assert_eq!(e.kind(), std::io::ErrorKind::WouldBlock),
        }
        match serial.read(&mut bytes) {
            Ok(n) => serial_bytes.extend_from_slice(&bytes[..n]),
            Err(e) => assert_eq!(e.kind(), std::io::ErrorKind::WouldBlock),
        }
        loop {
            match socket.read_frame() {
                Ok(wire) => {
                    let frame = Frame {
                        id: wire.raw_id(),
                        data: wire.data().try_into().unwrap(),
                    };
                    if let Some(feedback) = parse_feedback(&frame, params) {
                        assert_eq!(feedback.motor_id, motor.id);
                        assert_eq!(feedback.mode_status, 0);
                        assert!(feedback.position.is_finite());
                        feedback_count += 1;
                    }
                }
                Err(e) => {
                    assert_eq!(e.kind(), std::io::ErrorKind::WouldBlock);
                    break;
                }
            }
        }
        tokio::time::sleep(Duration::from_millis(5)).await;
    }
    assert!(feedback_count > 10, "missing active CAN feedback");
    if config.power_board.is_some() {
        assert_eq!(power_kinds, [1, 2, 3, 5, 6, 7, 8].into_iter().collect());
    }
    if config.imu.is_some() {
        let text = std::str::from_utf8(&serial_bytes).unwrap();
        assert!(text.contains("<ok>"));
        let records: Vec<_> = text
            .lines()
            .filter_map(|line| line.find('*').map(|i| &line[i + 1..]))
            .collect();
        assert!(records.len() > 20, "missing EBIMU samples");
        for line in records.iter().take(20) {
            assert_eq!(
                line.split(',').filter(|v| v.parse::<f64>().is_ok()).count(),
                11
            );
        }
    }
    let a = tokio::time::timeout(Duration::from_secs(1), poses.message())
        .await
        .unwrap()
        .unwrap()
        .unwrap();
    let b = tokio::time::timeout(Duration::from_secs(1), poses.message())
        .await
        .unwrap()
        .unwrap()
        .unwrap();
    assert!(b.sequence > a.sequence && b.sample_monotonic_ns > a.sample_monotonic_ns);
    if config.torso_guide.is_some() {
        // Both lines arrive together: stdio prefetch must not swallow the reset.
        operator_input.write_all(b"\n\n").unwrap();
        tokio::time::sleep(Duration::from_millis(100)).await;
        use std::io::{Seek, SeekFrom};
        process_log.seek(SeekFrom::Start(0)).unwrap();
        let mut text = String::new();
        process_log.read_to_string(&mut text).unwrap();
        assert!(text.contains("Torso guide released."), "{text}");
        assert!(
            text.contains("Torso guide reset; motors disarmed."),
            "{text}"
        );
        operator_input.write_all(b"\n").unwrap();
        tokio::time::sleep(Duration::from_millis(200)).await;
    }
    nix::sys::signal::kill(
        nix::unistd::Pid::from_raw(process.0.id() as i32),
        nix::sys::signal::Signal::SIGTERM,
    )
    .unwrap();
    let start = Instant::now();
    loop {
        if let Some(status) = process.0.try_wait().unwrap() {
            use std::io::{Seek, SeekFrom};
            process_log.seek(SeekFrom::Start(0)).unwrap();
            let mut log = String::new();
            process_log.read_to_string(&mut log).unwrap();
            assert!(status.success(), "{log}");
            assert!(log.contains("llvmpipe"), "{log}");
            assert!(video_path.metadata().unwrap().len() > 1000);
            if let Ok(directory) = std::env::var("BELLO_TEST_VIDEO_ARTIFACT_DIR") {
                std::fs::create_dir_all(&directory).unwrap();
                std::fs::copy(
                    &video_path,
                    std::path::Path::new(&directory).join("emulator.mp4"),
                )
                .unwrap();
            }
            break;
        }
        assert!(
            start.elapsed() < Duration::from_secs(20),
            "native emulator did not exit"
        );
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
}
