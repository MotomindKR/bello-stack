use super::*;
use bello_hardware_client::raw::proto as p;
use p::hardware_service_server::{HardwareService, HardwareServiceServer};
use std::sync::atomic::{AtomicUsize, Ordering};
use tokio_stream::StreamExt;

#[derive(Clone, Default)]
struct Hardware {
    mode: Arc<AtomicUsize>,
    inventories: Arc<AtomicUsize>,
    inventory: p::ListHardwareResponse,
    telemetry: Option<Vec<p::WatchTelemetryResponse>>,
}

#[tonic::async_trait]
impl HardwareService for Hardware {
    type WatchTelemetryStream =
        Pin<Box<dyn Stream<Item = Result<p::WatchTelemetryResponse, Status>> + Send>>;
    type StreamCommandsStream =
        Pin<Box<dyn Stream<Item = Result<p::StreamCommandsResponse, Status>> + Send>>;

    async fn list_hardware(
        &self,
        _: Request<p::ListHardwareRequest>,
    ) -> Result<Response<p::ListHardwareResponse>, Status> {
        self.inventories.fetch_add(1, Ordering::SeqCst);
        Ok(Response::new(p::ListHardwareResponse {
            serial_devices: vec![p::SerialDeviceInfo {
                name: "serial".into(),
                port: "none".into(),
                baud_rate: 115200,
                ..Default::default()
            }],
            imus: vec![p::ImuInfo {
                name: "imu".into(),
                serial_device: "serial".into(),
                source_frame: "pelvis".into(),
                output_frame: "pelvis".into(),
                ..Default::default()
            }],
            ..self.inventory.clone()
        }))
    }

    async fn watch_telemetry(
        &self,
        _: Request<p::WatchTelemetryRequest>,
    ) -> Result<Response<Self::WatchTelemetryStream>, Status> {
        if let Some(telemetry) = &self.telemetry {
            return Ok(Response::new(Box::pin(tokio_stream::iter(
                telemetry.clone().into_iter().map(Ok),
            ))));
        }
        let mode = self.mode.clone();
        Ok(Response::new(Box::pin(async_stream::try_stream! {
            let mut timestamp = 100_000_000;
            loop {
                match mode.load(Ordering::SeqCst) {
                    1 => break,
                    2 => Err(Status::unavailable("simulated hardware restart"))?,
                    3 => { tokio::time::sleep(Duration::from_millis(10)).await; continue; }
                    _ => {}
                }
                timestamp += 10_000_000;
                yield p::WatchTelemetryResponse { monotonic_ns: timestamp,
                    payload: Some(p::watch_telemetry_response::Payload::Imu(p::ImuTelemetry {
                        imu_name: "imu".into(),
                        orientation: Some(p::Quaternion { w: 1., ..Default::default() }),
                        angular_velocity: Some(p::Vector3::default()),
                        linear_acceleration: Some(p::Vector3 { z: 9.81, ..Default::default() }),
                        ..Default::default()
                    }))
                };
                tokio::time::sleep(Duration::from_millis(10)).await;
            }
        })))
    }

    async fn get_state_frame(
        &self,
        _: Request<p::GetStateFrameRequest>,
    ) -> Result<Response<p::GetStateFrameResponse>, Status> {
        unreachable!()
    }
    async fn acquire_lease(
        &self,
        _: Request<p::AcquireLeaseRequest>,
    ) -> Result<Response<p::AcquireLeaseResponse>, Status> {
        unreachable!()
    }
    async fn renew_lease(
        &self,
        _: Request<p::RenewLeaseRequest>,
    ) -> Result<Response<p::RenewLeaseResponse>, Status> {
        unreachable!()
    }
    async fn release_lease(
        &self,
        _: Request<p::ReleaseLeaseRequest>,
    ) -> Result<Response<p::ReleaseLeaseResponse>, Status> {
        unreachable!()
    }
    async fn enable_group(
        &self,
        _: Request<p::EnableGroupRequest>,
    ) -> Result<Response<p::EnableGroupResponse>, Status> {
        unreachable!()
    }
    async fn start_control(
        &self,
        _: Request<p::StartControlRequest>,
    ) -> Result<Response<p::StartControlResponse>, Status> {
        unreachable!()
    }
    async fn stop_control(
        &self,
        _: Request<p::StopControlRequest>,
    ) -> Result<Response<p::StopControlResponse>, Status> {
        unreachable!()
    }
    async fn disable_group(
        &self,
        _: Request<p::DisableGroupRequest>,
    ) -> Result<Response<p::DisableGroupResponse>, Status> {
        unreachable!()
    }
    async fn recover_group(
        &self,
        _: Request<p::RecoverGroupRequest>,
    ) -> Result<Response<p::RecoverGroupResponse>, Status> {
        unreachable!()
    }
    async fn stream_commands(
        &self,
        _: Request<tonic::Streaming<p::StreamCommandsRequest>>,
    ) -> Result<Response<Self::StreamCommandsStream>, Status> {
        unreachable!()
    }
}

async fn wait_pose(
    receiver: &mut watch::Receiver<Option<StampedPose>>,
    present: bool,
) -> Option<StampedPose> {
    tokio::time::timeout(Duration::from_secs(5), async {
        loop {
            let pose = receiver.borrow_and_update().clone();
            if pose.is_some() == present {
                return pose;
            }
            receiver.changed().await.unwrap();
        }
    })
    .await
    .unwrap()
}

#[tokio::test]
async fn absent_hardware_eof_restart_and_stall_do_not_stop_monitor() {
    let temporary = tempfile::tempdir().unwrap();
    let urdf = temporary.path().join("robot.urdf");
    std::fs::write(&urdf, r#"<robot name="synthetic"><link name="pelvis"/>
        <link name="left"><collision><geometry><box size=".1 .1 .02"/></geometry></collision></link>
        <link name="right"><collision><geometry><box size=".1 .1 .02"/></geometry></collision></link>
        <joint name="left_fixed" type="fixed"><parent link="pelvis"/><child link="left"/><origin xyz="0 .1 -.8"/></joint>
        <joint name="right_fixed" type="fixed"><parent link="pelvis"/><child link="right"/><origin xyz="0 -.1 -.8"/></joint></robot>"#).unwrap();
    let reservation = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let address = reservation.local_addr().unwrap();
    drop(reservation);
    let config: EstimatorConfig = serde_yaml::from_str(&format!(
        r#"
urdf: {}
hardware_endpoint: http://{}
listen: 127.0.0.1:0
imu_frame: pelvis
tracked_frame: pelvis
left_foot: left
right_foot: right
planar: true
ground_height_m: 0
"#,
        urdf.display(),
        address
    ))
    .unwrap();
    let description = RobotDescription::load(&urdf).unwrap();
    let model = description
        .kinematic_model("pelvis", "left", "right")
        .unwrap();
    let (poses, mut receiver) = watch::channel(None);
    let (stop, shutdown) = watch::channel(false);
    let monitor = tokio::spawn(monitor_hardware(
        config,
        model,
        ImuFrameTransform::new(nalgebra::Isometry3::identity()),
        poses,
        shutdown,
    ));
    tokio::time::sleep(Duration::from_millis(250)).await;
    assert!(!monitor.is_finished());
    assert!(receiver.borrow().is_none());
    let hardware = Hardware::default();
    let listener = TcpListener::bind(address).await.unwrap();
    let server = tokio::spawn(
        tonic::transport::Server::builder()
            .add_service(HardwareServiceServer::new(hardware.clone()))
            .serve_with_incoming(TcpListenerStream::new(listener)),
    );
    let mut last = wait_pose(&mut receiver, true).await.unwrap();
    for failure in [1, 2, 3] {
        hardware.mode.store(failure, Ordering::SeqCst);
        wait_pose(&mut receiver, false).await;
        assert!(!monitor.is_finished());
        hardware.mode.store(0, Ordering::SeqCst);
        let next = wait_pose(&mut receiver, true).await.unwrap();
        assert!(next.sequence > last.sequence);
        assert!((next.pose.position - last.pose.position).norm() < 1e-6);
        last = next;
    }
    assert!(hardware.inventories.load(Ordering::SeqCst) >= 4);
    hardware.mode.store(1, Ordering::SeqCst);
    wait_pose(&mut receiver, false).await;
    stop.send(true).unwrap();
    tokio::time::timeout(Duration::from_secs(1), monitor)
        .await
        .unwrap()
        .unwrap()
        .unwrap();
    server.abort();
    let _ = server.await;
}

#[tokio::test]
async fn watch_pose_reports_age_of_oldest_encoder_used() {
    use p::watch_telemetry_response::Payload;

    let motor = |event_ms, feedback_age_ms| p::WatchTelemetryResponse {
        monotonic_ns: event_ms * 1_000_000_u64,
        payload: Some(Payload::Motor(p::MotorTelemetry {
            group: "legs".into(),
            motor_name: "motor".into(),
            feedback_age_ms,
            ..Default::default()
        })),
    };
    let imu = |event_ms, sample_age_ms| p::WatchTelemetryResponse {
        monotonic_ns: event_ms * 1_000_000_u64,
        payload: Some(Payload::Imu(p::ImuTelemetry {
            imu_name: "imu".into(),
            orientation: Some(p::Quaternion {
                w: 1.0,
                ..Default::default()
            }),
            angular_velocity: Some(p::Vector3::default()),
            linear_acceleration: Some(p::Vector3 {
                z: 9.81,
                ..Default::default()
            }),
            sample_age_ms,
            ..Default::default()
        })),
    };
    let address = p::MotorAddress {
        bus: "bus".into(),
        id: 1,
    };
    let hardware = Hardware {
        inventory: p::ListHardwareResponse {
            buses: vec![p::BusInfo {
                name: "bus".into(),
                transport: "test".into(),
                ..Default::default()
            }],
            groups: vec![p::GroupInfo {
                name: "legs".into(),
                control_rate_hz: 100.0,
                motors: vec![address.clone()],
                ..Default::default()
            }],
            motors: vec![p::MotorInfo {
                name: "motor".into(),
                coordinate: "left_joint".into(),
                group: "legs".into(),
                address: Some(address),
                limits: Some(p::MotorLimits {
                    position_min: -1.0,
                    position_max: 1.0,
                    velocity_min: -1.0,
                    velocity_max: 1.0,
                    torque_min: -1.0,
                    torque_max: 1.0,
                    kp_max: 1.0,
                    kd_max: 1.0,
                    ..Default::default()
                }),
                ..Default::default()
            }],
            ..Default::default()
        },
        telemetry: Some(vec![
            motor(110, 10),
            motor(400, 0),
            imu(420, 30),
            imu(700, 260),
        ]),
        ..Default::default()
    };
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let endpoint = format!("http://{}", listener.local_addr().unwrap());
    let server = tokio::spawn(
        tonic::transport::Server::builder()
            .add_service(HardwareServiceServer::new(hardware))
            .serve_with_incoming(TcpListenerStream::new(listener)),
    );
    let urdf = tempfile::NamedTempFile::new().unwrap();
    std::fs::write(urdf.path(), r#"<robot name="synthetic"><link name="pelvis"/>
        <link name="left"><collision><geometry><box size=".1 .1 .02"/></geometry></collision></link>
        <link name="right"><collision><geometry><box size=".1 .1 .02"/></geometry></collision></link>
        <joint name="left_joint" type="revolute"><parent link="pelvis"/><child link="left"/><origin xyz="0 .1 -.8"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint>
        <joint name="right_fixed" type="fixed"><parent link="pelvis"/><child link="right"/><origin xyz="0 -.1 -.8"/></joint></robot>"#).unwrap();
    let config: EstimatorConfig = serde_yaml::from_str(&format!(
        "urdf: {}\nhardware_endpoint: {endpoint}\nlisten: 127.0.0.1:0\nimu_frame: pelvis\nplanar: true\nground_height_m: 0\nimu_stale_ms: 250\nmotor_stale_ms: 300\n",
        urdf.path().display(),
    )).unwrap();
    let description = RobotDescription::load(urdf.path()).unwrap();
    let model = description
        .kinematic_model("pelvis", "left", "right")
        .unwrap();
    let (poses, receiver) = watch::channel(None);
    consume_hardware(
        Robot::connect(endpoint).await.unwrap(),
        &config,
        &model,
        &ImuFrameTransform::new(nalgebra::Isometry3::identity()),
        &poses,
        &mut ContactFilter::new(&config),
        &mut 0,
    )
    .await
    .unwrap();
    drop(poses);

    let server_pose = PoseServer {
        reference_frame: Arc::from("odom"),
        tracked_frame: Arc::from("pelvis"),
        poses: receiver,
        maximum_age: config.imu_stale(),
    };
    let mut stream = server_pose
        .watch_pose(Request::new(WatchPoseRequest {
            reference_frame: "odom".into(),
            tracked_frame: "pelvis".into(),
        }))
        .await
        .unwrap()
        .into_inner();
    let response = stream
        .next()
        .await
        .expect("fresh inputs should publish a pose")
        .unwrap();
    assert_eq!(response.sequence, 1);
    assert_eq!(response.sample_monotonic_ns, 390_000_000);
    assert!(
        response.sample_age_ms >= 320,
        "published age was {} ms",
        response.sample_age_ms
    );
    server.abort();
}

#[test]
fn permanent_errors_are_not_retried() {
    for status in [
        Status::invalid_argument("bad frames"),
        Status::permission_denied("credentials"),
        Status::data_loss("bad payload"),
    ] {
        assert!(!retryable_hardware_error(
            &HardwareError::Rpc(status).into()
        ));
    }
    assert!(retryable_hardware_error(
        &anyhow::Error::new(HardwareError::Rpc(Status::unavailable("offline"))).context("stream")
    ));
}

#[tokio::test]
async fn new_subscribers_never_receive_cached_stale_pose() {
    let (sender, receiver) = watch::channel(Some(StampedPose {
        sequence: 1,
        pose: PoseEstimate {
            position: Vector3::zeros(),
            orientation: UnitQuaternion::identity(),
            linear_velocity: Vector3::zeros(),
        },
        angular_velocity_local: Vector3::zeros(),
        produced: Instant::now(),
        source_age: Duration::from_secs(1),
        sample_monotonic_ns: 1,
    }));
    let server = PoseServer {
        reference_frame: Arc::from("odom"),
        tracked_frame: Arc::from("pelvis"),
        poses: receiver,
        maximum_age: Duration::from_millis(100),
    };
    let mut stream = server
        .watch_pose(Request::new(WatchPoseRequest {
            reference_frame: "odom".into(),
            tracked_frame: "pelvis".into(),
        }))
        .await
        .unwrap()
        .into_inner();
    assert!(
        tokio::time::timeout(Duration::from_millis(30), stream.next())
            .await
            .is_err()
    );
    let mut fresh = sender.borrow().clone().unwrap();
    fresh.sequence = 2;
    fresh.source_age = Duration::ZERO;
    fresh.produced = Instant::now();
    sender.send_replace(Some(fresh));
    assert_eq!(
        tokio::time::timeout(Duration::from_secs(1), stream.next())
            .await
            .unwrap()
            .unwrap()
            .unwrap()
            .sequence,
        2
    );
}
