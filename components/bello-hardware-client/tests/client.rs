use std::pin::Pin;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use bello_hardware_client::raw::proto::hardware_service_server::{
    HardwareService, HardwareServiceServer,
};
use bello_hardware_client::raw::proto::watch_telemetry_response::Payload;
use bello_hardware_client::raw::proto::*;
use bello_hardware_client::raw::{
    CommandStreamOptions, CommandValidity, EnumValue, HardwareClient, LeaseDuration, LeaseStatus,
    TelemetryFilter,
};
use bello_hardware_client::{ControlOptions, CoordinateTarget, Error, Robot};
use tokio::net::TcpListener;
use tokio::sync::{Notify, oneshot};
use tokio_stream::Stream;
use tokio_stream::wrappers::TcpListenerStream;
use tonic::transport::Server;
use tonic::{Request, Response, Status, Streaming};

const GROUP: &str = "test_group";
const TOKEN: &str = "550e8400-e29b-41d4-a716-446655440000";

type ResponseStream<T> = Pin<Box<dyn Stream<Item = Result<T, Status>> + Send + 'static>>;

#[derive(Clone)]
struct FakeHardware {
    state: Arc<Mutex<Recorded>>,
    settings: Arc<FakeSettings>,
}

struct FakeSettings {
    acquire_expires_ms: AtomicU64,
    renew_delay_ms: AtomicU64,
    renew_started: Notify,
    renew_in_flight: AtomicBool,
    fail_renew: AtomicBool,
    fail_start: AtomicBool,
    reject_commands: AtomicBool,
    fail_release: AtomicBool,
}

impl Default for FakeHardware {
    fn default() -> Self {
        Self {
            state: Arc::new(Mutex::new(Recorded::default())),
            settings: Arc::new(FakeSettings {
                acquire_expires_ms: AtomicU64::new(10_000),
                renew_delay_ms: AtomicU64::new(0),
                renew_started: Notify::new(),
                renew_in_flight: AtomicBool::new(false),
                fail_renew: AtomicBool::new(false),
                fail_start: AtomicBool::new(false),
                reject_commands: AtomicBool::new(false),
                fail_release: AtomicBool::new(false),
            }),
        }
    }
}

#[derive(Default)]
struct Recorded {
    acquired: Vec<AcquireLeaseRequest>,
    renewed: Vec<RenewLeaseRequest>,
    released: Vec<ReleaseLeaseRequest>,
    lifecycle: Vec<LifecycleCall>,
    starts: Vec<StartControlRequest>,
    commands: Vec<StreamCommandsRequest>,
    telemetry: Vec<WatchTelemetryRequest>,
}

#[derive(Debug, PartialEq, Eq)]
struct LifecycleCall {
    action: &'static str,
    group: String,
    token: String,
}

impl FakeHardware {
    fn record_lifecycle(&self, action: &'static str, group: String, token: String) {
        self.state
            .lock()
            .expect("fake state mutex poisoned")
            .lifecycle
            .push(LifecycleCall {
                action,
                group,
                token,
            });
    }
}

#[tonic::async_trait]
impl HardwareService for FakeHardware {
    type WatchTelemetryStream = ResponseStream<WatchTelemetryResponse>;
    type StreamCommandsStream = ResponseStream<StreamCommandsResponse>;

    async fn list_hardware(
        &self,
        _: Request<ListHardwareRequest>,
    ) -> Result<Response<ListHardwareResponse>, Status> {
        Ok(Response::new(ListHardwareResponse {
            buses: vec![BusInfo {
                name: "test_bus".into(),
                interface: "vcan-test".into(),
                host_id: 127,
                status: Some(DeviceStatus {
                    state: DeviceState::Online as i32,
                    detail: "bus online".into(),
                    transition_count: 2,
                }),
            }],
            groups: vec![GroupInfo {
                name: GROUP.into(),
                control_rate_hz: 250.0,
                state: GroupState::Idle as i32,
                motors: vec![MotorAddress {
                    bus: "test_bus".into(),
                    id: 1,
                }],
            }],
            motors: vec![MotorInfo {
                name: "motor_a".into(),
                coordinate: "coordinate_a".into(),
                group: GROUP.into(),
                address: Some(MotorAddress {
                    bus: "test_bus".into(),
                    id: 1,
                }),
                model: "test_motor".into(),
                direction: 1,
                status: Some(DeviceStatus {
                    state: DeviceState::Online as i32,
                    detail: "motor online".into(),
                    transition_count: 1,
                }),
                limits: Some(bello_hardware_api::MotorLimits {
                    position_min: -1.0,
                    position_max: 1.0,
                    velocity_min: -2.0,
                    velocity_max: 2.0,
                    torque_min: -3.0,
                    torque_max: 3.0,
                    kp_min: 0.0,
                    kp_max: 100.0,
                    kd_min: 0.0,
                    kd_max: 10.0,
                }),
            }],
            serial_devices: vec![SerialDeviceInfo {
                name: "test_serial".into(),
                port: "/dev/ttyTEST0".into(),
                baud_rate: 921_600,
                status: Some(DeviceStatus {
                    state: DeviceState::Online as i32,
                    detail: "serial online".into(),
                    transition_count: 4,
                }),
            }],
            imus: vec![ImuInfo {
                name: "test_imu".into(),
                serial_device: "test_serial".into(),
                status: Some(DeviceStatus {
                    state: DeviceState::Online as i32,
                    detail: "IMU online".into(),
                    transition_count: 5,
                }),
            }],
        }))
    }

    async fn watch_telemetry(
        &self,
        request: Request<WatchTelemetryRequest>,
    ) -> Result<Response<Self::WatchTelemetryStream>, Status> {
        let filter = request.into_inner();
        self.state
            .lock()
            .expect("fake state mutex poisoned")
            .telemetry
            .push(filter.clone());
        let group = WatchTelemetryResponse {
            monotonic_ns: 42,
            payload: Some(Payload::Group(GroupTelemetry {
                group: GROUP.into(),
                state: GroupState::Idle as i32,
                ..Default::default()
            })),
        };
        let motor = WatchTelemetryResponse {
            monotonic_ns: 43,
            payload: Some(Payload::Motor(MotorTelemetry {
                group: GROUP.into(),
                motor_name: "motor_a".into(),
                position: 0.25,
                velocity: 0.5,
                temperature: 32.0,
                ..Default::default()
            })),
        };
        let mut events = vec![Ok(group)];
        if filter.include_motors {
            events.push(Ok(motor));
        }
        if filter.include_imu {
            events.push(Ok(WatchTelemetryResponse {
                monotonic_ns: 44,
                payload: Some(Payload::Imu(ImuTelemetry {
                    imu_name: "test_imu".into(),
                    orientation: Some(Quaternion {
                        w: 1.0,
                        ..Default::default()
                    }),
                    angular_velocity: Some(Vector3 {
                        x: 0.1,
                        ..Default::default()
                    }),
                    linear_acceleration: Some(Vector3 {
                        z: 9.81,
                        ..Default::default()
                    }),
                    device_timestamp_ms: 123,
                    sample_age_ms: 4,
                })),
            }));
        }
        if filter.include_health {
            events.push(Ok(WatchTelemetryResponse {
                monotonic_ns: 45,
                payload: Some(Payload::Health(HealthTelemetry {
                    component: "can/test_bus".into(),
                    device_status: Some(DeviceStatus {
                        state: DeviceState::Degraded as i32,
                        detail: "test degradation".into(),
                        transition_count: 3,
                    }),
                    dropped_events: 7,
                })),
            }));
        }
        if filter.include_command_policy_events {
            events.push(Ok(WatchTelemetryResponse {
                monotonic_ns: 46,
                payload: Some(Payload::CommandPolicy(CommandPolicyTelemetry {
                    group: GROUP.into(),
                    motor_name: "motor_a".into(),
                    check: CommandPolicyCheck::Limit as i32,
                    behavior: CommandBehavior::Clamp as i32,
                    detail: "position clamped".into(),
                    command_modified: true,
                    command_rejected: false,
                    fault_response: FaultResponse::Unspecified as i32,
                })),
            }));
        }
        Ok(Response::new(Box::pin(tokio_stream::iter(events))))
    }

    async fn get_state_frame(
        &self,
        _: Request<GetStateFrameRequest>,
    ) -> Result<Response<GetStateFrameResponse>, Status> {
        Err(Status::unimplemented("not used by this fake"))
    }

    async fn acquire_lease(
        &self,
        request: Request<AcquireLeaseRequest>,
    ) -> Result<Response<AcquireLeaseResponse>, Status> {
        self.state
            .lock()
            .expect("fake state mutex poisoned")
            .acquired
            .push(request.into_inner());
        Ok(Response::new(AcquireLeaseResponse {
            group: GROUP.into(),
            token: TOKEN.into(),
            expires_in_ms: self.settings.acquire_expires_ms.load(Ordering::Relaxed),
        }))
    }

    async fn renew_lease(
        &self,
        request: Request<RenewLeaseRequest>,
    ) -> Result<Response<RenewLeaseResponse>, Status> {
        let request = request.into_inner();
        let expires_in_ms = if request.requested_duration_ms == 0 {
            10_000
        } else {
            request.requested_duration_ms
        };
        self.state
            .lock()
            .expect("fake state mutex poisoned")
            .renewed
            .push(request);
        if self.settings.fail_renew.load(Ordering::Relaxed) {
            return Err(Status::unavailable("renewal unavailable"));
        }
        self.settings.renew_in_flight.store(true, Ordering::SeqCst);
        self.settings.renew_started.notify_one();
        tokio::time::sleep(Duration::from_millis(
            self.settings.renew_delay_ms.load(Ordering::Relaxed),
        ))
        .await;
        self.settings.renew_in_flight.store(false, Ordering::SeqCst);
        Ok(Response::new(RenewLeaseResponse {
            group: GROUP.into(),
            token: TOKEN.into(),
            expires_in_ms,
        }))
    }

    async fn release_lease(
        &self,
        request: Request<ReleaseLeaseRequest>,
    ) -> Result<Response<ReleaseLeaseResponse>, Status> {
        self.state
            .lock()
            .expect("fake state mutex poisoned")
            .released
            .push(request.into_inner());
        if self.settings.renew_in_flight.load(Ordering::SeqCst) {
            return Err(Status::aborted("release raced renewal"));
        }
        if self.settings.fail_release.load(Ordering::Relaxed) {
            return Err(Status::failed_precondition(
                "release reaction entered fault",
            ));
        }
        Ok(Response::new(ReleaseLeaseResponse {
            state: GroupState::Idle as i32,
        }))
    }

    async fn enable_group(
        &self,
        request: Request<EnableGroupRequest>,
    ) -> Result<Response<EnableGroupResponse>, Status> {
        let request = request.into_inner();
        self.record_lifecycle("enable", request.group, request.lease_token);
        Ok(Response::new(EnableGroupResponse {
            state: GroupState::Damped as i32,
        }))
    }

    async fn start_control(
        &self,
        request: Request<StartControlRequest>,
    ) -> Result<Response<StartControlResponse>, Status> {
        let request = request.into_inner();
        self.record_lifecycle("start", request.group.clone(), request.lease_token.clone());
        self.state
            .lock()
            .expect("fake state mutex poisoned")
            .starts
            .push(request);
        if self.settings.fail_start.load(Ordering::Relaxed) {
            return Err(Status::unavailable("start response unavailable"));
        }
        Ok(Response::new(StartControlResponse {
            state: GroupState::Running as i32,
        }))
    }

    async fn stop_control(
        &self,
        request: Request<StopControlRequest>,
    ) -> Result<Response<StopControlResponse>, Status> {
        let request = request.into_inner();
        self.record_lifecycle("stop", request.group, request.lease_token);
        Ok(Response::new(StopControlResponse {
            state: GroupState::Damped as i32,
        }))
    }

    async fn disable_group(
        &self,
        request: Request<DisableGroupRequest>,
    ) -> Result<Response<DisableGroupResponse>, Status> {
        let request = request.into_inner();
        self.record_lifecycle("disable", request.group, request.lease_token);
        Ok(Response::new(DisableGroupResponse {
            state: GroupState::Idle as i32,
        }))
    }

    async fn recover_group(
        &self,
        request: Request<RecoverGroupRequest>,
    ) -> Result<Response<RecoverGroupResponse>, Status> {
        let request = request.into_inner();
        self.record_lifecycle("recover", request.group, request.lease_token);
        Ok(Response::new(RecoverGroupResponse {
            state: GroupState::Idle as i32,
        }))
    }

    async fn stream_commands(
        &self,
        request: Request<Streaming<StreamCommandsRequest>>,
    ) -> Result<Response<Self::StreamCommandsStream>, Status> {
        let mut requests = request.into_inner();
        let state = Arc::clone(&self.state);
        let settings = Arc::clone(&self.settings);
        let output = async_stream::try_stream! {
            while let Some(command) = requests.message().await? {
                state
                    .lock()
                    .expect("fake state mutex poisoned")
                    .commands
                    .push(command.clone());
                let lease_lost = command
                    .commands
                    .iter()
                    .any(|motor| motor.motor_name == "lease_lost");
                let rejected = settings.reject_commands.load(Ordering::Relaxed);
                yield StreamCommandsResponse {
                    group: command.group,
                    sequence: command.sequence,
                    result: if lease_lost {
                        CommandResult::LeaseLost as i32
                    } else if rejected {
                        CommandResult::Unsafe as i32
                    } else {
                        CommandResult::Accepted as i32
                    },
                    detail: if lease_lost {
                        "lease lost".into()
                    } else if rejected {
                        "unsafe target".into()
                    } else {
                        "accepted".into()
                    },
                    state: if lease_lost {
                        GroupState::Damped as i32
                    } else if rejected {
                        GroupState::Fault as i32
                    } else {
                        GroupState::Running as i32
                    },
                };
            }
        };
        Ok(Response::new(Box::pin(output)))
    }
}

struct FakeServer {
    endpoint: String,
    state: Arc<Mutex<Recorded>>,
    settings: Arc<FakeSettings>,
    shutdown: Option<oneshot::Sender<()>>,
    worker: tokio::task::JoinHandle<Result<(), tonic::transport::Error>>,
}

impl FakeServer {
    async fn start() -> Self {
        let listener = TcpListener::bind("127.0.0.1:0")
            .await
            .expect("bind fake server");
        let address = listener.local_addr().expect("read fake server address");
        let service = FakeHardware::default();
        let state = Arc::clone(&service.state);
        let settings = Arc::clone(&service.settings);
        let (shutdown, shutdown_rx) = oneshot::channel();
        let worker = tokio::spawn(async move {
            Server::builder()
                .add_service(HardwareServiceServer::new(service))
                .serve_with_incoming_shutdown(TcpListenerStream::new(listener), async {
                    let _ = shutdown_rx.await;
                })
                .await
        });
        Self {
            endpoint: format!("http://{address}"),
            state,
            settings,
            shutdown: Some(shutdown),
            worker,
        }
    }

    async fn stop(mut self) {
        if let Some(shutdown) = self.shutdown.take() {
            let _ = shutdown.send(());
        }
        self.worker
            .await
            .expect("join fake server")
            .expect("run fake server");
    }
}

#[tokio::test]
async fn canonical_workflow_binds_group_token_sequences_and_filters() {
    let server = FakeServer::start().await;
    let client = HardwareClient::connect(server.endpoint.clone())
        .await
        .expect("connect client");

    let inventory = client.inventory().await.expect("read inventory");
    assert_eq!(inventory.groups[0].name, GROUP);

    let filter = TelemetryFilter::default()
        .selected_groups([GROUP])
        .include_motors(true)
        .include_health(true)
        .include_command_policy_events(true);
    let mut telemetry = client.watch(filter).await.expect("watch telemetry");
    let event = telemetry
        .message()
        .await
        .expect("receive telemetry")
        .expect("telemetry event");
    assert_eq!(event.monotonic_ns, 42);
    assert!(matches!(event.payload, Payload::Group(group) if group.group == GROUP));
    let event = telemetry
        .message()
        .await
        .expect("receive motor telemetry")
        .expect("motor telemetry event");
    assert_eq!(event.monotonic_ns, 43);
    assert!(matches!(event.payload, Payload::Motor(motor) if motor.motor_name == "motor_a"));
    let event = telemetry
        .message()
        .await
        .expect("receive health telemetry")
        .expect("health telemetry event");
    assert!(matches!(event.payload, Payload::Health(health) if health.component == "can/test_bus"));
    let event = telemetry
        .message()
        .await
        .expect("receive command-policy telemetry")
        .expect("command-policy telemetry event");
    assert!(
        matches!(event.payload, Payload::CommandPolicy(policy) if policy.motor_name == "motor_a")
    );
    assert!(telemetry.message().await.expect("telemetry EOF").is_none());

    let mut lease = client
        .acquire_lease(
            GROUP,
            "integration-test",
            LeaseDuration::For(Duration::from_millis(1500)),
        )
        .await
        .expect("acquire lease");
    assert_eq!(lease.status(), LeaseStatus::Active);
    assert!(!format!("{lease:?}").contains(TOKEN));
    assert_eq!(
        lease.enable().await.expect("enable group"),
        EnumValue::Known(GroupState::Damped)
    );
    assert_eq!(
        lease
            .start(
                CommandValidity::For(Duration::from_millis(100)),
                vec![MotorCommand {
                    motor_name: "motor_a".into(),
                    kp: 10.0,
                    kd: 1.0,
                    ..Default::default()
                }],
            )
            .await
            .expect("start control"),
        EnumValue::Known(GroupState::Running)
    );

    let stream = lease
        .open_command_stream(CommandStreamOptions::default().queue_capacity(2))
        .await
        .expect("open command stream");
    assert!(matches!(
        lease
            .open_command_stream(CommandStreamOptions::default())
            .await,
        Err(Error::CommandStreamAlreadyOpen)
    ));
    let (mut commands, mut acknowledgements) = stream.split();
    let sequence = commands
        .send(
            CommandValidity::For(Duration::from_millis(100)),
            vec![MotorCommand {
                motor_name: "motor_a".into(),
                kp: 10.0,
                kd: 1.0,
                ..Default::default()
            }],
        )
        .await
        .expect("send command");
    assert_eq!(sequence, 1);
    let acknowledgement = acknowledgements
        .message()
        .await
        .expect("receive acknowledgement")
        .expect("command acknowledgement")
        .require_accepted()
        .expect("accepted command");
    assert_eq!(acknowledgement.sequence, 1);
    let second_sequence = commands
        .send(
            CommandValidity::For(Duration::from_millis(100)),
            vec![MotorCommand {
                motor_name: "motor_a".into(),
                ..Default::default()
            }],
        )
        .await
        .expect("send second command");
    assert_eq!(second_sequence, 2);
    assert_eq!(
        acknowledgements
            .message()
            .await
            .expect("receive second acknowledgement")
            .expect("second command acknowledgement")
            .require_accepted()
            .expect("accepted second command")
            .sequence,
        2
    );

    let original_deadline = lease.deadline();
    let shortened_deadline = lease
        .renew(LeaseDuration::For(Duration::from_millis(500)))
        .await
        .expect("renew lease");
    assert!(shortened_deadline.expires_at() < original_deadline.expires_at());
    assert_eq!(
        lease.stop().await.expect("stop control"),
        EnumValue::Known(GroupState::Damped)
    );
    assert_eq!(
        lease.disable().await.expect("disable group"),
        EnumValue::Known(GroupState::Idle)
    );
    assert_eq!(
        lease.release().await.expect("release lease"),
        EnumValue::Known(GroupState::Idle)
    );
    assert!(matches!(
        commands
            .send(CommandValidity::GroupDefault, Vec::new())
            .await,
        Err(Error::LeaseInactive(LeaseStatus::Released))
    ));

    drop(commands);
    drop(acknowledgements);
    drop(telemetry);
    drop(client);
    {
        let recorded = server.state.lock().expect("fake state mutex poisoned");
        assert_eq!(recorded.acquired.len(), 1);
        assert_eq!(recorded.acquired[0].group, GROUP);
        assert_eq!(recorded.acquired[0].requested_duration_ms, 1500);
        assert_eq!(recorded.renewed[0].token, TOKEN);
        assert_eq!(recorded.commands.len(), 2);
        assert_eq!(recorded.commands[0].group, GROUP);
        assert_eq!(recorded.commands[0].lease_token, TOKEN);
        assert_eq!(recorded.commands[0].sequence, 1);
        assert_eq!(recorded.commands[0].valid_for_ms, 100);
        assert_eq!(recorded.commands[0].commands[0].motor_name, "motor_a");
        assert_eq!(recorded.commands[1].sequence, 2);
        assert_eq!(recorded.released[0].token, TOKEN);
        assert_eq!(
            recorded
                .lifecycle
                .iter()
                .map(|call| call.action)
                .collect::<Vec<_>>(),
            ["enable", "start", "stop", "disable"]
        );
        assert_eq!(recorded.telemetry[0].groups, [GROUP]);
        assert!(recorded.telemetry[0].include_motors);
        assert!(recorded.telemetry[0].include_health);
    }
    server.stop().await;
}

#[tokio::test]
async fn dropping_a_lease_stops_bound_senders_without_hidden_release() {
    let server = FakeServer::start().await;
    let client = HardwareClient::connect(server.endpoint.clone())
        .await
        .expect("connect client");
    let lease = client
        .acquire_lease(GROUP, "drop-test", LeaseDuration::ServerDefault)
        .await
        .expect("acquire lease");
    let (mut commands, responses) = lease
        .open_command_stream(CommandStreamOptions::default())
        .await
        .expect("open command stream")
        .split();

    drop(lease);
    assert!(matches!(
        commands
            .send(CommandValidity::GroupDefault, Vec::new())
            .await,
        Err(Error::LeaseInactive(LeaseStatus::Dropped))
    ));
    drop(commands);
    drop(responses);
    drop(client);
    assert!(
        server
            .state
            .lock()
            .expect("fake state mutex poisoned")
            .released
            .is_empty()
    );
    server.stop().await;
}

#[tokio::test]
async fn successful_in_flight_renewal_can_restore_a_locally_expired_deadline() {
    let server = FakeServer::start().await;
    server
        .settings
        .acquire_expires_ms
        .store(20, Ordering::Relaxed);
    server.settings.renew_delay_ms.store(60, Ordering::Relaxed);
    let client = HardwareClient::connect(server.endpoint.clone())
        .await
        .expect("connect client");
    let mut lease = client
        .acquire_lease(GROUP, "renew-race", LeaseDuration::ServerDefault)
        .await
        .expect("acquire lease");

    let renewal = lease.renew(LeaseDuration::For(Duration::from_secs(1)));
    let observe_expiry = async {
        tokio::time::sleep(Duration::from_millis(35)).await;
        assert_eq!(lease.status(), LeaseStatus::Expired);
    };
    let (renewed, ()) = tokio::join!(renewal, observe_expiry);
    assert!(renewed.expect("complete renewal").remaining() > Duration::from_millis(800));
    assert_eq!(lease.status(), LeaseStatus::Active);
    lease.release().await.expect("release renewed lease");

    drop(lease);
    drop(client);
    server.stop().await;
}

#[tokio::test]
async fn locally_expired_lease_still_allows_best_effort_cleanup() {
    let server = FakeServer::start().await;
    server
        .settings
        .acquire_expires_ms
        .store(10, Ordering::Relaxed);
    let client = HardwareClient::connect(server.endpoint.clone())
        .await
        .expect("connect client");
    let mut lease = client
        .acquire_lease(GROUP, "cleanup", LeaseDuration::ServerDefault)
        .await
        .expect("acquire lease");
    tokio::time::sleep(Duration::from_millis(15)).await;
    assert_eq!(lease.status(), LeaseStatus::Expired);

    assert_eq!(
        lease.stop().await.expect("best-effort stop"),
        EnumValue::Known(GroupState::Damped)
    );
    assert_eq!(
        lease.disable().await.expect("best-effort disable"),
        EnumValue::Known(GroupState::Idle)
    );
    assert_eq!(
        lease.release().await.expect("best-effort release"),
        EnumValue::Known(GroupState::Idle)
    );

    drop(lease);
    drop(client);
    server.stop().await;
}

#[tokio::test]
async fn lease_lost_acknowledgement_invalidates_the_lease_and_sender() {
    let server = FakeServer::start().await;
    let client = HardwareClient::connect(server.endpoint.clone())
        .await
        .expect("connect client");
    let lease = client
        .acquire_lease(GROUP, "lease-lost", LeaseDuration::ServerDefault)
        .await
        .expect("acquire lease");
    let (mut commands, mut responses) = lease
        .open_command_stream(CommandStreamOptions::default())
        .await
        .expect("open command stream")
        .split();
    commands
        .send(
            CommandValidity::GroupDefault,
            vec![MotorCommand {
                motor_name: "lease_lost".into(),
                ..Default::default()
            }],
        )
        .await
        .expect("send command");
    let acknowledgement = responses
        .message()
        .await
        .expect("receive acknowledgement")
        .expect("lease-lost acknowledgement");
    assert_eq!(
        acknowledgement.result,
        EnumValue::Known(CommandResult::LeaseLost)
    );
    assert_eq!(lease.status(), LeaseStatus::Lost);
    assert!(matches!(
        commands
            .send(CommandValidity::GroupDefault, Vec::new())
            .await,
        Err(Error::LeaseInactive(LeaseStatus::Lost))
    ));

    drop(commands);
    drop(responses);
    drop(lease);
    drop(client);
    server.stop().await;
}

#[tokio::test]
async fn successful_release_is_authoritative_over_a_lease_lost_acknowledgement() {
    let server = FakeServer::start().await;
    let client = HardwareClient::connect(server.endpoint.clone())
        .await
        .expect("connect client");
    let mut lease = client
        .acquire_lease(GROUP, "release-ack-race", LeaseDuration::ServerDefault)
        .await
        .expect("acquire lease");
    let (mut commands, mut responses) = lease
        .open_command_stream(CommandStreamOptions::default())
        .await
        .expect("open command stream")
        .split();
    commands
        .send(
            CommandValidity::GroupDefault,
            vec![MotorCommand {
                motor_name: "lease_lost".into(),
                ..Default::default()
            }],
        )
        .await
        .expect("send command");

    let (release, acknowledgement) = tokio::join!(lease.release(), responses.message());
    assert_eq!(
        release.expect("release lease"),
        EnumValue::Known(GroupState::Idle)
    );
    assert_eq!(
        acknowledgement
            .expect("receive acknowledgement")
            .expect("lease-lost acknowledgement")
            .result,
        EnumValue::Known(CommandResult::LeaseLost)
    );
    assert_eq!(lease.status(), LeaseStatus::Released);

    drop(commands);
    drop(responses);
    drop(lease);
    drop(client);
    server.stop().await;
}

#[tokio::test]
async fn failed_release_is_terminal_locally() {
    let server = FakeServer::start().await;
    server.settings.fail_release.store(true, Ordering::Relaxed);
    let client = HardwareClient::connect(server.endpoint.clone())
        .await
        .expect("connect client");
    let mut lease = client
        .acquire_lease(GROUP, "release-failure", LeaseDuration::ServerDefault)
        .await
        .expect("acquire lease");

    let error = lease.release().await.expect_err("release should fail");
    assert_eq!(
        error.rpc_status().map(Status::code),
        Some(tonic::Code::FailedPrecondition)
    );
    assert_eq!(lease.status(), LeaseStatus::Lost);
    assert!(matches!(
        lease.enable().await,
        Err(Error::LeaseInactive(LeaseStatus::Lost))
    ));

    drop(lease);
    drop(client);
    server.stop().await;
}

#[tokio::test]
async fn release_waits_for_an_in_flight_renewal() {
    let server = FakeServer::start().await;
    server.settings.renew_delay_ms.store(60, Ordering::Relaxed);
    let client = HardwareClient::connect(server.endpoint.clone())
        .await
        .expect("connect client");
    let mut lease = client
        .acquire_lease(GROUP, "renew-release", LeaseDuration::ServerDefault)
        .await
        .expect("acquire lease");
    let renewer = lease.renewer();
    let renewal = tokio::spawn(async move {
        renewer
            .renew(LeaseDuration::For(Duration::from_secs(1)))
            .await
    });
    server.settings.renew_started.notified().await;

    assert_eq!(
        lease.release().await.expect("release after renewal"),
        EnumValue::Known(GroupState::Idle)
    );
    renewal
        .await
        .expect("join renewal")
        .expect("complete renewal");
    assert_eq!(lease.status(), LeaseStatus::Released);

    drop(lease);
    drop(client);
    server.stop().await;
}

#[tokio::test]
async fn reopening_a_command_stream_keeps_the_lease_sequence() {
    let server = FakeServer::start().await;
    let client = HardwareClient::connect(server.endpoint.clone())
        .await
        .expect("connect client");
    let lease = client
        .acquire_lease(GROUP, "reopen", LeaseDuration::ServerDefault)
        .await
        .expect("acquire lease");

    let mut first = lease
        .open_command_stream(CommandStreamOptions::default())
        .await
        .expect("open first stream");
    assert_eq!(
        first
            .send(CommandValidity::GroupDefault, Vec::new())
            .await
            .expect("send first command"),
        1
    );
    first
        .message()
        .await
        .expect("first acknowledgement")
        .expect("first acknowledgement value");
    drop(first);

    let mut second = lease
        .open_command_stream(CommandStreamOptions::default())
        .await
        .expect("open replacement stream");
    assert_eq!(
        second
            .send(CommandValidity::GroupDefault, Vec::new())
            .await
            .expect("send second command"),
        2
    );

    drop(second);
    drop(lease);
    drop(client);
    server.stop().await;
}

#[tokio::test]
async fn ergonomic_robot_api_hides_lease_stream_and_acknowledgement_workflow() {
    let server = FakeServer::start().await;
    let raw = HardwareClient::connect(server.endpoint.clone())
        .await
        .expect("connect client");
    let robot = Robot::from_hardware_client(raw)
        .await
        .expect("load robot inventory");
    let group = robot.group(GROUP).expect("resolve group");
    assert_eq!(
        group.coordinate_names().collect::<Vec<_>>(),
        ["coordinate_a"]
    );
    let bus = robot.buses().next().expect("configured bus");
    assert_eq!(bus.name(), "test_bus");
    assert_eq!(bus.interface(), "vcan-test");
    assert_eq!(bus.host_id(), 127);
    assert_eq!(
        bus.initial_status().map(|status| status.state()),
        Some(bello_hardware_client::DeviceState::Online)
    );
    let serial = robot
        .serial_devices()
        .next()
        .expect("configured serial device");
    assert_eq!(serial.name(), "test_serial");
    assert_eq!(serial.port(), "/dev/ttyTEST0");
    assert_eq!(serial.baud_rate(), 921_600);
    let imu = robot.imus().next().expect("configured IMU");
    assert_eq!(imu.name(), "test_imu");
    assert_eq!(imu.serial_device(), "test_serial");

    let initial = group
        .frame()
        .coordinate(
            "coordinate_a",
            CoordinateTarget::position(0.0).gains(20.0, 1.0),
        )
        .expect("assign initial target")
        .build()
        .expect("build initial frame");
    let next = group
        .frame()
        .coordinate(
            "coordinate_a",
            CoordinateTarget::position(0.1).gains(20.0, 1.0),
        )
        .expect("assign next target")
        .build()
        .expect("build next frame");

    let mut control = group
        .control_with(
            ControlOptions::new("ergonomic-test")
                .lease_duration(Duration::from_secs(1))
                .command_validity(Duration::from_millis(100)),
        )
        .await
        .expect("acquire control");
    control.start(initial).await.expect("start control");
    control.send(next).await.expect("send accepted frame");
    control.shutdown().await.expect("clean shutdown");

    {
        let recorded = server.state.lock().expect("fake state mutex poisoned");
        assert_eq!(recorded.starts.len(), 1);
        assert_eq!(recorded.starts[0].commands[0].motor_name, "motor_a");
        assert_eq!(recorded.starts[0].valid_for_ms, 100);
        assert_eq!(recorded.commands.len(), 1);
        assert_eq!(recorded.commands[0].commands[0].motor_name, "motor_a");
        assert_eq!(
            recorded
                .lifecycle
                .iter()
                .map(|call| call.action)
                .collect::<Vec<_>>(),
            ["enable", "start", "stop", "disable"]
        );
        assert_eq!(recorded.released.len(), 1);
    }

    server.stop().await;
}

#[tokio::test]
async fn ergonomic_state_stream_aggregates_events_without_protobuf_matching() {
    let server = FakeServer::start().await;
    let robot = Robot::connect(server.endpoint.clone())
        .await
        .expect("connect robot");
    let mut states = robot.states().await.expect("open robot state stream");
    let state = states
        .changed()
        .await
        .expect("receive state")
        .expect("state event");
    let group = state.group(GROUP).expect("aggregated group state");
    assert_eq!(group.monotonic_ns, 42);
    assert_eq!(group.state, bello_hardware_client::GroupState::Idle);
    assert!(matches!(
        state.last_change(),
        Some(bello_hardware_client::RobotStateChange::Group(changed))
            if changed.group == GROUP
    ));
    let state = states
        .changed()
        .await
        .expect("receive motor state")
        .expect("motor state event");
    let coordinate = state
        .coordinate("coordinate_a")
        .expect("aggregated coordinate state");
    assert_eq!(coordinate.position, 0.25);
    assert_eq!(coordinate.temperature, 32.0);
    assert!(matches!(
        state.last_change(),
        Some(bello_hardware_client::RobotStateChange::Motor(changed))
            if changed.motor == "motor_a"
    ));
    let state = states
        .changed()
        .await
        .expect("receive IMU state")
        .expect("IMU state event");
    let imu = state.imu("test_imu").expect("aggregated IMU state");
    assert_eq!(imu.sample_age, Duration::from_millis(4));
    assert_eq!(imu.linear_acceleration.z, 9.81);
    assert!(matches!(
        state.last_change(),
        Some(bello_hardware_client::RobotStateChange::Imu(changed))
            if changed.imu == "test_imu"
    ));
    let state = states
        .changed()
        .await
        .expect("receive health state")
        .expect("health state event");
    let health = state.health("can/test_bus").expect("aggregated bus health");
    assert_eq!(health.dropped_events, 7);
    assert_eq!(
        health.status.as_ref().map(|status| status.state()),
        Some(bello_hardware_client::DeviceState::Degraded)
    );
    assert_eq!(state.health_snapshots().count(), 1);
    assert!(matches!(
        state.last_change(),
        Some(bello_hardware_client::RobotStateChange::Health(changed))
            if changed.component == "can/test_bus"
    ));
    let state = states
        .changed()
        .await
        .expect("receive command-policy state")
        .expect("command-policy state event");
    let policies = state.command_policies().collect::<Vec<_>>();
    assert_eq!(policies.len(), 1);
    assert_eq!(
        policies[0].check,
        bello_hardware_client::CommandPolicyCheck::Limit
    );
    assert_eq!(
        policies[0].behavior,
        bello_hardware_client::CommandBehavior::Clamp
    );
    assert!(policies[0].command_modified);
    assert!(!policies[0].command_rejected);
    assert!(matches!(
        state.last_change(),
        Some(bello_hardware_client::RobotStateChange::CommandPolicy(changed))
            if changed.motor == "motor_a"
    ));

    server.stop().await;
}

#[tokio::test]
async fn ergonomic_control_renews_short_leases_automatically() {
    let server = FakeServer::start().await;
    server
        .settings
        .acquire_expires_ms
        .store(40, Ordering::Relaxed);
    let robot = Robot::connect(server.endpoint.clone())
        .await
        .expect("connect robot");
    let group = robot.group(GROUP).expect("resolve group");
    let control = group
        .control_with(
            ControlOptions::new("automatic-renewal").lease_duration(Duration::from_millis(40)),
        )
        .await
        .expect("acquire control");

    tokio::time::timeout(Duration::from_secs(1), async {
        loop {
            if !server
                .state
                .lock()
                .expect("fake state mutex poisoned")
                .renewed
                .is_empty()
            {
                break;
            }
            tokio::time::sleep(Duration::from_millis(2)).await;
        }
    })
    .await
    .expect("automatic renewal");

    control.shutdown().await.expect("release control");
    server.stop().await;
}

#[tokio::test]
async fn ergonomic_control_surfaces_automatic_renewal_failure() {
    let server = FakeServer::start().await;
    server
        .settings
        .acquire_expires_ms
        .store(40, Ordering::Relaxed);
    server.settings.fail_renew.store(true, Ordering::Relaxed);
    let robot = Robot::connect(server.endpoint.clone())
        .await
        .expect("connect robot");
    let group = robot.group(GROUP).expect("resolve group");
    let mut control = group
        .control_with(
            ControlOptions::new("failed-renewal").lease_duration(Duration::from_millis(40)),
        )
        .await
        .expect("acquire control");
    let frame = group
        .frame()
        .coordinate("coordinate_a", CoordinateTarget::damping(1.0))
        .expect("assign target")
        .build()
        .expect("build frame");

    tokio::time::timeout(Duration::from_secs(1), async {
        loop {
            if !server
                .state
                .lock()
                .expect("fake state mutex poisoned")
                .renewed
                .is_empty()
            {
                break;
            }
            tokio::time::sleep(Duration::from_millis(2)).await;
        }
    })
    .await
    .expect("failed renewal attempt");
    tokio::time::sleep(Duration::from_millis(5)).await;

    assert!(matches!(
        control.start(frame).await,
        Err(Error::RenewalFailed { .. })
    ));
    let _ = control.shutdown().await;
    server.stop().await;
}

#[tokio::test]
async fn ergonomic_control_never_retries_after_a_rejected_running_command() {
    let server = FakeServer::start().await;
    let robot = Robot::connect(server.endpoint.clone())
        .await
        .expect("connect robot");
    let group = robot.group(GROUP).expect("resolve group");
    let frame = || {
        group
            .frame()
            .coordinate("coordinate_a", CoordinateTarget::damping(1.0))
            .expect("assign target")
            .build()
            .expect("build frame")
    };
    let mut control = group
        .control("rejection-test")
        .await
        .expect("acquire control");
    control.start(frame()).await.expect("start control");
    server
        .settings
        .reject_commands
        .store(true, Ordering::Relaxed);

    assert!(matches!(
        control.send(frame()).await,
        Err(Error::RejectedCommand { .. })
    ));
    assert_eq!(
        control.state(),
        bello_hardware_client::ControlState::Uncertain
    );
    assert!(matches!(
        control.send(frame()).await,
        Err(Error::InvalidSessionState { .. })
    ));

    control.shutdown().await.expect("clean up control");
    server.stop().await;
}

#[tokio::test]
async fn ergonomic_resume_rolls_an_ambiguous_start_back_to_damped() {
    let server = FakeServer::start().await;
    let robot = Robot::connect(server.endpoint.clone())
        .await
        .expect("connect robot");
    let group = robot.group(GROUP).expect("resolve group");
    let frame = || {
        group
            .frame()
            .coordinate("coordinate_a", CoordinateTarget::damping(1.0))
            .expect("assign target")
            .build()
            .expect("build frame")
    };
    let mut control = group
        .control("resume-rollback")
        .await
        .expect("acquire control");

    control.start(frame()).await.expect("start control");
    control.stop().await.expect("stop control");
    server.settings.fail_start.store(true, Ordering::Relaxed);
    assert!(control.resume(frame()).await.is_err());
    assert_eq!(control.state(), bello_hardware_client::ControlState::Damped);

    control.shutdown().await.expect("shutdown damped control");
    server.stop().await;
}
