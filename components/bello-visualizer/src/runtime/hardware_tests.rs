use std::collections::{BTreeMap, BTreeSet};
use std::fs;
use std::pin::Pin;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use bello_hardware_client::raw::proto;
use bello_hardware_client::raw::proto::hardware_service_server::{
    HardwareService, HardwareServiceServer,
};
use bello_hardware_client::raw::proto::watch_telemetry_response::Payload;
use futures_core::Stream;
use tokio::net::TcpListener;
use tokio::sync::{oneshot, watch};
use tokio_stream::wrappers::TcpListenerStream;
use tonic::transport::Server;
use tonic::{Request, Response, Status, Streaming};

use super::*;
use crate::config::{BoundedDefault, CoordinateControlProfile, ResolvedGroupControlProfile};

const GROUP: &str = "legs";
const ISOLATED_GROUP: &str = "arms";
const BUS: &str = "body/can";
const SERIAL: &str = "body/serial";
const IMU: &str = "body_imu";
const MOTOR_A: &str = "hip_motor";
const MOTOR_B: &str = "knee_motor";
const MOTOR_C: &str = "shoulder_motor";
const COORDINATE_A: &str = "hip";
const COORDINATE_B: &str = "knee";
const COORDINATE_C: &str = "shoulder";
const TOKEN: &str = "visualizer-test-token";
const SOURCE_NS: u64 = 9_876_543;

type ResponseStream<T> = Pin<Box<dyn Stream<Item = Result<T, Status>> + Send + 'static>>;

fn motor_limits() -> proto::MotorLimits {
    proto::MotorLimits {
        position_min: -2.0,
        position_max: 2.0,
        velocity_min: -5.0,
        velocity_max: 5.0,
        torque_min: -8.0,
        torque_max: 8.0,
        kp_min: 0.0,
        kp_max: 100.0,
        kd_min: 0.0,
        kd_max: 10.0,
    }
}

#[derive(Clone)]
struct FakeHardware {
    recorded: Arc<Mutex<Recorded>>,
    disconnect: watch::Sender<bool>,
    inventory: proto::ListHardwareResponse,
    reject_after_first_command_for: Option<String>,
}

#[derive(Default)]
struct Recorded {
    inventory_requests: usize,
    telemetry_requests: Vec<proto::WatchTelemetryRequest>,
    acquired: Vec<proto::AcquireLeaseRequest>,
    renewed: Vec<proto::RenewLeaseRequest>,
    released: Vec<proto::ReleaseLeaseRequest>,
    lifecycle: Vec<LifecycleCall>,
    commands: Vec<TimedCommand>,
}

struct TimedCommand {
    received: Instant,
    request: proto::StreamCommandsRequest,
}

#[derive(Debug, PartialEq, Eq)]
struct LifecycleCall {
    action: &'static str,
    group: String,
    token: String,
}

impl FakeHardware {
    fn lifecycle(&self, action: &'static str, group: String, token: String) {
        self.recorded
            .lock()
            .expect("fake hardware state mutex poisoned")
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
    type WatchTelemetryStream = ResponseStream<proto::WatchTelemetryResponse>;
    type StreamCommandsStream = ResponseStream<proto::StreamCommandsResponse>;

    async fn list_hardware(
        &self,
        _: Request<proto::ListHardwareRequest>,
    ) -> Result<Response<proto::ListHardwareResponse>, Status> {
        self.recorded
            .lock()
            .expect("fake hardware state mutex poisoned")
            .inventory_requests += 1;
        Ok(Response::new(self.inventory.clone()))
    }

    async fn watch_telemetry(
        &self,
        request: Request<proto::WatchTelemetryRequest>,
    ) -> Result<Response<Self::WatchTelemetryStream>, Status> {
        let filter = request.into_inner();
        self.recorded
            .lock()
            .expect("fake hardware state mutex poisoned")
            .telemetry_requests
            .push(filter.clone());
        let mut disconnect = self.disconnect.subscribe();
        let output = async_stream::try_stream! {
            for event in telemetry(&filter) {
                yield event;
            }
            while !*disconnect.borrow() {
                if disconnect.changed().await.is_err() {
                    break;
                }
            }
        };
        Ok(Response::new(Box::pin(output)))
    }

    async fn get_state_frame(
        &self,
        _: Request<proto::GetStateFrameRequest>,
    ) -> Result<Response<proto::GetStateFrameResponse>, Status> {
        Err(Status::unimplemented("not used by this fake"))
    }

    async fn acquire_lease(
        &self,
        request: Request<proto::AcquireLeaseRequest>,
    ) -> Result<Response<proto::AcquireLeaseResponse>, Status> {
        let request = request.into_inner();
        let group = request.group.clone();
        let token = lease_token(&group);
        self.recorded
            .lock()
            .expect("fake hardware state mutex poisoned")
            .acquired
            .push(request);
        Ok(Response::new(proto::AcquireLeaseResponse {
            group,
            token,
            expires_in_ms: 10_000,
        }))
    }

    async fn renew_lease(
        &self,
        request: Request<proto::RenewLeaseRequest>,
    ) -> Result<Response<proto::RenewLeaseResponse>, Status> {
        let request = request.into_inner();
        let group = lease_group(&request.token);
        let token = request.token.clone();
        let expires_in_ms = request.requested_duration_ms.max(10_000);
        self.recorded
            .lock()
            .expect("fake hardware state mutex poisoned")
            .renewed
            .push(request);
        Ok(Response::new(proto::RenewLeaseResponse {
            group,
            token,
            expires_in_ms,
        }))
    }

    async fn release_lease(
        &self,
        request: Request<proto::ReleaseLeaseRequest>,
    ) -> Result<Response<proto::ReleaseLeaseResponse>, Status> {
        self.recorded
            .lock()
            .expect("fake hardware state mutex poisoned")
            .released
            .push(request.into_inner());
        Ok(Response::new(proto::ReleaseLeaseResponse {
            state: proto::GroupState::Idle as i32,
        }))
    }

    async fn enable_group(
        &self,
        request: Request<proto::EnableGroupRequest>,
    ) -> Result<Response<proto::EnableGroupResponse>, Status> {
        let request = request.into_inner();
        self.lifecycle("enable", request.group, request.lease_token);
        Ok(Response::new(proto::EnableGroupResponse {
            state: proto::GroupState::Damped as i32,
        }))
    }

    async fn start_control(
        &self,
        request: Request<proto::StartControlRequest>,
    ) -> Result<Response<proto::StartControlResponse>, Status> {
        let request = request.into_inner();
        self.lifecycle("start", request.group, request.lease_token);
        Ok(Response::new(proto::StartControlResponse {
            state: proto::GroupState::Running as i32,
        }))
    }

    async fn stop_control(
        &self,
        request: Request<proto::StopControlRequest>,
    ) -> Result<Response<proto::StopControlResponse>, Status> {
        let request = request.into_inner();
        self.lifecycle("stop", request.group, request.lease_token);
        Ok(Response::new(proto::StopControlResponse {
            state: proto::GroupState::Damped as i32,
        }))
    }

    async fn disable_group(
        &self,
        request: Request<proto::DisableGroupRequest>,
    ) -> Result<Response<proto::DisableGroupResponse>, Status> {
        let request = request.into_inner();
        self.lifecycle("disable", request.group, request.lease_token);
        Ok(Response::new(proto::DisableGroupResponse {
            state: proto::GroupState::Idle as i32,
        }))
    }

    async fn recover_group(
        &self,
        request: Request<proto::RecoverGroupRequest>,
    ) -> Result<Response<proto::RecoverGroupResponse>, Status> {
        let request = request.into_inner();
        self.lifecycle("recover", request.group, request.lease_token);
        Ok(Response::new(proto::RecoverGroupResponse {
            state: proto::GroupState::Idle as i32,
        }))
    }

    async fn stream_commands(
        &self,
        request: Request<Streaming<proto::StreamCommandsRequest>>,
    ) -> Result<Response<Self::StreamCommandsStream>, Status> {
        let mut requests = request.into_inner();
        let recorded = Arc::clone(&self.recorded);
        let reject_after_first_command_for = self.reject_after_first_command_for.clone();
        let output = async_stream::try_stream! {
            while let Some(command) = requests.message().await? {
                let command_number = {
                    let mut recorded = recorded
                        .lock()
                        .expect("fake hardware state mutex poisoned");
                    recorded.commands.push(TimedCommand {
                        received: Instant::now(),
                        request: command.clone(),
                    });
                    recorded
                        .commands
                        .iter()
                        .filter(|timed| timed.request.group == command.group)
                        .count()
                };
                let rejected = reject_after_first_command_for.as_deref()
                    == Some(command.group.as_str())
                    && command_number > 1;
                yield proto::StreamCommandsResponse {
                    group: command.group,
                    sequence: command.sequence,
                    result: if rejected {
                        proto::CommandResult::Unsafe as i32
                    } else {
                        proto::CommandResult::Accepted as i32
                    },
                    detail: if rejected {
                        "synthetic isolated rejection".into()
                    } else {
                        "accepted".into()
                    },
                    state: if rejected {
                        proto::GroupState::Fault as i32
                    } else {
                        proto::GroupState::Running as i32
                    },
                };
            }
        };
        Ok(Response::new(Box::pin(output)))
    }
}

fn lease_token(group: &str) -> String {
    if group == GROUP {
        TOKEN.into()
    } else {
        format!("{TOKEN}:{group}")
    }
}

fn lease_group(token: &str) -> String {
    token
        .strip_prefix(&format!("{TOKEN}:"))
        .unwrap_or(GROUP)
        .to_owned()
}

struct FakeServer {
    endpoint: String,
    recorded: Arc<Mutex<Recorded>>,
    disconnect: watch::Sender<bool>,
    shutdown: Option<oneshot::Sender<()>>,
    worker: tokio::task::JoinHandle<Result<(), tonic::transport::Error>>,
}

impl FakeServer {
    async fn start() -> Self {
        Self::start_with(inventory(), None).await
    }

    async fn start_with(
        inventory: proto::ListHardwareResponse,
        reject_after_first_command_for: Option<&str>,
    ) -> Self {
        let listener = TcpListener::bind("127.0.0.1:0")
            .await
            .expect("bind fake hardware server");
        let address = listener.local_addr().expect("fake server address");
        let (disconnect, _) = watch::channel(false);
        let service = FakeHardware {
            recorded: Arc::new(Mutex::new(Recorded::default())),
            disconnect: disconnect.clone(),
            inventory,
            reject_after_first_command_for: reject_after_first_command_for.map(str::to_owned),
        };
        let recorded = Arc::clone(&service.recorded);
        let (shutdown, shutdown_receiver) = oneshot::channel();
        let worker = tokio::spawn(async move {
            Server::builder()
                .add_service(HardwareServiceServer::new(service))
                .serve_with_incoming_shutdown(TcpListenerStream::new(listener), async {
                    let _ = shutdown_receiver.await;
                })
                .await
        });
        Self {
            endpoint: format!("http://{address}"),
            recorded,
            disconnect,
            shutdown: Some(shutdown),
            worker,
        }
    }

    fn disconnect_telemetry(&self) {
        self.disconnect.send_replace(true);
    }

    fn resume_telemetry(&self) {
        self.disconnect.send_replace(false);
    }

    async fn stop(mut self) {
        if let Some(shutdown) = self.shutdown.take() {
            let _ = shutdown.send(());
        }
        self.worker
            .await
            .expect("join fake hardware server")
            .expect("run fake hardware server");
    }
}

fn inventory() -> proto::ListHardwareResponse {
    let online = |detail: &str, transition_count| proto::DeviceStatus {
        state: proto::DeviceState::Online as i32,
        detail: detail.into(),
        transition_count,
    };
    proto::ListHardwareResponse {
        buses: vec![proto::BusInfo {
            name: BUS.into(),
            interface: "vcan42".into(),
            host_id: 127,
            status: Some(online("bus online", 2)),
        }],
        serial_devices: vec![proto::SerialDeviceInfo {
            name: SERIAL.into(),
            port: "/dev/ttyTEST0".into(),
            baud_rate: 921_600,
            status: Some(online("serial online", 3)),
        }],
        groups: vec![proto::GroupInfo {
            name: GROUP.into(),
            control_rate_hz: 250.0,
            state: proto::GroupState::Idle as i32,
            motors: vec![
                proto::MotorAddress {
                    bus: BUS.into(),
                    id: 1,
                },
                proto::MotorAddress {
                    bus: BUS.into(),
                    id: 2,
                },
            ],
        }],
        motors: vec![
            proto::MotorInfo {
                name: MOTOR_A.into(),
                coordinate: COORDINATE_A.into(),
                group: GROUP.into(),
                address: Some(proto::MotorAddress {
                    bus: BUS.into(),
                    id: 1,
                }),
                model: "test-a".into(),
                direction: 1,
                status: Some(online("hip online", 4)),
                limits: Some(motor_limits()),
            },
            proto::MotorInfo {
                name: MOTOR_B.into(),
                coordinate: COORDINATE_B.into(),
                group: GROUP.into(),
                address: Some(proto::MotorAddress {
                    bus: BUS.into(),
                    id: 2,
                }),
                model: "test-b".into(),
                direction: -1,
                status: Some(online("knee online", 5)),
                limits: Some(motor_limits()),
            },
        ],
        imus: vec![proto::ImuInfo {
            name: IMU.into(),
            serial_device: SERIAL.into(),
            status: Some(online("IMU online", 6)),
        }],
    }
}

fn isolation_inventory() -> proto::ListHardwareResponse {
    let mut inventory = inventory();
    inventory.groups.push(proto::GroupInfo {
        name: ISOLATED_GROUP.into(),
        control_rate_hz: 250.0,
        state: proto::GroupState::Idle as i32,
        motors: vec![proto::MotorAddress {
            bus: BUS.into(),
            id: 3,
        }],
    });
    inventory.motors.push(proto::MotorInfo {
        name: MOTOR_C.into(),
        coordinate: COORDINATE_C.into(),
        group: ISOLATED_GROUP.into(),
        address: Some(proto::MotorAddress {
            bus: BUS.into(),
            id: 3,
        }),
        model: "test-c".into(),
        direction: 1,
        status: Some(proto::DeviceStatus {
            state: proto::DeviceState::Online as i32,
            detail: "shoulder online".into(),
            transition_count: 7,
        }),
        limits: Some(motor_limits()),
    });
    inventory
}

fn telemetry(filter: &proto::WatchTelemetryRequest) -> Vec<proto::WatchTelemetryResponse> {
    let mut events = vec![response(Payload::Group(proto::GroupTelemetry {
        group: GROUP.into(),
        state: proto::GroupState::Running as i32,
        command_count: 81,
        control_tick: 82,
        jitter_us: -7,
        detail: String::new(),
        cause: proto::GroupCause::Unspecified as i32,
    }))];
    if filter.include_motors {
        events.extend([
            response(Payload::Motor(proto::MotorTelemetry {
                group: GROUP.into(),
                motor_name: MOTOR_A.into(),
                position: 0.25,
                velocity: 1.5,
                torque: 2.5,
                temperature: 42.5,
                fault_bits: 0x12,
                mode_status: 0x34,
                feedback_age_ms: 6,
            })),
            response(Payload::Motor(proto::MotorTelemetry {
                group: GROUP.into(),
                motor_name: MOTOR_B.into(),
                position: -0.75,
                velocity: -1.25,
                torque: -2.25,
                temperature: 43.5,
                fault_bits: 0x56,
                mode_status: 0x78,
                feedback_age_ms: 7,
            })),
        ]);
    }
    if filter.include_imu {
        events.push(response(Payload::Imu(proto::ImuTelemetry {
            imu_name: IMU.into(),
            orientation: Some(proto::Quaternion {
                x: 0.1,
                y: 0.2,
                z: 0.3,
                w: 0.9,
            }),
            angular_velocity: Some(proto::Vector3 {
                x: 1.0,
                y: 2.0,
                z: 3.0,
            }),
            linear_acceleration: Some(proto::Vector3 {
                x: 4.0,
                y: 5.0,
                z: 6.0,
            }),
            device_timestamp_ms: 1234,
            sample_age_ms: 8,
        })));
    }
    if filter.include_health {
        events.push(response(Payload::Health(proto::HealthTelemetry {
            component: format!("can/{BUS}"),
            device_status: Some(proto::DeviceStatus {
                state: proto::DeviceState::Degraded as i32,
                detail: "synthetic congestion".into(),
                transition_count: 9,
            }),
            dropped_events: 3,
        })));
    }
    if filter.include_command_policy_events {
        events.push(response(Payload::CommandPolicy(
            proto::CommandPolicyTelemetry {
                group: GROUP.into(),
                motor_name: MOTOR_A.into(),
                check: proto::CommandPolicyCheck::Limit as i32,
                behavior: proto::CommandBehavior::Clamp as i32,
                detail: "synthetic clamp".into(),
                command_modified: true,
                command_rejected: false,
                fault_response: proto::FaultResponse::Unspecified as i32,
            },
        )));
    }
    events
}

fn response(payload: Payload) -> proto::WatchTelemetryResponse {
    proto::WatchTelemetryResponse {
        monotonic_ns: SOURCE_NS,
        payload: Some(payload),
    }
}

fn endpoint(endpoint: &str) -> EndpointConfig {
    EndpointConfig {
        endpoint: endpoint.into(),
        tls: None,
    }
}

async fn wait_until(mut predicate: impl FnMut() -> bool) {
    tokio::time::timeout(Duration::from_secs(2), async {
        loop {
            if predicate() {
                return;
            }
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
    })
    .await
    .expect("condition did not become true before timeout");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn monitor_publishes_complete_equal_timestamp_state_freezes_and_reconnects() {
    let server = FakeServer::start().await;
    let latest = Arc::new(RwLock::new(DisplaySnapshot::default()));
    let (shutdown, shutdown_receiver) = watch::channel(false);
    let monitor = tokio::spawn(monitor_hardware(
        endpoint(&server.endpoint),
        240.0,
        BTreeMap::from([(GROUP.into(), Duration::from_millis(150))]),
        Arc::clone(&latest),
        shutdown_receiver,
    ));

    wait_until(|| {
        let display = latest.read();
        display.hardware_connected
            && display.motors.len() == 2
            && display
                .motors
                .values()
                .all(|motor| motor.feedback_available)
            && display
                .imus
                .get(IMU)
                .is_some_and(|imu| imu.sample_available)
            && display.health.contains_key(&format!("can/{BUS}"))
            && display
                .events
                .iter()
                .any(|event| event.detail.contains("synthetic clamp"))
    })
    .await;

    {
        let display = latest.read();
        let bus = &display.buses[BUS];
        assert_eq!(bus.interface, "vcan42");
        assert_eq!(bus.host_id, 127);
        assert_eq!(
            bus.status.as_ref().map(|status| status.state),
            Some(DeviceState::Degraded)
        );
        assert_eq!(display.serial_devices[SERIAL].baud_rate, 921_600);

        let hip = &display.motors[COORDINATE_A];
        assert_eq!(hip.motor, MOTOR_A);
        assert_eq!(hip.group, GROUP);
        assert_eq!(hip.bus, BUS);
        assert_eq!(hip.id, 1);
        assert_eq!(hip.model, "test-a");
        assert_eq!(hip.direction, 1);
        assert!(hip.feedback_current_connection);
        assert_eq!(hip.monotonic_ns, Some(SOURCE_NS));
        assert_eq!(hip.position, 0.25);
        assert_eq!(hip.velocity, 1.5);
        assert_eq!(hip.effort, 2.5);
        assert_eq!(hip.temperature, 42.5);
        assert_eq!(hip.fault_bits, 0x12);
        assert_eq!(hip.mode_status, 0x34);
        assert_eq!(hip.stale_after, Duration::from_millis(150));

        let knee = &display.motors[COORDINATE_B];
        assert_eq!(knee.monotonic_ns, Some(SOURCE_NS));
        assert_eq!(knee.position, -0.75);
        assert_eq!(knee.temperature, 43.5);

        let group = &display.groups[GROUP];
        assert_eq!(group.state, GroupState::Running);
        assert!(group.telemetry_available);
        assert!(group.telemetry_current_connection);
        assert_eq!(group.monotonic_ns, Some(SOURCE_NS));
        assert_eq!(group.command_count, 81);
        assert_eq!(group.control_tick, 82);
        assert_eq!(group.jitter_us, -7);

        let imu = &display.imus[IMU];
        assert_eq!(imu.serial_device, SERIAL);
        assert!(imu.sample_current_connection);
        assert_eq!(imu.monotonic_ns, Some(SOURCE_NS));
        assert_eq!(imu.orientation.x, 0.1);
        assert_eq!(imu.orientation.w, 0.9);
        assert_eq!(imu.angular_velocity.z, 3.0);
        assert_eq!(imu.linear_acceleration.x, 4.0);
        assert!(imu.sample_age >= Duration::from_millis(8));
        assert_eq!(imu.device_timestamp, Duration::from_millis(1234));

        let health = &display.health[&format!("can/{BUS}")];
        assert_eq!(health.monotonic_ns, SOURCE_NS);
        assert_eq!(health.dropped_events, 3);
        assert_eq!(display.telemetry_dropped_total, 3);
        assert!(display.events.iter().any(|event| {
            event.category == "command policy"
                && event.component == format!("{GROUP}/{MOTOR_A}")
                && event.monotonic_ns == Some(SOURCE_NS)
        }));
    }

    {
        let recorded = server
            .recorded
            .lock()
            .expect("fake hardware state mutex poisoned");
        assert_eq!(recorded.inventory_requests, 1);
        assert_eq!(recorded.telemetry_requests.len(), 1);
        let filter = &recorded.telemetry_requests[0];
        assert!(filter.include_motors);
        assert!(filter.include_imu);
        assert!(filter.include_health);
        assert!(filter.include_command_policy_events);
    }

    let frozen_hip = latest.read().motors[COORDINATE_A].position;
    server.disconnect_telemetry();
    wait_until(|| !latest.read().hardware_connected).await;
    let age_at_disconnect = latest.read().motors[COORDINATE_A].age;
    tokio::time::sleep(Duration::from_millis(30)).await;
    {
        let display = latest.read();
        assert!(display.hardware_warning.is_some());
        assert_eq!(display.motors[COORDINATE_A].position, frozen_hip);
        assert!(display.motors[COORDINATE_A].feedback_available);
        assert!(!display.motors[COORDINATE_A].feedback_current_connection);
        assert!(!display.groups[GROUP].telemetry_current_connection);
        assert!(!display.imus[IMU].sample_current_connection);
        assert!(display.motors[COORDINATE_A].age >= age_at_disconnect);
    }

    server.resume_telemetry();
    wait_until(|| {
        let reconnected = server
            .recorded
            .lock()
            .expect("fake hardware state mutex poisoned")
            .inventory_requests
            >= 2;
        let display = latest.read();
        reconnected
            && display.hardware_connected
            && display.motors[COORDINATE_A].feedback_current_connection
            && display.groups[GROUP].telemetry_current_connection
            && display.imus[IMU].sample_current_connection
    })
    .await;
    assert_eq!(latest.read().telemetry_dropped_total, 6);

    let _ = shutdown.send(true);
    monitor.await.expect("join hardware monitor");
    server.stop().await;
}

fn bounds(min: f64, max: f64, default: f64) -> BoundedDefault {
    BoundedDefault { min, max, default }
}

fn coordinate_profile() -> CoordinateControlProfile {
    CoordinateControlProfile {
        position: bounds(-2.0, 2.0, 0.0),
        velocity: bounds(-5.0, 5.0, 0.125),
        torque_feedforward: bounds(-8.0, 8.0, 0.25),
        kp: bounds(0.0, 100.0, 20.0),
        kd: bounds(0.0, 10.0, 1.5),
    }
}

fn profile() -> ResolvedGroupControlProfile {
    ResolvedGroupControlProfile {
        command_rate_hz: 25.0,
        validity_ms: 90,
        telemetry_staleness_ms: 200,
        position_slew_rate: 0.5,
        coordinates: BTreeMap::from([
            (COORDINATE_A.into(), coordinate_profile()),
            (COORDINATE_B.into(), coordinate_profile()),
        ]),
    }
}

fn profile_config() -> GroupControlProfile {
    GroupControlProfile {
        command_rate_hz: 25.0,
        validity_ms: 90,
        telemetry_staleness_ms: 200,
        position_slew_rate: 0.5,
        velocity: bounds(-5.0, 5.0, 0.125),
        torque_feedforward: bounds(-8.0, 8.0, 0.25),
        kp: bounds(0.0, 100.0, 20.0),
        kd: bounds(0.0, 10.0, 1.5),
    }
}

fn control_description() -> RobotDescription {
    let directory = tempfile::tempdir().unwrap();
    let path = directory.path().join("control.urdf");
    fs::write(
        &path,
        r#"<robot name="control">
          <link name="base"/><link name="hip_link"/><link name="knee_link"/><link name="shoulder_link"/>
          <joint name="hip" type="revolute">
            <parent link="base"/><child link="hip_link"/>
            <limit lower="-2" upper="2" effort="8" velocity="5"/>
          </joint>
          <joint name="knee" type="revolute">
            <parent link="hip_link"/><child link="knee_link"/>
            <limit lower="-2" upper="2" effort="8" velocity="5"/>
          </joint>
          <joint name="shoulder" type="revolute">
            <parent link="knee_link"/><child link="shoulder_link"/>
            <limit lower="-2" upper="2" effort="8" velocity="5"/>
          </joint>
        </robot>"#,
    )
    .unwrap();
    RobotDescription::load(&path).unwrap()
}

fn feedback_motor(
    group: &str,
    motor: &str,
    coordinate: &str,
    id: u32,
    position: f64,
) -> DisplayMotor {
    DisplayMotor {
        motor: motor.into(),
        coordinate: coordinate.into(),
        group: group.into(),
        bus: BUS.into(),
        id,
        model: "fake".into(),
        direction: 1,
        limits: CoordinateLimits {
            position_min: -2.0,
            position_max: 2.0,
            velocity_min: -5.0,
            velocity_max: 5.0,
            torque_min: -8.0,
            torque_max: 8.0,
            kp_min: 0.0,
            kp_max: 100.0,
            kd_min: 0.0,
            kd_max: 10.0,
        },
        status: None,
        feedback_available: true,
        feedback_current_connection: true,
        monotonic_ns: Some(SOURCE_NS),
        position,
        velocity: 0.0,
        effort: 0.0,
        temperature: 35.0,
        fault_bits: 0,
        mode_status: 0,
        age: Duration::from_millis(1),
        stale_after: Duration::from_millis(200),
    }
}

fn updated_targets() -> BTreeMap<String, CoordinateCommand> {
    BTreeMap::from([
        (
            COORDINATE_A.into(),
            CoordinateCommand {
                position: 0.7,
                velocity: 0.2,
                torque_feedforward: 0.3,
                kp: 22.0,
                kd: 1.7,
            },
        ),
        (
            COORDINATE_B.into(),
            CoordinateCommand {
                position: -0.8,
                velocity: -0.2,
                torque_feedforward: -0.3,
                kp: 23.0,
                kd: 1.8,
            },
        ),
    ])
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn group_control_uses_fresh_immutable_seed_complete_frames_rate_and_cleanup() {
    let server = FakeServer::start().await;
    let robot = connect_hardware(&endpoint(&server.endpoint))
        .await
        .expect("connect ergonomic robot client");
    let group = robot.group(GROUP).expect("resolve fake motor group");
    let profile = profile();
    let latest = Arc::new(RwLock::new(DisplaySnapshot::default()));
    {
        let display = &mut *latest.write();
        display.motors.insert(
            COORDINATE_A.into(),
            feedback_motor(GROUP, MOTOR_A, COORDINATE_A, 1, 0.2),
        );
        display.motors.insert(
            COORDINATE_B.into(),
            feedback_motor(GROUP, MOTOR_B, COORDINATE_B, 2, -0.4),
        );
    }

    let initial = seed_targets(GROUP, &profile, &latest).expect("seed from fresh feedback");
    assert_eq!(initial[COORDINATE_A].position, 0.2);
    assert_eq!(initial[COORDINATE_B].position, -0.4);
    assert_eq!(initial[COORDINATE_A].velocity, 0.125);
    assert_eq!(initial[COORDINATE_A].torque_feedforward, 0.25);
    assert_eq!(initial[COORDINATE_A].kp, 20.0);
    assert_eq!(initial[COORDINATE_A].kd, 1.5);

    // Observed positions do not define command limits. Clamp finite feedback
    // to the legal target range even when the mechanism is beyond that range.
    latest
        .write()
        .motors
        .get_mut(COORDINATE_A)
        .unwrap()
        .position = 2.0005;
    let boundary = seed_targets(GROUP, &profile, &latest).expect("clamp endpoint feedback");
    assert_eq!(boundary[COORDINATE_A].position, 2.0);
    latest
        .write()
        .motors
        .get_mut(COORDINATE_A)
        .unwrap()
        .position = 2.002;
    let outside = seed_targets(GROUP, &profile, &latest).expect("clamp out-of-range feedback");
    assert_eq!(outside[COORDINATE_A].position, 2.0);

    latest
        .write()
        .motors
        .get_mut(COORDINATE_A)
        .unwrap()
        .position = f64::INFINITY;
    assert!(seed_targets(GROUP, &profile, &latest).is_err());

    // A later display update must not mutate the already captured start frame.
    latest
        .write()
        .motors
        .get_mut(COORDINATE_A)
        .unwrap()
        .position = 1.9;
    latest
        .write()
        .motors
        .get_mut(COORDINATE_B)
        .unwrap()
        .position = 1.8;
    assert_eq!(initial[COORDINATE_A].position, 0.2);
    assert_eq!(initial[COORDINATE_B].position, -0.4);
    latest
        .write()
        .motors
        .get_mut(COORDINATE_A)
        .unwrap()
        .position = 0.2;
    latest
        .write()
        .motors
        .get_mut(COORDINATE_B)
        .unwrap()
        .position = -0.4;

    let (targets, receiver) = watch::channel(Some(initial.clone()));
    let (shutdown, shutdown_receiver) = watch::channel(false);
    let (finished, mut finished_receiver) = tokio::sync::mpsc::unbounded_channel();
    let worker_latest = Arc::clone(&latest);
    let worker = tokio::spawn(run_group_control(
        group,
        profile.clone(),
        initial,
        receiver,
        worker_latest,
        shutdown_receiver,
        finished,
    ));

    wait_until(|| {
        !server
            .recorded
            .lock()
            .expect("fake hardware state mutex poisoned")
            .commands
            .is_empty()
    })
    .await;
    targets
        .send(Some(updated_targets()))
        .expect("publish updated command targets");
    wait_until(|| {
        server
            .recorded
            .lock()
            .expect("fake hardware state mutex poisoned")
            .commands
            .len()
            >= 5
    })
    .await;
    targets.send(None).expect("request command stop");
    assert_eq!(
        tokio::time::timeout(Duration::from_secs(2), finished_receiver.recv())
            .await
            .expect("control worker did not finish")
            .expect("control worker completion channel closed"),
        GROUP
    );
    worker.await.expect("join group control worker");
    let _ = shutdown.send(true);

    {
        let recorded = server
            .recorded
            .lock()
            .expect("fake hardware state mutex poisoned");
        assert_eq!(recorded.acquired.len(), 1);
        assert_eq!(recorded.acquired[0].group, GROUP);
        assert_eq!(recorded.acquired[0].client_name, "bello-visualizer");
        assert!(recorded.renewed.is_empty());
        assert_eq!(recorded.released.len(), 1);
        assert_eq!(recorded.released[0].token, TOKEN);
        assert_eq!(
            recorded.lifecycle,
            [
                LifecycleCall {
                    action: "enable",
                    group: GROUP.into(),
                    token: TOKEN.into(),
                },
                LifecycleCall {
                    action: "start",
                    group: GROUP.into(),
                    token: TOKEN.into(),
                },
                LifecycleCall {
                    action: "stop",
                    group: GROUP.into(),
                    token: TOKEN.into(),
                },
                LifecycleCall {
                    action: "disable",
                    group: GROUP.into(),
                    token: TOKEN.into(),
                },
            ]
        );

        assert!(recorded.commands.len() >= 5);
        let first = &recorded.commands[0].request;
        assert_eq!(first.valid_for_ms, 90);
        assert_eq!(
            command_positions(first),
            [(MOTOR_A.into(), 0.2), (MOTOR_B.into(), -0.4)]
        );
        assert!(recorded.commands.iter().all(|timed| {
            timed.request.group == GROUP
                && timed.request.lease_token == TOKEN
                && timed.request.valid_for_ms == 90
                && timed.request.commands.len() == 2
                && timed
                    .request
                    .commands
                    .iter()
                    .map(|command| command.motor_name.as_str())
                    .collect::<BTreeSet<_>>()
                    == BTreeSet::from([MOTOR_A, MOTOR_B])
        }));
        let last_positions = command_positions(&recorded.commands.last().unwrap().request);
        assert!(last_positions[0].1 > 0.2 && last_positions[0].1 <= 0.7);
        assert!(last_positions[1].1 < -0.4 && last_positions[1].1 >= -0.8);
        assert!(recorded.commands.windows(2).all(|pair| {
            let before = command_positions(&pair[0].request);
            let after = command_positions(&pair[1].request);
            before
                .iter()
                .zip(after.iter())
                .all(|(before, after)| (after.1 - before.1).abs() <= 0.020_000_001)
        }));
        assert!(
            recorded
                .commands
                .windows(2)
                .skip(1)
                .all(|pair| pair[1].received.duration_since(pair[0].received)
                    >= Duration::from_millis(20))
        );
        assert!(
            recorded
                .commands
                .windows(2)
                .skip(1)
                .all(|pair| pair[1].received.duration_since(pair[0].received)
                    < Duration::from_millis(200))
        );
    }
    assert_eq!(
        latest.read().control.get(GROUP),
        Some(&ControlStatus::Monitoring)
    );

    drop(robot);
    server.stop().await;
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn rejected_command_stops_only_its_group_while_other_group_keeps_running() {
    let server = FakeServer::start_with(isolation_inventory(), Some(ISOLATED_GROUP)).await;
    let latest = Arc::new(RwLock::new(DisplaySnapshot::default()));
    {
        let display = &mut *latest.write();
        display.motors.insert(
            COORDINATE_A.into(),
            feedback_motor(GROUP, MOTOR_A, COORDINATE_A, 1, 0.2),
        );
        display.motors.insert(
            COORDINATE_B.into(),
            feedback_motor(GROUP, MOTOR_B, COORDINATE_B, 2, -0.4),
        );
        display.motors.insert(
            COORDINATE_C.into(),
            feedback_motor(ISOLATED_GROUP, MOTOR_C, COORDINATE_C, 3, 0.6),
        );
    }

    let profiles = BTreeMap::from([
        (GROUP.into(), profile_config()),
        (ISOLATED_GROUP.into(), profile_config()),
    ]);
    let (commands, command_receiver) = crossbeam_channel::unbounded();
    let (shutdown, shutdown_receiver) = watch::channel(false);
    let supervisor = tokio::spawn(control_supervisor(
        endpoint(&server.endpoint),
        profiles,
        control_description(),
        command_receiver,
        Arc::clone(&latest),
        shutdown_receiver,
    ));

    commands
        .send(ControlCommand::Start(vec![
            GROUP.into(),
            ISOLATED_GROUP.into(),
        ]))
        .expect("request concurrent group control");
    wait_until(|| {
        let display = latest.read();
        let controls_isolated =
            matches!(
                display.control.get(ISOLATED_GROUP),
                Some(ControlStatus::Failed(detail))
                    if detail.contains("synthetic isolated rejection")
            ) && matches!(display.control.get(GROUP), Some(ControlStatus::Running));
        let recorded = server
            .recorded
            .lock()
            .expect("fake hardware state mutex poisoned");
        controls_isolated
            && recorded
                .commands
                .iter()
                .filter(|command| command.request.group == ISOLATED_GROUP)
                .count()
                >= 2
            && recorded
                .released
                .iter()
                .any(|release| release.token == lease_token(ISOLATED_GROUP))
    })
    .await;

    let unaffected_count = server
        .recorded
        .lock()
        .expect("fake hardware state mutex poisoned")
        .commands
        .iter()
        .filter(|command| command.request.group == GROUP)
        .count();
    wait_until(|| {
        server
            .recorded
            .lock()
            .expect("fake hardware state mutex poisoned")
            .commands
            .iter()
            .filter(|command| command.request.group == GROUP)
            .count()
            >= unaffected_count + 3
    })
    .await;

    {
        let display = latest.read();
        assert!(matches!(
            display.control.get(ISOLATED_GROUP),
            Some(ControlStatus::Failed(detail))
                if detail.contains("synthetic isolated rejection")
        ));
        assert_eq!(display.control.get(GROUP), Some(&ControlStatus::Running));
    }
    {
        let recorded = server
            .recorded
            .lock()
            .expect("fake hardware state mutex poisoned");
        assert_eq!(
            recorded
                .commands
                .iter()
                .filter(|command| command.request.group == ISOLATED_GROUP)
                .count(),
            2
        );
        assert!(
            recorded
                .lifecycle
                .iter()
                .any(|call| { call.group == ISOLATED_GROUP && call.action == "disable" })
        );
        assert!(
            !recorded
                .lifecycle
                .iter()
                .any(|call| call.group == GROUP && call.action == "disable")
        );
        assert!(
            !recorded
                .released
                .iter()
                .any(|release| release.token == lease_token(GROUP))
        );
    }

    commands
        .send(ControlCommand::StopAll)
        .expect("stop unaffected group");
    wait_until(|| {
        latest.read().control.get(GROUP) == Some(&ControlStatus::Monitoring)
            && server
                .recorded
                .lock()
                .expect("fake hardware state mutex poisoned")
                .released
                .iter()
                .any(|release| release.token == lease_token(GROUP))
    })
    .await;

    let _ = shutdown.send(true);
    supervisor.await.expect("join control supervisor");
    server.stop().await;
}

fn command_positions(request: &proto::StreamCommandsRequest) -> Vec<(String, f64)> {
    let mut values = request
        .commands
        .iter()
        .map(|command| (command.motor_name.clone(), command.position))
        .collect::<Vec<_>>();
    values.sort_by(|left, right| left.0.cmp(&right.0));
    values
}
