use std::net::{Ipv4Addr, UdpSocket};
use std::path::Path;

use bello_sim_can::{SimBus, SimSocket};
use robstride_protocol::{Feedback, MotorParams, Request, feedback_frame, parse_request};

use super::*;
use crate::config::{BusTransportConfig, Config, ResolvedTopology};
use crate::domain::{MotorCommand, MotorName, MotorTelemetry, TelemetryEvent};
use crate::hardware::{can::BusManager, robstride::MotorManager};

#[test]
fn safe_position_transient_faults_enabled_group() {
    assert_transient_fault(
        Feedback {
            position: 2.0,
            mode_status: 2,
            ..healthy_feedback()
        },
        GroupCause::SafePosition,
        GroupState::Damped,
    );
}

#[test]
fn reported_fault_transient_faults_running_group() {
    assert_transient_fault(
        Feedback {
            fault_bits: 1,
            mode_status: 2,
            ..healthy_feedback()
        },
        GroupCause::ReportedMotorFault,
        GroupState::Running,
    );
}

#[test]
fn wrong_mode_transient_faults_enabled_group() {
    assert_transient_fault(
        Feedback {
            mode_status: 1,
            ..healthy_feedback()
        },
        GroupCause::ModeStatus,
        GroupState::Damped,
    );
}

fn assert_transient_fault(bad: Feedback, cause: GroupCause, state: GroupState) {
    let reservation = UdpSocket::bind((Ipv4Addr::LOCALHOST, 0)).unwrap();
    let peer_reservation = UdpSocket::bind((Ipv4Addr::LOCALHOST, 0)).unwrap();
    let sim_bus = SimBus {
        daemon_port: reservation.local_addr().unwrap().port(),
        simulator_port: peer_reservation.local_addr().unwrap().port(),
    };
    let mut config: Config = serde_yaml::from_str(
        r#"
simulation: true
buses:
  - { name: can, transport: { type: sim, daemon_port: 1, simulator_port: 2 } }
motors:
  - { name: motor, coordinate: direct_joint, bus: can, id: 1, model: ROBSTRIDE_03,
      limits: { position_min: -1, position_max: 1 },
      health: { safe_position_min: -1, safe_position_max: 1 } }
motor_groups:
  - { name: test, motors: [motor], enable_retries: 1, feedback_timeout_ms: 2000,
      command_timeout_ms: 1000 }
"#,
    )
    .unwrap();
    config.buses[0].transport = BusTransportConfig::Sim(sim_bus);
    config.robot_description =
        Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/transmissions.urdf");
    let topology = ResolvedTopology::resolve(config).unwrap();
    let telemetry = Telemetry::new(64);
    let mut events = telemetry.subscribe();
    drop(peer_reservation);
    let simulator = sim_bus.open_simulator().unwrap();
    simulator
        .set_read_timeout(Some(Duration::from_secs(2)))
        .unwrap();
    drop(reservation);
    let buses = BusManager::start(&topology, telemetry.clone()).unwrap();
    let motors = MotorManager::start(&topology, &buses, telemetry.clone()).unwrap();
    let motor = motors.controller(&MotorName::from("motor")).unwrap();
    let spec = topology.groups().next().unwrap().1.clone();
    let (_lifecycle_tx, lifecycle_rx) = mpsc::sync_channel(1);
    let (_wake_tx, wake_rx) = mpsc::sync_channel(1);
    let snapshot = Arc::new(RwLock::new(GroupSnapshot::default()));
    let mut group = GroupController::new(
        spec,
        vec![GroupMotor::new(motor.clone())],
        ControllerInputs {
            lifecycle_rx,
            wake_rx,
            command_mailbox: Arc::new(Mutex::new(None)),
            lease_lost: Arc::new(AtomicBool::new(false)),
            emergency_pending: Arc::new(AtomicBool::new(false)),
            lease_generation: Arc::new(AtomicU64::new(1)),
            lease_loss_waiters: Arc::new(Mutex::new(Vec::new())),
            shutdown: Arc::new(AtomicBool::new(false)),
            snapshot: snapshot.clone(),
        },
        telemetry,
    );
    let params = motor.spec.params;
    let healthy = healthy_feedback();

    loop {
        if matches!(
            parse_request(&simulator.recv().unwrap()),
            Some(Request::ActiveReport { enabled: true, .. })
        ) {
            break;
        }
    }
    enable_with_feedback(&mut group, &simulator, params, healthy);
    group.control_tick(0, false);
    assert_eq!(snapshot.read().state, GroupState::Damped);
    if state == GroupState::Running {
        let started = group.handle_action(
            LifecycleAction::Start {
                batch: CommandBatch {
                    group: motor.group.clone(),
                    lease_token: Default::default(),
                    sequence: 1,
                    valid_for: None,
                    commands: vec![MotorCommand {
                        motor: motor.spec.name.clone(),
                        impedance: ImpedanceCommand::default(),
                    }],
                },
                submitted_at: Instant::now(),
            },
            1,
        );
        assert!(started.ok, "{}", started.message);
    }
    assert_eq!(snapshot.read().state, state);

    send(&simulator, params, bad);
    let healthy_after_fault = Feedback {
        temperature: 26.0,
        ..healthy
    };
    send(&simulator, params, healthy_after_fault);
    wait_for_motor_sample(&mut events, |sample| {
        sample.temperature == healthy_after_fault.temperature
    });
    group.control_tick(0, false);
    assert_eq!(snapshot.read().state, GroupState::Fault, "{cause:?}");
    assert_eq!(snapshot.read().cause, cause);
    group.control_tick(0, false);
    assert_eq!(snapshot.read().state, GroupState::Fault);
    assert_eq!(snapshot.read().cause, cause);

    send(
        &simulator,
        params,
        Feedback {
            fault_bits: 1,
            ..healthy
        },
    );
    wait_for_motor_sample(&mut events, |sample| sample.fault_bits != 0);
    let failed = group.handle_action(LifecycleAction::Recover, 1);
    assert_eq!(failed.to, GroupState::Fault);
    assert_eq!(snapshot.read().cause, GroupCause::Recover);

    send(
        &simulator,
        params,
        Feedback {
            mode_status: 0,
            ..healthy
        },
    );
    wait_for_motor_sample(&mut events, |sample| sample.mode_status == 0);
    let recovered = group.handle_action(LifecycleAction::Recover, 1);
    assert!(recovered.ok, "{}", recovered.message);
    assert_eq!(recovered.to, GroupState::Idle);
    enable_with_feedback(&mut group, &simulator, params, healthy);
}

fn healthy_feedback() -> Feedback {
    Feedback {
        motor_id: 1,
        mode_status: 2,
        temperature: 25.0,
        ..Feedback::default()
    }
}

fn enable_with_feedback(
    group: &mut GroupController,
    simulator: &SimSocket,
    params: MotorParams,
    healthy: Feedback,
) {
    let enabled = AtomicBool::new(false);
    thread::scope(|scope| {
        scope.spawn(|| {
            loop {
                if matches!(
                    parse_request(&simulator.recv().unwrap()),
                    Some(Request::Enable { .. })
                ) {
                    break;
                }
            }
            send(
                simulator,
                params,
                Feedback {
                    mode_status: 0,
                    ..healthy
                },
            );
            let deadline = Instant::now() + Duration::from_secs(2);
            while !enabled.load(Ordering::Acquire) && Instant::now() < deadline {
                thread::sleep(Duration::from_millis(5));
                send(simulator, params, healthy);
            }
        });
        let transition = group.handle_action(LifecycleAction::Enable, 1);
        enabled.store(true, Ordering::Release);
        assert_eq!(transition.to, GroupState::Damped, "{}", transition.message);
    });
}

fn send(simulator: &SimSocket, params: MotorParams, value: Feedback) {
    simulator.send(feedback_frame(0xfd, value, params)).unwrap();
}

fn wait_for_motor_sample(
    events: &mut tokio::sync::broadcast::Receiver<TelemetryEvent>,
    matches_sample: impl Fn(&MotorTelemetry) -> bool,
) {
    let deadline = Instant::now() + Duration::from_secs(2);
    loop {
        match events.try_recv() {
            Ok(event) if matches!(&event.payload, TelemetryPayload::Motor(sample) if matches_sample(sample)) =>
            {
                return;
            }
            Ok(_) => {}
            Err(tokio::sync::broadcast::error::TryRecvError::Empty) => {
                assert!(Instant::now() < deadline, "motor feedback did not arrive");
                thread::sleep(Duration::from_millis(1));
            }
            Err(error) => panic!("{error}"),
        }
    }
}
