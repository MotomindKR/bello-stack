use std::io::Write;
use std::net::{Ipv4Addr, UdpSocket};

use bello_sim_can::{SimBus, SimSocket};
use robstride_protocol::{Feedback, Frame, Request, feedback_frame, parse_request};

use super::*;
use crate::config::{BusTransportConfig, Config};
use crate::domain::MotorName;
use crate::hardware::robstride::MotorManager;

#[test]
fn maintenance_isolates_one_bus_and_restores_fresh_monitoring() {
    let (first_ports, first_guard, first_peer) = sim_bus();
    let (second_ports, second_guard, second_peer) = sim_bus();
    let mut config: Config = serde_yaml::from_str(
        r#"
simulation: true
buses:
  - { name: main, transport: { type: sim, daemon_port: 1, simulator_port: 2 } }
  - { name: other, transport: { type: sim, daemon_port: 3, simulator_port: 4 } }
motors:
  - { name: first, coordinate: direct_joint, bus: main, id: 1, model: RS_03,
      limits: { position_min: -1, position_max: 1 },
      health: { safe_position_min: -1, safe_position_max: 1 } }
  - { name: second, coordinate: second_joint, bus: other, id: 1, model: RS_03,
      limits: { position_min: -1, position_max: 1 },
      health: { safe_position_min: -1, safe_position_max: 1 } }
motor_groups:
  - { name: test, motors: [first, second], feedback_timeout_ms: 250 }
runtime:
  motor_velocity_filter: { cutoff_hz: 1 }
"#,
    )
    .unwrap();
    config.buses[0].transport = BusTransportConfig::Sim(first_ports);
    config.buses[1].transport = BusTransportConfig::Sim(second_ports);
    let mut urdf = tempfile::NamedTempFile::new().unwrap();
    write!(
        urdf,
        r#"<robot name="maintenance">
          <link name="base"/><link name="first"/><link name="second"/>
          <joint name="direct_joint" type="revolute">
            <parent link="base"/><child link="first"/>
            <limit lower="-1" upper="1" effort="5" velocity="10"/>
          </joint>
          <joint name="second_joint" type="revolute">
            <parent link="base"/><child link="second"/>
            <limit lower="-1" upper="1" effort="5" velocity="10"/>
          </joint>
        </robot>"#,
    )
    .unwrap();
    config.robot_description = urdf.path().into();
    let topology = ResolvedTopology::resolve(config).unwrap();
    let telemetry = Telemetry::new(64);
    drop(first_guard);
    drop(second_guard);
    let buses = BusManager::start(&topology, telemetry.clone()).unwrap();
    let motors = MotorManager::start(&topology, &buses, telemetry).unwrap();
    let first_name = MotorName::from("first");
    let second_name = MotorName::from("second");
    let first_motor = motors.controller(&first_name).unwrap();
    let second_motor = motors.controller(&second_name).unwrap();
    let first_bus = buses.handle(&BusName::from("main")).unwrap();
    wait_for_monitoring_request(&first_peer);
    wait_for_monitoring_request(&second_peer);

    let feedback = |velocity| Feedback {
        motor_id: 1,
        position: 0.0,
        velocity,
        torque: 0.0,
        temperature: 25.0,
        mode_status: 0,
        fault_bits: 0,
    };
    let first_frame = |velocity| feedback_frame(0xfd, feedback(velocity), first_motor.spec.params);
    let second_frame =
        |velocity| feedback_frame(0xfd, feedback(velocity), second_motor.spec.params);
    first_peer.send(first_frame(5.0)).unwrap();
    wait_until(|| motors.feedback(&first_name).is_some());

    let reserved = first_bus.reserve_maintenance().unwrap();
    assert!(first_bus.clone().reserve_maintenance().is_err());
    assert!(motors.feedback(&first_name).is_none());
    assert!(matches!(
        first_motor.configure_enable(),
        Err(BusError::Maintenance { .. })
    ));
    assert!(matches!(
        first_bus.submit_safety_batch(vec![CanFrame {
            id: 1,
            data: [1; 8]
        }]),
        Err(BusError::Maintenance { .. })
    ));
    let periodic = first_motor.periodic_control(
        Duration::from_millis(10),
        Arc::new(AtomicU64::new(1)),
        Arc::new(AtomicBool::new(false)),
    );
    periodic.publish(
        vec![CanFrame {
            id: 3,
            data: [3; 8],
        }],
        Instant::now() + Duration::from_secs(1),
        1,
    );

    let request = Frame {
        id: 0x0b00fd01,
        data: [7; 8],
    };
    reserved.send(request).unwrap();
    assert_eq!(first_peer.recv().unwrap(), request);
    let during_maintenance = first_frame(4.0);
    first_peer.send(during_maintenance).unwrap();
    second_peer.send(second_frame(6.0)).unwrap();
    assert_eq!(
        reserved.recv(Duration::from_secs(1)).unwrap(),
        Some(during_maintenance)
    );
    wait_until(|| motors.feedback(&second_name).is_some());
    assert!(motors.feedback(&first_name).is_none());
    assert_eq!(reserved.recv(Duration::ZERO).unwrap(), None);
    first_peer
        .set_read_timeout(Some(Duration::from_millis(30)))
        .unwrap();
    assert!(matches!(
        first_peer.recv().unwrap_err().kind(),
        std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut
    ));

    // Leave maintenance-era feedback pending at the release boundary.
    first_peer.send(first_frame(8.0)).unwrap();
    drop(reserved);
    first_peer
        .set_read_timeout(Some(Duration::from_secs(1)))
        .unwrap();
    assert!(matches!(
        parse_request(&first_peer.recv().unwrap()),
        Some(Request::ActiveReport { enabled: true, .. })
    ));
    assert!(motors.feedback(&first_name).is_none());
    first_peer.send(first_frame(-5.0)).unwrap();
    wait_until(|| motors.feedback(&first_name).is_some());
    let latest = motors.feedback(&first_name).unwrap();
    assert!(latest.value.velocity < -4.9);
    assert_eq!(latest.published_velocity, latest.value.velocity);
}

fn sim_bus() -> (SimBus, UdpSocket, SimSocket) {
    let daemon = UdpSocket::bind((Ipv4Addr::LOCALHOST, 0)).unwrap();
    let peer = UdpSocket::bind((Ipv4Addr::LOCALHOST, 0)).unwrap();
    let spec = SimBus {
        daemon_port: daemon.local_addr().unwrap().port(),
        simulator_port: peer.local_addr().unwrap().port(),
    };
    drop(peer);
    let peer = spec.open_simulator().unwrap();
    peer.set_read_timeout(Some(Duration::from_secs(1))).unwrap();
    (spec, daemon, peer)
}

fn wait_for_monitoring_request(peer: &SimSocket) {
    assert!(matches!(
        parse_request(&peer.recv().unwrap()),
        Some(Request::ActiveReport { enabled: true, .. })
    ));
}

fn wait_until(mut predicate: impl FnMut() -> bool) {
    let until = Instant::now() + Duration::from_secs(1);
    while !predicate() {
        assert!(Instant::now() < until);
        thread::sleep(Duration::from_millis(1));
    }
}
