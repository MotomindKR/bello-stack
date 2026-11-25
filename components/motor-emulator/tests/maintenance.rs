use std::collections::HashSet;
use std::time::Duration;

use robstride_emulator::config::{ActuatorConfig, CanInterface, Config, DynamicsConfig};
use robstride_emulator::emulator::{Emulator, MotorState};
use robstride_protocol::{
    ControlCommand, Frame, MotorParams, control_frame, enable_frame, param_write_u8_frame,
    parse_feedback, stop_frame,
};

const HOST: u8 = 0xfd;
const MOTOR: u8 = 7;
const ZERO: [u8; 8] = [1, 0, 0, 0, 0, 0, 0, 0];
const VERSION_QUERY: [u8; 8] = [0, 0xc4, 0, 0, 0, 0, 0, 0];
const IMAGE: &[u8] = b"BELLO-SIM-FW/1\n0.6.0.2";

fn wire(kind: u8, field: u16, target: u8, data: [u8; 8]) -> Frame {
    Frame {
        id: (u32::from(kind) << 24) | (u32::from(field) << 8) | u32::from(target),
        data,
    }
}

fn config(interface: &str, motors: &[(u8, &str, i8)]) -> Config {
    Config {
        interface: CanInterface::Socketcan(interface.into()),
        host_id: HOST,
        feedback_rate_hz: 100.,
        dynamics: DynamicsConfig::default(),
        actuators: motors
            .iter()
            .map(|&(id, model, direction)| ActuatorConfig {
                id,
                model: model.into(),
                direction,
                initial_position: 1.25,
                initial_temperature_c: 25.,
            })
            .collect(),
        events: vec![],
    }
}

fn emulator(direction: i8) -> Emulator {
    Emulator::new(config("vcan-test", &[(MOTOR, "RS_06", direction)])).unwrap()
}

fn reply(emulator: &mut Emulator, request: Frame) -> Frame {
    emulator.handle(request);
    let reports = emulator.advance_feedback(Duration::ZERO);
    assert_eq!(reports.len(), 1, "request {request:?}");
    assert_eq!(reports[0].delay, Duration::ZERO);
    reports[0].frame
}

fn identity(emulator: &mut Emulator, motor: u8) -> [u8; 8] {
    let answer = reply(emulator, wire(0, HOST.into(), motor, [0; 8]));
    assert_eq!(answer.id, wire(0, motor.into(), HOST, [0; 8]).id);
    answer.data
}

fn version(emulator: &mut Emulator, motor: u8) -> [u8; 4] {
    let answer = reply(emulator, wire(4, HOST.into(), motor, VERSION_QUERY));
    assert_eq!(answer.id, wire(2, motor.into(), HOST, [0; 8]).id);
    assert_eq!(&answer.data[..3], &[0, 0xc4, 0x56]);
    assert_eq!(answer.data[7], 0);
    answer.data[3..7].try_into().unwrap()
}

fn start(emulator: &mut Emulator, uuid: [u8; 8]) -> u8 {
    ota(emulator, 11, HOST.into(), uuid).0
}

fn ota(emulator: &mut Emulator, kind: u8, field: u16, data: [u8; 8]) -> (u8, [u8; 8]) {
    let answer = reply(emulator, wire(kind, field, MOTOR, data));
    assert_eq!(
        answer.id & 0xff00_ffff,
        wire(kind, MOTOR.into(), HOST, [0; 8]).id
    );
    ((answer.id >> 16) as u8, answer.data)
}

fn info(image: &[u8]) -> [u8; 8] {
    let mut data = [0; 8];
    data[..4].copy_from_slice(&(image.len() as u32).to_le_bytes());
    data[4..].copy_from_slice(&(image.len().div_ceil(8) as u32).to_le_bytes());
    data
}

fn end(count: u32) -> [u8; 8] {
    let mut data = [0; 8];
    data[..4].copy_from_slice(&count.to_le_bytes());
    data
}

fn packet(bytes: &[u8]) -> [u8; 8] {
    let mut data = [0xff; 8];
    data[..bytes.len()].copy_from_slice(bytes);
    data
}

#[test]
fn identity_and_version_distinguish_models_and_buses() {
    let models = [
        (7, "RS_00", 1),
        (8, "RS_01", 1),
        (9, "RS_02", 1),
        (10, "RS_03", 1),
        (11, "RS_04", 1),
        (12, "RS_05", 1),
        (13, "RS_06", 1),
        (14, "RS_10P", 1),
    ];
    let mut bus = Emulator::new(config("first-bus", &models)).unwrap();
    let mut restarted = Emulator::new(config("first-bus", &models)).unwrap();
    let mut other_bus = Emulator::new(config("second-bus", &models)).unwrap();
    let mut identities = HashSet::new();
    for ((motor, _, _), family) in models.into_iter().zip([0, 1, 2, 3, 4, 5, 6, 10]) {
        let uuid = identity(&mut bus, motor);
        assert!(identities.insert(uuid));
        assert_eq!(identity(&mut restarted, motor), uuid);
        assert!(identities.insert(identity(&mut other_bus, motor)));
        assert_eq!(version(&mut bus, motor), [0, family, 0, 1]);
        let image = format!("BELLO-SIM-FW/1\n0.{family}.0.2");
        let image = image.as_bytes();
        assert_eq!(
            reply(&mut bus, wire(11, HOST.into(), motor, uuid)).id >> 16 & 0xff,
            0
        );
        assert_eq!(
            reply(&mut bus, wire(12, HOST.into(), motor, info(image))).id >> 16 & 0xff,
            0
        );
        for (index, bytes) in image.chunks(8).enumerate() {
            assert_eq!(
                reply(&mut bus, wire(13, index as u16, motor, packet(bytes))).id >> 16 & 0xff,
                0
            );
        }
        let count = image.len().div_ceil(8) as u32;
        assert_eq!(
            reply(&mut bus, wire(14, 0, motor, end(count))).id >> 16 & 0xff,
            0
        );
        assert_eq!(version(&mut bus, motor), [0, family, 0, 2]);
        assert_eq!(identity(&mut bus, motor), uuid);
    }
    bus.handle(wire(0, 0xfc, MOTOR, [0; 8]));
    bus.handle(wire(0, HOST.into(), 99, [0; 8]));
    assert!(bus.advance_feedback(Duration::ZERO).is_empty());
}

#[test]
fn re_zero_preserves_physical_positions_and_maps_new_commands() {
    for direction in [-1, 1] {
        let mut emulator = emulator(direction);
        let params = MotorParams::for_model("RS_06", direction).unwrap();
        let stop = stop_frame(HOST, MOTOR, false);
        let zero = wire(6, HOST.into(), MOTOR, ZERO);
        let control = |position| {
            control_frame(
                params,
                ControlCommand {
                    motor_id: MOTOR,
                    position,
                    kp: 20.,
                    ..Default::default()
                },
            )
        };

        emulator.handle(param_write_u8_frame(HOST, MOTOR, 0x7005, 0));
        emulator.handle(enable_frame(HOST, MOTOR));
        reply(&mut emulator, control(1.5));
        assert_eq!(version(&mut emulator, MOTOR), [0, 6, 0, 1]);
        emulator.handle(wire(4, 0x0100 | u16::from(HOST), MOTOR, VERSION_QUERY));
        assert!(emulator.advance_feedback(Duration::ZERO).is_empty());
        assert!((emulator.command(MOTOR).unwrap().unwrap().position - 1.5).abs() < 0.001);
        emulator.handle(zero);
        assert!(emulator.advance_feedback(Duration::ZERO).is_empty());

        let stopped = parse_feedback(&reply(&mut emulator, stop), params).unwrap();
        assert_eq!(stopped.mode_status, 0);
        assert!((stopped.position - 1.25).abs() < 0.001);
        let zeroed = parse_feedback(&reply(&mut emulator, zero), params).unwrap();
        assert_eq!(zeroed.mode_status, 0);
        assert!(zeroed.position.abs() < 0.001);
        let stationary = control(0.);
        emulator.handle(enable_frame(HOST, MOTOR));
        emulator.handle(stationary);
        let feedback = emulator.advance(Duration::from_millis(100));
        assert_eq!(feedback.len(), 1);
        let feedback = parse_feedback(&feedback[0].frame, params).unwrap();
        assert!(feedback.position.abs() < 0.001);
        assert!(feedback.velocity.abs() < 0.01);

        // Bello Sim's physics supplies absolute positions and consumes absolute targets.
        emulator
            .set_motor_state(
                MOTOR,
                MotorState {
                    position: 1.65,
                    velocity: 0.1,
                    torque: 0.2,
                },
            )
            .unwrap();
        let feedback = parse_feedback(&reply(&mut emulator, control(0.4)), params).unwrap();
        assert!((feedback.position - 0.4).abs() < 0.001);
        assert!((feedback.velocity - 0.1).abs() < 0.01);
        assert!((feedback.torque - 0.2).abs() < 0.01);
        assert!((emulator.command(MOTOR).unwrap().unwrap().position - 1.65).abs() < 0.001);

        reply(&mut emulator, stop);
        reply(&mut emulator, zero);
        emulator.handle(enable_frame(HOST, MOTOR));
        reply(&mut emulator, control(0.));
        assert!((emulator.command(MOTOR).unwrap().unwrap().position - 1.65).abs() < 0.001);
    }
}

#[test]
fn ota_requires_stop_and_uuid_and_handles_retries_and_order() {
    let mut emulator = emulator(1);
    let uuid = identity(&mut emulator, MOTOR);
    let initial = version(&mut emulator, MOTOR);
    let info = info(IMAGE);
    let end = end(3);
    let packets: Vec<_> = IMAGE.chunks(8).map(packet).collect();

    emulator.handle(enable_frame(HOST, MOTOR));
    assert_ne!(start(&mut emulator, uuid), 0);
    reply(&mut emulator, stop_frame(HOST, MOTOR, false));
    assert_ne!(ota(&mut emulator, 12, HOST.into(), info).0, 0);
    assert_ne!(start(&mut emulator, [0; 8]), 0);
    assert_eq!(start(&mut emulator, uuid), 0);
    assert_eq!(start(&mut emulator, uuid), 0);
    assert_ne!(ota(&mut emulator, 13, 0, packets[0]).0, 0);
    let mut wrong_info = info;
    wrong_info[4] += 1;
    assert_ne!(ota(&mut emulator, 12, HOST.into(), wrong_info).0, 0);
    assert_eq!(ota(&mut emulator, 12, HOST.into(), info).0, 0);
    emulator.handle(enable_frame(HOST, MOTOR));
    assert_eq!(ota(&mut emulator, 13, 0, packets[0]).0, 0);
    assert_eq!(ota(&mut emulator, 12, HOST.into(), info).0, 0);
    assert_eq!(ota(&mut emulator, 13, 0, packets[0]).0, 0);
    assert_ne!(ota(&mut emulator, 13, 0, [0; 8]).0, 0);
    assert_eq!(
        ota(&mut emulator, 13, 2, packets[2]),
        (0x0f, [1, 0, 0, 0, 0, 0, 0, 0])
    );
    assert_ne!(ota(&mut emulator, 14, 0, end).0, 0);
    assert_eq!(version(&mut emulator, MOTOR), initial);
    assert_eq!(ota(&mut emulator, 13, 1, packets[1]).0, 0);
    assert_eq!(ota(&mut emulator, 13, 2, packets[2]).0, 0);
    assert_ne!(ota(&mut emulator, 14, 1, end).0, 0);
    assert_eq!(version(&mut emulator, MOTOR), initial);
    assert_eq!(ota(&mut emulator, 14, 0, end).0, 0);
    assert_eq!(ota(&mut emulator, 14, 0, end).0, 0);
    assert_eq!(version(&mut emulator, MOTOR), [0, 6, 0, 2]);
    assert_eq!(identity(&mut emulator, MOTOR), uuid);
}

#[test]
fn invalid_images_do_not_install_and_a_new_start_can_recover() {
    let mut emulator = emulator(1);
    let uuid = identity(&mut emulator, MOTOR);
    for (image, padding) in [
        (&b"BELLO-SIM-FW/1\n0.5.0.2"[..], 0xff),
        (b"BELLO-SIM-FW/1\n0.6.0.2\n", 0xff),
        (b"BELLO-SIM-FW/1\n0.6.0.+2", 0xff),
        (IMAGE, 0),
        (IMAGE, 0xff),
    ] {
        assert_eq!(start(&mut emulator, uuid), 0);
        assert_eq!(ota(&mut emulator, 12, HOST.into(), info(image)).0, 0);
        for (index, bytes) in image.chunks(8).enumerate() {
            let mut data = [padding; 8];
            data[..bytes.len()].copy_from_slice(bytes);
            assert_eq!(ota(&mut emulator, 13, index as u16, data).0, 0);
        }
        let count = image.len().div_ceil(8) as u32;
        let status = ota(&mut emulator, 14, 0, end(count)).0;
        if image == IMAGE && padding == 0xff {
            assert_eq!(status, 0);
            assert_eq!(version(&mut emulator, MOTOR), [0, 6, 0, 2]);
        } else {
            assert_ne!(status, 0);
            assert_eq!(version(&mut emulator, MOTOR), [0, 6, 0, 1]);
        }
    }
}

#[cfg(unix)]
#[test]
fn standalone_emulator_answers_maintenance_over_loopback_can() {
    use std::io::{ErrorKind, Write};
    use std::net::{Ipv4Addr, UdpSocket};
    use std::process::{Child, Command, Stdio};
    use std::time::Instant;

    use bello_sim_can::SimBus;

    struct EmulatorProcess(Child);

    impl Drop for EmulatorProcess {
        fn drop(&mut self) {
            let _ = self.0.kill();
            let _ = self.0.wait();
        }
    }

    let reserve_port = || UdpSocket::bind((Ipv4Addr::LOCALHOST, 0)).unwrap();
    let daemon_port = reserve_port();
    let simulator_port = reserve_port();
    let bus = SimBus {
        daemon_port: daemon_port.local_addr().unwrap().port(),
        simulator_port: simulator_port.local_addr().unwrap().port(),
    };
    drop(daemon_port);
    let daemon = bus.open_daemon().unwrap();
    drop(simulator_port);
    daemon
        .set_read_timeout(Some(Duration::from_millis(100)))
        .unwrap();
    let query = wire(4, HOST.into(), MOTOR, VERSION_QUERY);
    daemon.send(query).unwrap();
    assert!(matches!(
        daemon.recv().unwrap_err().kind(),
        ErrorKind::WouldBlock | ErrorKind::TimedOut
    ));

    let mut process = EmulatorProcess(
        Command::new(env!("CARGO_BIN_EXE_robstride-emulator"))
            .args(["--config", "/dev/stdin"])
            .stdin(Stdio::piped())
            .stdout(Stdio::null())
            .spawn()
            .unwrap(),
    );
    writeln!(
        process.0.stdin.take().unwrap(),
        "interface: {{ daemon_port: {}, simulator_port: {} }}\nactuators:\n  - {{ id: {MOTOR}, model: RS_06, initial_position: 1.25 }}",
        bus.daemon_port, bus.simulator_port
    )
    .unwrap();

    let ping = wire(0, HOST.into(), MOTOR, [0; 8]);
    let ping_id = wire(0, MOTOR.into(), HOST, [0; 8]).id;
    let deadline = Instant::now() + Duration::from_secs(3);
    let uuid = loop {
        daemon.send(ping).unwrap();
        match daemon.recv() {
            Ok(frame) => {
                assert_eq!(frame.id, ping_id);
                break frame.data;
            }
            Err(error) => {
                assert!(matches!(
                    error.kind(),
                    ErrorKind::WouldBlock | ErrorKind::TimedOut
                ));
                assert!(process.0.try_wait().unwrap().is_none());
                assert!(Instant::now() < deadline, "emulator identity timed out");
            }
        }
    };
    let exchange = |request, expected_id| {
        daemon.send(request).unwrap();
        loop {
            let frame = daemon.recv().unwrap();
            if frame.id == expected_id {
                break frame.data;
            }
            assert_eq!(
                frame,
                Frame {
                    id: ping_id,
                    data: uuid
                }
            );
        }
    };
    let feedback_id = wire(2, MOTOR.into(), HOST, [0; 8]).id;
    assert_eq!(exchange(query, feedback_id), [0, 0xc4, 0x56, 0, 6, 0, 1, 0]);
    let stop = stop_frame(HOST, MOTOR, false);
    let zero = wire(6, HOST.into(), MOTOR, ZERO);
    let params = MotorParams::for_model("RS_06", 1).unwrap();
    exchange(stop, feedback_id);
    exchange(zero, feedback_id);
    let stopped = parse_feedback(
        &Frame {
            id: feedback_id,
            data: exchange(stop, feedback_id),
        },
        params,
    )
    .unwrap();
    assert_eq!(stopped.mode_status, 0);
    assert!(stopped.position.abs() < 0.001);

    let transfer = |kind, field, data| {
        exchange(
            wire(kind, field, MOTOR, data),
            wire(kind, MOTOR.into(), HOST, [0; 8]).id,
        )
    };
    transfer(11, HOST.into(), uuid);
    transfer(12, HOST.into(), info(IMAGE));
    for (index, bytes) in IMAGE.chunks(8).enumerate() {
        transfer(13, index as u16, packet(bytes));
    }
    transfer(14, 0, end(3));
    assert_eq!(exchange(query, feedback_id), [0, 0xc4, 0x56, 0, 6, 0, 2, 0]);
    assert_eq!(exchange(ping, ping_id), uuid);
}
