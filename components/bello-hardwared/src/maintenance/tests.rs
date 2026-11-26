use std::cell::RefCell;
use std::collections::VecDeque;

use robstride_protocol::{MotorParams, feedback_frame};

use super::*;
use crate::config::MotorConfig;

const HOST: u8 = 253;
const ID: u8 = 7;
const UUID: [u8; 8] = [15; 8];

fn motor() -> MotorSpec {
    let config: MotorConfig =
        serde_yaml::from_str("name: motor\ncoordinate: joint\nbus: test\nid: 7\nmodel: RS_03")
            .unwrap();
    MotorSpec::try_from(&config).unwrap()
}

struct MotorOnWire {
    now: Instant,
    received: VecDeque<Frame>,
    sent: Vec<Frame>,
    uuid: [u8; 8],
    version: [u8; 4],
    position: f64,
    moving: bool,
    noisy: bool,
    enabled_peer: Option<u8>,
    fault_bits: u16,
    accept_zero: bool,
    ota: Option<OtaBehavior>,
}

#[derive(Default)]
struct OtaBehavior {
    packets: Vec<[u8; 8]>,
    lose_data_ack: bool,
    lose_end_ack: bool,
    resume: Option<u16>,
    reject_install: bool,
}

impl Default for MotorOnWire {
    fn default() -> Self {
        Self {
            now: Instant::now(),
            received: VecDeque::new(),
            sent: Vec::new(),
            uuid: UUID,
            version: [0, 3, 0, 1],
            position: 0.4,
            moving: false,
            noisy: false,
            enabled_peer: None,
            fault_bits: 0,
            accept_zero: true,
            ota: None,
        }
    }
}

impl MotorOnWire {
    fn report(&mut self, kind: u8, status: u8, data: [u8; 8]) {
        self.received.push_back(Frame {
            id: (u32::from(kind) << 24)
                | (u32::from(status) << 16)
                | (u32::from(ID) << 8)
                | u32::from(HOST),
            data,
        });
    }
}

impl Wire for RefCell<MotorOnWire> {
    fn now(&self) -> Instant {
        self.borrow().now
    }

    fn sleep(&self, duration: Duration) {
        self.borrow_mut().now += duration;
    }

    fn send(&self, frame: Frame) -> Result<()> {
        let mut motor = self.borrow_mut();
        motor.sent.push(frame);
        let kind = (frame.id >> 24) as u8;
        let field = (frame.id >> 8) as u16;
        if kind == 24 {
            assert_eq!(field, u16::from(HOST));
            if frame.data[6] != 0 {
                let peer = frame.id as u8;
                let mode = if motor.enabled_peer == Some(peer) {
                    2
                } else {
                    0
                };
                motor.received.push_back(Frame {
                    id: (2 << 24) | (mode << 22) | (u32::from(peer) << 8) | u32::from(HOST),
                    data: [0; 8],
                });
            }
            return Ok(());
        }
        assert_eq!(frame.id as u8, ID);
        match kind {
            0 => {
                assert_eq!(field, u16::from(HOST));
                let uuid = motor.uuid;
                motor.report(0, 0, uuid);
            }
            4 => {
                assert_eq!(field, u16::from(HOST));
                if frame.data == [0, 0xc4, 0, 0, 0, 0, 0, 0] {
                    let [a, b, c, d] = motor.version;
                    motor.report(2, 0, [0, 0xc4, 0x56, a, b, c, d, 0]);
                } else {
                    assert_eq!(frame.data, [0; 8]);
                    if motor.moving {
                        motor.position += 0.01;
                    }
                    let status = feedback_frame(
                        HOST,
                        Feedback {
                            motor_id: ID,
                            position: motor.position,
                            fault_bits: motor.fault_bits,
                            ..Feedback::default()
                        },
                        MotorParams::for_model("RS_03", 1).unwrap(),
                    );
                    motor.received.push_back(status);
                }
            }
            6 => {
                assert_eq!(field, u16::from(HOST));
                assert_eq!(frame.data, [1, 0, 0, 0, 0, 0, 0, 0]);
                if motor.accept_zero {
                    motor.position = 0.0;
                }
            }
            11 => {
                assert_eq!(field, u16::from(HOST));
                assert_eq!(frame.data, motor.uuid);
                motor.ota.as_mut().unwrap().packets.clear();
                motor.report(kind, 0, [0; 8]);
            }
            12 => {
                assert_eq!(field, u16::from(HOST));
                assert_eq!(frame.data, [22, 0, 0, 0, 3, 0, 0, 0]);
                motor.report(kind, 0, [0; 8]);
            }
            13 => {
                let ota = motor.ota.as_mut().unwrap();
                if std::mem::take(&mut ota.lose_data_ack) {
                    motor.report(kind, 0, [0; 8]);
                    motor.received.back_mut().unwrap().id ^= 1 << 8; // Wrong source ID.
                    return Ok(());
                }
                if field == 1
                    && let Some(next) = ota.resume.take()
                {
                    ota.packets.truncate(usize::from(next));
                    let mut payload = [0; 8];
                    payload[..2].copy_from_slice(&next.to_le_bytes());
                    motor.report(kind, 0xf0, payload);
                } else {
                    assert_eq!(usize::from(field), ota.packets.len());
                    ota.packets.push(frame.data);
                    motor.report(kind, 0, [0; 8]);
                }
            }
            14 => {
                assert_eq!(field, 0);
                assert_eq!(frame.data, [3, 0, 0, 0, 0, 0, 0, 0]);
                let ota = motor.ota.as_ref().unwrap();
                let bytes: Vec<_> = ota.packets.iter().flatten().copied().collect();
                assert_eq!(bytes, b"BELLO-SIM-FW/1\n0.3.0.2\xff\xff");
                let lose_end_ack = ota.lose_end_ack;
                if !ota.reject_install {
                    motor.version = [0, 3, 0, 2];
                }
                if !lose_end_ack {
                    motor.report(kind, 0, [0; 8]);
                }
            }
            _ => panic!("unexpected frame {frame:?}"),
        }
        Ok(())
    }

    fn recv(&self, timeout: Duration) -> Result<Option<Frame>> {
        let mut motor = self.borrow_mut();
        if let Some(frame) = motor.received.pop_front() {
            return Ok(Some(frame));
        }
        let noise_interval = Duration::from_millis(400);
        if motor.noisy && timeout >= noise_interval {
            motor.now += noise_interval;
            return Ok(Some(Frame {
                id: 0x1800_08fd,
                data: [0; 8],
            }));
        }
        motor.now += timeout;
        Ok(None)
    }
}

fn write_kinds(wire: &RefCell<MotorOnWire>) -> Vec<u32> {
    wire.borrow()
        .sent
        .iter()
        .map(|frame| frame.id >> 24)
        .filter(|kind| matches!(kind, 6 | 11..=14))
        .collect()
}

#[test]
fn enabled_peer_refuses_maintenance_before_stopping_or_silencing_motors() {
    let wire = RefCell::new(MotorOnWire {
        enabled_peer: Some(8),
        ..MotorOnWire::default()
    });
    let spec = motor();
    let error = Link::new(&wire, &spec, HOST)
        .prepare(&[ID, 8], false, true)
        .err()
        .expect("an externally enabled peer must block maintenance");
    assert!(error.to_string().contains("motor 8"));
    assert!(
        wire.borrow()
            .sent
            .iter()
            .all(|frame| frame.id >> 24 == 24 && frame.data[6] != 0)
    );
}

#[test]
fn ota_retries_wrong_ack_rewinds_and_verifies_after_lost_end_ack() {
    let wire = RefCell::new(MotorOnWire {
        ota: Some(OtaBehavior {
            lose_data_ack: true,
            lose_end_ack: true,
            resume: Some(0),
            ..OtaBehavior::default()
        }),
        ..MotorOnWire::default()
    });
    let spec = motor();
    let link = Link::new(&wire, &spec, HOST);
    let prepared = link.prepare(&[8, ID, 8], true, true).unwrap();
    assert_eq!(prepared.uuid, UUID);
    assert_eq!(prepared.current_version, "0.3.0.1");
    assert_eq!(prepared.target_version.as_deref(), Some("0.3.0.2"));
    assert!(write_kinds(&wire).is_empty());

    let progress = RefCell::new(Vec::new());
    link.execute(
        &prepared,
        || Ok(()),
        |value| progress.borrow_mut().push(value),
    )
    .unwrap();
    assert_eq!(wire.borrow().version, [0, 3, 0, 2]);
    assert_eq!(
        write_kinds(&wire),
        [11, 12, 13, 13, 13, 13, 13, 13, 14, 14, 14]
    );
    assert_eq!(*progress.borrow(), [0, 31, 63, 95, 97, 100]);
    for id in [ID, 8] {
        assert_eq!(
            wire.borrow()
                .sent
                .iter()
                .filter(|f| f.id >> 24 == 24 && f.id as u8 == id && f.data[6] == 0)
                .count(),
            if id == ID { 3 } else { 2 },
        );
    }
}

#[test]
fn ota_refuses_forward_resume_and_requires_post_update_version() {
    for ota in [
        OtaBehavior {
            resume: Some(2),
            ..OtaBehavior::default()
        },
        OtaBehavior {
            reject_install: true,
            ..OtaBehavior::default()
        },
    ] {
        let expect_end = ota.reject_install;
        let wire = RefCell::new(MotorOnWire {
            ota: Some(ota),
            ..MotorOnWire::default()
        });
        let spec = motor();
        let link = Link::new(&wire, &spec, HOST);
        let prepared = link.prepare(&[], true, true).unwrap();
        let completed = RefCell::new(false);
        assert!(
            link.execute(
                &prepared,
                || Ok(()),
                |value| {
                    if value == 100 {
                        *completed.borrow_mut() = true;
                    }
                }
            )
            .is_err()
        );
        assert!(!*completed.borrow());
        assert_eq!(write_kinds(&wire).contains(&14), expect_end);
        assert_eq!(wire.borrow().version, [0, 3, 0, 1]);
    }
}

#[test]
fn execute_blocks_writes_if_identity_version_or_bus_activity_changes() {
    for change in [
        |motor: &mut MotorOnWire| motor.uuid = [3; 8],
        |motor: &mut MotorOnWire| motor.version = [0, 3, 0, 2],
        |motor: &mut MotorOnWire| motor.noisy = true,
        |motor: &mut MotorOnWire| motor.fault_bits = 1,
        |motor: &mut MotorOnWire| motor.moving = true,
        |motor: &mut MotorOnWire| motor.position += 0.1,
    ] {
        let wire = RefCell::new(MotorOnWire::default());
        let spec = motor();
        let link = Link::new(&wire, &spec, HOST);
        let prepared = link.prepare(&[], false, true).unwrap();
        change(&mut wire.borrow_mut());
        assert!(link.execute(&prepared, || Ok(()), |_| {}).is_err());
        assert!(write_kinds(&wire).is_empty());
    }
}

#[test]
fn re_zero_requires_stationary_position_readback() {
    let wire = RefCell::new(MotorOnWire {
        accept_zero: false,
        ..MotorOnWire::default()
    });
    let spec = motor();
    let link = Link::new(&wire, &spec, HOST);
    let prepared = link.prepare(&[], false, true).unwrap();
    assert_eq!(prepared.target_version, None);
    assert!(link.execute(&prepared, || Ok(()), |_| {}).is_err());

    assert_eq!(write_kinds(&wire), [6]);
    wire.borrow_mut().accept_zero = true;
    link.execute(&prepared, || Ok(()), |_| {}).unwrap();
    assert_eq!(write_kinds(&wire), [6, 6]);
}

#[test]
fn feedback_after_a_long_gap_must_establish_a_new_stationary_window() {
    let mut check = StationaryCheck::new(2.0 * std::f64::consts::TAU);
    let start = Instant::now();
    let mut observe = |ms| check.observe(Feedback::default(), start + Duration::from_millis(ms));

    for ms in [0, 25, 50, 75, 200, 225, 250, 275] {
        assert!(!observe(ms), "premature stationary verdict at {ms} ms");
    }
    assert!(observe(300));
}
