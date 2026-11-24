//! EBIMU UART encoding. Physics samples stay in the physical sensor frame.
use crate::Simulation;
use crate::uart::Uart;
use anyhow::{Result, ensure};
use nalgebra::{Quaternion, UnitQuaternion, Vector3};
use std::path::Path;

const GRAVITY: f64 = 9.80665;

pub struct Sample {
    pub time: f64,
    pub orientation: UnitQuaternion<f64>,
    pub gyro: Vector3<f64>,
    pub specific_force: Vector3<f64>,
    pub local_velocity: Vector3<f64>,
    pub position: Vector3<f64>,
}

impl Sample {
    pub fn read(sim: &Simulation, name: &str) -> Result<Self> {
        let q = sim.sensor::<4>(&format!("{name}_orientation"))?;
        let id = sim.id(crate::ffi::mjtObj::mjOBJ_SITE, &format!("{name}_site"))? as usize;
        let v = sim
            .data
            .object_velocity(crate::ffi::mjtObj::mjOBJ_SITE, id, true);
        Ok(Self {
            time: sim.data().time,
            orientation: UnitQuaternion::new_normalize(Quaternion::new(q[0], q[1], q[2], q[3])),
            gyro: sim.sensor::<3>(&format!("{name}_gyro"))?.into(),
            specific_force: sim.sensor::<3>(&format!("{name}_accelerometer"))?.into(),
            local_velocity: Vector3::new(v[3], v[4], v[5]),
            position: sim.data.site_xpos()[id].into(),
        })
    }
}

#[derive(Clone)]
pub struct Encoder {
    interval_ms: u16,
    ascii: bool,
    quaternion: bool,
    gyro: bool,
    accel: u8,
    magnet: bool,
    distance: u8,
    temperature: bool,
    timestamp: bool,
    running: bool,
    power_on: bool,
    temperature_c: f64,
    origin: Option<Vector3<f64>>,
    command: Option<Vec<u8>>,
    next: f64,
    previous: f64,
}

impl Encoder {
    pub fn new(rate_hz: f64, temperature_c: f64) -> Result<Self> {
        ensure!(
            rate_hz.is_finite() && (1. ..=1000.).contains(&rate_hz),
            "invalid EBIMU rate"
        );
        ensure!(temperature_c.is_finite(), "invalid EBIMU temperature");
        Ok(Self {
            interval_ms: (1000. / rate_hz).round() as u16,
            ascii: true,
            quaternion: true,
            gyro: true,
            accel: 2,
            magnet: false,
            distance: 0,
            temperature: false,
            timestamp: true,
            running: true,
            power_on: true,
            temperature_c,
            origin: None,
            command: None,
            next: 0.,
            previous: 0.,
        })
    }

    pub fn update(&mut self, sample: &Sample, input: &[u8]) -> Vec<u8> {
        if sample.time < self.previous {
            self.next = sample.time;
        }
        self.previous = sample.time;
        let mut output = Vec::new();
        for &byte in input {
            if byte == b'<' {
                self.command = Some(Vec::new());
            } else if let Some(command) = &mut self.command {
                if byte == b'>' {
                    let bytes = self.command.take().unwrap();
                    let reply = std::str::from_utf8(&bytes)
                        .ok()
                        .map(|s| self.configure(s, sample))
                        .unwrap_or("<er>");
                    output.extend_from_slice(reply.as_bytes());
                } else if command.len() < 128 {
                    command.push(byte);
                } else {
                    self.command = None;
                    output.extend_from_slice(b"<er>");
                }
            } else if byte == b'*' && self.interval_ms == 0 {
                output.extend(self.encode(sample));
            } else if byte == b'/' {
                self.origin = Some(sample.position);
            }
        }
        if input.is_empty()
            && self.running
            && self.interval_ms > 0
            && sample.time + 1e-12 >= self.next
        {
            output.extend(self.encode(sample));
            let period = f64::from(self.interval_ms) / 1000.;
            self.next += period;
            if self.next < sample.time - period {
                self.next = sample.time + period;
            }
        }
        output
    }

    fn configure(&mut self, command: &str, sample: &Sample) -> &'static str {
        match command {
            "ver" => return "<imu9dofv610>",
            "start" => {
                self.running = true;
                self.next = sample.time;
                return "<ok>";
            }
            "stop" => {
                self.running = false;
                return "<ok>";
            }
            "reset" => {
                self.running = self.power_on;
                self.next = sample.time;
                return "<ok>";
            }
            "posz" => {
                self.origin = Some(sample.position);
                return "<ok>";
            }
            "lf" => {
                *self = Self::new(100., self.temperature_c).unwrap();
                self.quaternion = false;
                self.gyro = false;
                self.accel = 0;
                self.timestamp = false;
                self.next = sample.time;
                return "<ok>";
            }
            _ => {}
        }
        let split = command
            .find(|c: char| c.is_ascii_digit())
            .unwrap_or(command.len());
        let (key, value) = command.split_at(split);
        let Ok(value) = value.parse::<u16>() else {
            return "<er>";
        };
        match (key, value) {
            ("soc", 1..=2) => self.ascii = value == 1,
            ("sof", 1..=2) => self.quaternion = value == 2,
            ("sog", 0..=1) => self.gyro = value == 1,
            ("soa", 0..=5) => self.accel = value as u8,
            ("som", 0..=1) => self.magnet = value == 1,
            ("sod", 0..=2) => self.distance = value as u8,
            ("sot", 0..=1) => self.temperature = value == 1,
            ("sots", 0..=1) => self.timestamp = value == 1,
            ("pons", 0..=1) => self.power_on = value == 1,
            ("sor", 0..=1000) => {
                self.interval_ms = value;
                self.next = sample.time;
            }
            ("sb", 1..=8) | ("sem", 0..=2) | ("ssg", 1..=5) | ("ssa", 1..=4) | ("lpfa", 0..=9) => {}
            _ => return "<er>",
        }
        "<ok>"
    }

    pub fn encode(&mut self, sample: &Sample) -> Vec<u8> {
        let q = sample.orientation;
        let mut fields: Vec<(String, f64, f64)> = Vec::new();
        let mut add = |value: f64, precision: usize, scale: f64| {
            fields.push((format!("{value:.precision$}"), value, scale))
        };
        if self.quaternion {
            for v in [-q.k, -q.j, q.i, q.w] {
                add(v, 4, 10000.);
            }
        } else {
            let attitude = UnitQuaternion::new_normalize(Quaternion::new(q.w, q.i, -q.j, -q.k));
            let (roll, pitch, yaw) = attitude.euler_angles();
            for v in [roll, pitch, yaw] {
                add(v.to_degrees(), 2, 100.);
            }
        }
        if self.gyro {
            for v in sample.gyro.iter() {
                add(v.to_degrees(), 2, 10.);
            }
        }
        let local_linear =
            sample.specific_force + q.inverse_transform_vector(&Vector3::new(0., 0., -GRAVITY));
        let acceleration = match self.accel {
            1 => sample.specific_force / GRAVITY,
            2 => local_linear / GRAVITY,
            3 => q.transform_vector(&local_linear) / GRAVITY,
            4 => sample.local_velocity,
            5 => q.transform_vector(&sample.local_velocity),
            _ => Vector3::zeros(),
        };
        if self.accel != 0 {
            for &v in acceleration.iter() {
                add(v, 3, 1000.);
            }
        }
        if self.magnet {
            for &v in q
                .inverse_transform_vector(&Vector3::new(20., 0., -45.))
                .iter()
            {
                add(v, 1, 10.);
            }
        }
        if self.distance != 0 {
            let mut delta = sample.position - *self.origin.get_or_insert(sample.position);
            if self.distance == 1 {
                delta = q.inverse_transform_vector(&delta);
            }
            for &v in delta.iter() {
                add(v, 3, 1000.);
            }
        }
        if self.temperature {
            add(self.temperature_c, 1, 10.);
        }
        if self.timestamp {
            add((sample.time * 1000.).round_ties_even() % 60001., 0, 1.);
        }
        if self.ascii {
            return format!(
                "*{}\r\n",
                fields
                    .iter()
                    .map(|f| f.0.as_str())
                    .collect::<Vec<_>>()
                    .join(",")
            )
            .into_bytes();
        }
        let mut bytes = vec![0x55, 0x55];
        for (_, value, scale) in fields {
            bytes.extend_from_slice(
                &((value * scale).round_ties_even() as i64 as u16).to_be_bytes(),
            );
        }
        let checksum = bytes
            .iter()
            .fold(0u16, |a, &b| a.wrapping_add(u16::from(b)));
        bytes.extend_from_slice(&checksum.to_be_bytes());
        bytes
    }
}

pub struct Serial {
    uart: Uart<Encoder>,
}

impl Serial {
    pub fn open(path: &Path, rate: f64, temperature: f64) -> Result<Self> {
        Ok(Self {
            uart: Uart::open(path, Encoder::new(rate, temperature)?)?,
        })
    }
    pub fn update(&mut self, sample: &Sample) -> Result<()> {
        self.uart
            .update(|encoder, input| Ok(encoder.update(sample, input)))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs::File;
    use std::io::{Read, Write};
    use std::os::fd::AsRawFd;

    fn sample() -> Sample {
        let orientation = UnitQuaternion::from_euler_angles(0.2, -0.3, 0.4);
        Sample {
            time: 0.,
            orientation,
            gyro: Vector3::new(0.1, -0.2, 0.3),
            specific_force: orientation.inverse_transform_vector(&Vector3::new(0., 0., GRAVITY)),
            local_velocity: Vector3::zeros(),
            position: Vector3::new(0.1, 0.2, 0.3),
        }
    }

    #[test]
    fn default_wire_quaternion_and_acceleration_match_hardware_decoder() {
        let sample = sample();
        let bytes = Encoder::new(200., 25.).unwrap().encode(&sample);
        let text = std::str::from_utf8(&bytes).unwrap();
        let values: Vec<f64> = text
            .trim()
            .trim_start_matches('*')
            .split(',')
            .map(|s| s.parse().unwrap())
            .collect();
        assert_eq!(values.len(), 11);
        for (a, b) in [values[2], -values[1], -values[0], values[3]].iter().zip([
            sample.orientation.i,
            sample.orientation.j,
            sample.orientation.k,
            sample.orientation.w,
        ]) {
            assert!((a - b).abs() <= 0.00005);
        }
        for (v, expected) in values[4..7].iter().zip(sample.gyro.iter()) {
            assert!((v.to_radians() - expected).abs() < 0.0001);
        }
        assert_eq!(&values[7..10], &[0., 0., 0.]);
    }

    #[test]
    fn command_fragments_overflow_binary_checksum_and_clock_rewind() {
        let mut encoder = Encoder::new(200., 25.).unwrap();
        let mut sample = sample();
        assert!(encoder.update(&sample, b"<so").is_empty());
        assert_eq!(encoder.update(&sample, b"c2>"), b"<ok>");
        let bytes = encoder.update(&sample, b"");
        assert_eq!(&bytes[..2], &[0x55, 0x55]);
        let checksum = bytes[..bytes.len() - 2]
            .iter()
            .map(|&b| u16::from(b))
            .sum::<u16>();
        assert_eq!(&bytes[bytes.len() - 2..], &checksum.to_be_bytes());
        assert!(encoder.update(&sample, b"").is_empty());
        sample.time = 1.;
        assert!(!encoder.update(&sample, b"").is_empty());
        sample.time = 0.;
        assert!(!encoder.update(&sample, b"").is_empty());
        let bad = [b"<".as_slice(), &[b'a'; 129]].concat();
        assert_eq!(encoder.update(&sample, &bad), b"<er>");
        assert_eq!(encoder.update(&sample, b"<sor0>"), b"<ok>");
        assert!(encoder.update(&sample, b"").is_empty());
        assert!(!encoder.update(&sample, b"*").is_empty());
    }

    #[test]
    fn stalled_serial_client_can_reconnect_to_a_fresh_imu() {
        use std::io::ErrorKind;
        use std::time::{Duration, Instant};

        let name = format!("imu-stall-{}", std::process::id());
        let mut serial = Serial::open(Path::new(&format!("pty:{name}")), 200., 25.).unwrap();
        let mut sample = sample();
        let attach = |serial: &mut Serial, sample: &Sample| {
            std::thread::scope(|scope| {
                let client =
                    scope.spawn(|| bello_sim_serial::connect(&name, Duration::from_secs(2)));
                while !client.is_finished() {
                    serial.update(sample).unwrap();
                    std::thread::yield_now();
                }
                let (slave, control) = client.join().unwrap().unwrap();
                nix::fcntl::fcntl(
                    slave.as_raw_fd(),
                    nix::fcntl::FcntlArg::F_SETFL(nix::fcntl::OFlag::O_NONBLOCK),
                )
                .unwrap();
                (File::from(slave), control)
            })
        };
        let (mut stalled, mut control) = attach(&mut serial, &sample);
        stalled.write_all(b"<soc2>").unwrap();
        serial.update(&sample).unwrap();
        sample.time += 0.01;
        serial.update(&sample).unwrap();
        let mut bytes = Vec::new();
        let deadline = Instant::now() + Duration::from_secs(2);
        while !bytes.windows(2).any(|word| word == [0x55, 0x55]) {
            assert!(
                Instant::now() < deadline,
                "IMU did not switch to binary mode"
            );
            sample.time += 0.01;
            serial.update(&sample).unwrap();
            if let Err(e) = stalled.read_to_end(&mut bytes) {
                assert_eq!(e.kind(), ErrorKind::WouldBlock);
            }
        }
        control.set_nonblocking(true).unwrap();
        let deadline = Instant::now() + Duration::from_secs(5);
        loop {
            assert!(
                Instant::now() < deadline,
                "stalled reader was not disconnected"
            );
            sample.time += 0.01;
            serial.update(&sample).unwrap();
            match control.read(&mut [0]) {
                Ok(0) => break,
                Err(e) if e.kind() == ErrorKind::WouldBlock => {}
                result => panic!("unexpected control read: {result:?}"),
            }
        }
        let (mut restarted, _control) = attach(&mut serial, &sample);
        sample.time += 0.01;
        serial.update(&sample).unwrap();
        let mut pfd = [nix::poll::PollFd::new(
            std::os::fd::AsFd::as_fd(&restarted),
            nix::poll::PollFlags::POLLIN,
        )];
        assert_eq!(nix::poll::poll(&mut pfd, 2000u16).unwrap(), 1);
        let mut first = [0];
        restarted.read_exact(&mut first).unwrap();
        assert_eq!(first, *b"*", "new IMU session must start in ASCII mode");
    }

    #[test]
    fn serial_processes_fragmented_commands_on_a_real_pty() {
        let pty = nix::pty::openpty(None, None).unwrap();
        let path = nix::unistd::ttyname(&pty.slave).unwrap();
        let mut serial = Serial::open(&path, 200., 25.).unwrap();
        let mut peer = File::from(pty.master);
        nix::fcntl::fcntl(
            peer.as_raw_fd(),
            nix::fcntl::FcntlArg::F_SETFL(nix::fcntl::OFlag::O_NONBLOCK),
        )
        .unwrap();
        peer.write_all(b"<ver>").unwrap();
        serial.update(&sample()).unwrap();
        let mut bytes = [0; 64];
        let count = peer.read(&mut bytes).unwrap();
        assert_eq!(&bytes[..count], b"<imu9dofv610>");
    }
}
