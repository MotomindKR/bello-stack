//! Synthetic AGX COG telemetry. No battery physics or power-switching model.
use crate::transport::Uart;
use anyhow::{Result, ensure};
use cog_protocol::{Profile, encode};
use serde::Deserialize;
use std::path::PathBuf;
use std::time::Duration;

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Config {
    pub name: String,
    pub device: PathBuf,
    pub battery_voltage_v: f32,
    pub battery_current_a: f32,
    pub battery_soc_percent: f32,
    pub motor_voltage_v: f32,
    pub motor_current_a: f32,
    pub temperature_c: f32,
    pub pressure_pa: f32,
    pub local_rails: [f32; 6],
    pub gpi_bits: u32,
    pub info_bits: u32,
    pub warning_bits: u32,
    pub critical_bits: u32,
    pub mute_telemetry: bool,
}

impl Config {
    pub fn validate(&self) -> Result<()> {
        ensure!(
            !self.name.trim().is_empty() && !self.device.as_os_str().is_empty(),
            "empty power board name/device"
        );
        ensure!(
            [
                self.battery_voltage_v,
                self.battery_current_a,
                self.battery_soc_percent,
                self.motor_voltage_v,
                self.motor_current_a,
                self.temperature_c,
                self.pressure_pa
            ]
            .iter()
            .chain(self.local_rails.iter())
            .all(|v| v.is_finite()),
            "nonfinite power board fixture"
        );
        ensure!(
            (0. ..=100.).contains(&self.battery_soc_percent),
            "invalid battery percentage"
        );
        Ok(())
    }
}

pub struct Encoder {
    config: Config,
    sequence: u8,
    command: Vec<u8>,
    discarding: bool,
    last_sync: Option<Duration>,
    next_sample: Duration,
}

impl Encoder {
    pub fn new(config: Config) -> Result<Self> {
        config.validate()?;
        Ok(Self {
            config,
            sequence: 0,
            command: Vec::new(),
            discarding: false,
            last_sync: None,
            next_sample: Duration::ZERO,
        })
    }

    /// Internal heartbeat state; not an invented GPIO/IWC flag or motor interlock.
    pub fn host_valid(&self, now: Duration) -> bool {
        self.last_sync
            .is_some_and(|at| now.saturating_sub(at) < Duration::from_secs(5))
    }

    pub fn update(&mut self, now: Duration, commands: &[u8]) -> Result<Vec<u8>> {
        for &byte in commands {
            if byte == b'\n' {
                if !self.discarding && self.command == b"sync\r" {
                    self.last_sync = Some(now);
                }
                self.command.clear();
                self.discarding = false;
            } else if !self.discarding {
                if self.command.len() == 128 {
                    self.command.clear();
                    self.discarding = true;
                } else {
                    self.command.push(byte);
                }
            }
        }
        if self.config.mute_telemetry || now < self.next_sample {
            return Ok(Vec::new());
        }
        self.next_sample = now + Duration::from_secs(1);
        let c = &self.config;
        let floats = |values: &[f32]| {
            values
                .iter()
                .flat_map(|v| v.to_le_bytes())
                .collect::<Vec<_>>()
        };
        let packets = [
            (
                1,
                floats(&[
                    c.battery_voltage_v,
                    c.battery_current_a,
                    c.battery_soc_percent,
                ]),
            ),
            (2, c.gpi_bits.to_le_bytes().to_vec()),
            (
                3,
                [c.info_bits, c.warning_bits, c.critical_bits]
                    .into_iter()
                    .flat_map(u32::to_le_bytes)
                    .collect(),
            ),
            (5, floats(&[c.temperature_c, c.pressure_pa])),
            (6, floats(&c.local_rails)),
            (7, floats(&[c.motor_voltage_v, c.motor_current_a])),
            (8, (now.as_millis() as u32).to_le_bytes().to_vec()),
        ];
        let mut bytes = Vec::new();
        for (kind, payload) in packets {
            bytes.extend(encode(Profile::Agx, kind, self.sequence, &payload)?);
            self.sequence = self.sequence.wrapping_add(1);
        }
        Ok(bytes)
    }
}

pub struct Serial {
    uart: Uart,
    encoder: Encoder,
}

impl Serial {
    pub fn open(config: Config) -> Result<Self> {
        config.validate()?;
        let uart = Uart::open(&config.device)?;
        Ok(Self {
            uart,
            encoder: Encoder::new(config)?,
        })
    }
    pub fn update(&mut self, now: Duration) -> Result<()> {
        let mut buffer = [0; 4096];
        let count = self.uart.read(&mut buffer)?;
        self.uart
            .write(&self.encoder.update(now, &buffer[..count])?)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cog_protocol::{Framer, Reading};
    use std::io::{Read, Write};
    use std::os::fd::AsRawFd;

    fn config() -> Config {
        serde_yaml::from_str::<crate::runtime::Config>(include_str!("../config/emulator_mini.yaml"))
            .unwrap()
            .power_board
            .unwrap()
    }
    fn decode(bytes: Vec<u8>) -> Vec<cog_protocol::Frame> {
        let mut parser = Framer::default();
        bytes
            .into_iter()
            .filter_map(|b| parser.push(b, Profile::Agx))
            .map(Result::unwrap)
            .collect()
    }
    #[test]
    fn packets_match_daemon_decoder_and_flags_are_not_inferred() {
        let mut c = config();
        c.battery_current_a = -1.25;
        c.critical_bits = 1 << 31;
        let mut encoder = Encoder::new(c).unwrap();
        let frames = decode(encoder.update(Duration::ZERO, b"sy").unwrap());
        assert_eq!(
            frames.iter().map(|f| f.kind).collect::<Vec<_>>(),
            [1, 2, 3, 5, 6, 7, 8]
        );
        assert!(matches!(
            frames[0].reading,
            Some(Reading::Battery { current: -1.25, .. })
        ));
        assert!(matches!(
            frames[2].reading,
            Some(Reading::Iwc {
                critical: 0x80000000,
                ..
            })
        ));
        assert!(!encoder.host_valid(Duration::ZERO));
        assert!(
            encoder
                .update(Duration::from_millis(10), b"nc\r\n")
                .unwrap()
                .is_empty()
        );
        assert!(encoder.host_valid(Duration::from_secs(4)));
        assert!(!encoder.host_valid(Duration::from_secs(6)));
        // Missed output periods never cause a catch-up flood.
        assert_eq!(
            decode(encoder.update(Duration::from_secs(100), b"").unwrap()).len(),
            7
        );
        assert!(
            encoder
                .update(Duration::from_secs(100), b"")
                .unwrap()
                .is_empty()
        );
    }
    #[test]
    fn malformed_commands_and_mute_are_bounded() {
        let mut c = config();
        c.mute_telemetry = true;
        let mut e = Encoder::new(c).unwrap();
        assert!(e.update(Duration::ZERO, &[b'x'; 10000]).unwrap().is_empty());
        assert!(e.command.len() <= 128);
        e.update(Duration::ZERO, b"sync\r\n").unwrap();
        assert!(!e.host_valid(Duration::ZERO));
        e.update(Duration::ZERO, b"sync\r\n").unwrap();
        assert!(e.host_valid(Duration::ZERO));
        let mut c = config();
        c.battery_soc_percent = 101.;
        assert!(Encoder::new(c).is_err());
    }
    #[test]
    fn real_pty_produces_valid_wire_data() {
        let pty = nix::pty::openpty(None, None).unwrap();
        let mut c = config();
        c.device = nix::unistd::ttyname(&pty.slave).unwrap();
        let mut serial = Serial::open(c).unwrap();
        let mut peer = std::fs::File::from(pty.master);
        nix::fcntl::fcntl(
            peer.as_raw_fd(),
            nix::fcntl::FcntlArg::F_SETFL(nix::fcntl::OFlag::O_NONBLOCK),
        )
        .unwrap();
        peer.write_all(b"sync\r\n").unwrap();
        serial.update(Duration::ZERO).unwrap();
        let mut bytes = [0; 4096];
        let count = peer.read(&mut bytes).unwrap();
        assert_eq!(decode(bytes[..count].to_vec()).len(), 7);
        assert!(serial.encoder.host_valid(Duration::ZERO));
    }
}
