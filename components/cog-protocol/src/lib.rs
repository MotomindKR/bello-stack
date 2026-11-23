use anyhow::{Result, bail, ensure};
use serde::Deserialize;

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum Profile {
    Agx,
    NanoNx,
}

impl Profile {
    pub fn name(self) -> &'static str {
        match self {
            Self::Agx => "agx",
            Self::NanoNx => "nano_nx",
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub enum Reading {
    Battery {
        voltage: f32,
        current: f32,
        soc_percent: Option<f32>,
    },
    Gpi {
        bits: u32,
        second_bits: Option<u32>,
    },
    Iwc {
        info: u32,
        warning: u32,
        critical: u32,
    },
    LegacyFault {
        alarm: u32,
        fault: u32,
    },
    Environment {
        temperature_c: f32,
        pressure_pa: f32,
    },
    LocalRails {
        values: [f32; 6],
    },
    MotorRail {
        voltage: f32,
        current: f32,
    },
    Tick {
        count: u32,
    },
    Encoder {
        left_rpm: f32,
        right_rpm: f32,
    },
}

#[derive(Clone, Debug, PartialEq)]
pub struct Frame {
    pub kind: u8,
    pub sequence: u8,
    pub reading: Option<Reading>,
}

// A Nano/NX length is u16, but known packets are tiny. Bound allocation even on
// a corrupt stream; after overflow discard the entire block through its delimiter.
#[derive(Default)]
pub struct Framer {
    bytes: Vec<u8>,
    discarding: bool,
}

impl Framer {
    pub fn push(&mut self, byte: u8, profile: Profile) -> Option<Result<Frame>> {
        if byte == 0 {
            if self.discarding {
                self.discarding = false;
                return Some(Err(anyhow::anyhow!("COG block exceeded 1024 bytes")));
            }
            if self.bytes.is_empty() {
                return None;
            }
            let result = decode(profile, &self.bytes);
            self.bytes.clear();
            return Some(result);
        }
        if !self.discarding {
            if self.bytes.len() == 1024 {
                self.bytes.clear();
                self.discarding = true;
            } else {
                self.bytes.push(byte);
            }
        }
        None
    }
}

fn crc16(bytes: &[u8]) -> u16 {
    let mut crc = 0xffffu16;
    for byte in bytes {
        crc ^= u16::from(*byte) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ 0x1021
            } else {
                crc << 1
            };
        }
    }
    crc
}

fn crc32(bytes: &[u8]) -> u32 {
    let mut crc = 0xffffffffu32;
    for byte in bytes {
        crc ^= u32::from(*byte) << 24;
        for _ in 0..8 {
            crc = if crc & 0x80000000 != 0 {
                (crc << 1) ^ 0x04c11db7
            } else {
                crc << 1
            };
        }
    }
    crc
}

fn decode(profile: Profile, encoded: &[u8]) -> Result<Frame> {
    let mut raw = Vec::with_capacity(encoded.len());
    let mut at = 0;
    while at < encoded.len() {
        let code = usize::from(encoded[at]);
        at += 1;
        ensure!(
            code != 0 && at + code - 1 <= encoded.len(),
            "invalid COBS encoding"
        );
        raw.extend_from_slice(&encoded[at..at + code - 1]);
        at += code - 1;
        if code != 255 && at < encoded.len() {
            raw.push(0);
        }
    }
    let (header, checksum) = match profile {
        Profile::Agx => (3, 2),
        Profile::NanoNx => (4, 4),
    };
    ensure!(raw.len() >= header + checksum, "short COG frame");
    let length = if profile == Profile::Agx {
        usize::from(raw[2])
    } else {
        usize::from(u16::from_le_bytes([raw[2], raw[3]]))
    };
    ensure!(
        raw.len() == header + length + checksum,
        "COG length mismatch"
    );
    let body = &raw[..header + length];
    let trailer = &raw[header + length..];
    let valid = if profile == Profile::Agx {
        crc16(body) == u16::from_le_bytes(trailer.try_into().unwrap())
    } else {
        crc32(body) == u32::from_le_bytes(trailer.try_into().unwrap())
    };
    ensure!(valid, "COG checksum mismatch");
    let data = &raw[header..header + length];
    let expected = match (profile, raw[0]) {
        (Profile::Agx, 1 | 3) => 12,
        (Profile::Agx, 2 | 8) => 4,
        (Profile::Agx, 6) => 24,
        (Profile::NanoNx, 1..=4) | (_, 5) | (Profile::Agx, 7) => 8,
        _ => {
            return Ok(Frame {
                kind: raw[0],
                sequence: raw[1],
                reading: None,
            });
        }
    };
    ensure!(
        length == expected,
        "invalid payload length for COG type {}",
        raw[0]
    );
    let word = |i: usize| u32::from_le_bytes(data[i..i + 4].try_into().unwrap());
    let number = |i: usize| -> Result<f32> {
        let value = f32::from_bits(word(i));
        ensure!(value.is_finite(), "nonfinite COG reading");
        Ok(value)
    };
    let reading = match raw[0] {
        1 => {
            let soc_percent = if profile == Profile::Agx {
                let soc = number(8)?;
                ensure!((0.0..=100.0).contains(&soc), "invalid battery percentage");
                Some(soc)
            } else {
                None
            };
            Reading::Battery {
                voltage: number(0)?,
                current: number(4)?,
                soc_percent,
            }
        }
        2 => Reading::Gpi {
            bits: word(0),
            second_bits: (profile == Profile::NanoNx).then(|| word(4)),
        },
        3 if profile == Profile::Agx => Reading::Iwc {
            info: word(0),
            warning: word(4),
            critical: word(8),
        },
        3 => Reading::LegacyFault {
            alarm: word(0),
            fault: word(4),
        },
        4 => Reading::Encoder {
            left_rpm: number(0)?,
            right_rpm: number(4)?,
        },
        5 => Reading::Environment {
            temperature_c: number(0)?,
            pressure_pa: number(4)?,
        },
        6 => Reading::LocalRails {
            values: [
                number(0)?,
                number(4)?,
                number(8)?,
                number(12)?,
                number(16)?,
                number(20)?,
            ],
        },
        7 => Reading::MotorRail {
            voltage: number(0)?,
            current: number(4)?,
        },
        8 => Reading::Tick { count: word(0) },
        _ => bail!("unsupported COG type"),
    };
    Ok(Frame {
        kind: raw[0],
        sequence: raw[1],
        reading: Some(reading),
    })
}

/// Encode a bounded COG frame; payload semantics are checked by the decoder.
pub fn encode(profile: Profile, kind: u8, sequence: u8, payload: &[u8]) -> Result<Vec<u8>> {
    ensure!(payload.len() <= 255, "COG payload exceeds encoder bound");
    let mut raw = vec![kind, sequence, payload.len() as u8];
    if profile == Profile::NanoNx {
        raw.push(0);
    }
    raw.extend_from_slice(payload);
    match profile {
        Profile::Agx => raw.extend_from_slice(&crc16(&raw).to_le_bytes()),
        Profile::NanoNx => raw.extend_from_slice(&crc32(&raw).to_le_bytes()),
    }
    let mut encoded = vec![0];
    let mut index = 0;
    let mut code = 1u8;
    for byte in raw {
        if byte == 0 {
            encoded[index] = code;
            index = encoded.len();
            encoded.push(0);
            code = 1;
        } else {
            encoded.push(byte);
            code += 1;
            if code == 255 {
                encoded[index] = code;
                index = encoded.len();
                encoded.push(0);
                code = 1;
            }
        }
    }
    encoded[index] = code;
    encoded.push(0);
    Ok(encoded)
}

#[cfg(test)]
mod tests {
    use super::*;

    pub fn packet(profile: Profile, kind: u8, data: &[u8]) -> Vec<u8> {
        let mut raw = vec![kind, 7, data.len() as u8];
        if profile == Profile::NanoNx {
            raw.push(0);
        }
        raw.extend_from_slice(data);
        if profile == Profile::Agx {
            raw.extend_from_slice(&crc16(&raw).to_le_bytes());
        } else {
            raw.extend_from_slice(&crc32(&raw).to_le_bytes());
        }
        let mut encoded = vec![0];
        let mut code_at = 0;
        let mut code = 1u8;
        for byte in raw {
            if byte == 0 {
                encoded[code_at] = code;
                code_at = encoded.len();
                encoded.push(0);
                code = 1;
            } else {
                encoded.push(byte);
                code += 1;
            }
        }
        encoded[code_at] = code;
        encoded.push(0);
        encoded
    }

    #[test]
    fn checksums_match_reference() {
        assert_eq!(crc16(b"123456789"), 0x29b1);
        assert_eq!(crc32(b"123456789"), 0x0376e6e7);
    }

    #[test]
    fn profiles_preserve_optional_soc_and_signed_current() {
        for profile in [Profile::Agx, Profile::NanoNx] {
            let mut data: Vec<u8> = [48.5f32, -1.25]
                .into_iter()
                .flat_map(f32::to_le_bytes)
                .collect();
            if profile == Profile::Agx {
                data.extend_from_slice(&73f32.to_le_bytes());
            }
            let bytes = packet(profile, 1, &data);
            let frame = decode(profile, &bytes[..bytes.len() - 1]).unwrap();
            assert_eq!(
                frame.reading,
                Some(Reading::Battery {
                    voltage: 48.5,
                    current: -1.25,
                    soc_percent: (profile == Profile::Agx).then_some(73.0)
                })
            );
            let other = if profile == Profile::Agx {
                Profile::NanoNx
            } else {
                Profile::Agx
            };
            assert!(decode(other, &bytes[..bytes.len() - 1]).is_err());
        }
    }

    #[test]
    fn framing_recovers_after_overflow_corruption_and_split_chunks() {
        let mut framer = Framer::default();
        for _ in 0..2000 {
            assert!(framer.push(1, Profile::Agx).is_none());
        }
        assert!(framer.push(0, Profile::Agx).unwrap().is_err());
        let good = packet(Profile::Agx, 8, &123u32.to_le_bytes());
        let mut bad = good.clone();
        bad[2] ^= 1;
        let mut results = Vec::new();
        for byte in bad.into_iter().chain(good) {
            if let Some(frame) = framer.push(byte, Profile::Agx) {
                results.push(frame);
            }
        }
        assert!(results[0].is_err());
        assert_eq!(
            results[1].as_ref().unwrap().reading,
            Some(Reading::Tick { count: 123 })
        );
    }

    #[test]
    fn invalid_lengths_and_nonfinite_values_are_rejected() {
        for data in [
            vec![0; 8],
            [f32::NAN, 0.0, 50.0]
                .into_iter()
                .flat_map(f32::to_le_bytes)
                .collect(),
        ] {
            let p = packet(Profile::Agx, 1, &data);
            assert!(decode(Profile::Agx, &p[..p.len() - 1]).is_err());
        }
    }
}
