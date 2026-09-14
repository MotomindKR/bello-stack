use crate::{DeviceStatus, Error};
use bello_hardware_api as proto;
use std::collections::BTreeSet;
use std::time::{Duration, Instant};

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn freshness_optional_soc_and_malformed_samples() {
        let board = PowerBoard {
            name: "power".into(),
            serial_device: "uart".into(),
            profile: "agx".into(),
            stale_timeout: Duration::from_secs(3),
            sync_period: Duration::from_secs(1),
            initial_status: None,
        };
        let sample = proto::PowerBoardSample {
            sequence: 255,
            sample_age_ms: 4000,
            stale: false,
            reading: Some(proto::power_board_sample::Reading::Battery(
                proto::PowerBattery {
                    voltage_v: 48.,
                    current_a: -1.,
                    soc_percent: None,
                },
            )),
        };
        let mut value = proto::PowerBoardTelemetry {
            board_name: board.name.clone(),
            samples: vec![sample.clone()],
            ..Default::default()
        };
        let snapshot = PowerBoardSnapshot::decode(value.clone(), &board, 1).unwrap();
        assert!(snapshot.is_stale(&snapshot.samples[0]));
        assert!(matches!(
            snapshot.samples[0].reading,
            PowerReading::Battery {
                soc_percent: None,
                ..
            }
        ));
        value.samples.push(sample);
        assert!(PowerBoardSnapshot::decode(value.clone(), &board, 1).is_err());
        value.samples.pop();
        value.samples[0].sequence = 256;
        assert!(PowerBoardSnapshot::decode(value, &board, 1).is_err());
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct PowerBoard {
    pub name: String,
    pub serial_device: String,
    pub profile: String,
    pub stale_timeout: Duration,
    pub sync_period: Duration,
    pub initial_status: Option<DeviceStatus>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct PowerFlags {
    pub bits: u32,
    pub active_names: Vec<String>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct VoltageCurrent {
    pub voltage_v: f64,
    pub current_a: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub enum PowerReading {
    Battery {
        voltage_v: f64,
        current_a: f64,
        soc_percent: Option<f64>,
    },
    Gpi {
        bits: u32,
        second_bits: Option<u32>,
    },
    Iwc {
        info: PowerFlags,
        warning: PowerFlags,
        critical: PowerFlags,
    },
    LegacyFault {
        alarm: PowerFlags,
        fault: PowerFlags,
    },
    Environment {
        temperature_c: f64,
        pressure_pa: f64,
    },
    LocalRails {
        rail_20v0: VoltageCurrent,
        rail_5v0: VoltageCurrent,
        rail_3v3: VoltageCurrent,
    },
    MotorRail(VoltageCurrent),
    TickCount(u32),
    Encoder {
        left_rpm: f64,
        right_rpm: f64,
    },
}

impl PowerReading {
    pub fn name(&self) -> &'static str {
        match self {
            Self::Battery { .. } => "battery",
            Self::Gpi { .. } => "gpi",
            Self::Iwc { .. } => "iwc",
            Self::LegacyFault { .. } => "legacy_fault",
            Self::Environment { .. } => "environment",
            Self::LocalRails { .. } => "local_rails",
            Self::MotorRail(_) => "motor_rail",
            Self::TickCount(_) => "tick_count",
            Self::Encoder { .. } => "encoder",
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct PowerSample {
    pub sequence: u8,
    pub age_at_receipt: Duration,
    pub stale_at_receipt: bool,
    pub reading: PowerReading,
}

#[derive(Clone, Debug, PartialEq)]
pub struct PowerBoardSnapshot {
    pub monotonic_ns: u64,
    pub board: String,
    pub connection_generation: u64,
    pub status: Option<DeviceStatus>,
    pub samples: Vec<PowerSample>,
    pub invalid_frames: u64,
    pub unknown_frames: u64,
    pub stale_timeout: Duration,
    received: Instant,
}

impl PowerBoardSnapshot {
    pub fn age(&self, sample: &PowerSample) -> Duration {
        sample
            .age_at_receipt
            .saturating_add(self.received.elapsed())
    }
    pub fn is_stale(&self, sample: &PowerSample) -> bool {
        sample.stale_at_receipt || self.age(sample) >= self.stale_timeout
    }

    pub(crate) fn decode(
        value: proto::PowerBoardTelemetry,
        board: &PowerBoard,
        monotonic_ns: u64,
    ) -> Result<Self, Error> {
        let invalid = || Error::Protocol("invalid power board telemetry".into());
        let rail = |v: Option<proto::PowerVoltageCurrent>| -> Result<VoltageCurrent, Error> {
            let v = v.ok_or_else(invalid)?;
            if !v.voltage_v.is_finite() || !v.current_a.is_finite() {
                return Err(invalid());
            }
            Ok(VoltageCurrent {
                voltage_v: v.voltage_v,
                current_a: v.current_a,
            })
        };
        let flags = |v: Option<proto::PowerFlags>| -> Result<PowerFlags, Error> {
            let v = v.ok_or_else(invalid)?;
            if v.active_names.len() > 32 {
                return Err(invalid());
            }
            Ok(PowerFlags {
                bits: v.bits,
                active_names: v.active_names,
            })
        };
        let mut seen = BTreeSet::new();
        let mut samples = Vec::new();
        for sample in value.samples {
            use proto::power_board_sample::Reading as R;
            let reading = match sample.reading.ok_or_else(invalid)? {
                R::Battery(v) => {
                    if !v.voltage_v.is_finite()
                        || !v.current_a.is_finite()
                        || v.soc_percent
                            .is_some_and(|x| !x.is_finite() || !(0.0..=100.0).contains(&x))
                    {
                        return Err(invalid());
                    }
                    PowerReading::Battery {
                        voltage_v: v.voltage_v,
                        current_a: v.current_a,
                        soc_percent: v.soc_percent,
                    }
                }
                R::Gpi(v) => PowerReading::Gpi {
                    bits: v.bits,
                    second_bits: v.second_bits,
                },
                R::Iwc(v) => PowerReading::Iwc {
                    info: flags(v.info)?,
                    warning: flags(v.warning)?,
                    critical: flags(v.critical)?,
                },
                R::LegacyFault(v) => PowerReading::LegacyFault {
                    alarm: flags(v.alarm)?,
                    fault: flags(v.fault)?,
                },
                R::Environment(v) => {
                    if !v.temperature_c.is_finite() || !v.pressure_pa.is_finite() {
                        return Err(invalid());
                    }
                    PowerReading::Environment {
                        temperature_c: v.temperature_c,
                        pressure_pa: v.pressure_pa,
                    }
                }
                R::LocalRails(v) => PowerReading::LocalRails {
                    rail_20v0: rail(v.rail_20v0)?,
                    rail_5v0: rail(v.rail_5v0)?,
                    rail_3v3: rail(v.rail_3v3)?,
                },
                R::MotorRail(v) => PowerReading::MotorRail(rail(Some(v))?),
                R::TickCount(v) => PowerReading::TickCount(v),
                R::Encoder(v) => {
                    if !v.left_rpm.is_finite() || !v.right_rpm.is_finite() {
                        return Err(invalid());
                    }
                    PowerReading::Encoder {
                        left_rpm: v.left_rpm,
                        right_rpm: v.right_rpm,
                    }
                }
            };
            if !seen.insert(reading.name()) {
                return Err(invalid());
            }
            samples.push(PowerSample {
                sequence: sample.sequence.try_into().map_err(|_| invalid())?,
                age_at_receipt: Duration::from_millis(sample.sample_age_ms),
                stale_at_receipt: sample.stale,
                reading,
            });
        }
        Ok(Self {
            monotonic_ns,
            board: value.board_name,
            connection_generation: value.connection_generation,
            status: value.status.map(Into::into),
            samples,
            invalid_frames: value.invalid_frames,
            unknown_frames: value.unknown_frames,
            stale_timeout: board.stale_timeout,
            received: Instant::now(),
        })
    }
}
