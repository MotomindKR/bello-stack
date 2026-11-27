use crate::{DeviceStatus, Error};
use bello_hardware_api as proto;
pub use proto::power_board_sample::Reading as PowerReading;
use std::collections::HashSet;
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
            PowerReading::Battery(proto::PowerBattery {
                soc_percent: None,
                ..
            })
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
        let rail =
            |v: &proto::PowerVoltageCurrent| v.voltage_v.is_finite() && v.current_a.is_finite();
        let flags = |v: &proto::PowerFlags| v.active_names.len() <= 32;
        let mut seen = HashSet::new();
        let mut samples = Vec::new();
        for sample in value.samples {
            let reading = sample.reading.ok_or_else(invalid)?;
            let valid = match &reading {
                PowerReading::Battery(v) => {
                    v.voltage_v.is_finite()
                        && v.current_a.is_finite()
                        && v.soc_percent.is_none_or(|x| (0.0..=100.0).contains(&x))
                }
                PowerReading::Gpi(_) | PowerReading::TickCount(_) => true,
                PowerReading::Iwc(v) => {
                    v.info.as_ref().is_some_and(flags)
                        && v.warning.as_ref().is_some_and(flags)
                        && v.critical.as_ref().is_some_and(flags)
                }
                PowerReading::LegacyFault(v) => {
                    v.alarm.as_ref().is_some_and(flags) && v.fault.as_ref().is_some_and(flags)
                }
                PowerReading::Environment(v) => {
                    v.temperature_c.is_finite() && v.pressure_pa.is_finite()
                }
                PowerReading::LocalRails(v) => {
                    v.rail_20v0.as_ref().is_some_and(rail)
                        && v.rail_5v0.as_ref().is_some_and(rail)
                        && v.rail_3v3.as_ref().is_some_and(rail)
                }
                PowerReading::MotorRail(v) => rail(v),
                PowerReading::Encoder(v) => v.left_rpm.is_finite() && v.right_rpm.is_finite(),
            };
            if !valid || !seen.insert(std::mem::discriminant(&reading)) {
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
