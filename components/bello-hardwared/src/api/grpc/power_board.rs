use crate::hardware::power_board::{PowerTelemetry, protocol::Reading};
use bello_hardware_api as proto;

fn flags(bits: u32, names: &[&str]) -> proto::PowerFlags {
    proto::PowerFlags {
        bits,
        active_names: (0..32)
            .filter(|bit| bits & (1 << bit) != 0)
            .map(|bit| {
                names
                    .get(bit)
                    .map(|name| (*name).to_owned())
                    .unwrap_or_else(|| format!("UNKNOWN_BIT_{bit}"))
            })
            .collect(),
    }
}

const INFO: &[&str] = &[
    "EMERGENCY_EFFECTIVE",
    "EMERGENCY_HARDWARE",
    "EMERGENCY_SOFTWARE",
    "CHG_BQ_CONNECTED",
    "CHG_BYPASS_CONNECTED",
    "BATTERY_CHARGING",
];
const WARNING: &[&str] = &[
    "OVER_CURRENT_DISCHARGE_VBAT",
    "OVER_CURRENT_CHARGE_VBAT",
    "OVER_VOLTAGE_VBAT",
    "UNDER_VOLTAGE_VBAT",
    "OVER_CURRENT_VMOT",
    "OVER_VOLTAGE_VMOT",
    "OVER_TEMPERATURE_PCB",
    "UNDER_TEMPERATURE_PCB",
    "OVER_CURRENT_20V0",
    "OVER_CURRENT_5V0",
    "OVER_CURRENT_3V3",
];
const CRITICAL: &[&str] = &[
    "OVER_CURRENT_DISCHARGE_VBAT",
    "OVER_CURRENT_CHARGE_VBAT",
    "OVER_VOLTAGE_VBAT",
    "UNDER_VOLTAGE_VBAT",
    "OVER_CURRENT_VMOT",
    "OVER_VOLTAGE_VMOT",
    "OVER_TEMPERATURE_PCB",
    "UNDER_TEMPERATURE_PCB",
    "OVER_CURRENT_20V0",
    "OVER_CURRENT_5V0",
    "OVER_CURRENT_3V3",
    "POWER_FAULT_VMOT",
    "POWER_FAULT_3V3",
];

pub(super) fn convert(value: PowerTelemetry) -> proto::PowerBoardTelemetry {
    let rail = |v: f32, i: f32| proto::PowerVoltageCurrent {
        voltage_v: v.into(),
        current_a: i.into(),
    };
    let samples = value
        .samples
        .into_iter()
        .map(|sample| {
            use proto::power_board_sample::Reading as R;
            let reading = match sample.reading {
                Reading::Battery {
                    voltage,
                    current,
                    soc_percent,
                } => R::Battery(proto::PowerBattery {
                    voltage_v: voltage.into(),
                    current_a: current.into(),
                    soc_percent: soc_percent.map(f64::from),
                }),
                Reading::Gpi { bits, second_bits } => R::Gpi(proto::PowerGpi { bits, second_bits }),
                Reading::Iwc {
                    info,
                    warning,
                    critical,
                } => R::Iwc(proto::PowerIwc {
                    info: Some(flags(info, INFO)),
                    warning: Some(flags(warning, WARNING)),
                    critical: Some(flags(critical, CRITICAL)),
                }),
                Reading::LegacyFault { alarm, fault } => R::LegacyFault(proto::PowerLegacyFault {
                    alarm: Some(flags(alarm, &["EMERGENCY", "CHG_CONNECTED"])),
                    fault: Some(flags(
                        fault,
                        &[
                            "VBAT_OCP",
                            "VBAT_OVP",
                            "VBAT_UVLO",
                            "VMOT_OCP",
                            "VMOT_OVP",
                            "VMOT_UVLO",
                            "5V0_EXT",
                        ],
                    )),
                }),
                Reading::Environment {
                    temperature_c,
                    pressure_pa,
                } => R::Environment(proto::PowerEnvironment {
                    temperature_c: temperature_c.into(),
                    pressure_pa: pressure_pa.into(),
                }),
                Reading::LocalRails { values: v } => R::LocalRails(proto::PowerLocalRails {
                    rail_20v0: Some(rail(v[0], v[1])),
                    rail_5v0: Some(rail(v[2], v[3])),
                    rail_3v3: Some(rail(v[4], v[5])),
                }),
                Reading::MotorRail { voltage, current } => R::MotorRail(rail(voltage, current)),
                Reading::Tick { count } => R::TickCount(count),
                Reading::Encoder {
                    left_rpm,
                    right_rpm,
                } => R::Encoder(proto::PowerEncoder {
                    left_rpm: left_rpm.into(),
                    right_rpm: right_rpm.into(),
                }),
            };
            let age = sample.received.elapsed();
            proto::PowerBoardSample {
                sequence: sample.sequence.into(),
                sample_age_ms: age.as_millis().min(u64::MAX as u128) as u64,
                stale: age >= value.stale_timeout,
                reading: Some(reading),
            }
        })
        .collect();
    proto::PowerBoardTelemetry {
        board_name: value.name.to_string(),
        connection_generation: value.generation,
        status: Some(super::conversion::device_status(&value.status)),
        samples,
        invalid_frames: value.invalid_frames,
        unknown_frames: value.unknown_frames,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn emergency_and_unknown_bits_are_not_silently_lost() {
        let decoded = flags(1 | (1 << 31), INFO);
        assert_eq!(
            decoded.active_names,
            ["EMERGENCY_EFFECTIVE", "UNKNOWN_BIT_31"]
        );
    }
    #[test]
    fn each_measurement_keeps_its_own_age() {
        use crate::hardware::power_board::Sample;
        use std::time::{Duration, Instant};
        let packet = convert(PowerTelemetry {
            name: "board".into(),
            generation: 1,
            status: crate::domain::DeviceStatus {
                state: crate::domain::DeviceState::Online,
                detail: String::new(),
                transition_count: 1,
            },
            samples: vec![
                Sample {
                    sequence: 1,
                    received: Instant::now() - Duration::from_secs(5),
                    reading: Reading::Tick { count: 1 },
                },
                Sample {
                    sequence: 2,
                    received: Instant::now(),
                    reading: Reading::MotorRail {
                        voltage: 48.0,
                        current: 0.0,
                    },
                },
            ],
            stale_timeout: Duration::from_secs(3),
            invalid_frames: 0,
            unknown_frames: 0,
        });
        assert!(packet.samples[0].stale);
        assert!(!packet.samples[1].stale);
    }
}
