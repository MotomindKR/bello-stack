use std::time::Instant;

use anyhow::{Result, bail};

use super::{ImuSample, ImuSpec};

const G_TO_METERS_PER_SECOND_SQUARED: f64 = 9.80665;

pub(super) fn output_commands(config: &ImuSpec) -> Vec<String> {
    let interval = (1000.0 / config.output_rate_hz.max(1) as f64)
        .round()
        .clamp(1.0, 1000.0) as u32;
    vec![
        "<soc1>".into(),
        format!("<sof{}>", if config.quaternion { 2 } else { 1 }),
        format!("<sog{}>", u8::from(config.gyro)),
        format!("<soa{}>", if config.acceleration { 2 } else { 0 }),
        "<som0>".into(),
        "<sot0>".into(),
        format!("<sots{}>", u8::from(config.device_timestamp)),
        format!("<sor{interval}>"),
    ]
}

pub(super) fn parse_ascii_line(line: &str, config: &ImuSpec) -> Result<ImuSample> {
    let line = line.trim();
    if !line.starts_with('*') {
        bail!("not an EBIMU data line");
    }
    let values: Vec<f64> = line[1..]
        .split(',')
        .map(|field| field.trim().parse::<f64>().map_err(anyhow::Error::from))
        .collect::<Result<_>>()?;
    let mut cursor = 0;
    let mut take = |count: usize| -> Result<&[f64]> {
        if cursor + count > values.len() {
            bail!("missing fields");
        }
        let result = &values[cursor..cursor + count];
        cursor += count;
        Ok(result)
    };
    let orientation = if config.quaternion {
        let q = take(4)?;
        normalize([q[2], -q[1], -q[0], q[3]])?
    } else {
        let e = take(3)?;
        euler_xyz_to_quaternion(e[0].to_radians(), -e[1].to_radians(), -e[2].to_radians())
    };
    let gyro = if config.gyro {
        let g = take(3)?;
        [g[0].to_radians(), g[1].to_radians(), g[2].to_radians()]
    } else {
        [0.0; 3]
    };
    let acceleration = if config.acceleration {
        let a = take(3)?;
        [
            a[0] * G_TO_METERS_PER_SECOND_SQUARED,
            a[1] * G_TO_METERS_PER_SECOND_SQUARED,
            a[2] * G_TO_METERS_PER_SECOND_SQUARED,
        ]
    } else {
        [0.0; 3]
    };
    let timestamp = if config.device_timestamp {
        take(1)?[0].round().max(0.0) as u64
    } else {
        0
    };
    if cursor != values.len() {
        bail!("expected {cursor} fields, got {}", values.len());
    }
    Ok(ImuSample {
        orientation_xyzw: orientation,
        angular_velocity_xyz: gyro,
        linear_acceleration_xyz: acceleration,
        device_timestamp_ms: timestamp,
        received: Instant::now(),
    })
}

fn normalize(q: [f64; 4]) -> Result<[f64; 4]> {
    let norm = q.iter().map(|v| v * v).sum::<f64>().sqrt();
    if norm == 0.0 {
        bail!("zero quaternion");
    }
    Ok(q.map(|v| v / norm))
}

fn euler_xyz_to_quaternion(roll: f64, pitch: f64, yaw: f64) -> [f64; 4] {
    let (sr, cr) = (roll * 0.5).sin_cos();
    let (sp, cp) = (pitch * 0.5).sin_cos();
    let (sy, cy) = (yaw * 0.5).sin_cos();
    normalize([
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ])
    .expect("Euler quaternion cannot be zero")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::ImuConfig;

    #[test]
    fn parses_quaternion_axis_conversion_and_units() {
        let raw = ImuConfig {
            configure_on_start: false,
            ..Default::default()
        };
        let config = ImuSpec::try_from(&raw).unwrap();
        let sample = parse_ascii_line("*0,0,0,1,180,0,-180,1,0,-1,123", &config).unwrap();
        assert!((sample.orientation_xyzw[3] - 1.0).abs() < 1e-12);
        assert!((sample.angular_velocity_xyz[0] - std::f64::consts::PI).abs() < 1e-12);
        assert!((sample.linear_acceleration_xyz[2] + G_TO_METERS_PER_SECOND_SQUARED).abs() < 1e-12);
        assert_eq!(sample.device_timestamp_ms, 123);
    }
}
