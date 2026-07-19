use std::time::Instant;

use anyhow::{Result, bail};

use super::{ImuSample, ImuSpec};

const G_TO_METERS_PER_SECOND_SQUARED: f64 = 9.80665;

pub(super) fn output_commands(config: &ImuSpec) -> Vec<String> {
    let interval = config.output_interval.as_millis();
    vec![
        "<soc1>".into(),
        "<sof2>".into(),
        "<sog1>".into(),
        "<soa2>".into(),
        "<som0>".into(),
        "<sot0>".into(),
        "<sots1>".into(),
        format!("<sor{interval}>"),
    ]
}

pub(super) fn parse_ascii_line(line: &str) -> Result<ImuSample> {
    let line = line.trim();
    if !line.starts_with('*') {
        bail!("not an EBIMU data line");
    }
    let values: Vec<f64> = line[1..]
        .split(',')
        .map(|field| field.trim().parse::<f64>().map_err(anyhow::Error::from))
        .collect::<Result<_>>()?;
    if values.iter().any(|value| !value.is_finite()) {
        bail!("sample contains a non-finite field");
    }
    let mut cursor = 0;
    let mut take = |count: usize| -> Result<&[f64]> {
        if cursor + count > values.len() {
            bail!("missing fields");
        }
        let result = &values[cursor..cursor + count];
        cursor += count;
        Ok(result)
    };
    let q = take(4)?;
    let orientation = normalize([q[2], -q[1], -q[0], q[3]])?;
    let g = take(3)?;
    let gyro = [g[0].to_radians(), g[1].to_radians(), g[2].to_radians()];
    let a = take(3)?;
    let acceleration = [
        a[0] * G_TO_METERS_PER_SECOND_SQUARED,
        a[1] * G_TO_METERS_PER_SECOND_SQUARED,
        a[2] * G_TO_METERS_PER_SECOND_SQUARED,
    ];
    let timestamp = take(1)?[0].round().max(0.0) as u64;
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
    let norm = q.iter().fold(0.0_f64, |norm, value| norm.hypot(*value));
    if !norm.is_finite() || norm <= 1e-12 {
        bail!("quaternion norm is non-finite or too small");
    }
    Ok(q.map(|v| v / norm))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_quaternion_axis_conversion_and_units() {
        let sample = parse_ascii_line("*0,0,0,1,180,0,-180,1,0,-1,123").unwrap();
        assert!((sample.orientation_xyzw[3] - 1.0).abs() < 1e-12);
        assert!((sample.angular_velocity_xyz[0] - std::f64::consts::PI).abs() < 1e-12);
        assert!((sample.linear_acceleration_xyz[2] + G_TO_METERS_PER_SECOND_SQUARED).abs() < 1e-12);
        assert_eq!(sample.device_timestamp_ms, 123);
    }

    #[test]
    fn rejects_non_finite_sensor_fields() {
        assert!(parse_ascii_line("*NaN,0,0,1,0,0,0,0,0,0,1").is_err());
        assert!(parse_ascii_line("*0,0,0,1,inf,0,0,0,0,0,1").is_err());
    }

    #[test]
    fn rejects_near_zero_quaternion() {
        assert!(parse_ascii_line("*1e-20,0,0,0,0,0,0,0,0,0,1").is_err());
    }
}
