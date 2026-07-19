use anyhow::Result;

use crate::config::{RerunConfig, RerunMode};
#[cfg(feature = "rerun")]
use crate::domain::TelemetryPayload;
use crate::telemetry::Telemetry;

#[derive(Clone, Debug)]
pub struct RerunSpec {
    mode: RerunMode,
    #[cfg(feature = "rerun")]
    path: String,
    #[cfg(feature = "rerun")]
    address: String,
}

impl TryFrom<&RerunConfig> for RerunSpec {
    type Error = anyhow::Error;

    fn try_from(config: &RerunConfig) -> Result<Self> {
        match config.mode {
            RerunMode::Save if config.path.is_empty() => {
                anyhow::bail!("rerun.path must be non-empty in save mode")
            }
            RerunMode::Connect if config.address.is_empty() => {
                anyhow::bail!("rerun.address must be non-empty in connect mode")
            }
            _ => {}
        }
        Ok(Self {
            mode: config.mode,
            #[cfg(feature = "rerun")]
            path: config.path.clone(),
            #[cfg(feature = "rerun")]
            address: config.address.clone(),
        })
    }
}

#[cfg(not(feature = "rerun"))]
pub fn start(config: &RerunSpec, _telemetry: Telemetry) -> Result<()> {
    if !matches!(config.mode, RerunMode::Off) {
        anyhow::bail!("Rerun mode requested but binary was built without --features rerun");
    }
    Ok(())
}

#[cfg(feature = "rerun")]
pub fn start(config: &RerunSpec, telemetry: Telemetry) -> Result<()> {
    let recording = match config.mode {
        RerunMode::Off => return Ok(()),
        RerunMode::Save => {
            rerun::RecordingStreamBuilder::new("bello_hardware").save(&config.path)?
        }
        RerunMode::Connect => rerun::RecordingStreamBuilder::new("bello_hardware")
            .connect_grpc_opts(format!("rerun+http://{}/proxy", config.address))?,
    };
    let mut rx = telemetry.subscribe();
    tokio::spawn(async move {
        loop {
            match rx.recv().await {
                Ok(event) => {
                    if let Err(error) = log_event(&recording, event.payload) {
                        tracing::warn!(%error, "Rerun logging failed");
                    }
                }
                Err(tokio::sync::broadcast::error::RecvError::Lagged(dropped)) => {
                    tracing::warn!(dropped, "Rerun telemetry subscriber lagged");
                }
                Err(tokio::sync::broadcast::error::RecvError::Closed) => break,
            }
        }
    });
    Ok(())
}

#[cfg(feature = "rerun")]
fn log_event(recording: &rerun::RecordingStream, payload: TelemetryPayload) -> Result<()> {
    let scalar = |path: String, value: f64| -> Result<()> {
        recording.log(path, &rerun::Scalars::single(value))?;
        Ok(())
    };
    match payload {
        TelemetryPayload::Motor(value) => {
            let path = format!("motors/{}/{}", value.group, value.motor);
            scalar(format!("{path}/position"), value.position)?;
            scalar(format!("{path}/velocity"), value.velocity)?;
            scalar(format!("{path}/torque"), value.torque)?;
            scalar(format!("{path}/temperature"), value.temperature)?;
            scalar(format!("{path}/fault_bits"), value.fault_bits as f64)?;
        }
        TelemetryPayload::Imu(value) => {
            let path = format!("imus/{}", value.imu);
            for (axis, value) in ["x", "y", "z", "w"].into_iter().zip(value.orientation_xyzw) {
                scalar(format!("{path}/orientation/{axis}"), value)?;
            }
            for (axis, value) in ["x", "y", "z"].into_iter().zip(value.angular_velocity_xyz) {
                scalar(format!("{path}/angular_velocity/{axis}"), value)?;
            }
            for (axis, value) in ["x", "y", "z"]
                .into_iter()
                .zip(value.linear_acceleration_xyz)
            {
                scalar(format!("{path}/linear_acceleration/{axis}"), value)?;
            }
        }
        TelemetryPayload::Group(value) => {
            scalar(
                format!("groups/{}/control_tick", value.group),
                value.control_tick as f64,
            )?;
            scalar(
                format!("groups/{}/jitter_us", value.group),
                value.jitter_us as f64,
            )?;
            scalar(
                format!("groups/{}/command_count", value.group),
                value.command_count as f64,
            )?;
        }
        TelemetryPayload::CommandPolicy(value) => {
            recording.log(
                format!("command_policy/{}/{}", value.group, value.motor),
                &rerun::TextLog::new(format!(
                    "{} {}: {}; modified={}, rejected={}, response={}",
                    value.severity,
                    value.check,
                    value.detail,
                    value.command_modified,
                    value.command_rejected,
                    value.response
                )),
            )?;
        }
        TelemetryPayload::Health(value) => {
            recording.log(
                format!("health/{}", value.component),
                &rerun::TextLog::new(format!("{}: {}", value.status, value.detail)),
            )?;
        }
    }
    Ok(())
}
