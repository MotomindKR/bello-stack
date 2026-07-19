use anyhow::Result;
#[cfg(feature = "rerun")]
use std::collections::HashSet;
use std::time::Duration;

use crate::config::RerunConfig;
#[cfg(feature = "rerun")]
use crate::domain::{TelemetryEvent, TelemetryPayload};
use crate::telemetry::Telemetry;
use tokio::sync::watch;

#[derive(Clone, Debug)]
#[cfg_attr(not(feature = "rerun"), allow(dead_code))]
pub enum RerunSpec {
    Off,
    Save(String),
    Connect(String),
}

impl TryFrom<&RerunConfig> for RerunSpec {
    type Error = anyhow::Error;

    fn try_from(config: &RerunConfig) -> Result<Self> {
        match config {
            RerunConfig::Save { path } if path.trim().is_empty() => {
                anyhow::bail!("rerun.path must be non-empty in save mode")
            }
            RerunConfig::Connect { address } if address.trim().is_empty() => {
                anyhow::bail!("rerun.address must be non-empty in connect mode")
            }
            RerunConfig::Connect { address } if address.chars().any(char::is_whitespace) => {
                anyhow::bail!("rerun.address must not contain whitespace")
            }
            _ => {}
        }
        #[cfg(not(feature = "rerun"))]
        if !matches!(config, RerunConfig::Off) {
            anyhow::bail!("Rerun mode requested but binary was built without --features rerun");
        }
        Ok(match config {
            RerunConfig::Off => Self::Off,
            RerunConfig::Save { path } => Self::Save(path.clone()),
            RerunConfig::Connect { address } => Self::Connect(address.clone()),
        })
    }
}

#[cfg(not(feature = "rerun"))]
pub fn start(
    config: &RerunSpec,
    _telemetry: Telemetry,
    _shutdown: watch::Receiver<bool>,
) -> Result<RerunWorker> {
    if !matches!(config, RerunSpec::Off) {
        anyhow::bail!("Rerun mode requested but binary was built without --features rerun");
    }
    Ok(RerunWorker { join: None })
}

#[cfg(feature = "rerun")]
pub fn start(
    config: &RerunSpec,
    telemetry: Telemetry,
    mut shutdown: watch::Receiver<bool>,
) -> Result<RerunWorker> {
    let recording = match config {
        RerunSpec::Off => return Ok(RerunWorker { join: None }),
        RerunSpec::Save(path) => rerun::RecordingStreamBuilder::new("bello_hardware").save(path)?,
        RerunSpec::Connect(address) => rerun::RecordingStreamBuilder::new("bello_hardware")
            .connect_grpc_opts(format!("rerun+http://{address}/proxy"))?,
    };
    let mut rx = telemetry.subscribe();
    let join = tokio::spawn(async move {
        let mut configured_series = HashSet::new();
        loop {
            let event = tokio::select! {
                biased;
                changed = shutdown.changed() => {
                    if changed.is_err() || *shutdown.borrow() {
                        drain_pending(&recording, &mut configured_series, &mut rx);
                        break;
                    }
                    continue;
                }
                event = rx.recv() => event,
            };
            match event {
                Ok(event) => {
                    if let Err(error) = log_event(&recording, &mut configured_series, event) {
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
    Ok(RerunWorker { join: Some(join) })
}

#[cfg(feature = "rerun")]
fn drain_pending(
    recording: &rerun::RecordingStream,
    configured_series: &mut HashSet<String>,
    source: &mut tokio::sync::broadcast::Receiver<TelemetryEvent>,
) {
    loop {
        match source.try_recv() {
            Ok(event) => {
                if let Err(error) = log_event(recording, configured_series, event) {
                    tracing::warn!(%error, "Rerun logging failed during shutdown");
                }
            }
            Err(tokio::sync::broadcast::error::TryRecvError::Lagged(dropped)) => {
                tracing::warn!(dropped, "Rerun telemetry subscriber lagged during shutdown");
            }
            Err(
                tokio::sync::broadcast::error::TryRecvError::Empty
                | tokio::sync::broadcast::error::TryRecvError::Closed,
            ) => break,
        }
    }
}

pub struct RerunWorker {
    join: Option<tokio::task::JoinHandle<()>>,
}

impl RerunWorker {
    pub async fn join(mut self) {
        if let Some(mut join) = self.join.take() {
            match tokio::time::timeout(Duration::from_secs(2), &mut join).await {
                Ok(Ok(())) => {}
                Ok(Err(error)) => tracing::warn!(%error, "Rerun task failed during shutdown"),
                Err(_) => {
                    tracing::warn!("Rerun task did not stop within 2 seconds; aborting it");
                    join.abort();
                    let _ = join.await;
                }
            }
        }
    }
}

#[cfg(feature = "rerun")]
fn log_event(
    recording: &rerun::RecordingStream,
    configured_series: &mut HashSet<String>,
    event: TelemetryEvent,
) -> Result<()> {
    recording.set_time("time_since_start", Duration::from_nanos(event.monotonic_ns));
    match event.payload {
        TelemetryPayload::Motor(value) => {
            let path = format!("motors/{}/{}", value.group, value.motor);
            log_scalars(
                recording,
                configured_series,
                path,
                [
                    "position",
                    "velocity",
                    "torque",
                    "temperature",
                    "fault_bits",
                ],
                [
                    value.position,
                    value.velocity,
                    value.torque,
                    value.temperature,
                    f64::from(value.fault_bits),
                ],
            )?;
        }
        TelemetryPayload::Imu(value) => {
            let path = format!("imus/{}", value.imu);
            log_scalars(
                recording,
                configured_series,
                format!("{path}/orientation"),
                ["x", "y", "z", "w"],
                value.orientation_xyzw,
            )?;
            log_scalars(
                recording,
                configured_series,
                format!("{path}/angular_velocity"),
                ["x", "y", "z"],
                value.angular_velocity_xyz,
            )?;
            log_scalars(
                recording,
                configured_series,
                format!("{path}/linear_acceleration"),
                ["x", "y", "z"],
                value.linear_acceleration_xyz,
            )?;
        }
        TelemetryPayload::Group(value) => {
            log_scalars(
                recording,
                configured_series,
                format!("groups/{}", value.group),
                ["control_tick", "jitter_us", "command_count"],
                [
                    value.control_tick as f64,
                    value.jitter_us as f64,
                    value.command_count as f64,
                ],
            )?;
        }
        TelemetryPayload::CommandPolicy(value) => {
            recording.log(
                format!("command_policy/{}/{}", value.group, value.motor),
                &rerun::TextLog::new(format!(
                    "{:?} {:?}: {}; modified={}, rejected={}, response={:?}",
                    value.behavior,
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
                &rerun::TextLog::new(format!("{}: {}", value.status.state, value.status.detail)),
            )?;
        }
    }
    Ok(())
}

#[cfg(feature = "rerun")]
fn log_scalars<const N: usize>(
    recording: &rerun::RecordingStream,
    configured_series: &mut HashSet<String>,
    path: String,
    names: [&str; N],
    values: [f64; N],
) -> Result<()> {
    if configured_series.insert(path.clone()) {
        recording.log_static(path.as_str(), &rerun::SeriesLines::new().with_names(names))?;
    }
    recording.log(path, &rerun::Scalars::new(values))?;
    Ok(())
}
