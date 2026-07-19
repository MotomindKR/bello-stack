use std::collections::HashSet;
use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::mpsc::{Receiver, SyncSender, TrySendError, sync_channel};
use std::thread::JoinHandle;
use std::time::Duration;

use anyhow::{Context, Result, anyhow};
use bello_hardware_api::WatchTelemetryResponse;
use rerun::{RecordingStream, RecordingStreamBuilder};
use tokio::sync::oneshot;

use crate::config::OutputConfig;
use crate::mapping::{self, LogEntry};
use crate::timeline::Timeline;

const APPLICATION_ID: &str = "bello_hardware";
const TIMELINE_NAME: &str = "time_since_start";
const CONNECTION_PATH: &str = "health/bello_rerun_connection";
const TIMELINE_PATH: &str = "health/bello_rerun_timeline";
const WRITER_PATH: &str = "health/bello_rerun_writer";
const QUEUE_CAPACITY: usize = 4_096;
const SHUTDOWN_TIMEOUT: Duration = Duration::from_secs(2);

enum WriterCommand {
    ConnectionOpened(Connection),
    ConnectionLost(String),
    Telemetry {
        connection: Connection,
        response: WatchTelemetryResponse,
    },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct Connection {
    generation: u64,
    restored: bool,
}

pub(crate) struct RerunWriter {
    sender: Option<SyncSender<WriterCommand>>,
    dropped: Arc<AtomicU64>,
    finished: Option<oneshot::Receiver<()>>,
    worker: Option<JoinHandle<()>>,
    next_connection_generation: AtomicU64,
}

impl RerunWriter {
    pub(crate) fn start(output: &OutputConfig) -> Result<Self> {
        let recording = open_recording(output)?;
        let (sender, receiver) = sync_channel(QUEUE_CAPACITY);
        let dropped = Arc::new(AtomicU64::new(0));
        let worker_dropped = Arc::clone(&dropped);
        let (finished_tx, finished) = oneshot::channel();
        let worker = std::thread::Builder::new()
            .name("bello-rerun-writer".to_owned())
            .spawn(move || {
                run_writer(recording, receiver, &worker_dropped);
                let _ = finished_tx.send(());
            })
            .context("spawning Rerun writer thread")?;

        Ok(Self {
            sender: Some(sender),
            dropped,
            finished: Some(finished),
            worker: Some(worker),
            next_connection_generation: AtomicU64::new(1),
        })
    }

    pub(crate) fn connection_opened(&self, restored: bool) -> Result<Connection> {
        let connection = Connection {
            generation: self
                .next_connection_generation
                .fetch_update(Ordering::Relaxed, Ordering::Relaxed, |generation| {
                    Some(generation.checked_add(1).unwrap_or(1))
                })
                .unwrap_or_else(|generation| generation),
            restored,
        };
        self.enqueue(WriterCommand::ConnectionOpened(connection))?;
        Ok(connection)
    }

    pub(crate) fn connection_lost(&self, detail: String) -> Result<()> {
        self.enqueue(WriterCommand::ConnectionLost(detail))
    }

    pub(crate) fn telemetry(
        &self,
        connection: Connection,
        response: WatchTelemetryResponse,
    ) -> Result<()> {
        self.enqueue(WriterCommand::Telemetry {
            connection,
            response,
        })
    }

    fn enqueue(&self, command: WriterCommand) -> Result<()> {
        let sender = self
            .sender
            .as_ref()
            .ok_or_else(|| anyhow!("Rerun writer is shutting down"))?;
        match sender.try_send(command) {
            Ok(()) => Ok(()),
            Err(TrySendError::Full(_)) => {
                let already_dropped = self.dropped.fetch_add(1, Ordering::Relaxed);
                if already_dropped == 0 {
                    tracing::warn!(
                        capacity = QUEUE_CAPACITY,
                        "Rerun writer queue full; dropping observer events"
                    );
                }
                Ok(())
            }
            Err(TrySendError::Disconnected(_)) => Err(anyhow!("Rerun writer stopped unexpectedly")),
        }
    }

    pub(crate) async fn shutdown(mut self) {
        self.sender.take();
        let Some(finished) = self.finished.take() else {
            return;
        };

        match tokio::time::timeout(SHUTDOWN_TIMEOUT, finished).await {
            Ok(Ok(())) => self.join_finished_worker(),
            Ok(Err(_)) => {
                tracing::warn!("Rerun writer stopped without completing shutdown");
                self.join_finished_worker();
            }
            Err(_) => {
                tracing::warn!(
                    ?SHUTDOWN_TIMEOUT,
                    "Rerun writer shutdown deadline expired; detaching writer thread"
                );
            }
        }
    }

    fn join_finished_worker(&mut self) {
        if let Some(worker) = self.worker.take()
            && worker.join().is_err()
        {
            tracing::warn!("Rerun writer thread panicked");
        }
    }
}

fn open_recording(output: &OutputConfig) -> Result<RecordingStream> {
    match output {
        OutputConfig::Save { path } => RecordingStreamBuilder::new(APPLICATION_ID)
            .save(path)
            .with_context(|| format!("opening Rerun recording {path}")),
        OutputConfig::Connect { address } => RecordingStreamBuilder::new(APPLICATION_ID)
            .connect_grpc_opts(format!("rerun+http://{address}/proxy"))
            .with_context(|| format!("connecting Rerun output to {address}")),
    }
}

fn run_writer(recording: RecordingStream, receiver: Receiver<WriterCommand>, dropped: &AtomicU64) {
    let mut timeline = Timeline::default();
    let mut configured_series = HashSet::new();
    let mut active_connection = None;

    while let Ok(command) = receiver.recv() {
        report_dropped_events(&recording, &mut timeline, dropped);
        match command {
            WriterCommand::ConnectionOpened(connection) => enter_connection(
                &recording,
                &mut timeline,
                &mut active_connection,
                connection,
            ),
            WriterCommand::ConnectionLost(detail) => log_marker(
                &recording,
                &mut timeline,
                CONNECTION_PATH,
                format!("connection lost: {detail}"),
            ),
            WriterCommand::Telemetry {
                connection,
                response,
            } => {
                enter_connection(
                    &recording,
                    &mut timeline,
                    &mut active_connection,
                    connection,
                );
                log_response(&recording, &mut configured_series, &mut timeline, response);
            }
        }
    }

    report_dropped_events(&recording, &mut timeline, dropped);
    tracing::info!("flushing Rerun recording");
    if let Err(error) = recording.flush_with_timeout(SHUTDOWN_TIMEOUT) {
        tracing::warn!(%error, "failed to flush Rerun recording during shutdown");
    }
}

fn enter_connection(
    recording: &RecordingStream,
    timeline: &mut Timeline,
    active: &mut Option<u64>,
    connection: Connection,
) {
    if *active == Some(connection.generation) {
        return;
    }
    *active = Some(connection.generation);
    timeline.begin_connection();
    log_marker(
        recording,
        timeline,
        CONNECTION_PATH,
        if connection.restored {
            "connection restored"
        } else {
            "connected"
        },
    );
}

fn report_dropped_events(
    recording: &RecordingStream,
    timeline: &mut Timeline,
    dropped: &AtomicU64,
) {
    let dropped = dropped.swap(0, Ordering::Relaxed);
    if dropped == 0 {
        return;
    }
    tracing::warn!(
        dropped,
        "Rerun writer resumed after dropping observer events"
    );
    log_marker(
        recording,
        timeline,
        WRITER_PATH,
        format!("dropped {dropped} observer events while the writer was backpressured"),
    );
}

fn log_response(
    recording: &RecordingStream,
    configured_series: &mut HashSet<String>,
    timeline: &mut Timeline,
    response: WatchTelemetryResponse,
) {
    let mapped = mapping::map(response);
    let (timestamp_ns, reset) = timeline.map(mapped.source_timestamp_ns);
    recording.set_time(TIMELINE_NAME, Duration::from_nanos(timestamp_ns));

    if let Some(reset) = reset {
        let detail = format!(
            "source timestamp reset from {} ns to {} ns; rebased timeline",
            reset.previous_source_ns, reset.new_source_ns
        );
        log_text(recording, TIMELINE_PATH, detail);
    }
    for warning in mapped.warnings {
        tracing::warn!(%warning, "skipping malformed telemetry field");
    }
    for entry in mapped.entries {
        if let Err(error) = log_entry(recording, configured_series, entry) {
            tracing::warn!(%error, "Rerun telemetry logging failed");
        }
    }
}

fn log_marker(
    recording: &RecordingStream,
    timeline: &mut Timeline,
    path: &str,
    text: impl Into<String>,
) {
    recording.set_time(TIMELINE_NAME, Duration::from_nanos(timeline.marker_time()));
    log_text(recording, path, text);
}

fn log_entry(
    recording: &RecordingStream,
    configured_series: &mut HashSet<String>,
    entry: LogEntry,
) -> Result<()> {
    match entry {
        LogEntry::Scalars {
            path,
            names,
            values,
        } => {
            if !configured_series.contains(&path) {
                recording
                    .log_static(path.as_str(), &rerun::SeriesLines::new().with_names(names))?;
                configured_series.insert(path.clone());
            }
            recording.log(path, &rerun::Scalars::new(values))?;
        }
        LogEntry::Text { path, text } => log_text(recording, path, text),
    }
    Ok(())
}

fn log_text(recording: &RecordingStream, path: impl Into<String>, text: impl Into<String>) {
    if let Err(error) = recording.log(path.into(), &rerun::TextLog::new(text.into())) {
        tracing::warn!(%error, "Rerun marker logging failed");
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn full_queue_preserves_connection_identity_on_surviving_telemetry() {
        let (sender, receiver) = sync_channel(1);
        let writer = RerunWriter {
            sender: Some(sender),
            dropped: Arc::new(AtomicU64::new(0)),
            finished: None,
            worker: None,
            next_connection_generation: AtomicU64::new(1),
        };

        let first = writer.connection_opened(false).unwrap();
        let second = writer.connection_opened(true).unwrap();
        assert_eq!(writer.dropped.load(Ordering::Relaxed), 1);

        assert!(
            matches!(receiver.recv().unwrap(), WriterCommand::ConnectionOpened(value) if value == first)
        );
        writer
            .telemetry(second, WatchTelemetryResponse::default())
            .unwrap();
        assert!(matches!(
            receiver.recv().unwrap(),
            WriterCommand::Telemetry { connection, .. } if connection == second
        ));
    }
}
