use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::{Duration, Instant};

use anyhow::{Result, bail};
use parking_lot::RwLock;
use tracing::{info, warn};

use crate::config::ImuFilterConfig;
use crate::domain::{ImuName, ImuTelemetry, TelemetryPayload};
use crate::hardware::lifecycle::{DeviceLifecycle, DeviceState};
use crate::hardware::serial::{SerialChunk, SerialHandle};
use crate::telemetry::Telemetry;

mod ebimu;
mod filter;
mod spec;

use ebimu::{output_commands, parse_ascii_line};
use filter::ImuLowPass;
pub(crate) use spec::ImuSpec;
use spec::POLL_PERIOD;

// EBIMU setting commands may persist configuration before acknowledging. Real
// hardware takes roughly 650 ms, independently of the serial I/O timeout.
const COMMAND_ACK_TIMEOUT: Duration = Duration::from_secs(2);

#[derive(Clone, Debug)]
pub struct ImuSample {
    pub orientation_xyzw: [f64; 4],
    pub angular_velocity_xyz: [f64; 3],
    pub linear_acceleration_xyz: [f64; 3],
    pub device_timestamp_ms: u64,
    pub received: Instant,
}

#[derive(Clone)]
pub struct ImuHandle {
    pub name: ImuName,
    pub lifecycle: DeviceLifecycle,
    latest: Arc<RwLock<Option<ImuSample>>>,
}

impl ImuHandle {
    pub(crate) fn latest(&self) -> Option<ImuSample> {
        self.latest.read().clone()
    }
}

pub(crate) struct ImuWorker {
    pub(crate) handle: ImuHandle,
    stop: Arc<AtomicBool>,
    thread: Option<thread::JoinHandle<()>>,
}

impl ImuWorker {
    pub(crate) fn start(
        config: ImuSpec,
        filter_config: ImuFilterConfig,
        serial: SerialHandle,
        telemetry: Telemetry,
    ) -> Result<Self> {
        let stop = Arc::new(AtomicBool::new(false));
        let lifecycle = DeviceLifecycle::new(
            format!("imu/{}", config.name),
            DeviceState::Starting,
            "starting IMU sensor",
            telemetry.clone(),
        );
        let handle = ImuHandle {
            name: config.name.clone(),
            lifecycle,
            latest: Arc::new(RwLock::new(None)),
        };
        let worker_handle = handle.clone();
        let worker_stop = stop.clone();
        let worker_lifecycle = handle.lifecycle.clone();
        let thread_name = format!("{}-imu", config.name);
        let thread = thread::Builder::new().name(thread_name).spawn(move || {
            let run_stop = worker_stop.clone();
            let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                run_sensor(
                    config,
                    filter_config,
                    serial,
                    worker_handle,
                    run_stop,
                    telemetry,
                );
            }));
            if result.is_err() {
                worker_lifecycle.enter_fault("IMU worker panicked");
            } else if worker_stop.load(Ordering::Acquire) {
                worker_lifecycle.shutdown("IMU sensor stopped");
            } else {
                worker_lifecycle.enter_fault("IMU worker stopped unexpectedly");
            }
        })?;
        Ok(Self {
            handle,
            stop,
            thread: Some(thread),
        })
    }

    pub(crate) fn request_stop(&self) {
        self.stop.store(true, Ordering::Release);
    }
}

impl Drop for ImuWorker {
    fn drop(&mut self) {
        self.request_stop();
        if let Some(thread) = self.thread.take()
            && thread.join().is_err()
        {
            warn!(imu=%self.handle.name, "IMU worker panicked during shutdown");
        }
        self.handle.lifecycle.shutdown("IMU worker stopped");
    }
}

fn run_sensor(
    config: ImuSpec,
    filter_config: ImuFilterConfig,
    serial: SerialHandle,
    state: ImuHandle,
    stop: Arc<AtomicBool>,
    telemetry: Telemetry,
) {
    let mut configured_generation = None;
    let mut reconnect_requested_at = None;
    let mut last_valid = None;
    let mut frame_buffer = Vec::with_capacity(1024);
    let mut filter = ImuLowPass::new(filter_config);
    state
        .lifecycle
        .mark_connecting(format!("waiting for serial device {}", serial.spec.name));

    while !stop.load(Ordering::Acquire) {
        let transport = serial.status();
        if let Some(generation) = reconnect_requested_at {
            if !transport.state.is_online() || transport.transition_count == generation {
                if configured_generation.is_some() {
                    configured_generation = None;
                    last_valid = None;
                    frame_buffer.clear();
                    filter.reset();
                }
                thread::sleep(POLL_PERIOD);
                continue;
            }
            reconnect_requested_at = None;
        }
        if !transport.state.is_online() {
            if configured_generation.is_some() {
                state.lifecycle.begin_reconnect(format!(
                    "serial device {} unavailable: {} ({})",
                    serial.spec.name, transport.state, transport.detail
                ));
                configured_generation = None;
                last_valid = None;
                frame_buffer.clear();
                filter.reset();
            }
            thread::sleep(POLL_PERIOD);
            continue;
        }

        if configured_generation != Some(transport.transition_count) {
            state
                .lifecycle
                .mark_connecting(format!("configuring sensor on {}", serial.spec.name));
            if let Err(error) = configure_device(&config, &serial, &stop) {
                if stop.load(Ordering::Acquire) {
                    break;
                }
                let detail = format!("failed to configure {}: {error}", config.name);
                state.lifecycle.begin_reconnect(detail.clone());
                serial.request_reconnect(detail);
                reconnect_requested_at = Some(transport.transition_count);
                configured_generation = None;
                thread::sleep(POLL_PERIOD);
                continue;
            }
            configured_generation = Some(transport.transition_count);
            last_valid = Some(Instant::now());
            filter.reset();
        }

        match serial.recv_timeout(POLL_PERIOD) {
            Ok(chunk) => match frame_ascii_lines(&mut frame_buffer, chunk) {
                Ok(lines) => {
                    for (line, received) in lines {
                        match parse_serial_line(&line, received) {
                            Ok(sample) => {
                                last_valid = Some(sample.received);
                                state.lifecycle.mark_online("valid IMU samples streaming");
                                publish(&state, &telemetry, filter.apply(sample));
                            }
                            Err(error) => report_parse_error(&state, &config, &error),
                        }
                    }
                }
                Err(error) => report_parse_error(&state, &config, &error),
            },
            Err(crossbeam_channel::RecvTimeoutError::Timeout) => {}
            Err(crossbeam_channel::RecvTimeoutError::Disconnected) => {
                state
                    .lifecycle
                    .begin_reconnect("serial receive channel disconnected");
                break;
            }
        }

        if last_valid.is_some_and(|received| received.elapsed() >= config.stale_timeout) {
            let detail = format!(
                "no valid IMU sample for {} ms",
                last_valid.expect("checked above").elapsed().as_millis()
            );
            state.lifecycle.begin_reconnect(detail.clone());
            serial.request_reconnect(detail);
            reconnect_requested_at = Some(transport.transition_count);
            configured_generation = None;
            last_valid = None;
            frame_buffer.clear();
            filter.reset();
        }
    }
}

fn configure_device(config: &ImuSpec, serial: &SerialHandle, stop: &AtomicBool) -> Result<()> {
    let commands = std::iter::once("<stop>".to_owned())
        .chain(output_commands(config))
        .chain(std::iter::once("<start>".to_owned()));
    for command in commands {
        if stop.load(Ordering::Acquire) {
            bail!("IMU configuration cancelled by shutdown");
        }
        serial.send(command.clone().into_bytes(), stop)?;
        await_command_ack(serial, stop, &command)?;
    }
    info!(imu=%config.name, serial=%serial.spec.name, "EBIMU configured");
    Ok(())
}

fn await_command_ack(serial: &SerialHandle, stop: &AtomicBool, command: &str) -> Result<()> {
    let deadline = Instant::now() + COMMAND_ACK_TIMEOUT;
    let mut response = Vec::with_capacity(128);
    loop {
        if stop.load(Ordering::Acquire) {
            bail!("IMU configuration cancelled by shutdown");
        }
        let remaining = deadline.saturating_duration_since(Instant::now());
        if remaining.is_zero() {
            bail!("timed out waiting for EBIMU acknowledgement to {command}");
        }
        match serial.recv_timeout(POLL_PERIOD.min(remaining)) {
            Ok(chunk) => {
                response.extend_from_slice(&chunk.bytes);
                if contains_token(&response, b"<er>") {
                    bail!("EBIMU rejected configuration command {command}");
                }
                if contains_token(&response, b"<ok>") {
                    return Ok(());
                }
                if response.len() > 64 * 1024 {
                    bail!("EBIMU acknowledgement exceeded 64 KiB for {command}");
                }
            }
            Err(crossbeam_channel::RecvTimeoutError::Timeout) => {}
            Err(crossbeam_channel::RecvTimeoutError::Disconnected) => {
                bail!("serial receive channel disconnected while configuring EBIMU");
            }
        }
    }
}

fn contains_token(buffer: &[u8], token: &[u8]) -> bool {
    buffer.windows(token.len()).any(|window| window == token)
}

fn frame_ascii_lines(buffer: &mut Vec<u8>, chunk: SerialChunk) -> Result<Vec<(String, Instant)>> {
    buffer.extend_from_slice(&chunk.bytes);
    if buffer.len() > 64 * 1024 {
        buffer.clear();
        bail!("IMU serial frame exceeded 64 KiB without a newline");
    }
    let mut lines = Vec::new();
    while let Some(end) = buffer.iter().position(|byte| *byte == b'\n') {
        let bytes: Vec<_> = buffer.drain(..=end).collect();
        let line = std::str::from_utf8(&bytes)?.trim().to_owned();
        if !line.is_empty() {
            lines.push((line, chunk.received));
        }
    }
    Ok(lines)
}

fn parse_serial_line(line: &str, received: Instant) -> Result<ImuSample> {
    let mut sample = parse_ascii_line(line)?;
    sample.received = received;
    Ok(sample)
}

fn report_parse_error(state: &ImuHandle, config: &ImuSpec, error: &anyhow::Error) {
    state.lifecycle.degrade_or_update(error.to_string());
    warn!(imu=%config.name, %error, "discarding malformed IMU sample");
}

fn publish(state: &ImuHandle, telemetry: &Telemetry, sample: ImuSample) {
    *state.latest.write() = Some(sample.clone());
    telemetry.send(TelemetryPayload::Imu(ImuTelemetry {
        imu: state.name.clone(),
        orientation_xyzw: sample.orientation_xyzw,
        angular_velocity_xyz: sample.angular_velocity_xyz,
        linear_acceleration_xyz: sample.linear_acceleration_xyz,
        device_timestamp_ms: sample.device_timestamp_ms,
        sample_age_ms: sample.received.elapsed().as_millis().min(u64::MAX as u128) as u64,
    }));
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn imu_driver_frames_lines_across_transport_chunks() {
        let received = Instant::now();
        let mut buffer = Vec::new();
        assert!(
            frame_ascii_lines(
                &mut buffer,
                SerialChunk {
                    bytes: b"*1,2".to_vec(),
                    received,
                },
            )
            .unwrap()
            .is_empty()
        );
        let lines = frame_ascii_lines(
            &mut buffer,
            SerialChunk {
                bytes: b",3\r\n*4,5,6\n".to_vec(),
                received,
            },
        )
        .unwrap();
        assert_eq!(lines[0].0, "*1,2,3");
        assert_eq!(lines[1].0, "*4,5,6");
    }

    #[test]
    fn command_reply_tokens_are_found_among_stream_data() {
        assert!(contains_token(b"*0,0,0\r\n<ok>", b"<ok>"));
        assert!(contains_token(b"partial<er>trailing", b"<er>"));
        assert!(!contains_token(b"*0,0,0\r\n", b"<ok>"));
    }
}
