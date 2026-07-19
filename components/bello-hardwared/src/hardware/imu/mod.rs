use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::{Duration, Instant};

use anyhow::{Result, bail};
use parking_lot::RwLock;
use tracing::{info, warn};

use crate::domain::{ImuName, ImuTelemetry, SerialDeviceName, TelemetryEvent, TelemetryPayload};
use crate::hardware::lifecycle::{DeviceLifecycle, DeviceState};
use crate::hardware::serial::{SerialChunk, SerialHandle};
use crate::telemetry::Telemetry;

mod ebimu;
mod spec;

use ebimu::{output_commands, parse_ascii_line};
pub(crate) use spec::ImuSpec;

#[derive(Clone, Debug)]
pub struct ImuSample {
    pub orientation_xyzw: [f64; 4],
    pub angular_velocity_xyz: [f64; 3],
    pub linear_acceleration_xyz: [f64; 3],
    pub device_timestamp_ms: u64,
    pub received: Instant,
}

#[derive(Clone)]
pub struct ImuState {
    pub name: ImuName,
    pub serial_device: SerialDeviceName,
    pub lifecycle: DeviceLifecycle,
    pub latest: Arc<RwLock<Option<ImuSample>>>,
}

pub(crate) struct ImuWorker {
    pub(crate) state: ImuState,
    stop: Arc<AtomicBool>,
    thread: Option<thread::JoinHandle<()>>,
}

impl ImuWorker {
    pub(crate) fn start(
        config: ImuSpec,
        serial: SerialHandle,
        telemetry: Telemetry,
    ) -> Result<Self> {
        let stop = Arc::new(AtomicBool::new(false));
        let initial = if config.enabled {
            DeviceState::Starting
        } else {
            DeviceState::Disabled
        };
        let lifecycle = DeviceLifecycle::new(
            format!("imu/{}", config.name),
            initial,
            if config.enabled {
                "starting IMU sensor"
            } else {
                "IMU disabled by configuration"
            },
            telemetry.clone(),
        );
        let state = ImuState {
            name: config.name.clone(),
            serial_device: config.serial_device.clone(),
            lifecycle,
            latest: Arc::new(RwLock::new(None)),
        };
        if !config.enabled {
            return Ok(Self {
                state,
                stop,
                thread: None,
            });
        }
        let worker_state = state.clone();
        let worker_stop = stop.clone();
        let worker_lifecycle = state.lifecycle.clone();
        let thread_name = format!("{}-imu", config.name);
        let thread = thread::Builder::new().name(thread_name).spawn(move || {
            let run_stop = worker_stop.clone();
            let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                run_sensor(config, serial, worker_state, run_stop, telemetry);
            }));
            if result.is_err() {
                worker_lifecycle.fail("IMU worker panicked");
            } else if worker_stop.load(Ordering::Acquire) {
                let _ = worker_lifecycle.transition(DeviceState::Shutdown, "IMU sensor stopped");
            } else {
                worker_lifecycle.fail("IMU worker stopped unexpectedly");
            }
        })?;
        Ok(Self {
            state,
            stop,
            thread: Some(thread),
        })
    }
}

impl Drop for ImuWorker {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Release);
        if let Some(thread) = self.thread.take()
            && thread.join().is_err()
        {
            warn!(imu=%self.state.name, "IMU worker panicked during shutdown");
        }
        if self.state.lifecycle.state() != DeviceState::Shutdown {
            let _ = self
                .state
                .lifecycle
                .transition(DeviceState::Shutdown, "IMU worker stopped");
        }
    }
}

fn run_sensor(
    config: ImuSpec,
    serial: SerialHandle,
    state: ImuState,
    stop: Arc<AtomicBool>,
    telemetry: Telemetry,
) {
    let mut configured_generation = None;
    let mut last_valid = None;
    let mut frame_buffer = Vec::with_capacity(1024);
    let poll_period = Duration::from_millis(20);
    let _ = state.lifecycle.transition(
        DeviceState::Connecting,
        format!("waiting for serial device {}", serial.spec.name),
    );

    while !stop.load(Ordering::Acquire) {
        let transport = serial.status();
        if !transport.state.is_online() {
            if configured_generation.is_some() {
                enter_reconnecting(
                    &state.lifecycle,
                    format!(
                        "serial device {} unavailable: {} ({})",
                        serial.spec.name, transport.state, transport.detail
                    ),
                );
                configured_generation = None;
                last_valid = None;
                frame_buffer.clear();
            }
            thread::sleep(poll_period);
            continue;
        }

        if configured_generation != Some(transport.transition_count) {
            prepare_connecting(&state.lifecycle, &serial.spec.name);
            if let Err(error) = configure_device(&config, &serial) {
                let detail = format!("failed to configure {}: {error}", config.name);
                enter_reconnecting(&state.lifecycle, detail.clone());
                serial.request_reconnect(detail);
                configured_generation = None;
                thread::sleep(poll_period);
                continue;
            }
            configured_generation = Some(transport.transition_count);
            last_valid = Some(Instant::now());
        }

        match serial.recv_timeout(poll_period) {
            Ok(chunk) => match frame_ascii_lines(&mut frame_buffer, chunk) {
                Ok(lines) => {
                    for (line, received) in lines {
                        match parse_serial_line(&line, received, &config) {
                            Ok(sample) => {
                                last_valid = Some(sample.received);
                                if matches!(
                                    state.lifecycle.state(),
                                    DeviceState::Connecting | DeviceState::Degraded
                                ) {
                                    let _ = state.lifecycle.transition(
                                        DeviceState::Online,
                                        "valid IMU samples streaming",
                                    );
                                }
                                publish(&state, &telemetry, sample);
                            }
                            Err(error) => report_parse_error(&state, &config, &error),
                        }
                    }
                }
                Err(error) => report_parse_error(&state, &config, &error),
            },
            Err(crossbeam_channel::RecvTimeoutError::Timeout) => {}
            Err(crossbeam_channel::RecvTimeoutError::Disconnected) => {
                enter_reconnecting(
                    &state.lifecycle,
                    "serial receive channel disconnected".into(),
                );
                break;
            }
        }

        if last_valid.is_some_and(|received| received.elapsed() >= config.stale_timeout) {
            let detail = format!(
                "no valid IMU sample for {} ms",
                last_valid.expect("checked above").elapsed().as_millis()
            );
            enter_reconnecting(&state.lifecycle, detail.clone());
            serial.request_reconnect(detail);
            configured_generation = None;
            last_valid = None;
            frame_buffer.clear();
        }
    }
}

fn prepare_connecting(lifecycle: &DeviceLifecycle, serial: &SerialDeviceName) {
    match lifecycle.state() {
        DeviceState::Starting => {
            let _ = lifecycle.transition(
                DeviceState::Connecting,
                format!("configuring sensor on {serial}"),
            );
        }
        DeviceState::Reconnecting => {
            let _ = lifecycle.transition(
                DeviceState::Connecting,
                format!("reconfiguring sensor on {serial}"),
            );
        }
        DeviceState::Connecting => {
            let _ = lifecycle.transition(
                DeviceState::Connecting,
                format!("configuring sensor on {serial}"),
            );
        }
        _ => {}
    }
}

fn enter_reconnecting(lifecycle: &DeviceLifecycle, detail: String) {
    match lifecycle.state() {
        DeviceState::Online => {
            let _ = lifecycle.transition(DeviceState::Degraded, detail.clone());
            let _ = lifecycle.transition(DeviceState::Fault, detail.clone());
            let _ = lifecycle.transition(DeviceState::Reconnecting, detail);
        }
        DeviceState::Degraded | DeviceState::Connecting => {
            let _ = lifecycle.transition(DeviceState::Fault, detail.clone());
            let _ = lifecycle.transition(DeviceState::Reconnecting, detail);
        }
        DeviceState::Fault => {
            let _ = lifecycle.transition(DeviceState::Reconnecting, detail);
        }
        DeviceState::Reconnecting => {
            let _ = lifecycle.transition(DeviceState::Reconnecting, detail);
        }
        _ => {}
    }
}

fn configure_device(config: &ImuSpec, serial: &SerialHandle) -> Result<()> {
    if config.configure_on_start {
        for command in output_commands(config) {
            serial.send(command.into_bytes())?;
            thread::sleep(Duration::from_millis(20));
        }
    }
    info!(imu=%config.name, serial=%serial.spec.name, "EBIMU configured");
    Ok(())
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

fn parse_serial_line(line: &str, received: Instant, config: &ImuSpec) -> Result<ImuSample> {
    let mut sample = parse_ascii_line(line, config)?;
    sample.received = received;
    Ok(sample)
}

fn report_parse_error(state: &ImuState, config: &ImuSpec, error: &anyhow::Error) {
    if state.lifecycle.state() == DeviceState::Online {
        let _ = state
            .lifecycle
            .transition(DeviceState::Degraded, error.to_string());
    } else {
        let _ = state
            .lifecycle
            .transition(state.lifecycle.state(), error.to_string());
    }
    warn!(imu=%config.name, %error, "discarding malformed IMU sample");
}

fn publish(state: &ImuState, telemetry: &Telemetry, sample: ImuSample) {
    *state.latest.write() = Some(sample.clone());
    telemetry.send(TelemetryEvent {
        monotonic_ns: 0,
        payload: TelemetryPayload::Imu(ImuTelemetry {
            imu: state.name.clone(),
            serial_device: state.serial_device.clone(),
            status: state.lifecycle.status(),
            orientation_xyzw: sample.orientation_xyzw,
            angular_velocity_xyz: sample.angular_velocity_xyz,
            linear_acceleration_xyz: sample.linear_acceleration_xyz,
            device_timestamp_ms: sample.device_timestamp_ms,
            sample_age_ms: sample.received.elapsed().as_millis().min(u64::MAX as u128) as u64,
        }),
    });
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
}
