use std::collections::BTreeMap;
use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
};
use std::thread;
use std::time::{Duration, Instant};

use crate::config::PowerBoardConfig;
use crate::domain::{PowerBoardName, SerialDeviceName, TelemetryPayload};
use crate::hardware::lifecycle::{DeviceLifecycle, DeviceState, DeviceStatus};
use crate::hardware::serial::SerialHandle;
use crate::telemetry::Telemetry;
use anyhow::{Result, ensure};

pub(crate) use cog_protocol as protocol;
use protocol::{Framer, Profile, Reading};

const POLL: Duration = Duration::from_millis(20);
const PUBLISH: Duration = Duration::from_millis(250);

#[derive(Clone, Debug)]
pub(crate) struct PowerBoardSpec {
    pub(crate) name: PowerBoardName,
    pub(crate) serial_device: SerialDeviceName,
    pub(crate) profile: Profile,
    pub(crate) sync_period: Duration,
    pub(crate) stale_timeout: Duration,
}

impl TryFrom<&PowerBoardConfig> for PowerBoardSpec {
    type Error = anyhow::Error;
    fn try_from(raw: &PowerBoardConfig) -> Result<Self> {
        for value in [&raw.name, &raw.serial_device] {
            ensure!(
                !value.trim().is_empty() && value.trim() == value,
                "power board names must be nonempty without surrounding whitespace"
            );
        }
        let period = raw
            .sync_period_ms
            .unwrap_or(if raw.profile == Profile::Agx { 1000 } else { 0 });
        ensure!(
            period == 0 || (raw.profile == Profile::Agx && (100..5000).contains(&period)),
            "power board sync requires AGX and 100..4999 ms, or 0 for read-only"
        );
        ensure!(
            (250..=60_000).contains(&raw.stale_timeout_ms) && raw.stale_timeout_ms >= period * 2,
            "power board stale_timeout_ms must be 250..60000 and cover two sync periods"
        );
        Ok(Self {
            name: raw.name.as_str().into(),
            serial_device: raw.serial_device.as_str().into(),
            profile: raw.profile,
            sync_period: Duration::from_millis(period),
            stale_timeout: Duration::from_millis(raw.stale_timeout_ms),
        })
    }
}

#[derive(Clone, Debug)]
pub(crate) struct Sample {
    pub(crate) sequence: u8,
    pub(crate) received: Instant,
    pub(crate) reading: Reading,
}

#[derive(Clone, Debug)]
pub(crate) struct PowerTelemetry {
    pub(crate) name: PowerBoardName,
    pub(crate) generation: u64,
    pub(crate) status: DeviceStatus,
    pub(crate) samples: Vec<Sample>,
    pub(crate) stale_timeout: Duration,
    pub(crate) invalid_frames: u64,
    pub(crate) unknown_frames: u64,
}

pub(crate) struct PowerBoardWorker {
    pub(crate) lifecycle: DeviceLifecycle,
    stop: Arc<AtomicBool>,
    thread: Option<thread::JoinHandle<()>>,
}

impl PowerBoardWorker {
    pub(crate) fn start(
        spec: PowerBoardSpec,
        serial: SerialHandle,
        telemetry: Telemetry,
    ) -> Result<Self> {
        let lifecycle = DeviceLifecycle::new(
            format!("power_board/{}", spec.name),
            DeviceState::Starting,
            "starting power board",
            telemetry.clone(),
        );
        let stop = Arc::new(AtomicBool::new(false));
        let run_stop = stop.clone();
        let run_lifecycle = lifecycle.clone();
        let thread = thread::Builder::new()
            .name(format!("{}-power-board", spec.name))
            .spawn(move || {
                let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                    run(spec, serial, &run_lifecycle, &run_stop, telemetry)
                }));
                if result.is_err() {
                    run_lifecycle.enter_fault("power board worker panicked");
                } else if run_stop.load(Ordering::Acquire) {
                    run_lifecycle.shutdown("power board stopped");
                } else {
                    run_lifecycle.enter_fault("power board worker stopped unexpectedly");
                }
            })?;
        Ok(Self {
            lifecycle,
            stop,
            thread: Some(thread),
        })
    }

    pub(crate) fn request_stop(&self) {
        self.stop.store(true, Ordering::Release);
    }
}

impl Drop for PowerBoardWorker {
    fn drop(&mut self) {
        self.request_stop();
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
    }
}

fn run(
    spec: PowerBoardSpec,
    serial: SerialHandle,
    lifecycle: &DeviceLifecycle,
    stop: &AtomicBool,
    telemetry: Telemetry,
) {
    let mut framer = Framer::default();
    let mut samples: BTreeMap<u8, Sample> = BTreeMap::new();
    let mut generation = None;
    let mut waiting_reconnect = None;
    let mut last_valid = Instant::now();
    let mut next_sync = Instant::now();
    let mut next_publish = Instant::now();
    let mut invalid_frames = 0u64;
    let mut unknown_frames = 0u64;
    lifecycle.mark_connecting("waiting for power board serial transport");
    while !stop.load(Ordering::Acquire) {
        let transport = serial.status();
        let online =
            transport.state.is_online() && waiting_reconnect != Some(transport.transition_count);
        if !online {
            if generation.take().is_some() {
                samples.clear();
                framer = Framer::default();
                lifecycle.begin_reconnect("power board serial unavailable");
                next_publish = Instant::now();
            }
            thread::sleep(POLL);
        } else {
            if generation != Some(transport.transition_count) {
                generation = Some(transport.transition_count);
                waiting_reconnect = None;
                samples.clear();
                framer = Framer::default();
                last_valid = Instant::now();
                next_sync = Instant::now();
                lifecycle.mark_connecting("waiting for valid COG telemetry");
            }
            if !spec.sync_period.is_zero() && Instant::now() >= next_sync {
                if let Err(error) = serial.send(b"sync\r\n".to_vec(), stop) {
                    if stop.load(Ordering::Acquire) {
                        break;
                    }
                    lifecycle.begin_reconnect(format!("power board sync failed: {error}"));
                    serial.request_reconnect("power board sync failed");
                    waiting_reconnect = generation;
                    continue;
                }
                next_sync = Instant::now() + spec.sync_period;
            }
            match serial.recv_timeout(POLL) {
                Ok(chunk) => {
                    for byte in chunk.bytes {
                        match framer.push(byte, spec.profile) {
                            Some(Ok(frame)) => {
                                if let Some(reading) = frame.reading {
                                    last_valid = chunk.received;
                                    // State changes publish immediately; ordinary samples are coalesced.
                                    if matches!(frame.kind, 2 | 3)
                                        && samples
                                            .get(&frame.kind)
                                            .is_none_or(|old| old.reading != reading)
                                    {
                                        next_publish = Instant::now();
                                    }
                                    samples.insert(
                                        frame.kind,
                                        Sample {
                                            sequence: frame.sequence,
                                            received: chunk.received,
                                            reading,
                                        },
                                    );
                                    lifecycle.mark_online(
                                        "valid COG telemetry received; monitoring only",
                                    );
                                } else {
                                    unknown_frames = unknown_frames.saturating_add(1);
                                }
                            }
                            Some(Err(_)) => {
                                invalid_frames = invalid_frames.saturating_add(1);
                                lifecycle.degrade_or_update("invalid COG frame discarded");
                            }
                            None => {}
                        }
                    }
                }
                Err(crossbeam_channel::RecvTimeoutError::Timeout) => {}
                Err(crossbeam_channel::RecvTimeoutError::Disconnected) => break,
            }
            if last_valid.elapsed() >= spec.stale_timeout {
                lifecycle.begin_reconnect("no fresh valid COG telemetry");
                serial.request_reconnect("no fresh valid COG telemetry");
                waiting_reconnect = generation;
            }
        }
        if Instant::now() >= next_publish {
            telemetry.send(TelemetryPayload::PowerBoard(PowerTelemetry {
                name: spec.name.clone(),
                generation: generation.unwrap_or(transport.transition_count),
                status: lifecycle.status(),
                samples: samples.values().cloned().collect(),
                stale_timeout: spec.stale_timeout,
                invalid_frames,
                unknown_frames,
            }));
            next_publish = Instant::now() + PUBLISH;
        }
    }
}

// This fixture reopens a pseudo-terminal through the physical serial-port
// API. macOS PTYs reject the baud-rate ioctl required by this fixture.
// Protocol and configuration tests run on every platform.
#[cfg(all(test, target_os = "linux"))]
mod tests;

#[cfg(test)]
mod spec_tests {
    use super::*;
    #[test]
    fn profile_controls_heartbeat_and_staleness_validation() {
        let mut c = PowerBoardConfig {
            name: "pcb".into(),
            serial_device: "uart".into(),
            profile: Profile::Agx,
            sync_period_ms: None,
            stale_timeout_ms: 3000,
        };
        assert_eq!(
            PowerBoardSpec::try_from(&c).unwrap().sync_period,
            Duration::from_secs(1)
        );
        c.profile = Profile::NanoNx;
        assert!(PowerBoardSpec::try_from(&c).unwrap().sync_period.is_zero());
        c.sync_period_ms = Some(1000);
        assert!(PowerBoardSpec::try_from(&c).is_err());
        c.profile = Profile::Agx;
        c.stale_timeout_ms = 500;
        assert!(PowerBoardSpec::try_from(&c).is_err());
    }
}
