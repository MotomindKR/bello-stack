use std::collections::BTreeMap;
use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
};
use std::thread;
use std::time::{Duration, Instant};

use crate::config::{EmergencyStopMode, PowerBoardConfig};
use crate::domain::{PowerBoardName, SerialDeviceName, TelemetryPayload};
use crate::hardware::lifecycle::{DeviceLifecycle, DeviceState, DeviceStatus};
use crate::hardware::serial::SerialHandle;
use crate::telemetry::Telemetry;
use anyhow::{Result, ensure};

pub(crate) use cog_protocol as protocol;
use protocol::{Framer, Profile, Reading};

mod emergency;
pub(crate) use emergency::EmergencyHandler;
use emergency::EmergencyHandoff;

const POLL: Duration = Duration::from_millis(20);
const IDLE_PUBLISH: Duration = Duration::from_millis(250);

#[derive(Clone, Debug)]
pub(crate) struct PowerBoardSpec {
    pub(crate) name: PowerBoardName,
    pub(crate) serial_device: SerialDeviceName,
    pub(crate) profile: Profile,
    pub(crate) sync_period: Duration,
    pub(crate) stale_timeout: Duration,
    pub(crate) emergency_stop: EmergencyStopMode,
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
            .unwrap_or(if raw.profile == Profile::Agx { 100 } else { 0 });
        ensure!(
            period == 0 || (raw.profile == Profile::Agx && (100..5000).contains(&period)),
            "power board sync requires AGX and 100..4999 ms, or 0 for read-only"
        );
        ensure!(
            (250..=60_000).contains(&raw.stale_timeout_ms) && raw.stale_timeout_ms >= period * 2,
            "power board stale_timeout_ms must be 250..60000 and cover two sync periods"
        );
        ensure!(
            raw.emergency_stop != EmergencyStopMode::Damping
                || (raw.profile == Profile::Agx && period > 0 && raw.stale_timeout_ms < 5000),
            "emergency damping requires AGX, an active sync heartbeat, and stale_timeout_ms below 5000"
        );
        Ok(Self {
            name: raw.name.as_str().into(),
            serial_device: raw.serial_device.as_str().into(),
            profile: raw.profile,
            sync_period: Duration::from_millis(period),
            stale_timeout: Duration::from_millis(raw.stale_timeout_ms),
            emergency_stop: raw.emergency_stop,
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
        emergency: Option<EmergencyHandler>,
    ) -> Result<Self> {
        ensure!(
            emergency.is_some() == (spec.emergency_stop == EmergencyStopMode::Damping),
            "emergency damping requires a daemon safety handler"
        );
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
                    run(
                        spec,
                        serial,
                        &run_lifecycle,
                        &run_stop,
                        telemetry,
                        emergency,
                    )
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
    emergency: Option<EmergencyHandler>,
) {
    let mut emergency =
        emergency.map(|handler| EmergencyHandoff::new(serial.clone(), handler, spec.stale_timeout));
    let mut connected_at = Instant::now();
    let mut framer = Framer::default();
    let mut samples: BTreeMap<u8, Sample> = BTreeMap::new();
    let mut generation = None;
    let mut waiting_reconnect = None;
    let mut last_valid = Instant::now();
    let mut next_sync = Instant::now();
    let mut next_publish = Instant::now();
    let mut invalid_frames = 0u64;
    let mut unknown_frames = 0u64;
    let publish = |samples: &BTreeMap<u8, Sample>, generation, invalid_frames, unknown_frames| {
        telemetry.send(TelemetryPayload::PowerBoard(PowerTelemetry {
            name: spec.name.clone(),
            generation,
            status: lifecycle.status(),
            samples: samples.values().cloned().collect(),
            stale_timeout: spec.stale_timeout,
            invalid_frames,
            unknown_frames,
        }));
    };
    lifecycle.mark_connecting("waiting for power board serial transport");
    'receive: while !stop.load(Ordering::Acquire) {
        let transport = serial.status();
        let online =
            transport.state.is_online() && waiting_reconnect != Some(transport.transition_count);
        if !online {
            if let Some(handoff) = &mut emergency {
                handoff.disconnect();
            }
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
                connected_at = Instant::now();
                next_sync = Instant::now();
                lifecycle.mark_connecting("waiting for valid COG telemetry");
                if let Some(handoff) = &mut emergency
                    && let Err(error) = handoff.connect(stop)
                {
                    handoff.disconnect();
                    lifecycle.begin_reconnect(format!("emergency mode setup failed: {error}"));
                    serial.request_reconnect("emergency mode setup failed");
                    waiting_reconnect = generation;
                    continue;
                }
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
                                    if let Some(handoff) = &mut emergency
                                        && let Err(error) =
                                            handoff.observe(&reading, chunk.received, stop)
                                    {
                                        handoff.disconnect();
                                        lifecycle.begin_reconnect(format!(
                                            "emergency status failed: {error}"
                                        ));
                                        serial.request_reconnect("emergency status failed");
                                        waiting_reconnect = generation;
                                        continue 'receive;
                                    }
                                    last_valid = chunk.received;
                                    samples.insert(
                                        frame.kind,
                                        Sample {
                                            sequence: frame.sequence,
                                            received: chunk.received,
                                            reading,
                                        },
                                    );
                                    lifecycle.mark_online("valid COG telemetry received");
                                    publish(
                                        &samples,
                                        transport.transition_count,
                                        invalid_frames,
                                        unknown_frames,
                                    );
                                    next_publish = Instant::now() + IDLE_PUBLISH;
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
            if last_valid.elapsed() >= spec.stale_timeout
                || emergency
                    .as_ref()
                    .is_some_and(|handoff| handoff.stale(connected_at))
            {
                if let Some(handoff) = &mut emergency {
                    handoff.disconnect();
                }
                lifecycle.begin_reconnect("no fresh valid COG telemetry");
                serial.request_reconnect("no fresh valid COG telemetry");
                waiting_reconnect = generation;
            }
        }
        if Instant::now() >= next_publish {
            publish(
                &samples,
                generation.unwrap_or(transport.transition_count),
                invalid_frames,
                unknown_frames,
            );
            next_publish = Instant::now() + IDLE_PUBLISH;
        }
    }
}

// This fixture reopens a pseudo-terminal through the physical serial-port
// API. macOS PTYs reject the baud-rate ioctl required by this fixture.
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
            emergency_stop: EmergencyStopMode::Hardware,
            stale_timeout_ms: 3000,
        };
        assert_eq!(
            PowerBoardSpec::try_from(&c).unwrap().sync_period,
            Duration::from_millis(100)
        );
        c.profile = Profile::NanoNx;
        assert!(PowerBoardSpec::try_from(&c).unwrap().sync_period.is_zero());
        c.sync_period_ms = Some(1000);
        assert!(PowerBoardSpec::try_from(&c).is_err());
        c.profile = Profile::Agx;
        c.stale_timeout_ms = 500;
        assert!(PowerBoardSpec::try_from(&c).is_err());
        c.sync_period_ms = Some(100);
        c.emergency_stop = EmergencyStopMode::Damping;
        assert!(PowerBoardSpec::try_from(&c).is_ok());
        c.sync_period_ms = Some(0);
        assert!(PowerBoardSpec::try_from(&c).is_err());
        c.sync_period_ms = Some(100);
        c.stale_timeout_ms = 5000;
        assert!(PowerBoardSpec::try_from(&c).is_err());
        c.stale_timeout_ms = 500;
        c.profile = Profile::NanoNx;
        assert!(PowerBoardSpec::try_from(&c).is_err());
    }
}
