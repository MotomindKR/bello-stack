use std::thread;
use std::time::{Duration, Instant};

use anyhow::{Context, Result, ensure};
use robstride_protocol::{Feedback, Frame, active_report_frame, parse_feedback, stop_frame};

use crate::domain::MotorAddress;
use crate::hardware::can::MaintenanceBus;
use crate::hardware::robstride::{FirmwareVersion as Version, MotorSpec};

use firmware::Image;
use stationary::StationaryCheck;

mod firmware;
mod stationary;

const IDENTITY: u8 = 0;
const FEEDBACK: u8 = 2;
const SET_ZERO: u8 = 6;
const FAULT: u8 = 21;
const REPLY_TIMEOUT: Duration = Duration::from_millis(200);
const QUIET_WINDOW: Duration = Duration::from_millis(500);
const QUIET_SETTLE: Duration = Duration::from_millis(30);
const STOP_TIMEOUT: Duration = Duration::from_secs(1);
const ZERO_POSITION_LIMIT: f64 = 0.03;

pub(crate) struct Prepared {
    pub(crate) uuid: [u8; 8],
    pub(crate) current_version: String,
    pub(crate) reviewed_position_rad: Option<f64>,
    pub(crate) target_version: Option<String>,
    address: MotorAddress,
    model: String,
    host: u8,
    peers: Vec<u8>,
    installed: Version,
    image: Option<Image>,
    simulation: bool,
}

pub(crate) fn prepare(
    bus: &MaintenanceBus,
    motor: &MotorSpec,
    peers: &[u8],
    host: u8,
    firmware: bool,
    simulation: bool,
) -> Result<Prepared> {
    let link = Link::new(bus, motor, host);
    link.prepare(peers, firmware, simulation)
}

pub(crate) fn execute(
    bus: &MaintenanceBus,
    motor: &MotorSpec,
    host: u8,
    p: &Prepared,
    before_write: impl FnOnce() -> Result<()>,
    progress: impl Fn(u32),
) -> Result<()> {
    let link = Link::new(bus, motor, host);
    link.execute(p, before_write, progress)
}

trait Wire {
    fn send(&self, frame: Frame) -> Result<()>;
    fn recv(&self, timeout: Duration) -> Result<Option<Frame>>;
}

impl Wire for MaintenanceBus {
    fn send(&self, frame: Frame) -> Result<()> {
        self.send(frame)
    }

    fn recv(&self, timeout: Duration) -> Result<Option<Frame>> {
        self.recv(timeout)
    }
}

struct Link<'a> {
    wire: &'a dyn Wire,
    motor: &'a MotorSpec,
    id: u8,
    host: u8,
}

impl<'a> Link<'a> {
    fn new(wire: &'a dyn Wire, motor: &'a MotorSpec, host: u8) -> Self {
        Self {
            wire,
            motor,
            id: motor.address.id,
            host,
        }
    }

    fn prepare(&self, peers: &[u8], firmware: bool, simulation: bool) -> Result<Prepared> {
        let mut peers = peers.to_vec();
        peers.retain(|&id| id != self.id);
        peers.sort_unstable();
        peers.dedup();

        let (uuid, installed, position) = self.preflight(&peers)?;
        let image = firmware
            .then(|| Image::prepare(&self.motor.model, installed, simulation))
            .transpose()?;
        Ok(Prepared {
            uuid,
            current_version: installed.to_string(),
            reviewed_position_rad: (!firmware).then_some(position),
            target_version: image.as_ref().map(|image| image.version.to_string()),
            address: self.motor.address.clone(),
            model: self.motor.model.clone(),
            host: self.host,
            peers,
            installed,
            image,
            simulation,
        })
    }

    fn execute(
        &self,
        prepared: &Prepared,
        before_write: impl FnOnce() -> Result<()>,
        progress: impl Fn(u32),
    ) -> Result<()> {
        ensure!(
            self.motor.address == prepared.address
                && self.motor.model == prepared.model
                && self.host == prepared.host,
            "Motor selection changed since preparation"
        );
        progress(0);
        let (uuid, installed, position) = self.preflight(&prepared.peers)?;
        if let Some(reviewed) = prepared.reviewed_position_rad {
            ensure!(
                (reviewed - position).abs() < ZERO_POSITION_LIMIT,
                "shaft moved since zero was prepared"
            );
        }
        ensure!(
            uuid == prepared.uuid,
            "Motor UUID changed since preparation"
        );
        ensure!(
            installed == prepared.installed,
            "Firmware changed since preparation to {installed}"
        );

        before_write()?;
        if let Some(image) = &prepared.image {
            image.transfer(self, uuid, &progress).context(
                "Firmware transfer failed. Keep the motor stopped and verify its firmware before retrying",
            )?;
            progress(97);
            self.wait_for_restart(prepared.simulation).context(
                "Could not confirm firmware restart. Keep the motor stopped and verify its firmware before retrying",
            )?;
            self.verify(uuid, image.version, false)
                .context("Firmware transfer completed but readback did not verify")?;
        } else {
            self.send(SET_ZERO, self.host.into(), [1, 0, 0, 0, 0, 0, 0, 0])?;
            thread::sleep(Duration::from_millis(30));
            self.verify(uuid, installed, true)
                .context("Re-zero command sent but readback did not verify")?;
        }
        progress(100);
        Ok(())
    }

    fn preflight(&self, peers: &[u8]) -> Result<([u8; 8], Version, f64)> {
        self.discard_pending()?;
        for &id in peers.iter().chain(std::iter::once(&self.id)) {
            self.wire.send(active_report_frame(self.host, id, true))?;
            let status = self
                .wait_for(STOP_TIMEOUT, |frame| {
                    (frame.id >> 24 == u32::from(FEEDBACK)
                        && (frame.id >> 8) as u8 == id
                        && frame.id as u8 == self.host
                        && robstride_protocol::firmware_version_reply(&frame).is_none())
                    .then_some((frame.id >> 16) as u8)
                })?
                .with_context(|| format!("motor {id} did not confirm stopped mode"))?;
            ensure!(
                status == 0,
                "motor {id} must report stopped mode without faults"
            );
        }
        self.wire.send(stop_frame(self.host, self.id, false))?;
        for &id in peers.iter().chain(std::iter::once(&self.id)) {
            self.wire.send(active_report_frame(self.host, id, false))?;
        }
        // Allow the stop reply and any in-flight periodic reports to arrive.
        // A later quiet window must be uninterrupted.
        self.wait_for::<()>(QUIET_SETTLE, |_| None)?;
        ensure!(
            self.wire.recv(QUIET_WINDOW)?.is_none(),
            "CAN bus traffic detected during maintenance preflight. Disable other CAN tools and periodic reports"
        );

        let uuid = self.uuid()?;
        let version = self
            .version()?
            .context("No firmware version response before CAN deadline")?;
        version.check_model(&self.motor.model)?;
        let stopped = self.stationary()?;
        Ok((uuid, version, stopped.position))
    }

    fn verify(&self, uuid: [u8; 8], expected: Version, require_zero: bool) -> Result<()> {
        let actual = self
            .version()?
            .context("No firmware version response before CAN deadline")?;
        ensure!(self.uuid()? == uuid, "Motor UUID did not match");
        ensure!(
            actual == expected,
            "Motor reported firmware {actual}, expected {expected}"
        );

        let stopped = self.stationary()?;
        if require_zero {
            ensure!(
                stopped.position.abs() < ZERO_POSITION_LIMIT,
                "Stopped position {:.4} rad is outside the re-zero threshold of {:.2} rad",
                stopped.position,
                ZERO_POSITION_LIMIT
            );
        }
        Ok(())
    }

    fn wait_for_restart(&self, simulation: bool) -> Result<()> {
        if !simulation {
            thread::sleep(Duration::from_secs(2));
        }
        for _ in 0..10 {
            if self.version()?.is_some() {
                self.wire
                    .send(active_report_frame(self.host, self.id, false))?;
                return Ok(());
            }
            thread::sleep(Duration::from_millis(300));
        }
        anyhow::bail!("Motor did not report its firmware after OTA")
    }

    fn uuid(&self) -> Result<[u8; 8]> {
        self.send(IDENTITY, self.host.into(), [0; 8])?;
        self.wait_for(REPLY_TIMEOUT, |frame| {
            (frame.id >> 8 == u32::from(self.id) && [self.host, 0xfe].contains(&(frame.id as u8)))
                .then_some(frame.data)
        })?
        .context("No motor UUID response before CAN deadline")
    }

    fn version(&self) -> Result<Option<Version>> {
        self.wire
            .send(robstride_protocol::firmware_version_request(
                self.host, self.id,
            ))?;
        self.wait_for(REPLY_TIMEOUT, |frame| self.version_reply(frame))
    }

    fn version_reply(&self, frame: Frame) -> Option<Version> {
        if !self.is_reply(frame, FEEDBACK) {
            return None;
        }
        robstride_protocol::firmware_version_reply(&frame).map(Version)
    }

    fn stationary(&self) -> Result<Feedback> {
        let deadline = Instant::now() + STOP_TIMEOUT;
        let params = self.motor.params;
        let mut check = StationaryCheck::new(params.position_max - params.position_min);
        while let Some(remaining) = deadline.checked_duration_since(Instant::now()) {
            self.wire.send(stop_frame(self.host, self.id, false))?;
            let feedback = self.wait_for(remaining.min(Duration::from_millis(100)), |frame| {
                self.is_reply(frame, FEEDBACK)
                    .then(|| parse_feedback(&frame, params))
                    .flatten()
            })?;
            if let Some(feedback) = feedback {
                ensure!(
                    feedback.fault_bits == 0,
                    "Motor reported faults 0x{:02x} during maintenance",
                    feedback.fault_bits
                );
                if check.observe(feedback, Instant::now()) {
                    return Ok(feedback);
                }
            } else {
                check.clear();
            }
            thread::sleep(
                Duration::from_millis(10).min(deadline.saturating_duration_since(Instant::now())),
            );
        }
        anyhow::bail!("Could not confirm stopped mode and a stationary shaft")
    }

    fn send(&self, kind: u8, field: u16, data: [u8; 8]) -> Result<()> {
        self.wire.send(Frame {
            id: (u32::from(kind) << 24) | (u32::from(field) << 8) | u32::from(self.id),
            data,
        })
    }

    fn is_reply(&self, frame: Frame, kind: u8) -> bool {
        self.is_reply_to(frame, kind, self.host)
    }

    fn is_reply_to(&self, frame: Frame, kind: u8, host: u8) -> bool {
        frame.id >> 24 == u32::from(kind)
            && (frame.id >> 8) as u8 == self.id
            && frame.id as u8 == host
    }

    fn check_fault(&self, frame: Frame) -> Result<()> {
        if self.is_reply(frame, FAULT)
            || self.is_reply_to(frame, FAULT, robstride_protocol::OTA_HOST_ID)
        {
            let faults = u32::from_le_bytes(frame.data[..4].try_into().expect("four fault bytes"));
            ensure!(faults == 0, "Motor reported type-21 fault 0x{faults:08x}");
        }
        Ok(())
    }

    fn discard_pending(&self) -> Result<()> {
        for _ in 0..4096 {
            let Some(frame) = self.wire.recv(Duration::ZERO)? else {
                return Ok(());
            };
            self.check_fault(frame)?;
        }
        anyhow::bail!("CAN receive queue did not drain before maintenance")
    }

    fn wait_for<T>(
        &self,
        timeout: Duration,
        accept: impl Fn(Frame) -> Option<T>,
    ) -> Result<Option<T>> {
        let deadline = Instant::now() + timeout;
        while let Some(remaining) = deadline.checked_duration_since(Instant::now()) {
            let Some(frame) = self.wire.recv(remaining)? else {
                break;
            };
            self.check_fault(frame)?;
            if let Some(value) = accept(frame) {
                return Ok(Some(value));
            }
        }
        Ok(None)
    }
}

#[cfg(test)]
mod tests;
