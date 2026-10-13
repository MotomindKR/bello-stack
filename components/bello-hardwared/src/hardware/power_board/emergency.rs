use std::sync::{Arc, atomic::AtomicBool};
use std::time::{Duration, Instant};

use anyhow::{Result, bail};

use super::protocol::Reading;
use crate::hardware::serial::SerialHandle;

pub(crate) type EmergencyHandler = Arc<dyn Fn(bool) + Send + Sync>;

pub(super) struct EmergencyHandoff {
    serial: SerialHandle,
    handler: EmergencyHandler,
    blocked: Option<bool>,
    configured: bool,
    last_iwc: Option<Instant>,
    stale_timeout: Duration,
}

impl EmergencyHandoff {
    pub(super) fn new(
        serial: SerialHandle,
        handler: EmergencyHandler,
        stale_timeout: Duration,
    ) -> Self {
        let mut handoff = Self {
            serial,
            handler,
            blocked: None,
            configured: false,
            last_iwc: None,
            stale_timeout,
        };
        handoff.block(true);
        handoff
    }

    fn block(&mut self, blocked: bool) {
        if self.blocked != Some(blocked) {
            (self.handler)(blocked);
            self.blocked = Some(blocked);
        }
    }

    pub(super) fn connect(&mut self, stop: &AtomicBool) -> Result<()> {
        self.block(true);
        self.configured = false;
        self.last_iwc = None;
        self.serial
            .send(b"iwc emergency stop_category 0\r\n".to_vec(), stop)?;
        // The host latches motion inhibition by revoking leases. Do not turn a
        // physical press into a persistent firmware software-stop request.
        self.serial
            .send(b"iwc emergency latch 0\r\n".to_vec(), stop)?;
        Ok(())
    }

    pub(super) fn observe(
        &mut self,
        reading: &Reading,
        received: Instant,
        stop: &AtomicBool,
    ) -> Result<()> {
        if let Reading::Iwc { info, .. } = reading {
            if received.elapsed() >= self.stale_timeout {
                bail!("stale emergency status received");
            }
            self.last_iwc = Some(received);
            let pressed = info & 0b111 != 0;
            if !pressed && !self.configured {
                self.serial
                    .send(b"iwc emergency stop_category 1\r\n".to_vec(), stop)?;
                self.configured = true;
            }
            self.block(pressed || !self.configured);
        }
        Ok(())
    }

    pub(super) fn stale(&self, connected_at: Instant) -> bool {
        self.last_iwc.unwrap_or(connected_at).elapsed() >= self.stale_timeout
    }

    pub(super) fn disconnect(&mut self) {
        self.block(true);
        self.last_iwc = None;
        self.configured = false;
    }
}

impl Drop for EmergencyHandoff {
    fn drop(&mut self) {
        self.block(true);
        // The worker stop flag is already set on orderly shutdown. The serial
        // manager stays alive and bounds this independent cleanup write.
        if let Err(error) = self.serial.send(
            b"iwc emergency stop_category 0\r\n".to_vec(),
            &AtomicBool::new(false),
        ) {
            tracing::error!(%error, "could not restore hardware emergency cutoff; relying on firmware heartbeat expiry");
        }
        if let Err(error) = self.serial.send(
            b"iwc emergency latch 1\r\n".to_vec(),
            &AtomicBool::new(false),
        ) {
            tracing::error!(%error, "could not restore firmware emergency latch");
        }
    }
}
