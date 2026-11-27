use std::sync::{Arc, atomic::AtomicBool};
use std::time::{Duration, Instant};

use anyhow::{Result, bail};

use super::protocol::Reading;
use crate::hardware::serial::SerialHandle;

pub(crate) type EmergencyHandler = Arc<dyn Fn(bool) + Send + Sync>;

const WARNING_OVER_CURRENT_DISCHARGE_VBAT: u32 = 1 << 0;
const WARNING_OVER_CURRENT_VMOT: u32 = 1 << 4;

pub(super) struct BoardSafety {
    handoff: Option<SerialHandle>,
    handler: EmergencyHandler,
    blocked: bool,
    configured: bool,
    last_iwc: Option<Instant>,
    stale_timeout: Duration,
}

impl BoardSafety {
    pub(super) fn new(
        handoff: Option<SerialHandle>,
        handler: EmergencyHandler,
        stale_timeout: Duration,
    ) -> Self {
        Self {
            handoff,
            handler,
            blocked: false,
            configured: false,
            last_iwc: None,
            stale_timeout,
        }
    }

    fn block(&mut self, blocked: bool) {
        if self.blocked != blocked {
            (self.handler)(blocked);
            self.blocked = blocked;
        }
    }

    pub(super) fn connect(&mut self, stop: &AtomicBool) -> Result<()> {
        self.disconnect();
        let Some(serial) = &self.handoff else {
            return Ok(());
        };
        serial.send(b"iwc emergency stop_category 0\r\n".to_vec(), stop)?;
        // The host latches motion inhibition by revoking leases. Do not turn a
        // physical press into a persistent firmware software-stop request.
        serial.send(b"iwc emergency latch 0\r\n".to_vec(), stop)?;
        Ok(())
    }

    pub(super) fn observe(
        &mut self,
        reading: &Reading,
        received: Instant,
        stop: &AtomicBool,
    ) -> Result<()> {
        if let Reading::Iwc { info, warning, .. } = reading {
            let overcurrent =
                warning & (WARNING_OVER_CURRENT_DISCHARGE_VBAT | WARNING_OVER_CURRENT_VMOT) != 0;
            let Some(serial) = &self.handoff else {
                if overcurrent || received.elapsed() < self.stale_timeout {
                    self.block(overcurrent);
                }
                return Ok(());
            };
            if received.elapsed() >= self.stale_timeout {
                bail!("stale emergency status received");
            }
            self.last_iwc = Some(received);
            let inhibited = info & 0b111 != 0 || overcurrent;
            if !inhibited && !self.configured {
                serial.send(b"iwc emergency stop_category 1\r\n".to_vec(), stop)?;
                self.configured = true;
            }
            self.block(inhibited);
        }
        Ok(())
    }

    pub(super) fn stale(&self, connected_at: Instant) -> bool {
        self.handoff.is_some()
            && self.last_iwc.unwrap_or(connected_at).elapsed() >= self.stale_timeout
    }

    pub(super) fn disconnect(&mut self) {
        if self.handoff.is_some() {
            self.block(true);
        }
        self.last_iwc = None;
        self.configured = false;
    }
}

impl Drop for BoardSafety {
    fn drop(&mut self) {
        self.disconnect();
        let Some(serial) = &self.handoff else { return };
        // The worker stop flag is already set on orderly shutdown. The serial
        // manager stays alive and bounds this independent cleanup write.
        if let Err(error) = serial.send(
            b"iwc emergency stop_category 0\r\n".to_vec(),
            &AtomicBool::new(false),
        ) {
            tracing::error!(%error, "could not restore hardware emergency cutoff; relying on firmware heartbeat expiry");
        }
        if let Err(error) = serial.send(
            b"iwc emergency latch 1\r\n".to_vec(),
            &AtomicBool::new(false),
        ) {
            tracing::error!(%error, "could not restore firmware emergency latch");
        }
    }
}
