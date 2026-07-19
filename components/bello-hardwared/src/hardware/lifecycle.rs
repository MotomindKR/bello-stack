use std::sync::Arc;
use std::time::Duration;

use anyhow::{Result, bail};
use parking_lot::RwLock;

use crate::config::ReconnectConfig;
use crate::domain::{ComponentHealth, TelemetryEvent, TelemetryPayload};
use crate::telemetry::Telemetry;

pub use crate::domain::{DeviceState, DeviceStatus};

impl DeviceState {
    fn allows(self, next: Self) -> bool {
        if self == next || next == Self::Shutdown {
            return true;
        }
        match self {
            Self::Disabled => false,
            Self::Starting => matches!(next, Self::Connecting | Self::Online | Self::Fault),
            Self::Connecting => matches!(next, Self::Online | Self::Fault),
            Self::Online => matches!(next, Self::Degraded | Self::Fault),
            Self::Degraded => matches!(next, Self::Online | Self::Fault),
            Self::Reconnecting => matches!(next, Self::Connecting | Self::Fault),
            Self::Fault => matches!(next, Self::Reconnecting),
            Self::Shutdown => false,
        }
    }
}

#[derive(Clone, Debug)]
pub struct DeviceFsm {
    status: DeviceStatus,
}

impl DeviceFsm {
    pub fn new(initial: DeviceState, detail: impl Into<String>) -> Self {
        Self {
            status: DeviceStatus {
                state: initial,
                detail: detail.into(),
                transition_count: 0,
            },
        }
    }

    pub fn status(&self) -> &DeviceStatus {
        &self.status
    }

    pub fn transition(&mut self, next: DeviceState, detail: impl Into<String>) -> Result<()> {
        let from = self.status.state;
        if !from.allows(next) {
            bail!("invalid device transition {from} -> {next}");
        }
        let detail = detail.into();
        if from != next {
            self.status.transition_count += 1;
        }
        self.status.state = next;
        self.status.detail = detail;
        Ok(())
    }
}

#[derive(Clone)]
pub struct DeviceLifecycle {
    component: Arc<str>,
    fsm: Arc<RwLock<DeviceFsm>>,
    telemetry: Telemetry,
}

impl DeviceLifecycle {
    pub fn new(
        component: impl Into<Arc<str>>,
        initial: DeviceState,
        detail: impl Into<String>,
        telemetry: Telemetry,
    ) -> Self {
        let lifecycle = Self {
            component: component.into(),
            fsm: Arc::new(RwLock::new(DeviceFsm::new(initial, detail))),
            telemetry,
        };
        lifecycle.publish();
        lifecycle
    }

    pub fn status(&self) -> DeviceStatus {
        self.fsm.read().status().clone()
    }

    pub fn state(&self) -> DeviceState {
        self.fsm.read().status().state
    }

    pub fn transition(&self, next: DeviceState, detail: impl Into<String>) -> Result<()> {
        self.fsm.write().transition(next, detail)?;
        self.publish();
        Ok(())
    }

    /// Records an unexpected runtime failure without requiring each worker to
    /// duplicate the legal transition path into `Fault`.
    pub(crate) fn fail(&self, detail: impl Into<String>) {
        let detail = detail.into();
        match self.state() {
            DeviceState::Starting | DeviceState::Connecting | DeviceState::Reconnecting => {
                let _ = self.transition(DeviceState::Fault, detail);
            }
            DeviceState::Online => {
                let _ = self.transition(DeviceState::Degraded, detail.clone());
                let _ = self.transition(DeviceState::Fault, detail);
            }
            DeviceState::Degraded | DeviceState::Fault => {
                let _ = self.transition(DeviceState::Fault, detail);
            }
            DeviceState::Disabled | DeviceState::Shutdown => {}
        }
    }

    fn publish(&self) {
        let status = self.status();
        self.telemetry.send(TelemetryEvent {
            monotonic_ns: 0,
            payload: TelemetryPayload::Health(ComponentHealth {
                component: self.component.to_string(),
                status: status.state,
                detail: status.detail,
                transition_count: status.transition_count,
                dropped_events: 0,
            }),
        });
    }
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct ReconnectPolicy {
    initial_delay: Duration,
    maximum_delay: Duration,
}

impl ReconnectPolicy {
    pub(crate) fn delay(self, failures: u32) -> Duration {
        let exponent = failures.saturating_sub(1).min(31);
        self.initial_delay
            .saturating_mul(1_u32 << exponent)
            .min(self.maximum_delay)
    }
}

impl TryFrom<(&ReconnectConfig, &str)> for ReconnectPolicy {
    type Error = anyhow::Error;

    fn try_from((config, path): (&ReconnectConfig, &str)) -> Result<Self> {
        if config.initial_delay_ms == 0 {
            bail!("{path}.initial_delay_ms must be positive");
        }
        if config.maximum_delay_ms < config.initial_delay_ms {
            bail!("{path}.maximum_delay_ms must be >= initial_delay_ms");
        }
        Ok(Self {
            initial_delay: Duration::from_millis(config.initial_delay_ms),
            maximum_delay: Duration::from_millis(config.maximum_delay_ms),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rejects_invalid_device_transition() {
        let mut fsm = DeviceFsm::new(DeviceState::Starting, "starting");
        assert!(fsm.transition(DeviceState::Connecting, "opening").is_ok());
        assert!(fsm.transition(DeviceState::Online, "ready").is_ok());
        assert!(
            fsm.transition(DeviceState::Reconnecting, "skipped fault")
                .is_err()
        );
    }

    #[test]
    fn reconnect_delay_is_bounded() {
        let policy = ReconnectPolicy {
            initial_delay: Duration::from_millis(100),
            maximum_delay: Duration::from_secs(1),
        };
        assert_eq!(policy.delay(1), Duration::from_millis(100));
        assert_eq!(policy.delay(4), Duration::from_millis(800));
        assert_eq!(policy.delay(20), Duration::from_secs(1));
    }

    #[test]
    fn unexpected_online_worker_failure_is_faulted() {
        let lifecycle = DeviceLifecycle::new(
            "test/device",
            DeviceState::Starting,
            "starting",
            Telemetry::new(8),
        );
        lifecycle.transition(DeviceState::Online, "online").unwrap();
        lifecycle.fail("worker stopped");
        assert_eq!(lifecycle.state(), DeviceState::Fault);
        assert_eq!(lifecycle.status().detail, "worker stopped");
    }
}
