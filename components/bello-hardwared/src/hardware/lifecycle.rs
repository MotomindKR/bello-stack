use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::{Duration, Instant};

use anyhow::{Result, bail};
use garde::Validate;
use parking_lot::RwLock;

use crate::config::ReconnectConfig;
use crate::domain::{ComponentHealth, TelemetryPayload};
use crate::telemetry::Telemetry;

pub use crate::domain::{DeviceState, DeviceStatus};

impl DeviceState {
    fn allows(self, next: Self) -> bool {
        if self == next || next == Self::Shutdown {
            return true;
        }
        match self {
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

    pub(crate) fn mark_connecting(&self, detail: impl Into<String>) {
        let detail = detail.into();
        self.apply("mark connecting", move |fsm, updates| {
            match fsm.status().state {
                DeviceState::Starting | DeviceState::Reconnecting => {
                    record(fsm, updates, DeviceState::Connecting, detail)
                }
                DeviceState::Connecting if fsm.status().detail != detail => {
                    record(fsm, updates, DeviceState::Connecting, detail)
                }
                DeviceState::Connecting
                | DeviceState::Online
                | DeviceState::Degraded
                | DeviceState::Fault
                | DeviceState::Shutdown => Ok(()),
            }
        });
    }

    pub(crate) fn mark_online(&self, detail: impl Into<String>) {
        let detail = detail.into();
        self.apply("mark online", move |fsm, updates| {
            match fsm.status().state {
                DeviceState::Starting => {
                    record(fsm, updates, DeviceState::Connecting, "device detected")?;
                    record(fsm, updates, DeviceState::Online, detail)
                }
                DeviceState::Connecting | DeviceState::Degraded => {
                    record(fsm, updates, DeviceState::Online, detail)
                }
                DeviceState::Fault => {
                    record(fsm, updates, DeviceState::Reconnecting, "device recovered")?;
                    record(
                        fsm,
                        updates,
                        DeviceState::Connecting,
                        "validating recovered device",
                    )?;
                    record(fsm, updates, DeviceState::Online, detail)
                }
                DeviceState::Reconnecting => {
                    record(fsm, updates, DeviceState::Connecting, "device recovered")?;
                    record(fsm, updates, DeviceState::Online, detail)
                }
                DeviceState::Online if fsm.status().detail != detail => {
                    record(fsm, updates, DeviceState::Online, detail)
                }
                DeviceState::Online | DeviceState::Shutdown => Ok(()),
            }
        });
    }

    pub(crate) fn enter_fault(&self, detail: impl Into<String>) {
        let detail = detail.into();
        self.apply("enter fault", move |fsm, updates| {
            match fsm.status().state {
                DeviceState::Online => {
                    record(fsm, updates, DeviceState::Degraded, detail.clone())?;
                    record(fsm, updates, DeviceState::Fault, detail)
                }
                DeviceState::Starting
                | DeviceState::Connecting
                | DeviceState::Degraded
                | DeviceState::Reconnecting => record(fsm, updates, DeviceState::Fault, detail),
                DeviceState::Fault if fsm.status().detail != detail => {
                    record(fsm, updates, DeviceState::Fault, detail)
                }
                DeviceState::Fault | DeviceState::Shutdown => Ok(()),
            }
        });
    }

    pub(crate) fn begin_reconnect(&self, detail: impl Into<String>) {
        let detail = detail.into();
        self.apply("begin reconnect", move |fsm, updates| {
            match fsm.status().state {
                DeviceState::Online => {
                    record(fsm, updates, DeviceState::Degraded, detail.clone())?;
                    record(fsm, updates, DeviceState::Fault, detail.clone())?;
                    record(fsm, updates, DeviceState::Reconnecting, detail)
                }
                DeviceState::Starting | DeviceState::Connecting | DeviceState::Degraded => {
                    record(fsm, updates, DeviceState::Fault, detail.clone())?;
                    record(fsm, updates, DeviceState::Reconnecting, detail)
                }
                DeviceState::Fault => record(fsm, updates, DeviceState::Reconnecting, detail),
                DeviceState::Reconnecting if fsm.status().detail != detail => {
                    record(fsm, updates, DeviceState::Reconnecting, detail)
                }
                DeviceState::Reconnecting | DeviceState::Shutdown => Ok(()),
            }
        });
    }

    pub(crate) fn await_feedback(&self, detail: impl Into<String>) {
        let detail = detail.into();
        self.apply("await feedback", move |fsm, updates| {
            match fsm.status().state {
                DeviceState::Online => record(fsm, updates, DeviceState::Degraded, detail),
                DeviceState::Reconnecting => record(fsm, updates, DeviceState::Connecting, detail),
                DeviceState::Starting => record(fsm, updates, DeviceState::Connecting, detail),
                DeviceState::Connecting | DeviceState::Degraded
                    if fsm.status().detail != detail =>
                {
                    record(fsm, updates, fsm.status().state, detail)
                }
                DeviceState::Connecting
                | DeviceState::Degraded
                | DeviceState::Fault
                | DeviceState::Shutdown => Ok(()),
            }
        });
    }

    pub(crate) fn degrade_or_update(&self, detail: impl Into<String>) {
        let detail = detail.into();
        self.apply("record degraded detail", move |fsm, updates| {
            match fsm.status().state {
                DeviceState::Online => record(fsm, updates, DeviceState::Degraded, detail),
                DeviceState::Shutdown => Ok(()),
                state if fsm.status().detail != detail => record(fsm, updates, state, detail),
                _ => Ok(()),
            }
        });
    }

    pub(crate) fn shutdown(&self, detail: impl Into<String>) {
        let detail = detail.into();
        self.apply("shutdown", move |fsm, updates| {
            if fsm.status().state == DeviceState::Shutdown {
                Ok(())
            } else {
                record(fsm, updates, DeviceState::Shutdown, detail)
            }
        });
    }

    fn apply(
        &self,
        operation: &'static str,
        update: impl FnOnce(&mut DeviceFsm, &mut Vec<DeviceStatus>) -> Result<()>,
    ) {
        let result = {
            let mut current = self.fsm.write();
            let mut candidate = current.clone();
            let mut updates = Vec::new();
            let result = update(&mut candidate, &mut updates);
            if result.is_ok() {
                *current = candidate;
                // Publishing under the same lock preserves event ordering between
                // concurrent semantic operations without exposing partial state.
                for status in updates {
                    self.publish_status(status);
                }
            }
            result
        };
        if let Err(error) = result {
            tracing::warn!(component=%self.component, %error, operation, "rejected device lifecycle operation");
        }
    }

    fn publish_status(&self, status: DeviceStatus) {
        self.telemetry
            .send(TelemetryPayload::Health(ComponentHealth {
                component: self.component.to_string(),
                status,
                dropped_events: 0,
            }));
    }

    fn publish(&self) {
        self.publish_status(self.status());
    }
}

fn record(
    fsm: &mut DeviceFsm,
    updates: &mut Vec<DeviceStatus>,
    next: DeviceState,
    detail: impl Into<String>,
) -> Result<()> {
    fsm.transition(next, detail)?;
    updates.push(fsm.status().clone());
    Ok(())
}

pub(crate) fn wait_for_stop(stop: &AtomicBool, duration: Duration) -> bool {
    let deadline = Instant::now() + duration;
    while !stop.load(Ordering::Acquire) && Instant::now() < deadline {
        thread::sleep(
            Duration::from_millis(10).min(deadline.saturating_duration_since(Instant::now())),
        );
    }
    stop.load(Ordering::Acquire)
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

impl TryFrom<&ReconnectConfig> for ReconnectPolicy {
    type Error = anyhow::Error;

    fn try_from(config: &ReconnectConfig) -> Result<Self> {
        config.validate()?;
        if config.maximum_delay_ms < config.initial_delay_ms {
            bail!("maximum_delay_ms must be >= initial_delay_ms");
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
        lifecycle.mark_online("online");
        lifecycle.enter_fault("worker stopped");
        assert_eq!(lifecycle.state(), DeviceState::Fault);
        assert_eq!(lifecycle.status().detail, "worker stopped");
    }

    #[test]
    fn reconnect_operation_preserves_intermediate_health_events() {
        let telemetry = Telemetry::new(8);
        let lifecycle = DeviceLifecycle::new(
            "test/device",
            DeviceState::Online,
            "online",
            telemetry.clone(),
        );
        let mut events = telemetry.subscribe();

        lifecycle.begin_reconnect("link lost");

        let states: Vec<_> = (0..3)
            .map(|_| match events.try_recv().unwrap().payload {
                TelemetryPayload::Health(health) => health.status.state,
                _ => unreachable!(),
            })
            .collect();
        assert_eq!(
            states,
            [
                DeviceState::Degraded,
                DeviceState::Fault,
                DeviceState::Reconnecting
            ]
        );
        assert_eq!(lifecycle.status().transition_count, 3);
    }
}
