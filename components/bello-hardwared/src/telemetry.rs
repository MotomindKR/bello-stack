use std::time::Instant;

use tokio::sync::broadcast;

use crate::domain::{TelemetryEvent, TelemetryPayload};

#[derive(Clone)]
pub struct Telemetry {
    started: Instant,
    tx: broadcast::Sender<TelemetryEvent>,
}

impl Telemetry {
    pub fn new(capacity: usize) -> Self {
        let (tx, _) = broadcast::channel(capacity);
        Self {
            started: Instant::now(),
            tx,
        }
    }

    pub fn monotonic_ns(&self) -> u64 {
        self.started.elapsed().as_nanos().min(u64::MAX as u128) as u64
    }

    pub fn event(&self, payload: TelemetryPayload) -> TelemetryEvent {
        TelemetryEvent {
            monotonic_ns: self.monotonic_ns(),
            payload,
        }
    }

    pub fn send(&self, payload: TelemetryPayload) {
        let _ = self.tx.send(self.event(payload));
    }

    pub fn subscribe(&self) -> broadcast::Receiver<TelemetryEvent> {
        self.tx.subscribe()
    }
}
