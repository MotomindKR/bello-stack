use std::time::Instant;

use tokio::sync::broadcast;

use crate::domain::TelemetryEvent;

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
    pub fn send(&self, mut event: TelemetryEvent) {
        if event.monotonic_ns == 0 {
            event.monotonic_ns = self.monotonic_ns();
        }
        let _ = self.tx.send(event);
    }
    pub fn subscribe(&self) -> broadcast::Receiver<TelemetryEvent> {
        self.tx.subscribe()
    }
}
