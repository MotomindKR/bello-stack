use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::time::{Duration, Instant};

use crossbeam_channel::Sender;
use parking_lot::Mutex;

use super::{BusAccess, BusGeneration, CanFrame};
use crate::domain::{BusName, GroupName};

// Allow isolated missed slots, but revoke targets after four refresh periods
// without a completed write burst. This is independent of target validity.
const MAXIMUM_SILENCE_PERIODS: u32 = 4;

pub(super) struct PeriodicBatch {
    pub(super) bus_generation: BusGeneration,
    pub(super) frames: Arc<[CanFrame]>,
    pub(super) expires_at: Instant,
    generation: u64,
    lease: Arc<AtomicU64>,
    shutdown: Arc<AtomicBool>,
    permitted: Arc<AtomicBool>,
}

impl PeriodicBatch {
    pub(super) fn permitted(&self) -> bool {
        self.permitted.load(Ordering::Acquire)
            && self.generation != 0
            && self.lease.load(Ordering::Acquire) == self.generation
            && !self.shutdown.load(Ordering::Acquire)
    }

    fn valid(&self, now: Instant) -> bool {
        self.permitted() && now < self.expires_at
    }
}

#[derive(Default)]
struct Slot {
    batch: Option<Arc<PeriodicBatch>>,
    next_tick: Option<Instant>,
    last_sent: Option<Instant>,
    fault: Option<String>,
    missed_slots: u64,
}

/// One latest-target slot per group on a bus. Only the TX worker advances its clock.
#[derive(Clone)]
pub(crate) struct PeriodicControl {
    group: GroupName,
    bus: BusName,
    period: Duration,
    lease: Arc<AtomicU64>,
    shutdown: Arc<AtomicBool>,
    slot: Arc<Mutex<Slot>>,
    wake: Sender<()>,
    access: Arc<BusAccess>,
}

impl PeriodicControl {
    pub(super) fn new(
        group: GroupName,
        bus: BusName,
        period: Duration,
        lease: Arc<AtomicU64>,
        shutdown: Arc<AtomicBool>,
        wake: Sender<()>,
        access: Arc<BusAccess>,
    ) -> Self {
        Self {
            group,
            bus,
            period,
            lease,
            shutdown,
            slot: Arc::new(Mutex::new(Slot::default())),
            wake,
            access,
        }
    }

    pub(crate) fn publish(&self, frames: Vec<CanFrame>, expires_at: Instant, generation: u64) {
        let started = Instant::now();
        let mut slot = self.slot.lock();
        let Some((bus_generation, _)) = self
            .access
            .ordinary_traffic()
            .filter(|(_, window)| window.accepts(started))
        else {
            return;
        };
        self.discard_old_targets(&mut slot, Some(bus_generation));
        // Updates share the activation guard, so revocation also cancels a
        // previous target set already held by TX. Start creates a fresh guard.
        let permitted = slot.batch.as_ref().map_or_else(
            || Arc::new(AtomicBool::new(true)),
            |batch| batch.permitted.clone(),
        );
        slot.batch = Some(Arc::new(PeriodicBatch {
            bus_generation,
            frames: frames.into(),
            expires_at,
            generation,
            lease: self.lease.clone(),
            shutdown: self.shutdown.clone(),
            permitted,
        }));
        if slot.next_tick.is_none() {
            let now = Instant::now();
            slot.next_tick = Some(now);
            slot.last_sent = Some(now);
        }
        drop(slot);
        let _ = self.wake.try_send(());
    }

    pub(crate) fn revoke(&self) {
        let mut slot = self.slot.lock();
        if let Some(batch) = slot.batch.take() {
            batch.permitted.store(false, Ordering::Release);
        }
        slot.next_tick = None;
        slot.last_sent = None;
        slot.fault = None;
        drop(slot);
        let _ = self.wake.try_send(());
    }

    pub(crate) fn failure(&self, now: Instant) -> Option<String> {
        let mut slot = self.slot.lock();
        self.discard_old_targets(&mut slot, self.access.ordinary_generation());
        self.check_silence(&mut slot, now);
        slot.fault.clone()
    }

    fn discard_old_targets(&self, slot: &mut Slot, generation: Option<BusGeneration>) {
        if let Some(batch) = &slot.batch
            && Some(batch.bus_generation) != generation
        {
            batch.permitted.store(false, Ordering::Release);
            *slot = Slot::default();
        }
    }

    fn check_silence(&self, slot: &mut Slot, now: Instant) {
        if slot.fault.is_none()
            && slot.batch.as_ref().is_some_and(|batch| batch.valid(now))
            && slot.last_sent.is_some_and(|sent| {
                now.saturating_duration_since(sent) >= self.period * MAXIMUM_SILENCE_PERIODS
            })
        {
            let detail = format!(
                "bus {} group {} periodic control stalled for at least {} us",
                self.bus,
                self.group,
                (self.period * MAXIMUM_SILENCE_PERIODS).as_micros()
            );
            self.fail(slot, detail);
        }
    }

    fn fail(&self, slot: &mut Slot, detail: String) {
        if let Some(batch) = &slot.batch {
            batch.permitted.store(false, Ordering::Release);
        }
        slot.next_tick = None;
        slot.fault = Some(detail);
    }

    pub(super) fn transport_failed(&self, detail: &str) {
        let mut slot = self.slot.lock();
        if slot.batch.is_some() {
            self.fail(&mut slot, detail.into());
        }
    }

    pub(super) fn next_tick(&self, now: Instant) -> Option<Instant> {
        let mut slot = self.slot.lock();
        self.discard_old_targets(&mut slot, self.access.ordinary_generation());
        slot.batch
            .as_ref()
            .filter(|batch| batch.valid(now))
            .and(slot.next_tick)
    }

    pub(super) fn take_due(&self, now: Instant) -> Option<Arc<PeriodicBatch>> {
        let mut slot = self.slot.lock();
        self.check_silence(&mut slot, now);
        let due = slot.next_tick?;
        let batch = slot.batch.as_ref()?.clone();
        if now < due || !batch.valid(now) {
            return None;
        }
        let late = now.duration_since(due);
        let skipped = (late.as_nanos() / self.period.as_nanos()).min(u64::MAX as u128) as u64;
        slot.missed_slots = slot.missed_slots.saturating_add(skipped);
        let remainder = late.as_nanos() % self.period.as_nanos();
        slot.next_tick = Some(now + self.period - Duration::from_nanos(remainder as u64));
        if skipped != 0 {
            tracing::warn!(bus=%self.bus, group=%self.group, skipped_slots=skipped,
                total_missed_slots=slot.missed_slots, lateness_us=late.as_micros() as u64,
                "periodic CAN control skipped missed slots");
        }
        Some(batch)
    }

    pub(super) fn stall_deadline(&self) -> Option<Instant> {
        self.slot
            .lock()
            .last_sent
            .map(|sent| sent + self.period * MAXIMUM_SILENCE_PERIODS)
    }

    pub(super) fn permits_transmission(
        &self,
        batch: &PeriodicBatch,
        now: Instant,
        stall_deadline: Instant,
    ) -> bool {
        // Only TX records successful writes. The captured deadline therefore
        // stays fixed through this burst; normal frames need no slot lock.
        if now >= stall_deadline {
            self.check_silence(&mut self.slot.lock(), now);
        }
        batch.permitted()
    }

    pub(super) fn sent(&self, now: Instant) {
        let mut slot = self.slot.lock();
        self.check_silence(&mut slot, now);
        if slot.fault.is_none() && slot.batch.is_some() {
            slot.last_sent = Some(now);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crossbeam_channel::bounded;

    fn control() -> (PeriodicControl, Arc<AtomicU64>, Arc<AtomicBool>) {
        let lease = Arc::new(AtomicU64::new(1));
        let shutdown = Arc::new(AtomicBool::new(false));
        let control = PeriodicControl::new(
            GroupName::from("group"),
            BusName::from("can"),
            Duration::from_millis(10),
            lease.clone(),
            shutdown.clone(),
            bounded(1).0,
            Arc::new(BusAccess::default()),
        );
        (control, lease, shutdown)
    }

    fn publish(control: &PeriodicControl, id: u32, deadline: Instant) {
        control.publish(vec![CanFrame { id, data: [0; 8] }], deadline, 1);
    }

    #[test]
    fn late_completion_cannot_reset_the_silence_watchdog() {
        let (control, _, _) = control();
        publish(&control, 1, Instant::now() + Duration::from_secs(1));
        let due = control.next_tick(Instant::now()).unwrap();
        control.take_due(due).unwrap();
        let late = due + Duration::from_millis(45);
        control.sent(late);
        assert!(control.failure(late).is_some());
        assert!(control.next_tick(late).is_none());
    }

    #[test]
    fn latest_targets_replace_pending_without_resetting_clock_or_silence_watchdog() {
        let (control, _, _) = control();
        let deadline = Instant::now() + Duration::from_secs(1);
        publish(&control, 1, deadline);
        let due = control.next_tick(Instant::now()).unwrap();
        publish(&control, 2, deadline);
        assert_eq!(control.next_tick(due), Some(due));
        assert_eq!(control.take_due(due).unwrap().frames[0].id, 2);
        publish(&control, 3, deadline);
        assert!(control.failure(due + Duration::from_millis(40)).is_some());
        assert!(control.take_due(due + Duration::from_millis(40)).is_none());
    }

    #[test]
    fn isolated_late_tick_skips_slots_without_catchup() {
        let (control, _, _) = control();
        publish(&control, 1, Instant::now() + Duration::from_secs(1));
        let due = control.next_tick(Instant::now()).unwrap();
        let late = due + Duration::from_millis(25);
        assert!(control.take_due(late).is_some());
        control.sent(late);
        assert_eq!(control.slot.lock().missed_slots, 2);
        assert!(control.take_due(late).is_none());
        assert_eq!(
            control.next_tick(late),
            Some(due + Duration::from_millis(30))
        );
        assert!(control.failure(late).is_none());
    }

    #[test]
    fn revocation_cancels_inflight_old_targets_and_rearming_cannot_restore_them() {
        let (control, _, _) = control();
        let deadline = Instant::now() + Duration::from_secs(1);
        publish(&control, 1, deadline);
        let due = control.next_tick(Instant::now()).unwrap();
        let old = control.take_due(due).unwrap();
        publish(&control, 2, deadline);
        control.revoke();
        assert!(!old.permitted());
        publish(&control, 3, deadline);
        assert!(!old.permitted());
        assert!(
            control
                .take_due(control.next_tick(Instant::now()).unwrap())
                .unwrap()
                .permitted()
        );
    }

    #[test]
    fn lease_revocation_and_shutdown_block_tx_before_group_thread_runs() {
        let (control, lease, shutdown) = control();
        publish(&control, 1, Instant::now() + Duration::from_secs(1));
        let due = control.next_tick(Instant::now()).unwrap();
        let batch = control.take_due(due).unwrap();
        lease.store(2, Ordering::Release);
        assert!(!batch.permitted());
        assert!(control.next_tick(due).is_none());
        lease.store(1, Ordering::Release);
        shutdown.store(true, Ordering::Release);
        assert!(!batch.permitted());
    }

    #[test]
    fn repeats_keep_original_expiry_and_expiry_is_not_a_transport_fault() {
        let (control, _, _) = control();
        let deadline = Instant::now() + Duration::from_millis(30);
        publish(&control, 1, deadline);
        let due = control.next_tick(Instant::now()).unwrap();
        for tick in 0..3 {
            let now = due + Duration::from_millis(tick * 10);
            assert_eq!(control.take_due(now).unwrap().expires_at, deadline);
            control.sent(now);
        }
        assert!(control.take_due(deadline).is_none());
        assert!(control.failure(deadline + Duration::from_secs(1)).is_none());
    }
}
