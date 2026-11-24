use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::{Duration, Instant};

use crossbeam_channel::{Receiver, Sender, select_biased};
use tracing::{error, warn};

use super::PeriodicControl;
use super::periodic::PeriodicBatch;
use super::{
    BatchFailure, BusSpec, BusTxQueues, CanFrame, ControlBatch, PriorityBatch, PriorityBatchKind,
    ReceivedFrame,
};
use crate::config::BusTransportConfig;
use crate::hardware::lifecycle::{DeviceLifecycle, wait_for_stop};

enum Outbound {
    Control(ControlBatch),
    Priority(PriorityBatch),
    Periodic(PeriodicControl, Arc<PeriodicBatch>),
}

mod blocking;
mod sim;
#[cfg(target_os = "linux")]
mod socketcan;
mod usb;

trait FrameWriter {
    fn write_frame(&self, frame: CanFrame) -> Result<(), WriteFailure>;
}

#[derive(Debug)]
struct WriteFailure {
    queue_full: bool,
    detail: String,
}

impl WriteFailure {
    fn fatal(detail: impl Into<String>) -> Self {
        Self {
            queue_full: false,
            detail: detail.into(),
        }
    }
}

const BACKPRESSURE_LIMIT: Duration = Duration::from_millis(2);
const BACKPRESSURE_POLL: Duration = Duration::from_micros(100);

pub(super) fn run(
    spec: BusSpec,
    queues: BusTxQueues,
    frames: Sender<ReceivedFrame>,
    lifecycle: DeviceLifecycle,
    shutdown: Arc<AtomicBool>,
) {
    match &spec.transport {
        BusTransportConfig::Socketcan { .. } => {
            #[cfg(target_os = "linux")]
            blocking::run::<socketcan::SocketCanReader>(spec, queues, frames, lifecycle, shutdown);
            #[cfg(not(target_os = "linux"))]
            {
                lifecycle.enter_fault(
                    "SocketCAN requires Linux; use gsusb for hardware or sim for simulation",
                );
                reject_queued(&queues, "SocketCAN requires Linux");
            }
        }
        BusTransportConfig::Gsusb { .. } => usb::run(spec, queues, frames, lifecycle, shutdown),
        BusTransportConfig::Sim(_) => {
            blocking::run::<bello_sim_can::SimSocket>(spec, queues, frames, lifecycle, shutdown)
        }
    }
}

fn run_tx<W: FrameWriter>(
    writer: W,
    spec: &BusSpec,
    queues: &BusTxQueues,
    shutdown: &AtomicBool,
    session_stop: &AtomicBool,
    failure: &Sender<String>,
) {
    while let Some(outbound) = receive_prioritized(queues, shutdown, session_stop) {
        match outbound {
            Outbound::Periodic(control, batch) => {
                match transmit_periodic_batch(spec, &control, &batch, |frame| {
                    writer.write_frame(frame)
                }) {
                    Ok(true) => control.sent(Instant::now()),
                    Ok(false) => {}
                    Err(detail) => {
                        control.transport_failed(&detail);
                        error!(bus=%spec.name, %detail, "periodic CAN batch became indeterminate");
                        let _ = failure.try_send(detail);
                        break;
                    }
                }
            }
            Outbound::Control(batch) => match transmit_control_batch(&writer, spec, batch) {
                Ok(true) => {}
                Ok(false) => {
                    warn!(bus=%spec.name, "discarding expired CAN control batch");
                }
                Err(detail) => {
                    error!(bus=%spec.name, %detail, "CAN control batch became indeterminate");
                    let _ = failure.try_send(detail);
                    break;
                }
            },
            Outbound::Priority(batch) => {
                let result =
                    if priority_batch_can_finish(spec, batch.frames.len(), batch.expires_at) {
                        apply_priority_ordering(batch.kind, &queues.control);
                        transmit_priority_batch(&writer, batch.frames, batch.expires_at)
                    } else {
                        Err(BatchFailure::Expired)
                    };
                let failed = match &result {
                    Err(BatchFailure::Transmit(detail)) => Some(detail.clone()),
                    _ => None,
                };
                let _ = batch.completion.send(result);
                if let Some(detail) = failed {
                    error!(bus=%spec.name, %detail, "CAN priority batch failed");
                    let _ = failure.try_send(detail);
                    break;
                }
            }
        }
    }
}

fn transmit_periodic_batch(
    spec: &BusSpec,
    control: &PeriodicControl,
    batch: &PeriodicBatch,
    mut write: impl FnMut(CanFrame) -> Result<(), WriteFailure>,
) -> Result<bool, String> {
    let Some(stall_deadline) = control.stall_deadline() else {
        return Ok(false);
    };
    if !control_batch_can_finish(spec, batch.frames.len(), batch.expires_at) {
        return Ok(false);
    }
    let mut retry_until = None;
    for (index, frame) in batch.frames.iter().enumerate() {
        // Intentional revocation may interrupt a burst: its ordered safety
        // barrier follows on this same TX worker. It must never resume.
        let now = Instant::now();
        if !control.permits_transmission(batch, now, stall_deadline) {
            return Ok(false);
        }
        if now >= batch.expires_at {
            return if index == 0 {
                Ok(false)
            } else {
                Err("periodic control batch expired after partial transmission".into())
            };
        }
        if !write_with_backpressure(
            *frame,
            batch.expires_at,
            &mut retry_until,
            |now| control.permits_transmission(batch, now, stall_deadline),
            &mut write,
        )? {
            return Ok(false);
        }
    }
    let completed = Instant::now();
    if !control.permits_transmission(batch, completed, stall_deadline) {
        return Ok(false);
    }
    if completed > batch.expires_at {
        Err("periodic control batch completed after target validity deadline".into())
    } else {
        Ok(true)
    }
}

fn transmit_control_batch<W: FrameWriter>(
    writer: &W,
    spec: &BusSpec,
    batch: ControlBatch,
) -> Result<bool, String> {
    if !control_batch_can_finish(spec, batch.frames.len(), batch.expires_at) {
        return Ok(false);
    }
    let expires_at = batch.expires_at;
    let mut retry_until = None;
    write_control_batch(batch, Instant::now, |frame| {
        write_with_backpressure(
            frame,
            expires_at,
            &mut retry_until,
            |_| true,
            |frame| writer.write_frame(frame),
        )
        .map(|_| ())
    })
}

fn write_control_batch(
    batch: ControlBatch,
    clock: impl Fn() -> Instant,
    mut write: impl FnMut(CanFrame) -> Result<(), String>,
) -> Result<bool, String> {
    let started = clock();
    let total = batch.frames.len();
    for (index, frame) in batch.frames.into_iter().enumerate() {
        let now = clock();
        if now >= batch.expires_at {
            return if index == 0 {
                Ok(false)
            } else {
                Err(format!(
                    "control batch expired after partial transmission \
                     ({index} of {total} frames written in {} us, {} us of freshness left at the first write)",
                    now.saturating_duration_since(started).as_micros(),
                    batch
                        .expires_at
                        .saturating_duration_since(started)
                        .as_micros(),
                ))
            };
        }
        write(frame)?;
    }
    let completed = clock();
    if completed > batch.expires_at {
        Err(format!(
            "control batch completed after its freshness deadline \
             ({total} frames written in {} us, {} us of freshness left at the first write)",
            completed.saturating_duration_since(started).as_micros(),
            batch
                .expires_at
                .saturating_duration_since(started)
                .as_micros(),
        ))
    } else {
        Ok(true)
    }
}

fn transmit_priority_batch<W: FrameWriter>(
    writer: &W,
    frames: Vec<CanFrame>,
    expires_at: Instant,
) -> Result<(), BatchFailure> {
    let mut retry_until = None;
    for (index, frame) in frames.into_iter().enumerate() {
        if Instant::now() >= expires_at {
            return if index == 0 {
                Err(BatchFailure::Expired)
            } else {
                Err(BatchFailure::Transmit(
                    "priority batch expired after partial transmission".into(),
                ))
            };
        }
        write_with_backpressure(
            frame,
            expires_at,
            &mut retry_until,
            |_| true,
            |frame| writer.write_frame(frame),
        )
        .map_err(BatchFailure::Transmit)?;
    }
    if Instant::now() > expires_at {
        Err(BatchFailure::Transmit(
            "priority batch completed after its delivery deadline".into(),
        ))
    } else {
        Ok(())
    }
}

fn control_batch_can_finish(spec: &BusSpec, frame_count: usize, expires_at: Instant) -> bool {
    Instant::now()
        .checked_add(spec.control_batch_budget(frame_count))
        .is_some_and(|completion| completion <= expires_at)
}

fn priority_batch_can_finish(spec: &BusSpec, frame_count: usize, expires_at: Instant) -> bool {
    Instant::now()
        .checked_add(spec.priority_batch_budget(frame_count))
        .is_some_and(|completion| completion <= expires_at)
}

fn write_with_backpressure(
    frame: CanFrame,
    expires_at: Instant,
    retry_until: &mut Option<Instant>,
    mut permitted: impl FnMut(Instant) -> bool,
    mut write: impl FnMut(CanFrame) -> Result<(), WriteFailure>,
) -> Result<bool, String> {
    let mut retrying = false;
    loop {
        let now = Instant::now();
        if !permitted(now) {
            return Ok(false);
        }
        if now >= expires_at {
            return Err("CAN batch expired while waiting for transmit capacity".into());
        }
        if retrying && retry_until.is_some_and(|deadline| now >= deadline) {
            return Err("CAN transmit queue stayed full beyond its retry budget".into());
        }
        match write(frame) {
            Ok(()) => return Ok(true),
            Err(error) if error.queue_full => {
                let now = Instant::now();
                let deadline =
                    (*retry_until.get_or_insert(now + BACKPRESSURE_LIMIT)).min(expires_at);
                if now >= deadline {
                    return Err(format!("CAN transmit queue stayed full: {}", error.detail));
                }
                retrying = true;
                thread::sleep(BACKPRESSURE_POLL.min(deadline - now));
            }
            Err(error) => return Err(error.detail),
        }
    }
}

fn purge_control(control: &Receiver<ControlBatch>) {
    while control.try_recv().is_ok() {}
}

fn apply_priority_ordering(kind: PriorityBatchKind, control: &Receiver<ControlBatch>) {
    if kind == PriorityBatchKind::SafetyBarrier {
        purge_control(control);
    }
}

fn prepare_reconnect(
    spec: &BusSpec,
    lifecycle: &DeviceLifecycle,
    queues: &BusTxQueues,
    detail: String,
    shutdown: &AtomicBool,
    failures: &mut u32,
) {
    reject_queued(queues, &detail);
    if shutdown.load(Ordering::Acquire) {
        lifecycle.enter_fault(detail);
        return;
    }
    *failures = failures.saturating_add(1);
    let delay = spec.reconnect.delay(*failures);
    lifecycle.begin_reconnect(format!("{detail}; retrying in {} ms", delay.as_millis()));
    warn!(bus=%spec.name, %detail, delay_ms=delay.as_millis(), "CAN bus reconnect scheduled");
    if !wait_for_stop(shutdown, delay) {
        lifecycle.mark_connecting(format!("reopening {}", spec.transport.label()));
    }
}

fn reject_queued(queues: &BusTxQueues, reason: &str) {
    for control in queues.periodic.lock().values() {
        control.transport_failed(reason);
    }
    for queue in [&queues.safety, &queues.configuration] {
        while let Ok(batch) = queue.try_recv() {
            let _ = batch
                .completion
                .send(Err(BatchFailure::Transmit(reason.into())));
        }
    }
    while queues.control.try_recv().is_ok() {}
}

fn receive_prioritized(
    queues: &BusTxQueues,
    shutdown: &AtomicBool,
    session_stop: &AtomicBool,
) -> Option<Outbound> {
    loop {
        if shutdown.load(Ordering::Acquire) || session_stop.load(Ordering::Acquire) {
            return None;
        }
        if let Ok(batch) = queues.safety.try_recv() {
            return Some(Outbound::Priority(batch));
        }
        if let Ok(batch) = queues.configuration.try_recv() {
            return Some(Outbound::Priority(batch));
        }
        let now = Instant::now();
        let earliest = queues
            .periodic
            .lock()
            .values()
            .filter_map(|control| control.next_tick(now).map(|due| (due, control.clone())))
            .min_by_key(|(due, _)| *due);
        let idle_timeout = Duration::from_millis(20);
        let timeout = match earliest {
            Some((due, control)) if due <= now => {
                if let Some(batch) = control.take_due(now) {
                    return Some(Outbound::Periodic(control, batch));
                }
                continue;
            }
            Some((due, _)) => idle_timeout.min(due.duration_since(now)),
            None => idle_timeout,
        };
        select_biased! {
            recv(queues.safety) -> batch => return batch.ok().map(Outbound::Priority),
            recv(queues.configuration) -> batch => return batch.ok().map(Outbound::Priority),
            recv(queues.control) -> batch => return batch.ok().map(Outbound::Control),
            recv(queues.wake) -> _ => {},
            default(timeout) => {}
        }
    }
}

#[cfg(test)]
mod tests {
    use std::sync::atomic::AtomicU64;
    use std::sync::mpsc;
    use std::thread;

    use crossbeam_channel::bounded;

    use super::*;
    use crate::domain::GroupName;

    fn test_bus_spec() -> BusSpec {
        let config: crate::config::BusConfig = serde_yaml::from_str(
            "name: can\ntransport: { type: socketcan, interface: vcan0 }\nbitrate: 1000000\nmaximum_utilization: 1.0",
        )
        .unwrap();
        BusSpec::try_from(&config).unwrap()
    }

    fn periodic_fixture(valid_for: Duration) -> (BusSpec, PeriodicControl, Arc<PeriodicBatch>) {
        let spec = test_bus_spec();
        let control = PeriodicControl::new(
            crate::domain::GroupName::from("group"),
            spec.name.clone(),
            Duration::from_millis(10),
            Arc::new(AtomicU64::new(1)),
            Arc::new(AtomicBool::new(false)),
            bounded(1).0,
        );
        control.publish(
            vec![
                CanFrame {
                    id: 1,
                    data: [0; 8],
                },
                CanFrame {
                    id: 2,
                    data: [0; 8],
                },
            ],
            Instant::now() + valid_for,
            1,
        );
        let batch = control.take_due(Instant::now()).unwrap();
        (spec, control, batch)
    }

    #[test]
    fn scheduling_stall_during_a_burst_cancels_remaining_frames_and_latches_fault() {
        let (spec, control, batch) = periodic_fixture(Duration::from_secs(1));
        let mut written = Vec::new();
        assert!(
            !transmit_periodic_batch(&spec, &control, &batch, |frame| {
                written.push(frame.id);
                thread::sleep(Duration::from_millis(45));
                Ok(())
            })
            .unwrap()
        );
        assert_eq!(written, vec![1]);
        assert!(control.failure(Instant::now()).is_some());
    }

    #[test]
    fn periodic_revocation_interrupts_an_inflight_burst_without_resuming_old_frames() {
        let (spec, control, batch) = periodic_fixture(Duration::from_secs(1));
        let mut written = Vec::new();
        assert!(
            !transmit_periodic_batch(&spec, &control, &batch, |frame| {
                written.push(frame.id);
                control.revoke();
                Ok(())
            })
            .unwrap()
        );
        assert_eq!(written, vec![1]);
        assert!(control.next_tick(Instant::now()).is_none());
    }

    #[test]
    fn periodic_write_failure_after_partial_transmission_is_not_hidden() {
        let (spec, control, batch) = periodic_fixture(Duration::from_secs(1));
        let result = transmit_periodic_batch(&spec, &control, &batch, |frame| {
            if frame.id == 2 {
                Err(WriteFailure::fatal("injected CAN write failure"))
            } else {
                Ok(())
            }
        });
        assert_eq!(result, Err("injected CAN write failure".into()));
    }

    #[test]
    fn transient_queue_pressure_retries_only_the_rejected_frame() {
        let (spec, control, batch) = periodic_fixture(Duration::from_secs(1));
        let mut attempted = Vec::new();
        assert!(
            transmit_periodic_batch(&spec, &control, &batch, |frame| {
                attempted.push(frame.id);
                if attempted.len() == 2 {
                    Err(WriteFailure {
                        queue_full: true,
                        detail: "injected queue pressure".into(),
                    })
                } else {
                    Ok(())
                }
            })
            .unwrap()
        );
        assert_eq!(attempted, vec![1, 2, 2]);
    }

    #[test]
    fn revocation_during_queue_pressure_prevents_retry() {
        let (spec, control, batch) = periodic_fixture(Duration::from_secs(1));
        let mut attempted = Vec::new();
        assert!(
            !transmit_periodic_batch(&spec, &control, &batch, |frame| {
                attempted.push(frame.id);
                control.revoke();
                Err(WriteFailure {
                    queue_full: true,
                    detail: "injected queue pressure".into(),
                })
            })
            .unwrap()
        );
        assert_eq!(attempted, vec![1]);
    }

    #[test]
    fn persistent_queue_pressure_has_a_bounded_shared_budget() {
        let mut retry_until = None;
        let frame = CanFrame {
            id: 1,
            data: [0; 8],
        };
        let expires = Instant::now() + Duration::from_secs(1);
        let result = write_with_backpressure(
            frame,
            expires,
            &mut retry_until,
            |_| true,
            |_| {
                Err(WriteFailure {
                    queue_full: true,
                    detail: "injected queue pressure".into(),
                })
            },
        );
        assert!(result.unwrap_err().contains("queue stayed full"));
        let original_deadline = retry_until;
        let mut attempts = 0;
        let result = write_with_backpressure(
            frame,
            expires,
            &mut retry_until,
            |_| true,
            |_| {
                attempts += 1;
                Err(WriteFailure {
                    queue_full: true,
                    detail: "injected queue pressure".into(),
                })
            },
        );
        assert!(result.is_err());
        assert_eq!(attempts, 1);
        assert_eq!(retry_until, original_deadline);
    }

    #[test]
    fn expiry_during_queue_pressure_prevents_retry() {
        let mut retry_until = None;
        let frame = CanFrame {
            id: 1,
            data: [0; 8],
        };
        let expires = Instant::now() + Duration::from_millis(1);
        let mut attempts = 0;
        let result = write_with_backpressure(
            frame,
            expires,
            &mut retry_until,
            |_| true,
            |_| {
                attempts += 1;
                thread::sleep(Duration::from_millis(2));
                Err(WriteFailure {
                    queue_full: true,
                    detail: "injected queue pressure".into(),
                })
            },
        );
        assert!(result.is_err());
        assert_eq!(attempts, 1);
    }

    #[test]
    fn periodic_expiry_after_partial_transmission_is_an_error() {
        let (spec, control, batch) = periodic_fixture(Duration::from_millis(5));
        let result = transmit_periodic_batch(&spec, &control, &batch, |_| {
            thread::sleep(Duration::from_millis(10));
            Ok(())
        });
        assert!(result.unwrap_err().contains("partial transmission"));
    }

    #[test]
    fn safety_preempts_due_periodic_targets_and_shared_bus_groups_keep_separate_slots() {
        let (spec, first, _) = periodic_fixture(Duration::from_secs(1));
        let second = PeriodicControl::new(
            crate::domain::GroupName::from("other"),
            spec.name,
            Duration::from_millis(10),
            Arc::new(AtomicU64::new(1)),
            Arc::new(AtomicBool::new(false)),
            bounded(1).0,
        );
        second.publish(
            vec![CanFrame {
                id: 3,
                data: [0; 8],
            }],
            Instant::now() + Duration::from_secs(1),
            1,
        );
        first.revoke();
        let (safety_tx, safety) = bounded(1);
        // Keep the producer alive; receiver disconnection would end the worker.
        let (done, _) = mpsc::sync_channel(1);
        safety_tx
            .send(PriorityBatch {
                kind: PriorityBatchKind::SafetyBarrier,
                frames: Vec::new(),
                expires_at: Instant::now() + Duration::from_secs(1),
                completion: done,
            })
            .unwrap();
        let queues = BusTxQueues {
            safety,
            configuration: crossbeam_channel::never(),
            control: crossbeam_channel::never(),
            periodic: Arc::new(parking_lot::Mutex::new(std::collections::BTreeMap::from([
                (crate::domain::GroupName::from("group"), first),
                (crate::domain::GroupName::from("other"), second),
            ]))),
            wake: crossbeam_channel::never(),
        };
        assert!(matches!(
            receive_prioritized(&queues, &AtomicBool::new(false), &AtomicBool::new(false)),
            Some(Outbound::Priority(_))
        ));
        assert!(
            matches!(receive_prioritized(&queues, &AtomicBool::new(false), &AtomicBool::new(false)),
            Some(Outbound::Periodic(_, batch)) if batch.frames[0].id == 3)
        );
    }

    #[test]
    fn healthy_periodic_group_writes_when_the_earliest_group_stalls() {
        let spec = test_bus_spec();
        let period = Duration::from_secs(1);
        let expires_at = Instant::now() + Duration::from_secs(10);
        let (wake_tx, wake) = bounded(1);
        let mut periodic = std::collections::BTreeMap::new();
        for (name, id) in [("earliest", 1), ("healthy", 2)] {
            let group = GroupName::from(name);
            let control = PeriodicControl::new(
                group.clone(),
                spec.name.clone(),
                period,
                Arc::new(AtomicU64::new(1)),
                Arc::new(AtomicBool::new(false)),
                wake_tx.clone(),
            );
            control.publish(vec![CanFrame { id, data: [0; 8] }], expires_at, 1);
            periodic.insert(group, control);
        }
        let earliest = periodic[&GroupName::from("earliest")].clone();
        earliest.sent(Instant::now() - period * 5);
        wake.try_recv().unwrap();

        let (control_tx, control) = bounded(1);
        control_tx
            .send(ControlBatch {
                frames: vec![CanFrame {
                    id: 3,
                    data: [0; 8],
                }],
                expires_at,
            })
            .unwrap();
        let queues = BusTxQueues {
            safety: crossbeam_channel::never(),
            configuration: crossbeam_channel::never(),
            control,
            periodic: Arc::new(parking_lot::Mutex::new(periodic)),
            wake,
        };
        let selected =
            receive_prioritized(&queues, &AtomicBool::new(false), &AtomicBool::new(false));
        let now = Instant::now();
        assert!(earliest.next_tick(now).is_none());
        assert!(earliest.failure(now).is_some());
        let Some(Outbound::Periodic(control, batch)) = selected else {
            panic!("the due periodic group should be selected before queued control traffic");
        };
        let mut written = Vec::new();
        assert!(
            transmit_periodic_batch(&spec, &control, &batch, |frame| {
                written.push(frame.id);
                Ok(())
            })
            .unwrap()
        );
        assert_eq!(written, [2]);
    }

    #[test]
    fn reconnect_discards_control_and_rejects_priority_batches() {
        let (control_tx, control_rx) = bounded(1);
        let (safety_tx, safety_rx) = bounded(1);
        let (configuration_tx, configuration_rx) = bounded(1);
        let (safety_completion, safety_result) = mpsc::sync_channel(1);
        let (configuration_completion, configuration_result) = mpsc::sync_channel(1);
        control_tx
            .send(ControlBatch {
                frames: vec![CanFrame {
                    id: 1,
                    data: [0; 8],
                }],
                expires_at: Instant::now() + Duration::from_secs(1),
            })
            .unwrap();
        safety_tx
            .send(PriorityBatch {
                kind: PriorityBatchKind::SafetyBarrier,
                frames: vec![CanFrame {
                    id: 2,
                    data: [0; 8],
                }],
                expires_at: Instant::now() + Duration::from_secs(1),
                completion: safety_completion,
            })
            .unwrap();
        configuration_tx
            .send(PriorityBatch {
                kind: PriorityBatchKind::Configuration,
                frames: vec![CanFrame {
                    id: 3,
                    data: [0; 8],
                }],
                expires_at: Instant::now() + Duration::from_secs(1),
                completion: configuration_completion,
            })
            .unwrap();
        let queues = BusTxQueues {
            safety: safety_rx,
            configuration: configuration_rx,
            control: control_rx,
            periodic: Default::default(),
            wake: crossbeam_channel::never(),
        };
        reject_queued(&queues, "reconnecting");
        assert!(queues.control.is_empty());
        assert_eq!(
            safety_result.recv().unwrap(),
            Err(BatchFailure::Transmit("reconnecting".into()))
        );
        assert_eq!(
            configuration_result.recv().unwrap(),
            Err(BatchFailure::Transmit("reconnecting".into()))
        );
    }

    #[test]
    fn safety_batches_preempt_configuration_and_control() {
        let (safety_tx, safety_rx) = bounded(1);
        let (configuration_tx, configuration_rx) = bounded(1);
        let (control_tx, control_rx) = bounded(1);
        let (safety_completion, _) = mpsc::sync_channel(1);
        let (configuration_completion, _) = mpsc::sync_channel(1);
        let deadline = Instant::now() + Duration::from_secs(1);
        control_tx
            .send(ControlBatch {
                frames: Vec::new(),
                expires_at: deadline,
            })
            .unwrap();
        configuration_tx
            .send(PriorityBatch {
                kind: PriorityBatchKind::Configuration,
                frames: Vec::new(),
                expires_at: deadline,
                completion: configuration_completion,
            })
            .unwrap();
        safety_tx
            .send(PriorityBatch {
                kind: PriorityBatchKind::SafetyBarrier,
                frames: Vec::new(),
                expires_at: deadline,
                completion: safety_completion,
            })
            .unwrap();

        let shutdown = AtomicBool::new(false);
        let session_stop = AtomicBool::new(false);
        let queues = BusTxQueues {
            safety: safety_rx,
            configuration: configuration_rx,
            control: control_rx,
            periodic: Default::default(),
            wake: crossbeam_channel::never(),
        };
        assert!(matches!(
            receive_prioritized(&queues, &shutdown, &session_stop),
            Some(Outbound::Priority(PriorityBatch {
                kind: PriorityBatchKind::SafetyBarrier,
                ..
            }))
        ));
    }

    #[test]
    fn safety_barriers_purge_control_but_configuration_batches_do_not() {
        let (control_tx, control_rx) = bounded(2);
        for id in [1, 2] {
            control_tx
                .send(ControlBatch {
                    frames: vec![CanFrame { id, data: [0; 8] }],
                    expires_at: Instant::now() + Duration::from_secs(1),
                })
                .unwrap();
        }
        apply_priority_ordering(PriorityBatchKind::Configuration, &control_rx);
        assert_eq!(control_rx.len(), 2);
        apply_priority_ordering(PriorityBatchKind::SafetyBarrier, &control_rx);
        assert!(control_rx.is_empty());
    }

    #[test]
    fn a_late_control_batch_reports_how_long_its_writes_took() {
        let transmit = |frame_count: u32, write_time: Duration| {
            let now = std::cell::Cell::new(Instant::now());
            let batch = ControlBatch {
                frames: (1..=frame_count)
                    .map(|id| CanFrame { id, data: [0; 8] })
                    .collect(),
                expires_at: now.get() + Duration::from_millis(10),
            };
            let mut written = Vec::new();
            let result = write_control_batch(
                batch,
                || now.get(),
                |frame| {
                    written.push(frame.id);
                    now.set(now.get() + write_time);
                    Ok(())
                },
            );
            (result, written)
        };

        let (partial, written) = transmit(7, Duration::from_millis(4));
        assert_eq!(written, [1, 2, 3]);
        assert_eq!(
            partial.unwrap_err(),
            "control batch expired after partial transmission \
             (3 of 7 frames written in 12000 us, 10000 us of freshness left at the first write)"
        );

        let (late, written) = transmit(2, Duration::from_millis(8));
        assert_eq!(written, [1, 2]);
        assert_eq!(
            late.unwrap_err(),
            "control batch completed after its freshness deadline \
             (2 frames written in 16000 us, 10000 us of freshness left at the first write)"
        );

        let (on_time, written) = transmit(2, Duration::from_millis(5));
        assert_eq!(on_time, Ok(true));
        assert_eq!(written, [1, 2]);
    }

    #[test]
    fn freshness_is_checked_for_the_entire_control_batch() {
        let config: crate::config::BusConfig = serde_yaml::from_str(
            "name: can\ntransport: { type: socketcan, interface: vcan0 }\nbitrate: 10000\nmaximum_utilization: 1.0",
        )
        .unwrap();
        let spec = BusSpec::try_from(&config).unwrap();

        assert!(!control_batch_can_finish(
            &spec,
            25,
            Instant::now() + Duration::from_millis(100)
        ));
        assert!(control_batch_can_finish(
            &spec,
            25,
            Instant::now() + Duration::from_secs(1)
        ));
    }
}
