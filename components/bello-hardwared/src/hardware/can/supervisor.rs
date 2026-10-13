use std::io;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::{Duration, Instant};

use anyhow::{Context, Result};
use crossbeam_channel::{Receiver, Sender, bounded, select_biased};
use nix::errno::Errno;
use socketcan::id::CAN_EFF_FLAG;
use socketcan::{
    CanFrame as WireFrame, CanSocket, EmbeddedFrame, ExtendedId, Frame, Socket, SocketOptions,
};
use tracing::{error, warn};

use super::PeriodicControl;
use super::periodic::PeriodicBatch;
use super::{
    BatchFailure, BusSpec, BusTxQueues, CanFrame, ControlBatch, PriorityBatch, PriorityBatchKind,
    ReceivedFrame,
};
use crate::domain::BusName;
use crate::hardware::lifecycle::{DeviceLifecycle, wait_for_stop};

enum Outbound {
    Control(ControlBatch),
    Priority(PriorityBatch),
    Periodic(PeriodicControl, Arc<PeriodicBatch>),
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
    let mut failures = 0_u32;
    lifecycle.mark_connecting(format!("opening SocketCAN interface {}", spec.interface));
    while !shutdown.load(Ordering::Acquire) {
        match open_sockets(&spec) {
            Ok((writer, reader)) => {
                failures = 0;
                lifecycle.mark_online(format!("SocketCAN interface {} online", spec.interface));
                if let Err(detail) = run_session(writer, reader, &spec, &queues, &frames, &shutdown)
                {
                    if shutdown.load(Ordering::Acquire) {
                        break;
                    }
                    prepare_reconnect(&spec, &lifecycle, &queues, detail, &shutdown, &mut failures);
                }
            }
            Err(error) => prepare_reconnect(
                &spec,
                &lifecycle,
                &queues,
                error.to_string(),
                &shutdown,
                &mut failures,
            ),
        }
    }
    reject_queued(&queues, "CAN supervisor stopped");
    lifecycle.shutdown("CAN supervisor stopped");
}

fn open_sockets(spec: &BusSpec) -> Result<(CanSocket, CanSocket)> {
    let writer = CanSocket::open(&spec.interface).with_context(|| {
        format!(
            "opening SocketCAN writer {} ({})",
            spec.name, spec.interface
        )
    })?;
    writer.set_write_timeout(Duration::from_millis(20))?;
    let reader = CanSocket::open(&spec.interface).with_context(|| {
        format!(
            "opening SocketCAN reader {} ({})",
            spec.name, spec.interface
        )
    })?;
    reader
        .set_filters(&[(CAN_EFF_FLAG, CAN_EFF_FLAG)])
        .with_context(|| {
            format!(
                "restricting SocketCAN reader {} ({}) to extended frames",
                spec.name, spec.interface
            )
        })?;
    reader.set_read_timeout(Duration::from_millis(100))?;
    Ok((writer, reader))
}

fn run_session(
    writer: CanSocket,
    reader: CanSocket,
    spec: &BusSpec,
    queues: &BusTxQueues,
    frames: &Sender<ReceivedFrame>,
    shutdown: &Arc<AtomicBool>,
) -> Result<(), String> {
    let session_stop = Arc::new(AtomicBool::new(false));
    let (failure_tx, failure_rx) = bounded::<String>(2);

    let tx_queues = queues.clone();
    let tx_shutdown = shutdown.clone();
    let tx_stop = session_stop.clone();
    let tx_failure = failure_tx.clone();
    let tx_spec = spec.clone();
    let tx_bus = spec.name.to_string();
    let tx = thread::Builder::new()
        .name(format!("{tx_bus}-tx"))
        .spawn(move || {
            run_tx(
                writer,
                &tx_spec,
                &tx_queues,
                &tx_shutdown,
                &tx_stop,
                &tx_failure,
            )
        })
        .map_err(|error| error.to_string())?;

    let rx_shutdown = shutdown.clone();
    let rx_stop = session_stop.clone();
    let rx_failure = failure_tx;
    let rx_bus = spec.name.clone();
    let rx_frames = frames.clone();
    let rx = match thread::Builder::new()
        .name(format!("{rx_bus}-rx"))
        .spawn(move || {
            run_rx(
                reader,
                &rx_bus,
                &rx_frames,
                &rx_shutdown,
                &rx_stop,
                &rx_failure,
            )
        }) {
        Ok(thread) => thread,
        Err(error) => {
            session_stop.store(true, Ordering::Release);
            let _ = tx.join();
            return Err(error.to_string());
        }
    };

    let result = loop {
        if shutdown.load(Ordering::Acquire) {
            break Ok(());
        }
        match failure_rx.recv_timeout(Duration::from_millis(20)) {
            Ok(error) => break Err(error),
            Err(crossbeam_channel::RecvTimeoutError::Timeout) => {}
            Err(crossbeam_channel::RecvTimeoutError::Disconnected) => {
                break Err("CAN session workers disconnected".into());
            }
        }
    };
    session_stop.store(true, Ordering::Release);
    if tx.join().is_err() || rx.join().is_err() {
        return Err("CAN session worker panicked".into());
    }
    result
}

fn run_tx(
    writer: CanSocket,
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
                    write_frame(&writer, frame)
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
    mut write: impl FnMut(CanFrame) -> io::Result<()>,
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

fn transmit_control_batch(
    writer: &CanSocket,
    spec: &BusSpec,
    batch: ControlBatch,
) -> Result<bool, String> {
    if !control_batch_can_finish(spec, batch.frames.len(), batch.expires_at) {
        return Ok(false);
    }
    let mut retry_until = None;
    for (index, frame) in batch.frames.into_iter().enumerate() {
        if Instant::now() >= batch.expires_at {
            return if index == 0 {
                Ok(false)
            } else {
                Err("control batch expired after partial transmission".into())
            };
        }
        write_with_backpressure(
            frame,
            batch.expires_at,
            &mut retry_until,
            |_| true,
            |frame| write_frame(writer, frame),
        )?;
    }
    if Instant::now() > batch.expires_at {
        Err("control batch completed after its freshness deadline".into())
    } else {
        Ok(true)
    }
}

fn transmit_priority_batch(
    writer: &CanSocket,
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
            |frame| write_frame(writer, frame),
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
    mut write: impl FnMut(CanFrame) -> io::Result<()>,
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
            Err(error) if error.raw_os_error() == Some(Errno::ENOBUFS as i32) => {
                // SocketCAN reports a full device queue immediately, even with
                // SO_SNDTIMEO. Share one short retry budget across this burst.
                let now = Instant::now();
                let deadline =
                    (*retry_until.get_or_insert(now + BACKPRESSURE_LIMIT)).min(expires_at);
                if now >= deadline {
                    return Err(format!("CAN transmit queue stayed full: {error}"));
                }
                retrying = true;
                thread::sleep(BACKPRESSURE_POLL.min(deadline - now));
            }
            Err(error) => return Err(format!("CAN write failed: {error}")),
        }
    }
}

fn write_frame(writer: &CanSocket, frame: CanFrame) -> io::Result<()> {
    let id = ExtendedId::new(frame.id)
        .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidInput, "invalid extended CAN id"))?;
    let frame = WireFrame::new(id, &frame.data).ok_or_else(|| {
        io::Error::new(io::ErrorKind::InvalidInput, "could not construct CAN frame")
    })?;
    writer.write_frame(&frame)
}

fn run_rx(
    reader: CanSocket,
    bus: &BusName,
    frames: &Sender<ReceivedFrame>,
    shutdown: &AtomicBool,
    session_stop: &AtomicBool,
    failure: &Sender<String>,
) {
    while !shutdown.load(Ordering::Acquire) && !session_stop.load(Ordering::Acquire) {
        let wire = match reader.read_frame() {
            Ok(frame) => frame,
            Err(error)
                if matches!(
                    error.kind(),
                    std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut
                ) =>
            {
                continue;
            }
            Err(error) => {
                let _ = failure.try_send(format!("CAN read failed: {error}"));
                break;
            }
        };
        let Some(received) = decode_received(bus, &wire) else {
            warn!(bus=%bus, dlc=wire.data().len(), "discarding CAN frame with invalid payload length");
            continue;
        };
        if frames.try_send(received).is_err() {
            let _ = failure.try_send("CAN receive queue is full".into());
            break;
        }
    }
}

fn decode_received(bus: &BusName, wire: &WireFrame) -> Option<ReceivedFrame> {
    if !wire.is_extended() {
        return None;
    }
    Some(ReceivedFrame {
        bus: bus.clone(),
        frame: CanFrame {
            id: wire.raw_id(),
            data: wire.data().try_into().ok()?,
        },
        received: Instant::now(),
    })
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
        lifecycle.mark_connecting(format!("reopening SocketCAN interface {}", spec.interface));
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
        let mut timeout = Duration::from_millis(20);
        if let Some((due, control)) = earliest {
            if let Some(batch) = control.take_due(now) {
                return Some(Outbound::Periodic(control, batch));
            }
            // A revoked/stalled slot has no next tick. Avoid spinning on it.
            if control.next_tick(now).is_some() {
                timeout = timeout.min(due.saturating_duration_since(now));
            }
        }
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

    use super::*;

    fn periodic_fixture(valid_for: Duration) -> (BusSpec, PeriodicControl, Arc<PeriodicBatch>) {
        let config: crate::config::BusConfig = serde_yaml::from_str(
            "name: can\ninterface: vcan0\nbitrate: 1000000\nmaximum_utilization: 1.0",
        )
        .unwrap();
        let spec = BusSpec::try_from(&config).unwrap();
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
                Err(io::Error::other("injected CAN write failure"))
            } else {
                Ok(())
            }
        });
        assert_eq!(
            result,
            Err("CAN write failed: injected CAN write failure".into())
        );
    }

    #[test]
    fn transient_queue_pressure_retries_only_the_rejected_frame() {
        let (spec, control, batch) = periodic_fixture(Duration::from_secs(1));
        let mut attempted = Vec::new();
        assert!(
            transmit_periodic_batch(&spec, &control, &batch, |frame| {
                attempted.push(frame.id);
                if attempted.len() == 2 {
                    Err(io::Error::from_raw_os_error(Errno::ENOBUFS as i32))
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
                Err(io::Error::from_raw_os_error(Errno::ENOBUFS as i32))
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
            |_| Err(io::Error::from_raw_os_error(Errno::ENOBUFS as i32)),
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
                Err(io::Error::from_raw_os_error(Errno::ENOBUFS as i32))
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
                Err(io::Error::from_raw_os_error(Errno::ENOBUFS as i32))
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
    fn freshness_is_checked_for_the_entire_control_batch() {
        let config: crate::config::BusConfig = serde_yaml::from_str(
            "name: can\ninterface: vcan0\nbitrate: 10000\nmaximum_utilization: 1.0",
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

    #[test]
    fn short_can_payload_is_rejected() {
        let id = ExtendedId::new(1).unwrap();
        let wire = WireFrame::new(id, &[1, 2, 3]).unwrap();
        assert!(decode_received(&BusName::from("can"), &wire).is_none());
    }

    #[test]
    fn standard_can_frame_is_not_routed_to_robstride() {
        let id = socketcan::StandardId::new(0x27).unwrap();
        let wire = WireFrame::new(id, &[0x01, 1, 2, 3, 4, 5, 6]).unwrap();
        assert!(decode_received(&BusName::from("can"), &wire).is_none());
    }
}
