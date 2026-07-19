use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::{Duration, Instant};

use anyhow::{Context, Result};
use crossbeam_channel::{Receiver, Sender, bounded, select_biased};
use socketcan::{CanFrame as WireFrame, CanSocket, EmbeddedFrame, ExtendedId, Frame, Socket};
use tracing::{error, warn};

use super::{
    BatchFailure, BusSpec, BusTxQueues, CanFrame, ControlBatch, PriorityBatch, PriorityBatchKind,
    ReceivedFrame,
};
use crate::domain::BusName;
use crate::hardware::lifecycle::{DeviceLifecycle, wait_for_stop};

enum Outbound {
    Control(ControlBatch),
    Priority(PriorityBatch),
}

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

fn transmit_control_batch(
    writer: &CanSocket,
    spec: &BusSpec,
    batch: ControlBatch,
) -> Result<bool, String> {
    if !control_batch_can_finish(spec, batch.frames.len(), batch.expires_at) {
        return Ok(false);
    }
    for (index, frame) in batch.frames.into_iter().enumerate() {
        if Instant::now() >= batch.expires_at {
            return if index == 0 {
                Ok(false)
            } else {
                Err("control batch expired after partial transmission".into())
            };
        }
        write_frame(writer, frame)?;
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
        write_frame(writer, frame).map_err(BatchFailure::Transmit)?;
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

fn write_frame(writer: &CanSocket, frame: CanFrame) -> Result<(), String> {
    let id = ExtendedId::new(frame.id).ok_or_else(|| "invalid extended CAN id".to_owned())?;
    let frame = WireFrame::new(id, &frame.data)
        .ok_or_else(|| "could not construct CAN frame".to_owned())?;
    writer
        .write_frame(&frame)
        .map_err(|error| format!("CAN write failed: {error}"))
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
        select_biased! {
            recv(queues.safety) -> batch => return batch.ok().map(Outbound::Priority),
            recv(queues.configuration) -> batch => return batch.ok().map(Outbound::Priority),
            recv(queues.control) -> batch => return batch.ok().map(Outbound::Control),
            default(Duration::from_millis(20)) => {}
        }
    }
}

#[cfg(test)]
mod tests {
    use std::sync::mpsc;

    use super::*;

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
}
