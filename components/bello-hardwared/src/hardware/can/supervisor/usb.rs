use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::{Duration, Instant};

use crossbeam_channel::{Receiver, Sender, TryRecvError, bounded};
use gsusbio::{Bitrate, CanFrame as UsbFrame, CanId, CanSender, CanSocket, ReceiveEvent};
use tokio::runtime::{Builder, Handle};
use tracing::warn;

use super::{
    BusSpec, BusTxQueues, CanFrame, FrameWriter, ReceivedFrame, WriteFailure, prepare_reconnect,
    reject_queued, run_tx,
};
use crate::config::BusTransportConfig;
use crate::domain::BusName;
use crate::hardware::lifecycle::DeviceLifecycle;

const TRANSFER_TIMEOUT: Duration = Duration::from_millis(50);

pub(super) fn run(
    spec: BusSpec,
    queues: BusTxQueues,
    frames: Sender<ReceivedFrame>,
    lifecycle: DeviceLifecycle,
    shutdown: Arc<AtomicBool>,
) {
    let runtime = match Builder::new_current_thread().enable_all().build() {
        Ok(runtime) => runtime,
        Err(error) => {
            lifecycle.enter_fault(format!("creating GS-USB runtime: {error}"));
            reject_queued(&queues, "GS-USB runtime could not start");
            return;
        }
    };
    let mut failures = 0;
    lifecycle.mark_connecting(format!("opening {}", spec.transport.label()));
    while !shutdown.load(Ordering::Acquire) {
        let result = runtime.block_on(async {
            let socket = connect(&spec).await?;
            failures = 0;
            lifecycle.mark_online(format!("{} online", spec.transport.label()));
            run_session(socket, &spec, &queues, &frames, &shutdown).await
        });
        if shutdown.load(Ordering::Acquire) {
            break;
        }
        if let Err(detail) = result {
            prepare_reconnect(&spec, &lifecycle, &queues, detail, &shutdown, &mut failures);
        }
    }
    reject_queued(&queues, "GS-USB supervisor stopped");
    lifecycle.shutdown("GS-USB supervisor stopped");
}

async fn connect(spec: &BusSpec) -> Result<CanSocket, String> {
    let BusTransportConfig::Gsusb { serial, channel } = &spec.transport else {
        unreachable!("GS-USB supervisor requires a GS-USB transport")
    };
    let mut selected = None;
    for device in CanSocket::scan().await.map_err(|error| error.to_string())? {
        let found = match device.serial_number().await {
            Ok(found) => found,
            Err(error) => {
                warn!(%error, "could not read GS-USB device serial number");
                continue;
            }
        };
        if found.as_deref() == Some(serial.as_str()) && selected.replace(device).is_some() {
            return Err(format!("more than one GS-USB device has serial {serial}"));
        }
    }
    let device = selected.ok_or_else(|| format!("GS-USB device {serial} was not found"))?;
    CanSocket::builder(device)
        .channel(*channel)
        .nominal_bitrate(Bitrate::Custom(spec.bitrate))
        .command_timeout(TRANSFER_TIMEOUT)
        .connect()
        .await
        .map_err(|error| format!("connecting GS-USB device {serial}: {error}"))
}

struct UsbWriter {
    sender: CanSender,
    runtime: Handle,
}

impl FrameWriter for UsbWriter {
    fn write_frame(&self, frame: CanFrame) -> Result<(), WriteFailure> {
        let wire = encode_frame(frame).map_err(WriteFailure::fatal)?;
        self.runtime
            .block_on(self.sender.send(wire))
            .map_err(|error| WriteFailure::fatal(format!("GS-USB write failed: {error}")))
    }
}

fn encode_frame(frame: CanFrame) -> Result<UsbFrame, String> {
    let id = CanId::extended(frame.id).map_err(|error| error.to_string())?;
    UsbFrame::classic(id, &frame.data).map_err(|error| error.to_string())
}

fn decode_frame(bus: &BusName, frame: UsbFrame) -> Option<ReceivedFrame> {
    if frame.is_remote() || frame.is_fd() || !frame.id().is_extended() {
        return None;
    }
    Some(ReceivedFrame {
        bus: bus.clone(),
        frame: CanFrame {
            id: frame.id().value(),
            data: frame.data().try_into().ok()?,
        },
        received: Instant::now(),
    })
}

fn check_tx_failure(failures: &Receiver<String>) -> Result<(), String> {
    match failures.try_recv() {
        Ok(detail) => Err(detail),
        Err(TryRecvError::Disconnected) => Err("GS-USB transmit worker exited".into()),
        Err(TryRecvError::Empty) => Ok(()),
    }
}

async fn run_session(
    socket: CanSocket,
    spec: &BusSpec,
    queues: &BusTxQueues,
    frames: &Sender<ReceivedFrame>,
    shutdown: &Arc<AtomicBool>,
) -> Result<(), String> {
    let (sender, mut receiver) = socket.split();
    let session_stop = Arc::new(AtomicBool::new(false));
    let (failure_tx, failure_rx) = bounded(2);
    let writer = UsbWriter {
        sender: sender.clone(),
        runtime: Handle::current(),
    };
    let tx_spec = spec.clone();
    let tx_queues = queues.clone();
    let tx_shutdown = shutdown.clone();
    let tx_stop = session_stop.clone();
    let tx = thread::Builder::new()
        .name(format!("{}-usb-tx", spec.name))
        .spawn(move || {
            run_tx(
                writer,
                &tx_spec,
                &tx_queues,
                &tx_shutdown,
                &tx_stop,
                &failure_tx,
            )
        })
        .map_err(|error| error.to_string())?;
    let mut ticker = tokio::time::interval(Duration::from_millis(20));
    let result = loop {
        tokio::select! {
            _ = ticker.tick() => {
                if shutdown.load(Ordering::Acquire) {
                    break Ok(());
                }
                let stats = receiver.stats();
                if stats.usb_errors > 0 {
                    break Err("GS-USB transfer failed".into());
                }
                if stats.rx_dropped > 0 || stats.malformed_frames > 0 {
                    break Err(format!(
                        "GS-USB receive loss (dropped: {}, malformed: {})",
                        stats.rx_dropped, stats.malformed_frames
                    ));
                }
                if let Err(detail) = check_tx_failure(&failure_rx) {
                    break Err(detail);
                }
            }
            event = receiver.recv_event() => {
                match event {
                    Ok(Some(ReceiveEvent::Frame(received))) => {
                        let Some(frame) = decode_frame(&spec.name, received.frame) else {
                            warn!(bus=%spec.name, "discarding unsupported GS-USB CAN frame");
                            continue;
                        };
                        if frames.try_send(frame).is_err() {
                            break Err("CAN receive queue is full".into());
                        }
                    }
                    Ok(Some(ReceiveEvent::ErrorFrame { can_id, .. })) => {
                        break Err(format!("GS-USB CAN error frame {can_id:#x}"));
                    }
                    Ok(Some(ReceiveEvent::Overflow { .. })) => {
                        break Err("GS-USB receive overflow".into());
                    }
                    Ok(None) => break Err("GS-USB receiver closed".into()),
                    Err(error) => break Err(format!("GS-USB receive failed: {error}")),
                }
            }
        }
    };
    session_stop.store(true, Ordering::Release);
    let joined = tokio::task::spawn_blocking(move || tx.join()).await;
    let _ = tokio::time::timeout(Duration::from_secs(1), sender.close()).await;
    if !matches!(joined, Ok(Ok(()))) {
        return Err("GS-USB transmit worker panicked".into());
    }
    result
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn classic_extended_frames_round_trip_through_usb_boundary() {
        let bus = BusName::from("main");
        let original = CanFrame {
            id: 0x1234567,
            data: [1, 2, 3, 4, 5, 6, 7, 8],
        };
        let wire = encode_frame(original).unwrap();
        let received = decode_frame(&bus, wire).unwrap();
        assert_eq!(received.bus, bus);
        assert_eq!(received.frame, original);
    }

    #[test]
    fn rejects_frames_outside_robstride_wire_shape() {
        let bus = BusName::from("main");
        let standard = UsbFrame::classic(CanId::standard(1).unwrap(), &[0; 8]).unwrap();
        let short = UsbFrame::classic(CanId::extended(1).unwrap(), &[0; 7]).unwrap();
        assert!(decode_frame(&bus, standard).is_none());
        assert!(decode_frame(&bus, short).is_none());
        assert!(
            encode_frame(CanFrame {
                id: 1 << 29,
                data: [0; 8]
            })
            .is_err()
        );
    }

    #[test]
    fn disconnected_transmit_worker_ends_usb_session() {
        let (failure_tx, failure_rx) = bounded(1);
        assert!(check_tx_failure(&failure_rx).is_ok());
        drop(failure_tx);
        assert_eq!(
            check_tx_failure(&failure_rx),
            Err("GS-USB transmit worker exited".into())
        );
    }
}
