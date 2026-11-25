use std::collections::VecDeque;
use std::sync::Arc;
use std::time::{Duration, Instant};

use anyhow::{Result, anyhow, bail};
use crossbeam_channel::{Sender, bounded};
use parking_lot::{Condvar, Mutex};
use robstride_protocol::Frame;

use super::{CanFrame, ReceivedFrame};
use crate::domain::BusName;

const RECEIVE_CAPACITY: usize = 4096;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) struct BusGeneration(u64);

impl BusGeneration {
    pub(super) const INITIAL: Self = Self(0);
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct FeedbackWindow {
    since: Option<Instant>,
    generation: BusGeneration,
}

impl FeedbackWindow {
    pub(crate) const INITIAL: Self = Self {
        since: None,
        generation: BusGeneration::INITIAL,
    };

    pub(crate) fn accepts(self, received: Instant) -> bool {
        self.since.is_none_or(|since| received >= since)
    }
}

enum Reservation {
    Fencing,
    Active {
        since: Instant,
        frames: VecDeque<Frame>,
    },
    Failed(String),
}

enum Traffic {
    Ordinary(FeedbackWindow),
    Maintenance(Reservation),
}

struct State {
    generation: BusGeneration,
    connected: bool,
    // Some transports queue frames before exposing them without a host timestamp.
    received_count: Option<Box<dyn Fn() -> u64 + Send + Sync>>,
    receive_cutoff: u64,
    traffic: Traffic,
}

pub(super) struct BusAccess {
    state: Mutex<State>,
    received: Condvar,
}

impl Default for BusAccess {
    fn default() -> Self {
        Self {
            state: Mutex::new(State {
                generation: BusGeneration::INITIAL,
                connected: false,
                received_count: None,
                receive_cutoff: 0,
                traffic: Traffic::Ordinary(FeedbackWindow::INITIAL),
            }),
            received: Condvar::new(),
        }
    }
}

impl BusAccess {
    pub(super) fn ordinary_generation(&self) -> Option<BusGeneration> {
        self.ordinary_traffic().map(|(generation, _)| generation)
    }

    pub(super) fn feedback_window(&self) -> Option<FeedbackWindow> {
        self.ordinary_traffic().map(|(_, window)| window)
    }

    pub(super) fn ordinary_traffic(&self) -> Option<(BusGeneration, FeedbackWindow)> {
        let state = self.state.lock();
        match state.traffic {
            Traffic::Ordinary(window) => Some((state.generation, window)),
            Traffic::Maintenance(_) => None,
        }
    }

    pub(super) fn connected(&self, received_count: Option<Box<dyn Fn() -> u64 + Send + Sync>>) {
        let mut state = self.state.lock();
        state.connected = true;
        state.received_count = received_count;
        state.receive_cutoff = 0;
    }

    pub(super) fn disconnected(&self, detail: &str) {
        let mut state = self.state.lock();
        state.connected = false;
        state.received_count = None;
        if let Traffic::Maintenance(reservation) = &mut state.traffic
            && !matches!(reservation, Reservation::Failed(_))
        {
            *reservation = Reservation::Failed(detail.into());
            self.received.notify_all();
        }
    }

    pub(super) fn reserve(
        self: &Arc<Self>,
        bus: BusName,
        requests: Sender<MaintenanceRequest>,
    ) -> Result<MaintenanceBus> {
        let session = {
            let mut state = self.state.lock();
            if !state.connected {
                bail!("CAN bus {bus} has no active transport");
            }
            if matches!(state.traffic, Traffic::Maintenance(_)) {
                bail!("CAN bus {bus} already has a maintenance reservation");
            }
            state.generation.0 += 1;
            state.traffic = Traffic::Maintenance(Reservation::Fencing);
            MaintenanceBus {
                bus,
                access: self.clone(),
                requests,
                generation: state.generation,
                sending: Mutex::new(()),
            }
        };
        session.request(MaintenanceWork::Fence)?;
        Ok(session)
    }

    pub(super) fn activate(&self, generation: BusGeneration) {
        let mut state = self.state.lock();
        if state.generation == generation
            && matches!(state.traffic, Traffic::Maintenance(Reservation::Fencing))
        {
            state.cut_off_pending_receives();
            state.traffic = Traffic::Maintenance(Reservation::Active {
                since: Instant::now(),
                frames: VecDeque::new(),
            });
        }
    }

    pub(super) fn reject_unrepresentable_frame(
        &self,
        received: Instant,
        sequence: Option<u64>,
    ) -> Result<(), String> {
        let mut state = self.state.lock();
        if sequence.is_none_or(|sequence| sequence > state.receive_cutoff)
            && let Traffic::Maintenance(reservation @ Reservation::Active { .. }) =
                &mut state.traffic
            && matches!(reservation, Reservation::Active { since, .. } if received >= *since)
        {
            let detail = "maintenance received a frame outside the extended eight-byte CAN format";
            *reservation = Reservation::Failed(detail.into());
            self.received.notify_all();
            return Err(detail.into());
        }
        Ok(())
    }

    pub(super) fn route(
        &self,
        received: ReceivedFrame,
        ordinary: &Sender<ReceivedFrame>,
        sequence: Option<u64>,
    ) -> Result<(), String> {
        let mut state = self.state.lock();
        if sequence.is_some_and(|sequence| sequence <= state.receive_cutoff) {
            return Ok(());
        }
        match &mut state.traffic {
            Traffic::Ordinary(window) => {
                if window.accepts(received.received) {
                    ordinary
                        .try_send(received)
                        .map_err(|_| "CAN receive queue is full or disconnected")?;
                }
            }
            Traffic::Maintenance(reservation) => {
                if let Reservation::Active { since, frames } = reservation
                    && received.received >= *since
                {
                    if frames.len() >= RECEIVE_CAPACITY {
                        let detail = "maintenance CAN receive queue overflow";
                        *reservation = Reservation::Failed(detail.into());
                        self.received.notify_all();
                        return Err(detail.into());
                    }
                    frames.push_back(received.frame.into());
                    self.received.notify_one();
                }
            }
        }
        Ok(())
    }
}

impl State {
    fn cut_off_pending_receives(&mut self) {
        if let Some(received_count) = &self.received_count {
            self.receive_cutoff = received_count();
        }
    }

    fn reservation(&mut self, generation: BusGeneration) -> Result<&mut Reservation, String> {
        if self.generation == generation
            && let Traffic::Maintenance(reservation) = &mut self.traffic
        {
            return match reservation {
                Reservation::Failed(detail) => Err(detail.clone()),
                other => Ok(other),
            };
        }
        Err("maintenance reservation ended".into())
    }
}

pub(super) struct MaintenanceRequest {
    pub(super) generation: BusGeneration,
    pub(super) work: MaintenanceWork,
    pub(super) completion: Sender<Result<(), String>>,
}

#[derive(Clone, Copy)]
pub(super) enum MaintenanceWork {
    Fence,
    Write(CanFrame),
}

impl BusAccess {
    pub(super) fn check_request(&self, request: &MaintenanceRequest) -> Result<(), String> {
        self.state
            .lock()
            .reservation(request.generation)
            .and_then(|reservation| match (request.work, reservation) {
                (MaintenanceWork::Fence, Reservation::Fencing)
                | (MaintenanceWork::Write(_), Reservation::Active { .. }) => Ok(()),
                _ => Err("maintenance reservation is in the wrong phase".into()),
            })
    }
}

/// Exclusive access to one physical CAN bus. Dropping it restores ordinary traffic.
#[must_use = "keep the maintenance reservation alive while using the CAN bus"]
pub(crate) struct MaintenanceBus {
    bus: BusName,
    access: Arc<BusAccess>,
    requests: Sender<MaintenanceRequest>,
    generation: BusGeneration,
    sending: Mutex<()>,
}

impl MaintenanceBus {
    pub(crate) fn send(&self, frame: Frame) -> Result<()> {
        if frame.id > 0x1fff_ffff {
            bail!("CAN bus {}: invalid extended CAN ID", self.bus);
        }
        let _sending = self.sending.lock();
        self.request(MaintenanceWork::Write(frame.into()))
    }

    pub(crate) fn recv(&self, timeout: Duration) -> Result<Option<Frame>> {
        let started = Instant::now();
        let mut state = self.access.state.lock();
        loop {
            if let Reservation::Active { frames, .. } = state
                .reservation(self.generation)
                .map_err(|detail| self.error(detail))?
                && let Some(frame) = frames.pop_front()
            {
                return Ok(Some(frame));
            }
            let remaining = timeout.saturating_sub(started.elapsed());
            if remaining.is_zero() {
                return Ok(None);
            }
            self.access.received.wait_for(&mut state, remaining);
        }
    }

    fn request(&self, work: MaintenanceWork) -> Result<()> {
        let (completion, result) = bounded(1);
        {
            let mut state = self.access.state.lock();
            state
                .reservation(self.generation)
                .map_err(|detail| self.error(detail))?;
            self.requests
                .try_send(MaintenanceRequest {
                    generation: self.generation,
                    work,
                    completion,
                })
                .map_err(|_| self.error("maintenance transmit worker unavailable"))?;
        }

        result
            .recv()
            .map_err(|_| self.error("maintenance transmit worker disconnected"))?
            .map_err(|detail| self.error(detail))?;
        self.access
            .state
            .lock()
            .reservation(self.generation)
            .map_err(|detail| self.error(detail))?;
        Ok(())
    }

    fn error(&self, detail: impl std::fmt::Display) -> anyhow::Error {
        anyhow!("CAN bus {}: {detail}", self.bus)
    }
}

impl Drop for MaintenanceBus {
    fn drop(&mut self) {
        let mut state = self.access.state.lock();
        state.cut_off_pending_receives();
        state.traffic = Traffic::Ordinary(FeedbackWindow {
            since: Some(Instant::now()),
            generation: self.generation,
        });
    }
}

#[cfg(test)]
mod tests {
    use std::thread;

    use super::*;

    fn reserve_on_connected_bus(access: &Arc<BusAccess>) -> MaintenanceBus {
        let (requests, pending) = bounded::<MaintenanceRequest>(1);
        thread::scope(|scope| {
            scope.spawn(|| {
                let request = pending.recv().unwrap();
                assert!(matches!(request.work, MaintenanceWork::Fence));
                access.activate(request.generation);
                request.completion.send(Ok(())).unwrap();
            });
            access.reserve(BusName::from("test"), requests).unwrap()
        })
    }

    #[test]
    fn receive_overflow_invalidates_buffered_frames_and_writes() {
        let access = Arc::new(BusAccess::default());
        access.connected(None);
        let bus = reserve_on_connected_bus(&access);
        let (ordinary, monitor) = bounded(1);
        let frame = ReceivedFrame {
            bus: BusName::from("test"),
            frame: CanFrame {
                id: 1,
                data: [1; 8],
            },
            received: Instant::now(),
        };
        for _ in 0..RECEIVE_CAPACITY {
            access.route(frame.clone(), &ordinary, None).unwrap();
        }
        assert!(access.route(frame.clone(), &ordinary, None).is_err());
        assert!(
            bus.recv(Duration::ZERO)
                .unwrap_err()
                .to_string()
                .contains("overflow")
        );
        assert!(
            bus.send(frame.frame.into())
                .unwrap_err()
                .to_string()
                .contains("overflow")
        );
        assert!(monitor.is_empty());

        drop(bus);
        access
            .route(
                ReceivedFrame {
                    received: Instant::now(),
                    ..frame
                },
                &ordinary,
                None,
            )
            .unwrap();
        assert_eq!(monitor.try_recv().unwrap().frame.id, 1);
    }

    #[test]
    fn disconnect_wakes_receivers_and_reconnect_cannot_reuse_the_reservation() {
        let access = Arc::new(BusAccess::default());
        access.connected(None);
        let bus = reserve_on_connected_bus(&access);

        thread::scope(|scope| {
            let (started, waiting) = bounded(1);
            let active = &bus;
            let result = scope.spawn(move || {
                started.send(()).unwrap();
                active.recv(Duration::from_secs(3))
            });
            waiting.recv().unwrap();
            access.disconnected("transport disconnected");
            access.connected(None);
            assert!(
                result
                    .join()
                    .unwrap()
                    .unwrap_err()
                    .to_string()
                    .contains("transport disconnected")
            );
            assert!(bus.recv(Duration::ZERO).is_err());
            assert!(access.reserve(BusName::from("test"), bounded(1).0).is_err());
        });

        drop(bus);
        let new_bus = reserve_on_connected_bus(&access);
        assert_eq!(new_bus.recv(Duration::ZERO).unwrap(), None);
    }
}
