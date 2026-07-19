use std::collections::HashMap;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::Duration;

use anyhow::{Result, anyhow};
use robstride_protocol::parse_feedback;
use tracing::error;

use super::controller::MotorController;
use crate::config::ResolvedTopology;
use crate::domain::{DeviceState, DeviceStatus, MotorAddress, MotorName};
use crate::hardware::can::{BusManager, ReceivedFrame};
use crate::hardware::lifecycle::DeviceLifecycle;
use crate::telemetry::Telemetry;

/// Owns the feedback dispatcher and the set of configured motor devices.
pub(crate) struct MotorManager {
    controllers: HashMap<MotorName, MotorController>,
    shutdown: Arc<AtomicBool>,
    thread: Option<thread::JoinHandle<()>>,
}

impl MotorManager {
    pub(crate) fn start(
        topology: &ResolvedTopology,
        buses: &BusManager,
        telemetry: Telemetry,
    ) -> Result<Self> {
        let mut controllers = HashMap::new();
        let mut addresses = HashMap::new();
        let feedback_timeouts: HashMap<_, _> = topology
            .groups()
            .map(|(name, group)| (name.clone(), group.feedback_timeout))
            .collect();
        for (name, motor) in topology.motors() {
            let bus = buses.handle(&motor.spec.address.bus)?;
            let lifecycle = DeviceLifecycle::new(
                format!("motor/{name}"),
                DeviceState::Starting,
                format!("starting Robstride motor on {}", motor.spec.address.bus),
                telemetry.clone(),
            );
            lifecycle.transition(
                DeviceState::Connecting,
                "waiting for healthy motor feedback",
            )?;
            let controller = MotorController::new(
                motor.spec.clone(),
                motor.group.clone(),
                bus,
                *feedback_timeouts
                    .get(&motor.group)
                    .expect("resolved motor group has a feedback timeout"),
                lifecycle,
                telemetry.clone(),
            );
            addresses.insert(motor.spec.address.clone(), controller.clone());
            controllers.insert(name.clone(), controller);
        }

        let shutdown = Arc::new(AtomicBool::new(false));
        let worker_stop = shutdown.clone();
        let frames = buses.frames();
        let worker_controllers = addresses;
        let thread = thread::Builder::new()
            .name("robstride-rx".into())
            .spawn(move || {
                let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                    while !worker_stop.load(Ordering::Acquire) {
                        match frames.recv_timeout(Duration::from_millis(20)) {
                            Ok(received) => route_feedback(&worker_controllers, received),
                            Err(crossbeam_channel::RecvTimeoutError::Timeout) => {}
                            Err(crossbeam_channel::RecvTimeoutError::Disconnected) => break,
                        }
                        for controller in worker_controllers.values() {
                            controller.sync_bus();
                            controller.check_feedback_watchdog();
                        }
                    }
                }));
                if result.is_err() || !worker_stop.load(Ordering::Acquire) {
                    for controller in worker_controllers.values() {
                        controller
                            .lifecycle()
                            .fail("Robstride feedback dispatcher stopped unexpectedly");
                    }
                }
            })?;
        Ok(Self {
            controllers,
            shutdown,
            thread: Some(thread),
        })
    }

    pub(crate) fn controller(&self, name: &MotorName) -> Result<MotorController> {
        self.controllers
            .get(name)
            .cloned()
            .ok_or_else(|| anyhow!("unknown motor {name}"))
    }

    pub(crate) fn status(&self, name: &MotorName) -> Option<DeviceStatus> {
        self.controllers.get(name).map(MotorController::status)
    }
}

impl Drop for MotorManager {
    fn drop(&mut self) {
        self.shutdown.store(true, Ordering::Release);
        if let Some(thread) = self.thread.take()
            && thread.join().is_err()
        {
            error!("Robstride feedback worker panicked during shutdown");
        }
        for controller in self.controllers.values() {
            let _ = controller
                .lifecycle()
                .transition(DeviceState::Shutdown, "motor controller stopped");
        }
    }
}

fn route_feedback(controllers: &HashMap<MotorAddress, MotorController>, received: ReceivedFrame) {
    let address = MotorAddress {
        bus: received.bus,
        id: ((received.frame.id >> 8) & 0xff) as u8,
    };
    let Some(controller) = controllers.get(&address) else {
        return;
    };
    let frame = received.frame.into();
    if let Some(feedback) = parse_feedback(&frame, controller.spec.params) {
        controller.record_received_feedback(feedback, received.received);
    }
}
