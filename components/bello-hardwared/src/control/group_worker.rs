use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::mpsc;
use std::thread;

use anyhow::Result;
use parking_lot::{Mutex, RwLock};

use crate::domain::CommandError;
use crate::hardware::robstride::MotorManager;
use crate::telemetry::Telemetry;

use super::group::{ControllerInputs, GroupController, GroupMotor};
use super::group_handle::{GroupHandle, GroupSnapshot};
use super::spec::GroupSpec;

const LIFECYCLE_QUEUE_CAPACITY: usize = 16;
const WAKE_QUEUE_CAPACITY: usize = 1;

pub(crate) struct GroupWorker {
    handle: GroupHandle,
    join: Option<thread::JoinHandle<()>>,
}

impl GroupWorker {
    pub(crate) fn spawn(
        group: &GroupSpec,
        motor_manager: &MotorManager,
        telemetry: Telemetry,
    ) -> Result<Self> {
        let motors = group
            .motors
            .iter()
            .map(|name| motor_manager.controller(name).map(GroupMotor::new))
            .collect::<Result<Vec<_>>>()?;
        let (lifecycle_tx, lifecycle_rx) = mpsc::sync_channel(LIFECYCLE_QUEUE_CAPACITY);
        let (wake_tx, wake_rx) = mpsc::sync_channel(WAKE_QUEUE_CAPACITY);
        let command_mailbox = Arc::new(Mutex::new(None));
        let lease_lost = Arc::new(AtomicBool::new(false));
        let lease_generation = Arc::new(AtomicU64::new(0));
        let lease_loss_waiters = Arc::new(Mutex::new(Vec::new()));
        let shutdown = Arc::new(AtomicBool::new(false));
        let snapshot = Arc::new(RwLock::new(GroupSnapshot::default()));
        let controller = GroupController::new(
            group.clone(),
            motors,
            ControllerInputs {
                lifecycle_rx,
                wake_rx,
                command_mailbox: command_mailbox.clone(),
                lease_lost: lease_lost.clone(),
                lease_generation: lease_generation.clone(),
                lease_loss_waiters: lease_loss_waiters.clone(),
                shutdown: shutdown.clone(),
                snapshot: snapshot.clone(),
            },
            telemetry.clone(),
        );
        let panic_snapshot = snapshot.clone();
        let exit_mailbox = command_mailbox.clone();
        let exit_shutdown = shutdown.clone();
        let exit_waiters = lease_loss_waiters.clone();
        let join = thread::Builder::new()
            .name(format!("group-{}", group.name))
            .spawn(move || {
                let mut controller = controller;
                let panicked = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                    controller.run();
                }))
                .is_err();
                exit_shutdown.store(true, Ordering::Release);
                if let Some(pending) = exit_mailbox.lock().take() {
                    let _ = pending.reply.send(Err(CommandError::ControllerStopped));
                }
                if panicked {
                    let safety_panicked =
                        std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                            controller.handle_worker_panic()
                        }))
                        .is_err();
                    if safety_panicked {
                        let mut snapshot = panic_snapshot.write();
                        snapshot.state = crate::control::GroupState::Fault;
                        snapshot.cause = crate::control::GroupCause::Internal;
                        snapshot.detail = "group panic handler also panicked".into();
                        snapshot.updated = Some(std::time::Instant::now());
                        tracing::error!(
                            "group panic handler also panicked; safety state is unknown"
                        );
                    }
                }
                exit_waiters.lock().clear();
            })?;
        Ok(Self {
            handle: GroupHandle {
                lifecycle_tx,
                wake_tx,
                command_mailbox,
                lease_lost,
                lease_generation,
                lease_loss_waiters,
                shutdown,
                snapshot,
            },
            join: Some(join),
        })
    }

    pub(crate) fn handle(&self) -> GroupHandle {
        self.handle.clone()
    }

    fn request_shutdown_inner(&self) {
        self.handle.shutdown.store(true, Ordering::Release);
        if let Some(pending) = self.handle.command_mailbox.lock().take() {
            let _ = pending.reply.send(Err(CommandError::ControllerStopped));
        }
        let _ = self.handle.wake_tx.try_send(());
    }

    fn join_inner(&mut self) {
        if let Some(join) = self.join.take()
            && join.join().is_err()
        {
            tracing::error!("group controller panicked during shutdown");
        }
    }

    pub(crate) fn request_shutdown(&self) {
        self.request_shutdown_inner();
    }

    pub(crate) fn shutdown(mut self) {
        self.request_shutdown_inner();
        self.join_inner();
    }
}

impl Drop for GroupWorker {
    fn drop(&mut self) {
        self.request_shutdown_inner();
        self.join_inner();
    }
}
