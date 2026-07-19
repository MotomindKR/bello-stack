use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc;
use std::thread;

use anyhow::Result;
use parking_lot::{Mutex, RwLock};

use crate::domain::CommandError;
use crate::hardware::robstride::MotorManager;
use crate::telemetry::Telemetry;

use super::group::{ControllerInputs, GroupController, GroupMotor};
use super::group_handle::{GroupHandle, GroupSnapshot, LifecycleRequest};
use super::spec::GroupSpec;

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
        let (lifecycle_tx, lifecycle_rx) = mpsc::sync_channel(128);
        let command_mailbox = Arc::new(Mutex::new(None));
        let lease_lost = Arc::new(AtomicBool::new(false));
        let shutdown = Arc::new(AtomicBool::new(false));
        let snapshot = Arc::new(RwLock::new(GroupSnapshot::default()));
        let controller = GroupController::new(
            group.clone(),
            motors,
            ControllerInputs {
                lifecycle_rx,
                command_mailbox: command_mailbox.clone(),
                lease_lost: lease_lost.clone(),
                shutdown: shutdown.clone(),
                snapshot: snapshot.clone(),
            },
            telemetry,
        );
        let panic_snapshot = snapshot.clone();
        let join = thread::Builder::new()
            .name(format!("group-{}", group.name))
            .spawn(move || {
                if std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| controller.run()))
                    .is_err()
                {
                    let mut snapshot = panic_snapshot.write();
                    snapshot.state = crate::control::GroupState::Fault;
                    snapshot.fault_reason = "group controller panicked".into();
                }
            })?;
        Ok(Self {
            handle: GroupHandle {
                lifecycle_tx,
                command_mailbox,
                lease_lost,
                shutdown,
                snapshot,
            },
            join: Some(join),
        })
    }

    pub(crate) fn handle(&self) -> GroupHandle {
        self.handle.clone()
    }

    fn shutdown_inner(&mut self) {
        self.handle.shutdown.store(true, Ordering::Release);
        if let Some(pending) = self.handle.command_mailbox.lock().take() {
            let _ = pending.reply.send(Err(CommandError::ControllerStopped));
        }
        let _ = self.handle.lifecycle_tx.try_send(LifecycleRequest::Wake);
        if let Some(join) = self.join.take()
            && join.join().is_err()
        {
            tracing::error!("group controller panicked during shutdown");
        }
    }

    pub(crate) fn shutdown(mut self) {
        self.shutdown_inner();
    }
}

impl Drop for GroupWorker {
    fn drop(&mut self) {
        self.shutdown_inner();
    }
}
