use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Weak};
use std::time::{Duration, Instant};

use anyhow::{Result, anyhow};
use parking_lot::Mutex;
use tokio::sync::watch;

use crate::config::ResolvedTopology;
use crate::control::{GroupHandle, GroupWorker, LifecycleAction, Transition};
use crate::domain::{
    BusName, CommandBatch, CommandError, GroupName, ImuName, MotorName, SerialDeviceName,
};
use crate::hardware::can::BusManager;
use crate::hardware::imu::{ImuHandle, ImuWorker};
use crate::hardware::lifecycle::DeviceStatus;
use crate::hardware::robstride::MotorManager;
use crate::hardware::serial::SerialManager;
use crate::lease::{Lease, LeaseError, LeaseManager};
use crate::telemetry::Telemetry;

const LEASE_WATCHDOG_PERIOD: Duration = Duration::from_millis(5);

pub struct HardwareDaemon {
    pub topology: ResolvedTopology,
    pub(crate) groups: HashMap<GroupName, GroupHandle>,
    leases: LeaseManager,
    lease_gate: Mutex<()>,
    shutting_down: AtomicBool,
    pub telemetry: Telemetry,
    pub(crate) imus: HashMap<ImuName, ImuHandle>,
    group_workers: Mutex<Vec<GroupWorker>>,
    imu_workers: Vec<ImuWorker>,
    motor_workers: MotorManager,
    bus_workers: BusManager,
    serial_workers: SerialManager,
}

impl HardwareDaemon {
    pub fn start(topology: ResolvedTopology, telemetry: Telemetry) -> Result<Arc<Self>> {
        let bus_workers = BusManager::start(&topology, telemetry.clone())?;
        let motor_workers = MotorManager::start(&topology, &bus_workers, telemetry.clone())?;
        let serial_workers = SerialManager::start(&topology, telemetry.clone())?;
        let mut imus = HashMap::new();
        let mut imu_workers = Vec::new();
        for (name, imu) in topology.imus() {
            let serial = serial_workers.handle(&imu.serial_device)?;
            let worker = ImuWorker::start(imu.clone(), serial, telemetry.clone())?;
            imus.insert(name.clone(), worker.handle.clone());
            imu_workers.push(worker);
        }
        let mut groups = HashMap::new();
        let mut group_workers = Vec::new();
        for (name, group) in topology.groups() {
            let worker = GroupWorker::spawn(group, &motor_workers, telemetry.clone())?;
            groups.insert(name.clone(), worker.handle());
            group_workers.push(worker);
        }
        let leases = LeaseManager::new(
            topology.grpc().default_lease_ms,
            topology.grpc().maximum_lease_ms,
        );
        Ok(Arc::new(Self {
            topology,
            groups,
            leases,
            lease_gate: Mutex::new(()),
            shutting_down: AtomicBool::new(false),
            telemetry,
            imus,
            group_workers: Mutex::new(group_workers),
            imu_workers,
            motor_workers,
            bus_workers,
            serial_workers,
        }))
    }

    pub(crate) fn group(&self, name: &GroupName) -> Result<&GroupHandle> {
        self.groups
            .get(name)
            .ok_or_else(|| anyhow!("unknown group {name}"))
    }

    pub(crate) fn bus_status(&self, name: &BusName) -> Option<DeviceStatus> {
        self.bus_workers.status(name)
    }

    pub(crate) fn serial_status(&self, name: &SerialDeviceName) -> Option<DeviceStatus> {
        self.serial_workers.status(name)
    }

    pub(crate) fn motor_status(&self, name: &MotorName) -> Option<DeviceStatus> {
        self.motor_workers.status(name)
    }

    pub(crate) fn imu(&self, name: &ImuName) -> Option<&ImuHandle> {
        self.imus.get(name)
    }

    pub(crate) fn acquire_lease(
        &self,
        group: &GroupName,
        client: &str,
        requested_ms: u64,
    ) -> Result<Lease, LeaseError> {
        if self.shutting_down.load(Ordering::Acquire) {
            return Err(LeaseError::Unavailable);
        }
        let _gate = self.lease_gate.lock();
        if self.leases.expire_group(group).is_some()
            && let Some(controller) = self.groups.get(group)
        {
            controller.expire_lease();
        }
        let lease = self.leases.acquire(group, client, requested_ms)?;
        if let Some(controller) = self.groups.get(group) {
            controller.activate_lease(lease.generation);
        }
        Ok(lease)
    }

    pub(crate) fn renew_lease(
        &self,
        token: crate::domain::LeaseToken,
        requested_ms: u64,
    ) -> Result<Lease, LeaseError> {
        if self.shutting_down.load(Ordering::Acquire) {
            return Err(LeaseError::Unavailable);
        }
        let _gate = self.lease_gate.lock();
        let result = self.leases.renew(token, requested_ms);
        if matches!(result, Err(LeaseError::Expired)) {
            self.expire_and_notify();
        }
        result
    }

    pub(crate) async fn group_operation(
        &self,
        group: &GroupName,
        token: crate::domain::LeaseToken,
        action: LifecycleAction,
    ) -> Result<Transition> {
        if self.shutting_down.load(Ordering::Acquire) {
            return Err(anyhow!("hardware daemon is shutting down"));
        }
        let reply = {
            let _gate = self.lease_gate.lock();
            let generation = self
                .leases
                .verify(group, token)
                .map_err(anyhow::Error::from)?;
            self.groups
                .get(group)
                .ok_or_else(|| anyhow!("unknown group {group}"))?
                .enqueue_lifecycle(action, generation)?
        };
        Ok(reply.await?)
    }

    /// Verifies ownership and places a command in the group mailbox under the
    /// same lease gate used by expiry and release. The controller reply is
    /// awaited only after releasing the gate.
    pub(crate) async fn submit_command(&self, batch: CommandBatch) -> Result<(), CommandError> {
        let submitted_at = Instant::now();
        if self.shutting_down.load(Ordering::Acquire) {
            return Err(CommandError::ControllerStopped);
        }
        let reply = {
            let _gate = self.lease_gate.lock();
            let generation = self
                .leases
                .verify(&batch.group, batch.lease_token)
                .map_err(|_| CommandError::LeaseLost)?;
            self.groups
                .get(&batch.group)
                .ok_or(CommandError::UnknownGroup)?
                .enqueue_command(batch, generation, submitted_at)?
        };
        reply.await.map_err(|_| CommandError::ControllerStopped)?
    }

    pub(crate) async fn release_lease(
        &self,
        token: crate::domain::LeaseToken,
    ) -> Result<(Lease, Transition)> {
        let (lease, reply) = {
            let _gate = self.lease_gate.lock();
            let lease = self.leases.release(token).map_err(anyhow::Error::from)?;
            let reply = self
                .groups
                .get(&lease.group)
                .ok_or_else(|| anyhow!("unknown group {}", lease.group))?
                .invalidate_lease();
            (lease, reply)
        };
        Ok((lease, reply.await?))
    }

    fn expire_and_notify(&self) {
        for lease in self.leases.expire() {
            if let Some(group) = self.groups.get(&lease.group) {
                group.expire_lease();
            }
        }
    }

    pub async fn lease_watchdog(daemon: Weak<Self>, mut shutdown: watch::Receiver<bool>) {
        let mut timer = tokio::time::interval(LEASE_WATCHDOG_PERIOD);
        timer.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
        loop {
            tokio::select! {
                biased;
                changed = shutdown.changed() => {
                    if changed.is_err() || *shutdown.borrow() {
                        break;
                    }
                    continue;
                }
                _ = timer.tick() => {}
            }
            let Some(daemon) = daemon.upgrade() else {
                break;
            };
            let _gate = daemon.lease_gate.lock();
            daemon.expire_and_notify();
        }
    }

    /// Stops command admission and synchronously waits for every motor group to
    /// apply its configured exit response. CAN and motor workers remain alive
    /// until the daemon is dropped, so safety frames can still be delivered.
    pub fn shutdown(&self) {
        if self.shutting_down.swap(true, Ordering::AcqRel) {
            return;
        }
        let workers = std::mem::take(&mut *self.group_workers.lock());
        for worker in &workers {
            worker.request_shutdown();
        }
        for worker in workers {
            worker.shutdown();
        }
    }
}

impl Drop for HardwareDaemon {
    fn drop(&mut self) {
        self.shutdown();
        for worker in &self.imu_workers {
            worker.request_stop();
        }
        self.imu_workers.clear();
    }
}
