use std::collections::HashMap;
use std::sync::{Arc, Weak};
use std::time::Duration;

use anyhow::{Result, anyhow};
use parking_lot::Mutex;

use crate::config::ResolvedTopology;
use crate::control::{GroupHandle, GroupWorker};
use crate::domain::{
    BusName, CommandBatch, CommandError, GroupName, ImuName, MotorName, SerialDeviceName,
};
use crate::hardware::can::BusManager;
use crate::hardware::imu::{ImuState, ImuWorker};
use crate::hardware::lifecycle::DeviceStatus;
use crate::hardware::robstride::MotorManager;
use crate::hardware::serial::SerialManager;
use crate::lease::{Lease, LeaseError, LeaseManager};
use crate::telemetry::Telemetry;

pub struct HardwareDaemon {
    pub topology: ResolvedTopology,
    pub(crate) groups: HashMap<GroupName, GroupHandle>,
    leases: LeaseManager,
    lease_gate: Mutex<()>,
    pub telemetry: Telemetry,
    pub(crate) imus: HashMap<ImuName, ImuState>,
    group_workers: Vec<GroupWorker>,
    imu_workers: Vec<ImuWorker>,
    motor_workers: MotorManager,
    bus_workers: BusManager,
    serial_workers: SerialManager,
}

impl HardwareDaemon {
    pub fn start(topology: ResolvedTopology) -> Result<Arc<Self>> {
        let telemetry = Telemetry::new(topology.runtime().telemetry_capacity);
        let bus_workers = BusManager::start(&topology, telemetry.clone())?;
        let motor_workers = MotorManager::start(&topology, &bus_workers, telemetry.clone())?;
        let serial_workers = SerialManager::start(&topology, telemetry.clone())?;
        let mut imus = HashMap::new();
        let mut imu_workers = Vec::new();
        for (name, imu) in topology.imus() {
            let serial = serial_workers.handle(&imu.serial_device)?;
            let worker = ImuWorker::start(imu.clone(), serial, telemetry.clone())?;
            imus.insert(name.clone(), worker.state.clone());
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
            telemetry,
            imus,
            group_workers,
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

    pub(crate) fn imu(&self, name: &ImuName) -> Option<&ImuState> {
        self.imus.get(name)
    }

    pub(crate) fn acquire_lease(
        &self,
        group: &GroupName,
        client: &str,
        requested_ms: u64,
    ) -> Result<Lease, LeaseError> {
        let _gate = self.lease_gate.lock();
        if self.leases.expire_group(group).is_some()
            && let Some(controller) = self.groups.get(group)
        {
            controller.lease_lost();
        }
        self.leases.acquire(group, client, requested_ms)
    }

    pub(crate) fn renew_lease(
        &self,
        token: crate::domain::LeaseToken,
        requested_ms: u64,
    ) -> Result<Lease, LeaseError> {
        let _gate = self.lease_gate.lock();
        let result = self.leases.renew(token, requested_ms);
        if matches!(result, Err(LeaseError::Expired)) {
            self.expire_and_notify();
        }
        result
    }

    pub(crate) fn verify_lease(
        &self,
        group: &GroupName,
        token: crate::domain::LeaseToken,
    ) -> Result<(), LeaseError> {
        self.leases.verify(group, token)
    }

    /// Verifies ownership and places a command in the group mailbox under the
    /// same lease gate used by expiry and release. The controller reply is
    /// awaited only after releasing the gate.
    pub(crate) async fn submit_command(&self, batch: CommandBatch) -> Result<(), CommandError> {
        let reply = {
            let _gate = self.lease_gate.lock();
            self.leases
                .verify(&batch.group, batch.lease_token)
                .map_err(|_| CommandError::LeaseLost)?;
            self.groups
                .get(&batch.group)
                .ok_or(CommandError::UnknownGroup)?
                .enqueue_command(batch)?
        };
        reply.await.map_err(|_| CommandError::ControllerStopped)?
    }

    pub(crate) fn release_lease(
        &self,
        token: crate::domain::LeaseToken,
    ) -> Result<Lease, LeaseError> {
        let _gate = self.lease_gate.lock();
        let lease = self.leases.release(token)?;
        if let Some(group) = self.groups.get(&lease.group) {
            group.lease_lost();
        }
        Ok(lease)
    }

    pub(crate) fn lease_owner(&self, group: &GroupName) -> Option<String> {
        self.leases.owner(group)
    }

    fn expire_and_notify(&self) {
        for lease in self.leases.expire() {
            if let Some(group) = self.groups.get(&lease.group) {
                group.lease_lost();
            }
        }
    }

    pub async fn lease_watchdog(daemon: Weak<Self>) {
        let mut timer = tokio::time::interval(Duration::from_millis(20));
        loop {
            timer.tick().await;
            let Some(daemon) = daemon.upgrade() else {
                break;
            };
            let _gate = daemon.lease_gate.lock();
            daemon.expire_and_notify();
        }
    }
}

impl Drop for HardwareDaemon {
    fn drop(&mut self) {
        for worker in self.group_workers.drain(..) {
            worker.shutdown();
        }
        self.imu_workers.clear();
        // These fields intentionally remain alive until every group has emitted its stop frames.
        let _ = &self.motor_workers;
        let _ = &self.bus_workers;
        let _ = &self.serial_workers;
    }
}
