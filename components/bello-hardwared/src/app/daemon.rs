use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Weak};
use std::time::{Duration, Instant};

use anyhow::{Result, anyhow};
use parking_lot::Mutex;
use tokio::sync::watch;

use crate::config::{EmergencyStopMode, ResolvedTopology};
use crate::control::{GroupHandle, GroupWorker, LifecycleAction, Transition};
use crate::domain::PowerBoardName;
use crate::domain::{
    BusName, CommandBatch, CommandError, GroupName, ImuName, MotorName, SerialDeviceName,
};
use crate::hardware::can::BusManager;
use crate::hardware::imu::{ImuHandle, ImuWorker};
use crate::hardware::lifecycle::DeviceStatus;
use crate::hardware::power_board::PowerBoardWorker;
use crate::hardware::robstride::MotorManager;
use crate::hardware::robstride::TimedFeedback;
use crate::hardware::serial::SerialManager;
use crate::lease::{Lease, LeaseError, LeaseManager};
use crate::telemetry::Telemetry;

const LEASE_WATCHDOG_PERIOD: Duration = Duration::from_millis(5);

mod configuration;
use configuration::ConfigurationJobs;

pub struct HardwareDaemon {
    configuration: Arc<ConfigurationJobs>,
    state_directory: std::path::PathBuf,
    pub topology: ResolvedTopology,
    pub(crate) groups: HashMap<GroupName, GroupHandle>,
    leases: Arc<LeaseManager>,
    lease_gate: Arc<Mutex<()>>,
    shutting_down: AtomicBool,
    emergency_blocked: Arc<AtomicBool>,
    pub telemetry: Telemetry,
    pub(crate) imus: HashMap<ImuName, ImuHandle>,
    group_workers: Mutex<Vec<GroupWorker>>,
    imu_workers: Vec<ImuWorker>,
    pub(crate) power_workers: Mutex<HashMap<PowerBoardName, PowerBoardWorker>>,
    motor_workers: MotorManager,
    bus_workers: BusManager,
    serial_workers: SerialManager,
    _transport_ownership: Vec<std::fs::File>,
}

impl HardwareDaemon {
    pub fn start(
        topology: ResolvedTopology,
        telemetry: Telemetry,
        state_directory: std::path::PathBuf,
    ) -> Result<Arc<Self>> {
        let ownership = super::journal::claim_transports(&state_directory, &topology)?;
        for (_, bus) in topology.buses() {
            super::journal::check_clear(&super::journal::journal_path(&state_directory, bus))?;
        }
        let bus_workers = BusManager::start(&topology, telemetry.clone())?;
        let motor_workers = MotorManager::start(&topology, &bus_workers, telemetry.clone())?;
        let serial_workers = SerialManager::start(&topology, telemetry.clone())?;
        let mut imus = HashMap::new();
        let mut imu_workers = Vec::new();
        for (name, imu) in topology.imus() {
            let serial = serial_workers.handle(&imu.serial_device)?;
            let worker = ImuWorker::start(
                imu.clone(),
                topology.imu_filter(),
                serial,
                telemetry.clone(),
            )?;
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
        let emergency_blocked = topology
            .power_boards()
            .any(|(_, board)| board.emergency_stop == EmergencyStopMode::Damping);
        let daemon = Arc::new(Self {
            configuration: Arc::new(ConfigurationJobs::default()),
            state_directory,
            topology,
            groups,
            leases: Arc::new(leases),
            lease_gate: Arc::new(Mutex::new(())),
            shutting_down: AtomicBool::new(false),
            emergency_blocked: Arc::new(AtomicBool::new(emergency_blocked)),
            telemetry,
            imus,
            group_workers: Mutex::new(group_workers),
            imu_workers,
            power_workers: Mutex::new(HashMap::new()),
            motor_workers,
            bus_workers,
            serial_workers,
            _transport_ownership: ownership,
        });
        for (name, board) in daemon.topology.power_boards() {
            let serial = daemon.serial_workers.handle(&board.serial_device)?;
            let emergency = (board.emergency_stop == EmergencyStopMode::Damping)
                .then(|| daemon.emergency_handler());
            let worker = PowerBoardWorker::start(
                board.clone(),
                serial,
                daemon.telemetry.clone(),
                emergency,
            )?;
            daemon.power_workers.lock().insert(name.clone(), worker);
        }
        Ok(daemon)
    }

    fn emergency_handler(&self) -> crate::hardware::power_board::EmergencyHandler {
        let gate = self.lease_gate.clone();
        let inhibited = self.emergency_blocked.clone();
        let leases = self.leases.clone();
        let groups = self.groups.clone();
        let configuration = self.configuration.clone();
        Arc::new(move |blocked| {
            let _gate = gate.lock();
            inhibited.store(blocked, Ordering::Release);
            if blocked {
                configuration.cancel_pending();
                leases.revoke_all();
                for group in groups.values() {
                    group.request_emergency_damping();
                }
            }
        })
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

    pub(crate) fn motor_feedback(&self, name: &MotorName) -> Option<TimedFeedback> {
        self.motor_workers.feedback(name)
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
        if self.configuration.blocks(group)
            || self.emergency_blocked.load(Ordering::Acquire)
            || self
                .groups
                .get(group)
                .is_some_and(GroupHandle::emergency_pending)
        {
            return Err(LeaseError::Unavailable);
        }
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

    pub(crate) async fn start_control(&self, batch: CommandBatch) -> Result<Transition> {
        let submitted_at = Instant::now();
        if self.shutting_down.load(Ordering::Acquire) {
            return Err(anyhow!("hardware daemon is shutting down"));
        }
        let reply = {
            let _gate = self.lease_gate.lock();
            let generation = self
                .leases
                .verify(&batch.group, batch.lease_token)
                .map_err(anyhow::Error::from)?;
            self.groups
                .get(&batch.group)
                .ok_or_else(|| anyhow!("unknown group {}", batch.group))?
                .enqueue_lifecycle(
                    LifecycleAction::Start {
                        batch,
                        submitted_at,
                    },
                    generation,
                )?
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
        {
            let _gate = self.lease_gate.lock();
            if self.shutting_down.swap(true, Ordering::AcqRel) {
                return;
            }
        }
        for (name, handle) in &self.groups {
            if !self.configuration.blocks(name) {
                handle.request_shutdown();
            }
        }
        self.configuration.drain();
        let workers = std::mem::take(&mut *self.group_workers.lock());
        for worker in &workers {
            worker.request_shutdown();
        }
        for worker in workers {
            worker.shutdown();
        }
        let power_workers = std::mem::take(&mut *self.power_workers.lock());
        for worker in power_workers.values() {
            worker.request_stop();
        }
        drop(power_workers);
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::Config;

    #[test]
    fn emergency_revokes_every_lease_and_release_requires_new_ownership() {
        let mut config =
            Config::load_raw(concat!(env!("CARGO_MANIFEST_DIR"), "/config/bello.yaml")).unwrap();
        for (i, bus) in config.buses.iter_mut().enumerate() {
            bus.transport = crate::config::BusTransportConfig::Socketcan {
                interface: format!("no_emergency{i}"),
            };
        }
        for serial in &mut config.serial_devices {
            serial.port = "/dev/nonexistent-emergency-test".into();
        }
        config.power_boards.clear();
        let topology = ResolvedTopology::resolve(config).unwrap();
        let directory = tempfile::tempdir().unwrap();
        let daemon =
            HardwareDaemon::start(topology, Telemetry::new(4096), directory.path().into()).unwrap();
        let leases = daemon
            .groups
            .keys()
            .map(|group| daemon.acquire_lease(group, "test", 5000).unwrap())
            .collect::<Vec<_>>();
        let emergency = daemon.emergency_handler();
        emergency(true);
        for lease in &leases {
            assert!(daemon.leases.verify(&lease.group, lease.token).is_err());
            assert_eq!(daemon.groups[&lease.group].lease_generation_for_test(), 0);
            assert!(matches!(
                daemon.acquire_lease(&lease.group, "replacement", 5000),
                Err(LeaseError::Unavailable)
            ));
        }
        emergency(false);
        let deadline = Instant::now() + Duration::from_secs(2);
        while daemon.groups.values().any(GroupHandle::emergency_pending) {
            assert!(Instant::now() < deadline);
            std::thread::sleep(Duration::from_millis(1));
        }
        for lease in &leases {
            assert!(daemon.renew_lease(lease.token, 5000).is_err());
            let replacement = daemon
                .acquire_lease(&lease.group, "replacement", 5000)
                .unwrap();
            assert_ne!(replacement.generation, lease.generation);
            assert_eq!(
                daemon.groups[&lease.group].snapshot.read().state,
                crate::control::GroupState::Idle
            );
        }
        daemon.shutdown();
    }
}
