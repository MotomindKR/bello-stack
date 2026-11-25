use std::collections::HashMap;
use std::sync::Arc;
use std::sync::atomic::Ordering;
use std::time::{Duration, Instant};

use anyhow::{Context, Result, anyhow, ensure};
use bello_hardware_api::{
    MotorConfiguration, MotorConfigurationKind as Kind, MotorConfigurationPhase as Phase,
};
use parking_lot::{Condvar, Mutex};
use tonic::Status;

use super::HardwareDaemon;
use crate::app::journal::{Journal, PendingWrite};
use crate::config::BusTransportConfig;
use crate::control::GroupState;
use crate::domain::{GroupName, MotorName};
use crate::hardware::can::MaintenanceBus;
use crate::maintenance;

const REVIEW_TIMEOUT: Duration = Duration::from_secs(120);

struct Job {
    status: MotorConfiguration,
    groups: Vec<GroupName>,
    working: bool,
    cancel: bool,
    quarantine: Option<(MaintenanceBus, Journal)>,
}

#[derive(Default)]
pub(super) struct ConfigurationJobs {
    jobs: Mutex<HashMap<String, Job>>,
    changed: Condvar,
}

impl ConfigurationJobs {
    pub(super) fn blocks(&self, group: &GroupName) -> bool {
        self.jobs.lock().values().any(|job| {
            (job.working || job.status.phase() == Phase::Unverified) && job.groups.contains(group)
        })
    }

    pub(super) fn cancel_pending(&self) {
        for job in self.jobs.lock().values_mut() {
            job.cancel = true;
        }
        self.changed.notify_all();
    }

    pub(super) fn drain(&self) {
        let mut jobs = self.jobs.lock();
        for job in jobs.values_mut() {
            if job.status.phase() != Phase::Writing {
                job.cancel = true;
            }
        }
        self.changed.notify_all();
        while jobs.values().any(|job| job.working) {
            self.changed.wait(&mut jobs);
        }
    }
}

impl HardwareDaemon {
    pub(crate) fn prepare_configuration(
        self: &Arc<Self>,
        motor: String,
        kind: Kind,
    ) -> Result<MotorConfiguration, Status> {
        if kind == Kind::Unspecified {
            return Err(Status::invalid_argument("specify zero or firmware"));
        }
        let name = MotorName::from(motor.as_str());
        let target = self
            .topology
            .motor(&name)
            .map_err(|e| Status::not_found(e.to_string()))?;
        let mut groups: Vec<_> = self
            .topology
            .motors()
            .filter(|(_, m)| m.spec.address.bus == target.spec.address.bus)
            .map(|(_, m)| m.group.clone())
            .collect();
        groups.sort();
        groups.dedup();

        let _gate = self.lease_gate.lock();
        if self.shutting_down.load(Ordering::Acquire)
            || self.emergency_blocked.load(Ordering::Acquire)
        {
            return Err(Status::failed_precondition("control is inhibited"));
        }
        let mut jobs = self.configuration.jobs.lock();
        for group in &groups {
            let handle = &self.groups[group];
            if handle.snapshot.read().state != GroupState::Idle
                || handle.emergency_pending()
                || self.leases.occupied(group)
                || jobs.values().any(|job| {
                    (job.working || job.status.phase() == Phase::Unverified)
                        && job.groups.contains(group)
                })
            {
                return Err(Status::failed_precondition(format!(
                    "group {group} must be idle and unleased"
                )));
            }
        }
        let status = MotorConfiguration {
            id: uuid::Uuid::new_v4().to_string(),
            motor,
            kind: kind as i32,
            phase: Phase::Preparing as i32,
            ..Default::default()
        };
        jobs.retain(|_, j| j.status.motor != status.motor);
        jobs.insert(
            status.id.clone(),
            Job {
                status: status.clone(),
                groups,
                working: true,
                cancel: false,
                quarantine: None,
            },
        );
        let daemon = self.clone();
        let id = status.id.clone();
        if let Err(e) = std::thread::Builder::new()
            .name("motor-configuration".into())
            .spawn(move || daemon.configuration_worker(&id, name))
        {
            jobs.remove(&status.id);
            return Err(Status::unavailable(e.to_string()));
        }
        Ok(status)
    }

    pub(crate) fn configuration_status(&self, id: &str) -> Result<MotorConfiguration, Status> {
        self.configuration
            .jobs
            .lock()
            .get(id)
            .map(|j| j.status.clone())
            .ok_or_else(|| Status::not_found("unknown configuration id"))
    }

    pub(crate) fn configuration_decision(
        &self,
        id: &str,
        commit: bool,
    ) -> Result<MotorConfiguration, Status> {
        let _gate = self.lease_gate.lock();
        let mut jobs = self.configuration.jobs.lock();
        let job = jobs
            .get_mut(id)
            .ok_or_else(|| Status::not_found("unknown configuration id"))?;
        match (commit, job.status.phase()) {
            (true, Phase::Ready) if !job.cancel => {
                if self.shutting_down.load(Ordering::Acquire)
                    || self.emergency_blocked.load(Ordering::Acquire)
                    || job
                        .groups
                        .iter()
                        .any(|g| self.groups[g].emergency_pending())
                {
                    return Err(Status::failed_precondition("control is inhibited"));
                }
                job.status.phase = Phase::Writing as i32;
            }
            (false, Phase::Preparing | Phase::Ready) => job.cancel = true,
            (true, Phase::Writing | Phase::Succeeded | Phase::Unverified)
            | (false, Phase::Cancelled) => (),
            _ => {
                return Err(Status::failed_precondition(
                    "operation is not awaiting this decision",
                ));
            }
        }
        self.configuration.changed.notify_all();
        Ok(job.status.clone())
    }

    fn configuration_worker(&self, id: &str, name: MotorName) {
        let groups = self.configuration.jobs.lock()[id].groups.clone();
        let mut entered = Vec::new();
        let mut session = None;
        let mut journal = None;
        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| -> Result<Phase> {
            for group in &groups {
                self.groups[group]
                    .configure(GroupState::Configuring, &format!("configuring {name}"))?;
                entered.push(group.clone());
            }
            let motor = &self.topology.motor(&name)?.spec;
            let handle = self.bus_workers.handle(&motor.address.bus)?;
            let host = handle.spec.host_id;
            let peers = self
                .topology
                .motors()
                .filter(|(_, m)| m.spec.address.bus == motor.address.bus)
                .map(|(_, m)| m.spec.address.id)
                .collect::<Vec<_>>();
            let bus = session.insert(handle.reserve_maintenance()?);
            let kind = self.configuration.jobs.lock()[id].status.kind();
            let prepared = maintenance::prepare(
                bus,
                motor,
                &peers,
                host,
                kind == Kind::Firmware,
                matches!(handle.spec.transport, BusTransportConfig::Sim(_)),
            )?;
            {
                let mut jobs = self.configuration.jobs.lock();
                let status = &mut jobs.get_mut(id).unwrap().status;
                status.motor_uuid = prepared.uuid.to_vec();
                status.reviewed_position_rad = prepared.reviewed_position_rad;
                status.current_version.clone_from(&prepared.current_version);
                status.target_version.clone_from(&prepared.target_version);
                status.phase = Phase::Ready as i32;
                let deadline = Instant::now() + REVIEW_TIMEOUT;
                loop {
                    if jobs[id].cancel
                        || self.shutting_down.load(Ordering::Acquire)
                        || self.emergency_blocked.load(Ordering::Acquire)
                        || groups
                            .iter()
                            .any(|group| self.groups[group].emergency_pending())
                    {
                        return Ok(Phase::Cancelled);
                    }
                    if jobs[id].status.phase() == Phase::Writing {
                        break;
                    }
                    let remaining = deadline.saturating_duration_since(Instant::now());
                    if remaining.is_zero() {
                        jobs.get_mut(id).unwrap().cancel = true;
                        return Ok(Phase::Cancelled);
                    }
                    self.configuration
                        .changed
                        .wait_for(&mut jobs, remaining.min(Duration::from_millis(100)));
                }
            }
            let record = PendingWrite {
                id: id.into(),
                motor: name.to_string(),
                bus: motor.address.bus.to_string(),
                address: motor.address.id,
                uuid: prepared.uuid,
                expected_version: prepared
                    .target_version
                    .as_ref()
                    .unwrap_or(&prepared.current_version)
                    .clone(),
                zero: kind == Kind::Zero,
            };
            maintenance::execute(
                bus,
                motor,
                host,
                &prepared,
                || {
                    let _gate = self.lease_gate.lock();
                    ensure!(
                        !self.shutting_down.load(Ordering::Acquire)
                            && !self.emergency_blocked.load(Ordering::Acquire)
                            && !self.configuration.jobs.lock()[id].cancel,
                        "configuration interrupted before write"
                    );
                    journal = Some(Journal::begin(
                        &crate::app::journal::journal_path(&self.state_directory, &handle.spec),
                        &record,
                    )?);
                    Ok(())
                },
                |value| {
                    self.configuration
                        .jobs
                        .lock()
                        .get_mut(id)
                        .unwrap()
                        .status
                        .progress_percent = value;
                },
            )?;
            journal
                .as_ref()
                .context("missing configuration journal")?
                .clear()?;
            journal = None;
            Ok(Phase::Succeeded)
        }))
        .unwrap_or_else(|_| Err(anyhow!("motor configuration worker panicked")));
        let (mut phase, mut detail) = match result {
            Ok(phase) => (phase, String::new()),
            Err(err) if journal.is_some() => (
                Phase::Unverified,
                format!("{err:#}. Stop hardwared and run --verify-motor-configuration"),
            ),
            Err(err) => (Phase::Failed, format!("{err:#}")),
        };
        let check_health = session.is_some()
            && phase != Phase::Unverified
            && !self.shutting_down.load(Ordering::Acquire);
        let released = Instant::now();
        if phase != Phase::Unverified {
            drop(session.take());
        }
        for group in entered {
            let mut state = if phase == Phase::Unverified {
                GroupState::Fault
            } else {
                GroupState::Idle
            };
            if check_health
                && let Err(error) = self.wait_for_configuration_feedback(&group, released)
            {
                state = GroupState::Fault;
                detail = format!("{detail} group {group}: {error:#}").trim().into();
                if phase == Phase::Succeeded {
                    phase = Phase::Failed;
                }
            }
            if let Err(error) = self.groups[&group].configure(state, &detail) {
                detail =
                    format!("group {group} could not leave configuration: {error:#}. {detail}");
                if phase != Phase::Unverified {
                    phase = Phase::Failed;
                }
            }
        }
        let mut jobs = self.configuration.jobs.lock();
        let job = jobs.get_mut(id).unwrap();
        job.quarantine = session.zip(journal);
        job.status.phase = phase as i32;
        job.status.detail = detail;
        job.working = false;
        self.configuration.changed.notify_all();
    }

    fn wait_for_configuration_feedback(&self, name: &GroupName, since: Instant) -> Result<()> {
        let (_, group) = self
            .topology
            .groups()
            .find(|(g, _)| *g == name)
            .context("missing motor group")?;
        let deadline = Instant::now() + group.enable_timeout;
        loop {
            let mut feedback = HashMap::new();
            for name in &group.motors {
                if let Some(sample) = self.motor_feedback(name)
                    && sample.received >= since
                    && sample.received.elapsed() <= group.feedback_timeout
                    && sample.value.mode_status == 0
                    && sample.value.fault_bits == 0
                    && self.motor_status(name).is_some_and(|s| s.state.is_online())
                {
                    feedback.insert(name.clone(), sample.value);
                }
            }
            if feedback.len() == group.motors.len() {
                return group
                    .mechanical
                    .validate_feedback_positions(&feedback)
                    .map_err(|v| anyhow!(v.detail));
            }
            ensure!(
                Instant::now() < deadline,
                "missing fresh, healthy stopped feedback"
            );
            std::thread::sleep(Duration::from_millis(5));
        }
    }
}
