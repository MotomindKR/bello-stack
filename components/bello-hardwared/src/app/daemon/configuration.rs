use std::collections::HashMap;
use std::sync::Arc;
use std::sync::atomic::Ordering;
use std::time::{Duration, Instant};

use anyhow::{Context, Result, anyhow, ensure};
use bello_hardware_api::{
    GetMotorConfigurationResponse as OperationStatus, MotorConfigurationKind as Kind,
    MotorConfigurationPhase as Phase,
};
use parking_lot::{Condvar, Mutex};
use tonic::Status;

use super::HardwareDaemon;
use crate::config::BusTransportConfig;
use crate::control::GroupState;
use crate::domain::{GroupName, MotorName};
use crate::maintenance;

const REVIEW_TIMEOUT: Duration = Duration::from_secs(120);

struct Job {
    status: OperationStatus,
    groups: Vec<GroupName>,
    working: bool,
    cancel: bool,
}

#[derive(Default)]
pub(super) struct ConfigurationJobs {
    jobs: Mutex<HashMap<String, Job>>,
    changed: Condvar,
}

impl ConfigurationJobs {
    pub(super) fn blocks(&self, group: &GroupName) -> bool {
        self.jobs
            .lock()
            .values()
            .any(|job| job.working && job.groups.contains(group))
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
    ) -> Result<String, Status> {
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
                || jobs
                    .values()
                    .any(|job| job.working && job.groups.contains(group))
            {
                return Err(Status::failed_precondition(format!(
                    "group {group} must be idle and unleased"
                )));
            }
        }
        let status = OperationStatus {
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
        Ok(status.id)
    }

    pub(crate) fn configuration_status(&self, id: &str) -> Result<OperationStatus, Status> {
        self.configuration
            .jobs
            .lock()
            .get(id)
            .map(|j| j.status.clone())
            .ok_or_else(|| Status::not_found("unknown configuration id"))
    }

    pub(crate) fn configuration_decision(&self, id: &str, commit: bool) -> Result<(), Status> {
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
        Ok(())
    }

    fn configuration_worker(&self, id: &str, name: MotorName) {
        let groups = self.configuration.jobs.lock()[id].groups.clone();
        let mut entered = Vec::new();
        let mut session = None;
        let mut write_started = false;
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
                    write_started = true;
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
            Ok(Phase::Succeeded)
        }))
        .unwrap_or_else(|_| Err(anyhow!("motor configuration worker panicked")));
        let (phase, detail) = match result {
            Ok(phase) => (phase, String::new()),
            Err(err) if write_started => (Phase::Unverified, format!("{err:#}")),
            Err(err) => (Phase::Failed, format!("{err:#}")),
        };
        let check_health = session.is_some()
            && phase != Phase::Unverified
            && !self.shutting_down.load(Ordering::Acquire);
        let released = Instant::now();
        drop(session);
        let mut details = Vec::new();
        if !detail.is_empty() {
            details.push(detail.clone());
        }
        for group in entered {
            let (state, group_detail) = if check_health
                && let Err(error) = self.wait_for_configuration_feedback(&group, released)
            {
                let reason = format!("group {group}: {error:#}");
                details.push(reason.clone());
                (GroupState::Fault, reason)
            } else if phase == Phase::Unverified {
                (GroupState::Fault, detail.clone())
            } else {
                (GroupState::Idle, detail.clone())
            };
            if let Err(error) = self.groups[&group].configure(state, &group_detail) {
                details.push(format!(
                    "group {group} could not leave configuration: {error:#}"
                ));
            }
        }
        let mut jobs = self.configuration.jobs.lock();
        let job = jobs.get_mut(id).unwrap();
        job.status.phase = phase as i32;
        job.status.detail = details.join(". ");
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
            let samples = group
                .motors
                .iter()
                .map(|motor| {
                    let feedback = self
                        .stopped_feedback_since(motor, since, group.feedback_timeout)
                        .with_context(|| format!("motor {motor}"))?;
                    Ok((motor.clone(), feedback))
                })
                .collect::<Result<HashMap<_, _>>>();
            match samples {
                Ok(feedback) => {
                    return group
                        .mechanical
                        .validate_feedback_positions(&feedback)
                        .map_err(|v| anyhow!(v.detail));
                }
                Err(reason) if Instant::now() >= deadline => return Err(reason),
                Err(_) => std::thread::sleep(Duration::from_millis(5)),
            }
        }
    }

    fn stopped_feedback_since(
        &self,
        motor: &MotorName,
        since: Instant,
        timeout: Duration,
    ) -> Result<robstride_protocol::Feedback> {
        let sample = self
            .motor_feedback(motor)
            .context("no feedback after maintenance")?;
        ensure!(
            sample.received >= since && sample.received.elapsed() <= timeout,
            "stale feedback"
        );
        ensure!(
            sample.value.fault_bits == 0,
            "fault bits {:#x}",
            sample.value.fault_bits
        );
        ensure!(
            sample.value.mode_status == 0,
            "motor is in mode {}",
            sample.value.mode_status
        );
        let status = self.motor_status(motor).context("motor unavailable")?;
        ensure!(status.state.is_online(), "{}", status.detail);
        Ok(sample.value)
    }
}
