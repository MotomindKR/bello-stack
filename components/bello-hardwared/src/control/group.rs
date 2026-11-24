use std::collections::{BTreeMap, HashMap};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::mpsc::{self, Receiver};
use std::thread;
use std::time::{Duration, Instant};

use anyhow::Result;
use parking_lot::{Mutex, RwLock};

use crate::config::{CommandBehavior, CommandTransmission, FaultCause, FaultResponse};
use crate::domain::{
    CommandBatch, CommandError, CommandPolicyTelemetry, GroupTelemetry, ImpedanceCommand,
    TelemetryPayload,
};
use crate::hardware::can::PeriodicControl;
use crate::hardware::robstride::{MotorController, MotorFaultCause};
use crate::mechanics::MechanicalViolation;
use crate::telemetry::Telemetry;

use super::command_policy::{
    CommandPolicyCheck, CommandPolicyIssue, check_command, coupled_limit_behavior,
    select_position_reference,
};
use super::fsm::{GroupCause, GroupFsm, GroupState};
use super::group_handle::{CommandSubmission, GroupSnapshot, LifecycleAction, LifecycleRequest};
use super::spec::GroupSpec;

const GROUP_TELEMETRY_PERIOD: Duration = Duration::from_millis(40);

enum EnableFailure {
    LeaseLost(String),
    Shutdown(String),
    Failed(String),
}

struct CommandRefreshSchedule {
    refresh_period: Duration,
    command_arrival_grace: Duration,
    next_refresh: Instant,
}

impl CommandRefreshSchedule {
    fn new(now: Instant, refresh_period: Duration, command_arrival_grace: Duration) -> Self {
        Self {
            refresh_period,
            command_arrival_grace,
            next_refresh: now + command_arrival_grace,
        }
    }

    fn record_immediate_send(&mut self, now: Instant) {
        self.next_refresh = now + self.refresh_period + self.command_arrival_grace;
    }

    fn should_retransmit(&mut self, now: Instant) -> bool {
        if now < self.next_refresh {
            return false;
        }
        self.next_refresh = now + self.refresh_period;
        true
    }
}

impl EnableFailure {
    fn into_detail(self) -> String {
        match self {
            Self::LeaseLost(detail) | Self::Shutdown(detail) | Self::Failed(detail) => detail,
        }
    }
}

pub(super) struct GroupMotor {
    controller: MotorController,
    command: Option<ImpedanceCommand>,
}
impl GroupMotor {
    pub(super) fn new(controller: MotorController) -> Self {
        Self {
            controller,
            command: None,
        }
    }
}

pub(super) struct ControllerInputs {
    pub lifecycle_rx: Receiver<LifecycleRequest>,
    pub wake_rx: Receiver<()>,
    pub command_mailbox: Arc<Mutex<Option<CommandSubmission>>>,
    pub lease_lost: Arc<AtomicBool>,
    pub emergency_pending: Arc<AtomicBool>,
    pub lease_generation: Arc<AtomicU64>,
    pub lease_loss_waiters: Arc<Mutex<Vec<tokio::sync::oneshot::Sender<super::Transition>>>>,
    pub shutdown: Arc<AtomicBool>,
    pub snapshot: Arc<RwLock<GroupSnapshot>>,
}

pub(super) struct GroupController {
    spec: GroupSpec,
    motors: Vec<GroupMotor>,
    periodic_buses: Vec<(PeriodicControl, Vec<usize>)>,
    lifecycle_rx: Receiver<LifecycleRequest>,
    wake_rx: Receiver<()>,
    command_mailbox: Arc<Mutex<Option<CommandSubmission>>>,
    lease_lost: Arc<AtomicBool>,
    emergency_pending: Arc<AtomicBool>,
    lease_generation: Arc<AtomicU64>,
    lease_loss_waiters: Arc<Mutex<Vec<tokio::sync::oneshot::Sender<super::Transition>>>>,
    shutdown: Arc<AtomicBool>,
    fsm: GroupFsm,
    snapshot: Arc<RwLock<GroupSnapshot>>,
    telemetry: Telemetry,
    command_deadline: Option<Instant>,
    command_count: u64,
    command_generation: Option<u64>,
    last_sequence: Option<u64>,
    tick: u64,
    last_publish: Instant,
    last_published_state: GroupState,
    last_published_cause: GroupCause,
    last_published_detail: String,
}

impl GroupController {
    pub(super) fn new(
        spec: GroupSpec,
        motors: Vec<GroupMotor>,
        inputs: ControllerInputs,
        telemetry: Telemetry,
    ) -> Self {
        let mut periodic_buses = BTreeMap::new();
        if spec.command_transmission == CommandTransmission::Periodic {
            for (index, motor) in motors.iter().enumerate() {
                let entry = periodic_buses
                    .entry(motor.controller.spec.address.bus.clone())
                    .or_insert_with(|| {
                        (
                            motor.controller.periodic_control(
                                spec.command_refresh_period(),
                                inputs.lease_generation.clone(),
                                inputs.shutdown.clone(),
                            ),
                            Vec::new(),
                        )
                    });
                entry.1.push(index);
            }
        }
        Self {
            spec,
            motors,
            periodic_buses: periodic_buses.into_values().collect(),
            lifecycle_rx: inputs.lifecycle_rx,
            wake_rx: inputs.wake_rx,
            command_mailbox: inputs.command_mailbox,
            lease_lost: inputs.lease_lost,
            emergency_pending: inputs.emergency_pending,
            lease_generation: inputs.lease_generation,
            lease_loss_waiters: inputs.lease_loss_waiters,
            shutdown: inputs.shutdown,
            fsm: GroupFsm::default(),
            snapshot: inputs.snapshot,
            telemetry,
            command_deadline: None,
            command_count: 0,
            command_generation: None,
            last_sequence: None,
            tick: 0,
            last_publish: Instant::now(),
            last_published_state: GroupState::Idle,
            last_published_cause: GroupCause::Unspecified,
            last_published_detail: String::new(),
        }
    }

    pub(super) fn run(&mut self) {
        let period = self.spec.period();
        let now = Instant::now();
        let mut next_tick = now + period;
        let mut refresh_schedule = (self.spec.command_transmission
            == CommandTransmission::OnUpdate)
            .then(|| CommandRefreshSchedule::new(now, self.spec.command_refresh_period(), period));
        loop {
            if self.shutdown.load(Ordering::Acquire) {
                self.perform_shutdown();
                break;
            }
            if self.handle_emergency() {
                continue;
            }
            if self.lease_lost.swap(false, Ordering::AcqRel) {
                self.handle_lease_loss();
                continue;
            }
            let handled_lifecycle = match self.lifecycle_rx.try_recv() {
                Ok(request) => {
                    self.handle(request);
                    true
                }
                Err(mpsc::TryRecvError::Disconnected) => {
                    self.perform_shutdown();
                    break;
                }
                Err(mpsc::TryRecvError::Empty) => false,
            };
            if self.shutdown.load(Ordering::Acquire) {
                self.perform_shutdown();
                break;
            }
            if self.handle_emergency() {
                continue;
            }
            if self.lease_lost.swap(false, Ordering::AcqRel) {
                self.handle_lease_loss();
                continue;
            }
            let pending_command = { self.command_mailbox.lock().take() };
            if let Some(submission) = pending_command {
                let result = self.accept_commands(
                    submission.batch,
                    submission.lease_generation,
                    submission.submitted_at,
                    GroupState::Running,
                );
                if result.is_ok() {
                    if let Some(schedule) = &mut refresh_schedule {
                        self.control_tick(0, true);
                        schedule.record_immediate_send(Instant::now());
                    } else {
                        self.control_tick(0, false);
                        if self.fsm.state() == GroupState::Running {
                            self.publish_periodic_targets();
                        }
                    }
                }
                let _ = submission.reply.send(result);
                continue;
            }
            let now = Instant::now();
            if now >= next_tick {
                let jitter = now
                    .saturating_duration_since(next_tick)
                    .as_micros()
                    .min(i64::MAX as u128) as i64;
                let transmit = refresh_schedule
                    .as_mut()
                    .is_some_and(|schedule| schedule.should_retransmit(now));
                self.control_tick(jitter, transmit);
                next_tick += period;
                if next_tick < now {
                    next_tick = now + period;
                }
                continue;
            }
            if handled_lifecycle {
                // Drain queued lifecycle work without relying on one wake token
                // per request, but re-check the control deadline each iteration.
                continue;
            }
            let timeout = next_tick.saturating_duration_since(Instant::now());
            match self.wake_rx.recv_timeout(timeout) {
                Ok(()) => {}
                Err(mpsc::RecvTimeoutError::Timeout) => {}
                Err(mpsc::RecvTimeoutError::Disconnected) => {
                    self.perform_shutdown();
                    break;
                }
            }
        }
    }

    fn handle_emergency(&mut self) -> bool {
        if !self.emergency_pending.load(Ordering::Acquire) {
            return false;
        }
        self.clear_command_state();
        match self.fsm.state() {
            GroupState::Running | GroupState::Damped => {
                self.establish_damping(
                    GroupCause::Stop,
                    "power-board emergency or unavailable status; damping applied; leases revoked",
                );
            }
            GroupState::Fault => {
                if let Err(error) = self.send_damping() {
                    tracing::error!(group=%self.spec.name, %error, "emergency damping failed in fault state");
                }
            }
            GroupState::Idle | GroupState::Shutdown => {}
        }
        self.update_snapshot(0);
        self.emergency_pending.store(false, Ordering::Release);
        true
    }

    fn perform_shutdown(&mut self) {
        self.clear_command_state();
        if let Err(error) = self.respond(self.response(FaultCause::Exit)) {
            tracing::error!(group=%self.spec.name, %error, "failed to apply exit response during shutdown");
        }
        self.fsm.shutdown();
        self.publish(0);
    }

    fn handle(&mut self, request: LifecycleRequest) {
        match request {
            LifecycleRequest::Operation {
                action,
                lease_generation,
                reply,
            } => {
                if lease_generation == 0
                    || self.lease_generation.load(Ordering::Acquire) != lease_generation
                {
                    let state = self.fsm.state();
                    let _ = reply.send(super::Transition {
                        ok: false,
                        to: state,
                        message: "lease expired before operation execution".into(),
                    });
                    return;
                }
                let transition = self.handle_action(action, lease_generation);
                let _ = reply.send(transition);
            }
        }
    }

    fn handle_action(
        &mut self,
        action: LifecycleAction,
        lease_generation: u64,
    ) -> super::Transition {
        let transition = match action {
            LifecycleAction::Enable => {
                if let Some(rejection) = self.fsm.validate_enable() {
                    rejection
                } else if self.fsm.state() == GroupState::Damped {
                    self.fsm.enable()
                } else {
                    self.clear_command_state();
                    match self.enable_hardware() {
                        Ok(()) => {
                            self.establish_damping(GroupCause::Enable, "enabled with damping")
                        }
                        Err(EnableFailure::LeaseLost(error)) => self.fsm.idle(
                            GroupCause::LeaseLoss,
                            format!("enable interrupted by lease loss: {error}"),
                        ),
                        Err(EnableFailure::Shutdown(error)) => {
                            self.fsm.idle(GroupCause::Exit, error)
                        }
                        Err(EnableFailure::Failed(error)) => {
                            self.enter_fault(FaultCause::Enable, error)
                        }
                    }
                }
            }
            LifecycleAction::Start {
                batch,
                submitted_at,
            } => {
                if let Some(rejection) = self.fsm.validate_start() {
                    rejection
                } else {
                    match self.accept_commands(
                        batch,
                        lease_generation,
                        submitted_at,
                        GroupState::Damped,
                    ) {
                        Ok(()) => match self
                            .set_periodic_reporting(self.spec.periodic_feedback_during_control)
                        {
                            Ok(()) => {
                                let transition = self.fsm.start();
                                if self.spec.command_transmission == CommandTransmission::Periodic {
                                    self.control_tick(0, false);
                                    if self.fsm.state() == GroupState::Running {
                                        self.publish_periodic_targets();
                                    } else {
                                        return super::Transition {
                                            ok: false,
                                            to: self.fsm.state(),
                                            message: self.fsm.detail().into(),
                                        };
                                    }
                                }
                                transition
                            }
                            Err(error) => self.enter_fault(FaultCause::Transmission, error),
                        },
                        Err(error) => super::Transition {
                            ok: false,
                            to: self.fsm.state(),
                            message: error.to_string(),
                        },
                    }
                }
            }
            LifecycleAction::Stop => {
                if let Some(rejection) = self.fsm.validate_stop() {
                    rejection
                } else if self.fsm.state() == GroupState::Damped {
                    self.fsm.stop()
                } else {
                    self.clear_command_state();
                    self.establish_damping(GroupCause::Stop, "control stopped with damping")
                }
            }
            LifecycleAction::Disable => {
                if let Some(rejection) = self.fsm.validate_disable() {
                    rejection
                } else if self.fsm.state() == GroupState::Idle {
                    self.fsm.disable()
                } else {
                    self.clear_command_state();
                    match self.stop_motors(false) {
                        Ok(()) => self.fsm.disable(),
                        Err(error) => self.enter_fault(
                            FaultCause::Disable,
                            format!("failed to disable motors: {error}"),
                        ),
                    }
                }
            }
            LifecycleAction::Recover => {
                if let Some(rejection) = self.fsm.validate_recover() {
                    rejection
                } else {
                    self.clear_command_state();
                    match self.recover_hardware() {
                        Ok(()) => self.fsm.recover(),
                        Err(error) => self.enter_fault(
                            FaultCause::Recover,
                            format!("recovery guards failed: {error}"),
                        ),
                    }
                }
            }
        };
        self.update_snapshot(0);
        transition
    }

    fn handle_lease_loss(&mut self) {
        self.clear_command_state();
        let transition = match self.fsm.state() {
            GroupState::Running => {
                self.establish_damping(GroupCause::LeaseLoss, "control lease lost; damping applied")
            }
            GroupState::Damped => self.fsm.damp(
                GroupCause::LeaseLoss,
                "control lease lost; damping retained",
            ),
            state => super::Transition {
                ok: true,
                to: state,
                message: "lease released".into(),
            },
        };
        self.update_snapshot(0);
        for waiter in self.lease_loss_waiters.lock().drain(..) {
            let _ = waiter.send(transition.clone());
        }
    }

    fn clear_command_state(&mut self) {
        self.revoke_periodic_targets();
        if let Some(pending) = self.command_mailbox.lock().take() {
            let _ = pending.reply.send(Err(CommandError::Superseded));
        }
        for motor in &mut self.motors {
            motor.command = None;
        }
        self.command_deadline = None;
        self.command_generation = None;
        self.last_sequence = None;
    }

    fn enter_fault(&mut self, cause: FaultCause, reason: String) -> super::Transition {
        let group_cause = Self::group_cause(cause);
        let mut transition = self.latch_fault(group_cause, reason.clone());
        if let Err(error) = self.respond(self.response(cause)) {
            transition = self.fsm.fault(
                group_cause,
                format!("{reason}; fault response failed: {error}"),
            );
            self.update_snapshot(0);
        }
        transition
    }

    fn latch_fault(&mut self, cause: GroupCause, reason: String) -> super::Transition {
        self.clear_command_state();
        // Latch and publish the fault before a bounded response burst can block this worker.
        let transition = self.fsm.fault(cause, reason);
        self.update_snapshot(0);
        transition
    }

    fn establish_damping(&mut self, cause: GroupCause, success: &str) -> super::Transition {
        match self.send_damping() {
            Ok(()) => self.fsm.damp(cause, success),
            Err(error) => self.latch_fault(cause, format!("damping transmit failed: {error}")),
        }
    }

    pub(super) fn handle_worker_panic(&mut self) {
        self.enter_fault(FaultCause::Internal, "group controller panicked".into());
        self.update_snapshot(0);
    }

    fn enable_hardware(&mut self) -> Result<(), EnableFailure> {
        for attempt in 0..self.spec.enable_retries {
            for motor in &self.motors {
                self.check_enable_interrupted()?;
                motor
                    .controller
                    .configure_enable()
                    .map_err(|error| EnableFailure::Failed(error.to_string()))?;
            }
            let deadline = Instant::now() + self.spec.enable_timeout;
            while Instant::now() < deadline {
                self.check_enable_interrupted()?;
                let ready = self
                    .motors
                    .iter()
                    .all(|motor| motor.controller.feedback_ready());
                if ready {
                    self.validate_mechanical_feedback()
                        .map_err(EnableFailure::Failed)?;
                    return Ok(());
                }
                thread::sleep(Duration::from_millis(5));
            }
            if attempt + 1 < self.spec.enable_retries {
                let retry_at = Instant::now() + Duration::from_millis(20);
                while Instant::now() < retry_at {
                    self.check_enable_interrupted()?;
                    thread::sleep(Duration::from_millis(5));
                }
            }
        }
        Err(EnableFailure::Failed(
            "motors did not provide healthy feedback during enable".into(),
        ))
    }

    fn check_enable_interrupted(&self) -> Result<(), EnableFailure> {
        let interrupted = if self.shutdown.load(Ordering::Acquire) {
            Some((true, "shutdown requested during enable"))
        } else if self.emergency_pending.load(Ordering::Acquire) {
            Some((false, "power-board emergency interrupted enable"))
        } else if self.lease_lost.load(Ordering::Acquire) {
            Some((false, "control lease lost during enable"))
        } else {
            None
        };
        if let Some((shutdown, reason)) = interrupted {
            return match self.stop_motors(false) {
                Ok(()) if shutdown => Err(EnableFailure::Shutdown(reason.into())),
                Ok(()) => Err(EnableFailure::LeaseLost(reason.into())),
                Err(error) => Err(EnableFailure::Failed(format!(
                    "{reason}; motor stop failed: {error}"
                ))),
            };
        }
        Ok(())
    }

    fn recover_hardware(&self) -> Result<(), String> {
        if self.fsm.state() != GroupState::Fault {
            return Err("recover is only allowed from Fault".into());
        }
        self.stop_motors(true).map_err(|error| error.to_string())?;
        let deadline = Instant::now() + self.spec.enable_timeout;
        while Instant::now() < deadline {
            self.check_enable_interrupted()
                .map_err(EnableFailure::into_detail)?;
            // Keep the checked feedback and latches stable until the whole group passes recovery.
            if let Some(mut recovery) = self
                .motors
                .iter()
                .map(|motor| motor.controller.recovery_feedback())
                .collect::<Option<Vec<_>>>()
            {
                let feedback = self
                    .motors
                    .iter()
                    .zip(&recovery)
                    .map(|(motor, sample)| (motor.controller.spec.name.clone(), sample.feedback))
                    .collect();
                self.spec
                    .mechanical
                    .validate_feedback_positions(&feedback)
                    .map_err(|violation| violation.detail)?;
                for motor in &mut recovery {
                    motor.clear_latched_fault();
                }
                return Ok(());
            }
            thread::sleep(Duration::from_millis(5));
        }
        let unavailable = self
            .motors
            .iter()
            .filter(|motor| motor.controller.recovery_feedback().is_none())
            .map(|motor| {
                let status = motor.controller.status();
                format!(
                    "{}: {} ({})",
                    motor.controller.spec.name, status.state, status.detail
                )
            })
            .collect::<Vec<_>>()
            .join(", ");
        Err(format!("motors are not healthy: {unavailable}"))
    }

    fn accept_commands(
        &mut self,
        batch: CommandBatch,
        lease_generation: u64,
        submitted_at: Instant,
        required_state: GroupState,
    ) -> Result<(), CommandError> {
        if lease_generation == 0
            || self.lease_generation.load(Ordering::Acquire) != lease_generation
        {
            return Err(CommandError::LeaseLost);
        }
        if self.fsm.state() != required_state {
            return Err(CommandError::InvalidState);
        }
        let configured_timeout = self.spec.command_timeout;
        let valid_for = batch.valid_for.map_or(configured_timeout, |duration| {
            duration.min(configured_timeout)
        });
        let command_deadline = submitted_at + valid_for;
        if command_deadline <= Instant::now() {
            return Err(CommandError::Expired);
        }
        let same_generation = self.command_generation == Some(lease_generation);
        if same_generation
            && let Some(previous) = self
                .last_sequence
                .filter(|sequence| batch.sequence <= *sequence)
        {
            return Err(CommandError::StaleSequence {
                received: batch.sequence,
                previous,
            });
        }
        let mut supplied = HashMap::with_capacity(batch.commands.len());
        for command in batch.commands {
            if supplied.insert(command.motor, command.impedance).is_some() {
                return Err(CommandError::IncompleteGroup);
            }
        }
        if supplied.len() != self.motors.len() {
            return Err(CommandError::IncompleteGroup);
        }
        let mut staged = HashMap::with_capacity(self.motors.len());
        for motor in &self.motors {
            let Some(command) = supplied.remove(&motor.controller.spec.name) else {
                return Err(CommandError::IncompleteGroup);
            };
            let position_reference = select_position_reference(
                same_generation,
                motor.command.map(|previous| previous.position),
                motor
                    .controller
                    .feedback()
                    .map(|feedback| feedback.value.position),
            );
            let report = check_command(command, position_reference, &motor.controller.spec);
            let motor_name = motor.controller.spec.name.clone();
            for issue in &report.issues {
                let cause = self.command_fault_cause(issue.check);
                let response =
                    (issue.behavior == CommandBehavior::Fault).then(|| self.response(cause));
                self.publish_command_policy_event(&motor_name, issue, response);
                if issue.behavior == CommandBehavior::Fault {
                    let violation = issue.violation.clone();
                    self.enter_fault(
                        cause,
                        format!("{} {}: {}", motor_name, issue.check.as_str(), issue.detail),
                    );
                    self.update_snapshot(0);
                    return Err(CommandError::Unsafe {
                        motor: motor_name,
                        violation,
                    });
                }
                if matches!(issue.behavior, CommandBehavior::Reject) {
                    return Err(CommandError::Unsafe {
                        motor: motor_name,
                        violation: issue.violation.clone(),
                    });
                }
            }
            staged.insert(motor_name, report.command);
        }
        if let Err(violation) = self.spec.mechanical.validate_targets(&staged) {
            self.handle_mechanical_command(violation)?;
        }
        for motor in &mut self.motors {
            motor.command = staged.remove(&motor.controller.spec.name);
        }
        self.command_deadline = Some(command_deadline);
        self.command_count += 1;
        self.command_generation = Some(lease_generation);
        self.last_sequence = Some(batch.sequence);
        self.update_snapshot(0);
        Ok(())
    }

    fn control_tick(&mut self, jitter_us: i64, transmit_command: bool) {
        self.tick += 1;
        if self.handle_emergency() {
            return;
        }
        let state = self.fsm.state();
        if matches!(state, GroupState::Damped | GroupState::Running)
            && let Some((name, fault)) = self.motors.iter().find_map(|motor| {
                motor
                    .controller
                    .control_fault()
                    .map(|fault| (&motor.controller.spec.name, fault))
            })
        {
            self.enter_fault(
                self.motor_fault_cause(fault.cause),
                format!("motor {name} unavailable: {}", fault.detail),
            );
            self.update_snapshot(jitter_us);
            return;
        }
        if matches!(self.fsm.state(), GroupState::Damped | GroupState::Running)
            && let Err(detail) = self.validate_mechanical_feedback()
        {
            self.enter_fault(FaultCause::SafePosition, detail);
            self.update_snapshot(jitter_us);
            return;
        }
        if self.fsm.state() == GroupState::Running {
            if let Some(detail) = self
                .periodic_buses
                .iter()
                .find_map(|(control, _)| control.failure(Instant::now()))
            {
                self.enter_fault(FaultCause::Transmission, detail);
                self.update_snapshot(jitter_us);
                return;
            }
            if self
                .command_deadline
                .is_none_or(|deadline| Instant::now() >= deadline)
            {
                self.clear_command_state();
                self.establish_damping(
                    GroupCause::CommandTimeout,
                    "command watchdog expired; damping applied",
                );
                self.update_snapshot(jitter_us);
                return;
            }
            if transmit_command {
                for motor in &self.motors {
                    if let Some(command) = motor.command
                        && let Err(error) = motor.controller.send_control(command)
                    {
                        self.enter_fault(
                            FaultCause::Transmission,
                            format!(
                                "{} control transmit failed: {error}",
                                motor.controller.spec.name
                            ),
                        );
                        self.update_snapshot(jitter_us);
                        return;
                    }
                }
            }
        }
        self.update_snapshot(jitter_us);
    }

    fn validate_mechanical_feedback(&self) -> Result<(), String> {
        let feedback = self
            .motors
            .iter()
            .filter_map(|motor| {
                motor
                    .controller
                    .feedback()
                    .map(|value| (motor.controller.spec.name.clone(), value.value))
            })
            .collect();
        self.spec
            .mechanical
            .validate_feedback_positions(&feedback)
            .map_err(|violation| violation.detail)
    }

    fn mechanical_violation_behavior(&self, violation: &MechanicalViolation) -> CommandBehavior {
        coupled_limit_behavior(violation.affected_motors.iter().filter_map(|name| {
            self.motors
                .iter()
                .find(|motor| motor.controller.spec.name == *name)
                .map(|motor| &motor.controller.spec.command_policy)
        }))
    }

    fn handle_mechanical_command(
        &mut self,
        violation: MechanicalViolation,
    ) -> Result<(), CommandError> {
        let behavior = self.mechanical_violation_behavior(&violation);
        let issue = CommandPolicyIssue {
            check: CommandPolicyCheck::CommandLimit,
            behavior,
            violation: violation.violation.clone(),
            detail: violation.detail.clone(),
            command_modified: false,
            command_rejected: true,
        };
        self.publish_command_policy_event(
            &violation.motor,
            &issue,
            (behavior == CommandBehavior::Fault).then(|| self.response(FaultCause::CommandLimit)),
        );
        if behavior == CommandBehavior::Fault {
            self.enter_fault(FaultCause::CommandLimit, violation.detail);
            self.update_snapshot(0);
        }
        Err(CommandError::Unsafe {
            motor: violation.motor,
            violation: violation.violation,
        })
    }

    fn respond(&self, response: FaultResponse) -> Result<(), String> {
        match response {
            FaultResponse::Damping => self.send_damping(),
            FaultResponse::Disable => self.stop_motors(false),
        }
    }

    fn send_damping(&self) -> Result<(), String> {
        let commands = self.spec.mechanical.damping_commands();
        let safety = super::safety::send_damping(
            self.motors
                .iter()
                .map(|motor| (&motor.controller, commands[&motor.controller.spec.name])),
        );
        let reporting = self.set_periodic_reporting(true);
        safety.and(reporting)
    }

    fn stop_motors(&self, clear_fault: bool) -> Result<(), String> {
        let safety = super::safety::stop(
            self.motors.iter().map(|motor| &motor.controller),
            clear_fault,
        );
        let reporting = self.set_periodic_reporting(true);
        safety.and(reporting)
    }

    fn set_periodic_reporting(&self, enabled: bool) -> Result<(), String> {
        let mut errors = Vec::new();
        for motor in &self.motors {
            if let Err(error) = motor.controller.set_periodic_reporting(enabled) {
                errors.push(format!("{}: {error}", motor.controller.spec.name));
            }
        }
        if errors.is_empty() {
            Ok(())
        } else {
            Err(errors.join("; "))
        }
    }

    fn publish_periodic_targets(&self) {
        let deadline = self
            .command_deadline
            .expect("Running has a command deadline");
        let generation = self
            .command_generation
            .expect("Running has a command generation");
        for (control, indices) in &self.periodic_buses {
            let frames = indices
                .iter()
                .map(|index| {
                    let motor = &self.motors[*index];
                    motor
                        .controller
                        .control_frame(motor.command.expect("Running has complete targets"))
                })
                .collect();
            control.publish(frames, deadline, generation);
        }
    }

    fn revoke_periodic_targets(&self) {
        for (control, _) in &self.periodic_buses {
            control.revoke();
        }
    }
    fn command_fault_cause(&self, check: CommandPolicyCheck) -> FaultCause {
        match check {
            CommandPolicyCheck::CommandLimit => FaultCause::CommandLimit,
            CommandPolicyCheck::PositionStep => FaultCause::CommandPositionStep,
        }
    }
    fn motor_fault_cause(&self, cause: MotorFaultCause) -> FaultCause {
        match cause {
            MotorFaultCause::Communication => FaultCause::Communication,
            MotorFaultCause::StaleFeedback => FaultCause::StaleFeedback,
            MotorFaultCause::SafePosition => FaultCause::SafePosition,
            MotorFaultCause::Temperature => FaultCause::Temperature,
            MotorFaultCause::ReportedFault => FaultCause::ReportedMotorFault,
            MotorFaultCause::ModeStatus => FaultCause::ModeStatus,
        }
    }
    fn group_cause(cause: FaultCause) -> GroupCause {
        match cause {
            FaultCause::CommandLimit => GroupCause::CommandLimit,
            FaultCause::CommandPositionStep => GroupCause::CommandPositionStep,
            FaultCause::SafePosition => GroupCause::SafePosition,
            FaultCause::StaleFeedback => GroupCause::StaleFeedback,
            FaultCause::Temperature => GroupCause::Temperature,
            FaultCause::ReportedMotorFault => GroupCause::ReportedMotorFault,
            FaultCause::ModeStatus => GroupCause::ModeStatus,
            FaultCause::Communication => GroupCause::Communication,
            FaultCause::Enable => GroupCause::Enable,
            FaultCause::Transmission => GroupCause::Transmission,
            FaultCause::Disable => GroupCause::Disable,
            FaultCause::Recover => GroupCause::Recover,
            FaultCause::Exit => GroupCause::Exit,
            FaultCause::Internal => GroupCause::Internal,
        }
    }
    fn response(&self, cause: FaultCause) -> FaultResponse {
        self.spec.fault_policy.response(cause)
    }
    fn publish_command_policy_event(
        &self,
        motor: &crate::domain::MotorName,
        issue: &CommandPolicyIssue,
        response: Option<FaultResponse>,
    ) {
        self.telemetry
            .send(TelemetryPayload::CommandPolicy(CommandPolicyTelemetry {
                group: self.spec.name.clone(),
                motor: motor.clone(),
                check: issue.check,
                behavior: issue.behavior,
                detail: issue.detail.clone(),
                command_modified: issue.command_modified,
                command_rejected: issue.command_rejected,
                response,
            }));
    }
    fn update_snapshot(&mut self, jitter_us: i64) {
        let state = self.fsm.state();
        let cause = self.fsm.cause();
        let detail = self.fsm.detail().to_owned();
        *self.snapshot.write() = super::GroupSnapshot {
            state,
            cause,
            detail: detail.clone(),
            command_count: self.command_count,
            control_tick: self.tick,
            jitter_us,
            updated: Some(Instant::now()),
        };
        let changed = state != self.last_published_state
            || cause != self.last_published_cause
            || detail != self.last_published_detail;
        if changed || self.last_publish.elapsed() >= GROUP_TELEMETRY_PERIOD {
            self.publish(jitter_us);
            self.last_publish = Instant::now();
            self.last_published_state = state;
            self.last_published_cause = cause;
            self.last_published_detail = detail;
        }
    }
    fn publish(&self, jitter_us: i64) {
        self.telemetry.send(TelemetryPayload::Group(GroupTelemetry {
            group: self.spec.name.clone(),
            state: self.fsm.state(),
            cause: self.fsm.cause(),
            detail: self.fsm.detail().into(),
            command_count: self.command_count,
            control_tick: self.tick,
            jitter_us,
        }));
    }
}

#[cfg(test)]
mod tests {
    use std::time::{Duration, Instant};

    use super::CommandRefreshSchedule;

    #[test]
    fn emergency_clears_control_without_enabling_idle_motors_or_clearing_faults() {
        use super::*;
        let config = serde_yaml::from_str("name: test\nmotors: [motor]\n").unwrap();
        let spec = GroupSpec::try_from(&config).unwrap();
        let (_lifecycle_tx, lifecycle_rx) = mpsc::sync_channel(1);
        let (_wake_tx, wake_rx) = mpsc::sync_channel(1);
        let emergency_pending = Arc::new(AtomicBool::new(false));
        let mut controller = GroupController::new(
            spec,
            Vec::new(),
            ControllerInputs {
                lifecycle_rx,
                wake_rx,
                command_mailbox: Arc::new(Mutex::new(None)),
                lease_lost: Arc::new(AtomicBool::new(false)),
                emergency_pending: emergency_pending.clone(),
                lease_generation: Arc::new(AtomicU64::new(0)),
                lease_loss_waiters: Arc::new(Mutex::new(Vec::new())),
                shutdown: Arc::new(AtomicBool::new(false)),
                snapshot: Arc::new(RwLock::new(GroupSnapshot::default())),
            },
            Telemetry::new(16),
        );
        emergency_pending.store(true, Ordering::Release);
        assert!(controller.handle_emergency());
        assert_eq!(controller.fsm.state(), GroupState::Idle);
        controller.fsm.enable();
        controller.fsm.start();
        controller.command_deadline = Some(Instant::now() + Duration::from_secs(1));
        controller.command_generation = Some(1);
        controller.last_sequence = Some(42);
        emergency_pending.store(true, Ordering::Release);
        assert!(controller.handle_emergency());
        assert_eq!(controller.fsm.state(), GroupState::Damped);
        assert!(controller.command_deadline.is_none());
        assert!(controller.command_generation.is_none());
        assert!(controller.last_sequence.is_none());
        assert!(!controller.handle_emergency());
        assert_eq!(controller.fsm.state(), GroupState::Damped);
        controller
            .fsm
            .fault(GroupCause::Temperature, "overtemperature");
        emergency_pending.store(true, Ordering::Release);
        assert!(controller.handle_emergency());
        assert_eq!(controller.fsm.state(), GroupState::Fault);
        assert_eq!(controller.fsm.cause(), GroupCause::Temperature);
    }

    #[test]
    fn healthy_policy_commands_defer_periodic_refresh() {
        let start = Instant::now();
        let safety_period = Duration::from_millis(10);
        let mut schedule =
            CommandRefreshSchedule::new(start, Duration::from_millis(20), safety_period);

        for step in 0..50 {
            let command_at = start + Duration::from_millis(step * 20);
            schedule.record_immediate_send(command_at);
            assert!(!schedule.should_retransmit(command_at + safety_period));
            assert!(!schedule.should_retransmit(command_at + Duration::from_millis(20)));
        }
    }

    #[test]
    fn missed_policy_command_refreshes_after_one_safety_tick_of_grace() {
        let start = Instant::now();
        let mut schedule = CommandRefreshSchedule::new(
            start,
            Duration::from_millis(20),
            Duration::from_millis(10),
        );
        schedule.record_immediate_send(start);

        assert!(!schedule.should_retransmit(start + Duration::from_millis(20)));
        assert!(schedule.should_retransmit(start + Duration::from_millis(30)));
        assert!(!schedule.should_retransmit(start + Duration::from_millis(40)));
        assert!(schedule.should_retransmit(start + Duration::from_millis(50)));
    }
}

#[cfg(test)]
#[path = "group/fault_tests.rs"]
mod fault_tests;
