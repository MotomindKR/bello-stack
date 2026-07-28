use std::collections::HashMap;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::mpsc::{self, Receiver};
use std::thread;
use std::time::{Duration, Instant};

use anyhow::Result;
use parking_lot::{Mutex, RwLock};

use crate::config::{CommandBehavior, FaultCause, FaultResponse};
use crate::domain::{
    CommandBatch, CommandError, CommandPolicyTelemetry, GroupTelemetry, ImpedanceCommand,
    TelemetryPayload,
};
use crate::hardware::robstride::{MotorController, MotorFaultCause};
use crate::mechanics::MechanicalViolation;
use crate::telemetry::Telemetry;

use super::command_policy::{
    CommandPolicyCheck, CommandPolicyIssue, check_command, select_position_reference,
};
use super::fsm::{GroupFsm, GroupState};
use super::group_handle::{CommandSubmission, GroupSnapshot, LifecycleAction, LifecycleRequest};
use super::spec::GroupSpec;

const GROUP_TELEMETRY_PERIOD: Duration = Duration::from_millis(40);

enum DampingOutcome {
    Applied,
    Stopped(String),
}

pub(super) struct GroupMotor {
    // The group owns motion policy; protocol and transport remain behind this handle.
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
    pub lease_generation: Arc<AtomicU64>,
    pub lease_loss_waiters: Arc<Mutex<Vec<tokio::sync::oneshot::Sender<super::Transition>>>>,
    pub shutdown: Arc<AtomicBool>,
    pub snapshot: Arc<RwLock<GroupSnapshot>>,
}

pub(super) struct GroupController {
    spec: GroupSpec,
    motors: Vec<GroupMotor>,
    lifecycle_rx: Receiver<LifecycleRequest>,
    wake_rx: Receiver<()>,
    command_mailbox: Arc<Mutex<Option<CommandSubmission>>>,
    lease_lost: Arc<AtomicBool>,
    lease_generation: Arc<AtomicU64>,
    lease_loss_waiters: Arc<Mutex<Vec<tokio::sync::oneshot::Sender<super::Transition>>>>,
    shutdown: Arc<AtomicBool>,
    fsm: GroupFsm,
    fault_cause: Option<FaultCause>,
    snapshot: Arc<RwLock<GroupSnapshot>>,
    telemetry: Telemetry,
    command_deadline: Option<Instant>,
    command_count: u64,
    command_generation: Option<u64>,
    last_sequence: Option<u64>,
    tick: u64,
    last_publish: Instant,
    last_published_state: GroupState,
    last_published_fault_cause: Option<FaultCause>,
    last_published_fault_detail: String,
}

impl GroupController {
    pub(super) fn new(
        spec: GroupSpec,
        motors: Vec<GroupMotor>,
        inputs: ControllerInputs,
        telemetry: Telemetry,
    ) -> Self {
        Self {
            spec,
            motors,
            lifecycle_rx: inputs.lifecycle_rx,
            wake_rx: inputs.wake_rx,
            command_mailbox: inputs.command_mailbox,
            lease_lost: inputs.lease_lost,
            lease_generation: inputs.lease_generation,
            lease_loss_waiters: inputs.lease_loss_waiters,
            shutdown: inputs.shutdown,
            fsm: GroupFsm::default(),
            fault_cause: None,
            snapshot: inputs.snapshot,
            telemetry,
            command_deadline: None,
            command_count: 0,
            command_generation: None,
            last_sequence: None,
            tick: 0,
            last_publish: Instant::now(),
            last_published_state: GroupState::Idle,
            last_published_fault_cause: None,
            last_published_fault_detail: String::new(),
        }
    }

    pub(super) fn run(&mut self) {
        let period = self.spec.period();
        let mut next_tick = Instant::now() + period;
        loop {
            if self.shutdown.load(Ordering::Acquire) {
                self.perform_shutdown();
                break;
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
            if self.lease_lost.swap(false, Ordering::AcqRel) {
                self.handle_lease_loss();
                continue;
            }
            let now = Instant::now();
            if now >= next_tick {
                let jitter = now
                    .saturating_duration_since(next_tick)
                    .as_micros()
                    .min(i64::MAX as u128) as i64;
                self.control_tick(jitter);
                next_tick += period;
                if next_tick < now {
                    next_tick = now + period;
                }
                continue;
            }
            let pending_command = { self.command_mailbox.lock().take() };
            if let Some(submission) = pending_command {
                let result = self.accept_commands(
                    submission.batch,
                    submission.lease_generation,
                    submission.submitted_at,
                );
                let _ = submission.reply.send(result);
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

    fn perform_shutdown(&mut self) {
        self.clear_command_state();
        if let Err(error) = self.respond(self.response(FaultCause::Exit)) {
            tracing::error!(group=%self.spec.name, %error, "failed to apply exit response during shutdown");
        }
        self.fault_cause = None;
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
                } else {
                    self.clear_command_state();
                    match self.enable_hardware() {
                        Ok(()) => {
                            self.fault_cause = None;
                            self.fsm.enable()
                        }
                        Err(error) => self.enter_fault(FaultCause::Enable, error),
                    }
                }
            }
            LifecycleAction::Start => {
                if let Some(rejection) = self.fsm.validate_start() {
                    rejection
                } else {
                    let fresh = self
                        .command_deadline
                        .is_some_and(|deadline| deadline > Instant::now())
                        && self.command_generation == Some(lease_generation)
                        && self.motors.iter().all(|motor| motor.command.is_some());
                    if fresh {
                        self.fsm.start()
                    } else {
                        self.enter_fault(
                            FaultCause::Start,
                            "initial command is missing, stale, or unsafe".into(),
                        )
                    }
                }
            }
            LifecycleAction::Stop => {
                if let Some(rejection) = self.fsm.validate_stop() {
                    rejection
                } else {
                    self.clear_command_state();
                    match self.send_damping() {
                        Ok(DampingOutcome::Applied) => self.fsm.stop(),
                        Ok(DampingOutcome::Stopped(reason)) => self.latch_fault(
                            FaultCause::Stop,
                            format!(
                                "safe stop damping was unavailable ({reason}); motors were stopped"
                            ),
                        ),
                        Err(error) => self.enter_fault(
                            FaultCause::Stop,
                            format!("failed to apply stop damping: {error}"),
                        ),
                    }
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
                        Ok(()) => {
                            self.fault_cause = None;
                            self.fsm.recover()
                        }
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
        let transition = if matches!(self.fsm.state(), GroupState::Enabled | GroupState::Running) {
            let transition = self.enter_fault(FaultCause::LeaseLoss, "control lease lost".into());
            self.update_snapshot(0);
            transition
        } else {
            let state = self.fsm.state();
            super::Transition {
                ok: true,
                to: state,
                message: "lease released".into(),
            }
        };
        for waiter in self.lease_loss_waiters.lock().drain(..) {
            let _ = waiter.send(transition.clone());
        }
    }

    fn clear_command_state(&mut self) {
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
        let mut transition = self.latch_fault(cause, reason.clone());
        if let Err(error) = self.respond(self.response(cause)) {
            transition = self
                .fsm
                .fault(format!("{reason}; fault response failed: {error}"));
            self.update_snapshot(0);
        }
        transition
    }

    fn latch_fault(&mut self, cause: FaultCause, reason: String) -> super::Transition {
        self.clear_command_state();
        // Latch and publish the fault before a bounded response burst can block this worker.
        self.fault_cause = Some(cause);
        let transition = self.fsm.fault(reason);
        self.update_snapshot(0);
        transition
    }

    pub(super) fn handle_worker_panic(&mut self) {
        self.clear_command_state();
        self.enter_fault(FaultCause::Internal, "group controller panicked".into());
        self.update_snapshot(0);
    }

    fn enable_hardware(&mut self) -> Result<(), String> {
        for attempt in 0..self.spec.enable_retries {
            for motor in &self.motors {
                self.check_enable_interrupted()?;
                motor
                    .controller
                    .configure_enable()
                    .map_err(|e| e.to_string())?;
            }
            let deadline = Instant::now() + self.spec.enable_timeout;
            while Instant::now() < deadline {
                self.check_enable_interrupted()?;
                let ready = self
                    .motors
                    .iter()
                    .all(|motor| motor.controller.feedback_ready());
                if ready {
                    self.validate_mechanical_feedback()?;
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
        Err("motors did not provide healthy feedback during enable".into())
    }

    fn check_enable_interrupted(&self) -> Result<(), String> {
        let reason = if self.shutdown.load(Ordering::Acquire) {
            Some("shutdown requested during enable")
        } else if self.lease_lost.load(Ordering::Acquire) {
            Some("control lease lost during enable")
        } else {
            None
        };
        if let Some(reason) = reason {
            return match self.stop_motors(false) {
                Ok(()) => Err(reason.into()),
                Err(error) => Err(format!("{reason}; motor stop failed: {error}")),
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
            self.check_enable_interrupted()?;
            if self
                .motors
                .iter()
                .all(|motor| motor.controller.recovery_ready())
            {
                self.validate_mechanical_feedback()?;
                return Ok(());
            }
            thread::sleep(Duration::from_millis(5));
        }
        let unavailable = self
            .motors
            .iter()
            .filter(|motor| !motor.controller.recovery_ready())
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
    ) -> Result<(), CommandError> {
        if lease_generation == 0
            || self.lease_generation.load(Ordering::Acquire) != lease_generation
        {
            return Err(CommandError::LeaseLost);
        }
        if !matches!(self.fsm.state(), GroupState::Enabled | GroupState::Running) {
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
        if let Err(violation) = self.spec.mechanical.validate_commands(&staged, &feedback) {
            return self.reject_mechanical_command(violation);
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

    fn control_tick(&mut self, jitter_us: i64) {
        self.tick += 1;
        let state = self.fsm.state();
        if matches!(state, GroupState::Enabled | GroupState::Running)
            && let Some((name, status, fault)) = self.motors.iter().find_map(|motor| {
                let status = motor.controller.status();
                (!status.state.is_online()).then(|| {
                    (
                        motor.controller.spec.name.clone(),
                        status,
                        motor.controller.fault(),
                    )
                })
            })
        {
            let cause = fault.as_ref().map_or(FaultCause::Communication, |fault| {
                self.motor_fault_cause(fault.cause)
            });
            let detail = fault
                .as_ref()
                .map_or(status.detail.as_str(), |fault| fault.detail.as_str());
            self.enter_fault(
                cause,
                format!("motor {name} unavailable: {} ({})", status.state, detail),
            );
            self.update_snapshot(jitter_us);
            return;
        }
        if matches!(self.fsm.state(), GroupState::Enabled | GroupState::Running)
            && let Err(detail) = self.validate_mechanical_feedback()
        {
            self.enter_fault(FaultCause::SafePosition, detail);
            self.update_snapshot(jitter_us);
            return;
        }
        if self.fsm.state() == GroupState::Running {
            if self
                .command_deadline
                .is_none_or(|deadline| Instant::now() >= deadline)
            {
                self.enter_fault(
                    FaultCause::CommandTimeout,
                    "command watchdog expired".into(),
                );
                self.update_snapshot(jitter_us);
                return;
            }
            let commands = self
                .motors
                .iter()
                .filter_map(|motor| {
                    motor
                        .command
                        .map(|command| (motor.controller.spec.name.clone(), command))
                })
                .collect();
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
            if let Err(violation) = self.spec.mechanical.validate_commands(&commands, &feedback) {
                self.enter_fault(FaultCause::CommandLimit, violation.detail);
                self.update_snapshot(jitter_us);
                return;
            }
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

    fn reject_mechanical_command(
        &mut self,
        violation: MechanicalViolation,
    ) -> Result<(), CommandError> {
        let fault = violation.affected_motors.iter().any(|name| {
            self.motors.iter().any(|motor| {
                motor.controller.spec.name == *name
                    && motor.controller.spec.command_policy.limit == CommandBehavior::Fault
            })
        });
        let behavior = if fault {
            CommandBehavior::Fault
        } else {
            CommandBehavior::Reject
        };
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
            fault.then(|| self.response(FaultCause::CommandLimit)),
        );
        if fault {
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
            FaultResponse::Damping => {
                if let DampingOutcome::Stopped(reason) = self.send_damping()? {
                    tracing::warn!(
                        group = %self.spec.name,
                        %reason,
                        "safe damping was unavailable; sent motor stop frames"
                    );
                }
                Ok(())
            }
            FaultResponse::Disable => self.stop_motors(false),
        }
    }

    fn send_damping(&self) -> Result<DampingOutcome, String> {
        let feedback = self
            .motors
            .iter()
            .map(|motor| {
                let name = motor.controller.spec.name.clone();
                let measured = motor.controller.feedback().ok_or_else(|| {
                    format!("motor {name} has no feedback for damping torque validation")
                })?;
                if measured.received.elapsed() > self.spec.feedback_timeout {
                    return Err(format!(
                        "motor {name} feedback is stale for damping torque validation"
                    ));
                }
                Ok((name, measured.value))
            })
            .collect::<Result<HashMap<_, _>, String>>();
        let commands = feedback.and_then(|feedback| {
            self.spec
                .mechanical
                .damping_commands(&feedback)
                .map_err(|violation| violation.detail)
        });
        let commands = match commands {
            Ok(commands) => commands,
            Err(reason) => return self.stop_after_damping_failure(reason),
        };
        match super::safety::send_damping(
            self.motors
                .iter()
                .map(|motor| (&motor.controller, commands[&motor.controller.spec.name])),
        ) {
            Ok(()) => Ok(DampingOutcome::Applied),
            Err(error) => {
                self.stop_after_damping_failure(format!("damping transmit failed: {error}"))
            }
        }
    }

    fn stop_after_damping_failure(&self, reason: String) -> Result<DampingOutcome, String> {
        self.stop_motors(false)
            .map(|()| DampingOutcome::Stopped(reason.clone()))
            .map_err(|error| {
                format!("safe damping unavailable ({reason}); fallback motor stop failed: {error}")
            })
    }

    fn stop_motors(&self, clear_fault: bool) -> Result<(), String> {
        super::safety::stop(
            self.motors.iter().map(|motor| &motor.controller),
            clear_fault,
        )
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
        let fault_detail = self.fsm.fault_detail().to_owned();
        self.snapshot.write().state = state;
        let fault_cause = (state == GroupState::Fault)
            .then_some(self.fault_cause)
            .flatten();
        let changed = state != self.last_published_state
            || fault_cause != self.last_published_fault_cause
            || fault_detail != self.last_published_fault_detail;
        if changed || self.last_publish.elapsed() >= GROUP_TELEMETRY_PERIOD {
            self.publish(jitter_us);
            self.last_publish = Instant::now();
            self.last_published_state = state;
            self.last_published_fault_cause = fault_cause;
            self.last_published_fault_detail = fault_detail;
        }
    }
    fn publish(&self, jitter_us: i64) {
        self.telemetry.send(TelemetryPayload::Group(GroupTelemetry {
            group: self.spec.name.clone(),
            state: self.fsm.state(),
            fault_cause: (self.fsm.state() == GroupState::Fault)
                .then_some(self.fault_cause)
                .flatten(),
            fault_detail: self.fsm.fault_detail().into(),
            command_count: self.command_count,
            control_tick: self.tick,
            jitter_us,
        }));
    }
}
