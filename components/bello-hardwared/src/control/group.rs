use std::collections::HashSet;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, Receiver};
use std::thread;
use std::time::{Duration, Instant};

use anyhow::Result;
use parking_lot::{Mutex, RwLock};

use crate::config::{CommandViolationSeverity, FaultResponse};
use crate::domain::{
    CommandBatch, CommandError, CommandPolicyTelemetry, GroupTelemetry, ImpedanceCommand,
    LeaseToken, TelemetryEvent, TelemetryPayload,
};
use crate::hardware::can::BusError;
use crate::hardware::robstride::{MotorController, MotorFaultCause};
use crate::telemetry::Telemetry;

use super::command_policy::{
    CommandPolicyCheck, CommandPolicyIssue, check_command, damping_command,
};
use super::fsm::{GroupFsm, GroupState};
use super::group_handle::{CommandSubmission, GroupSnapshot, LifecycleRequest};
use super::spec::GroupSpec;

const GROUP_TELEMETRY_PERIOD: Duration = Duration::from_millis(40);

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
    pub command_mailbox: Arc<Mutex<Option<CommandSubmission>>>,
    pub lease_lost: Arc<AtomicBool>,
    pub shutdown: Arc<AtomicBool>,
    pub snapshot: Arc<RwLock<GroupSnapshot>>,
}

pub(super) struct GroupController {
    spec: GroupSpec,
    motors: Vec<GroupMotor>,
    lifecycle_rx: Receiver<LifecycleRequest>,
    command_mailbox: Arc<Mutex<Option<CommandSubmission>>>,
    lease_lost: Arc<AtomicBool>,
    shutdown: Arc<AtomicBool>,
    fsm: GroupFsm,
    snapshot: Arc<RwLock<GroupSnapshot>>,
    telemetry: Telemetry,
    command_deadline: Option<Instant>,
    command_count: u64,
    command_token: Option<LeaseToken>,
    last_sequence: Option<u64>,
    tick: u64,
    last_publish: Instant,
    last_published_state: GroupState,
    last_published_fault: String,
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
            command_mailbox: inputs.command_mailbox,
            lease_lost: inputs.lease_lost,
            shutdown: inputs.shutdown,
            fsm: GroupFsm::default(),
            snapshot: inputs.snapshot,
            telemetry,
            command_deadline: None,
            command_count: 0,
            command_token: None,
            last_sequence: None,
            tick: 0,
            last_publish: Instant::now(),
            last_published_state: GroupState::Idle,
            last_published_fault: String::new(),
        }
    }

    pub(super) fn run(mut self) {
        let period = self.spec.period();
        let mut next_tick = Instant::now() + period;
        loop {
            if self.shutdown.load(Ordering::Acquire) {
                self.perform_shutdown();
                break;
            }
            if self.lease_lost.swap(false, Ordering::AcqRel) {
                self.handle_lease_loss();
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
                let result = self.accept_commands(submission.batch);
                let _ = submission.reply.send(result);
                continue;
            }
            let timeout = next_tick.saturating_duration_since(Instant::now());
            match self.lifecycle_rx.recv_timeout(timeout) {
                Ok(request) => self.handle(request),
                Err(mpsc::RecvTimeoutError::Timeout) => {}
                Err(mpsc::RecvTimeoutError::Disconnected) => {
                    self.perform_shutdown();
                    break;
                }
            }
        }
    }

    fn perform_shutdown(&mut self) {
        if let Err(error) = self.respond(self.spec.fault_responses.exit) {
            tracing::error!(group=%self.spec.name, %error, "failed to apply exit response during shutdown");
        }
        self.fsm.shutdown();
        self.publish(0);
    }

    fn handle(&mut self, request: LifecycleRequest) {
        match request {
            LifecycleRequest::Enable(reply) => {
                let result = self.enable_hardware();
                let transition = match result {
                    Ok(()) => self.fsm.enable(true),
                    Err(error) => self.enter_fault(error, self.spec.fault_responses.enable),
                };
                self.update_snapshot(0);
                let _ = reply.send(transition);
            }
            LifecycleRequest::Start(reply) => {
                let fresh = self
                    .command_deadline
                    .is_some_and(|deadline| deadline > Instant::now())
                    && self.motors.iter().all(|motor| motor.command.is_some());
                let result = if fresh {
                    self.fsm.start(true)
                } else {
                    self.enter_fault(
                        "initial command is missing, stale, or unsafe".into(),
                        self.spec.fault_responses.start,
                    )
                };
                self.update_snapshot(0);
                let _ = reply.send(result);
            }
            LifecycleRequest::Stop(reply) => {
                let result = match self.send_damping() {
                    Ok(()) => self.fsm.stop(),
                    Err(error) => self.enter_fault(
                        format!("failed to apply stop damping: {error}"),
                        self.spec.fault_responses.stop,
                    ),
                };
                self.update_snapshot(0);
                let _ = reply.send(result);
            }
            LifecycleRequest::Disable(reply) => {
                let result = match self.stop_motors(false) {
                    Ok(()) => self.fsm.disable(),
                    Err(error) => self.enter_fault(
                        format!("failed to disable motors: {error}"),
                        self.spec.fault_responses.disable,
                    ),
                };
                self.update_snapshot(0);
                let _ = reply.send(result);
            }
            LifecycleRequest::Recover(reply) => {
                let result = match self.recover_hardware() {
                    Ok(()) => self.fsm.recover(true),
                    Err(error) => self.enter_fault(
                        format!("recovery guards failed: {error}"),
                        self.spec.fault_responses.disable,
                    ),
                };
                self.update_snapshot(0);
                let _ = reply.send(result);
            }
            LifecycleRequest::Fault(reason, reply) => {
                let result = self.enter_fault(reason, self.spec.fault_responses.manual);
                self.update_snapshot(0);
                let _ = reply.send(result);
            }
            LifecycleRequest::Wake => {}
        }
    }

    fn handle_lease_loss(&mut self) {
        if matches!(self.fsm.state(), GroupState::Enabled | GroupState::Running) {
            self.enter_fault(
                "control lease lost".into(),
                self.spec.fault_responses.lease_loss,
            );
            self.update_snapshot(0);
        }
    }

    fn enter_fault(&mut self, reason: String, response: FaultResponse) -> super::Transition {
        // Latch and publish the fault before a bounded response burst can block this worker.
        let mut transition = self.fsm.fault(reason.clone());
        self.update_snapshot(0);
        if let Err(error) = self.respond(response) {
            transition = self
                .fsm
                .fault(format!("{reason}; fault response failed: {error}"));
            self.update_snapshot(0);
        }
        transition
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

    fn accept_commands(&mut self, batch: CommandBatch) -> Result<(), CommandError> {
        if self.lease_lost.load(Ordering::Acquire) {
            return Err(CommandError::LeaseLost);
        }
        if !matches!(self.fsm.state(), GroupState::Enabled | GroupState::Running) {
            return Err(CommandError::InvalidState);
        }
        let expected: HashSet<&str> = self
            .motors
            .iter()
            .map(|motor| motor.controller.spec.name.as_str())
            .collect();
        let supplied: HashSet<&str> = batch
            .commands
            .iter()
            .map(|command| command.motor.as_str())
            .collect();
        if supplied.len() != batch.commands.len() || supplied != expected {
            return Err(CommandError::IncompleteGroup);
        }
        if self.command_token == Some(batch.lease_token) {
            if let Some(previous) = self
                .last_sequence
                .filter(|sequence| batch.sequence <= *sequence)
            {
                return Err(CommandError::StaleSequence {
                    received: batch.sequence,
                    previous,
                });
            }
        } else {
            self.command_token = Some(batch.lease_token);
            self.last_sequence = None;
        }
        let mut staged = Vec::new();
        for motor in &self.motors {
            let command = batch
                .commands
                .iter()
                .find(|command| command.motor == motor.controller.spec.name)
                .expect("set equality checked");
            let report = check_command(
                command.impedance,
                motor
                    .controller
                    .feedback()
                    .map(|feedback| feedback.value.position),
                &motor.controller.spec,
            );
            let motor_name = motor.controller.spec.name.clone();
            for issue in &report.issues {
                let response = self.command_fault_response(issue.check);
                self.publish_command_policy_event(&motor_name, issue, response);
                if issue.severity == CommandViolationSeverity::Fault {
                    let violation = issue.violation.clone();
                    self.enter_fault(
                        format!("{} {}: {}", motor_name, issue.check.as_str(), issue.detail),
                        response,
                    );
                    self.update_snapshot(0);
                    return Err(CommandError::Unsafe {
                        motor: motor_name,
                        violation,
                    });
                }
                if issue.command_rejected {
                    return Err(CommandError::Unsafe {
                        motor: motor_name,
                        violation: issue.violation.clone(),
                    });
                }
            }
            staged.push(report.command);
        }
        for (motor, command) in self.motors.iter_mut().zip(staged) {
            motor.command = Some(command);
        }
        let configured_timeout = self.spec.command_timeout;
        let valid_for = batch.valid_for.map_or(configured_timeout, |duration| {
            duration.min(configured_timeout)
        });
        self.command_deadline = Some(Instant::now() + valid_for);
        self.command_count += 1;
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
            let response = fault
                .as_ref()
                .map_or(self.spec.fault_responses.communication, |fault| {
                    self.motor_fault_response(fault.cause)
                });
            let detail = fault
                .as_ref()
                .map_or(status.detail.as_str(), |fault| fault.detail.as_str());
            self.enter_fault(
                format!("motor {name} unavailable: {} ({})", status.state, detail),
                response,
            );
            self.update_snapshot(jitter_us);
            return;
        }
        if self.fsm.state() == GroupState::Running {
            if self
                .command_deadline
                .is_none_or(|deadline| Instant::now() >= deadline)
            {
                self.enter_fault(
                    "command watchdog expired".into(),
                    self.spec.fault_responses.command_timeout,
                );
                self.update_snapshot(jitter_us);
                return;
            }
            for motor in &self.motors {
                if let Some(command) = motor.command
                    && let Err(error) = motor.controller.send_control(command)
                {
                    self.enter_fault(
                        format!(
                            "{} control transmit failed: {error}",
                            motor.controller.spec.name
                        ),
                        self.spec.fault_responses.transmission,
                    );
                    self.update_snapshot(jitter_us);
                    return;
                }
            }
        }
        self.update_snapshot(jitter_us);
    }

    fn respond(&self, response: FaultResponse) -> Result<(), BusError> {
        match response {
            FaultResponse::None => Ok(()),
            FaultResponse::Damping => self.send_damping_burst(),
            FaultResponse::Disable => self.stop_motors(false),
        }
    }
    fn send_damping_burst(&self) -> Result<(), BusError> {
        for index in 0..self.spec.damping_send_count {
            self.send_damping()?;
            if index + 1 < self.spec.damping_send_count {
                thread::sleep(self.spec.damping_send_period);
            }
        }
        Ok(())
    }
    fn send_damping(&self) -> Result<(), BusError> {
        for motor in &self.motors {
            let feedback = motor.controller.feedback().map(|feedback| feedback.value);
            let command = damping_command(feedback, &motor.controller.spec);
            motor.controller.send_safety_command(command)?;
        }
        Ok(())
    }
    fn stop_motors(&self, clear_fault: bool) -> Result<(), BusError> {
        for motor in &self.motors {
            motor.controller.stop(clear_fault)?;
        }
        Ok(())
    }
    fn command_fault_response(&self, check: CommandPolicyCheck) -> FaultResponse {
        match check {
            CommandPolicyCheck::CommandLimit => self.spec.fault_responses.command_limit,
            CommandPolicyCheck::PositionStep => self.spec.fault_responses.command_position_step,
        }
    }
    fn motor_fault_response(&self, cause: MotorFaultCause) -> FaultResponse {
        match cause {
            MotorFaultCause::Communication => self.spec.fault_responses.communication,
            MotorFaultCause::StaleFeedback => self.spec.fault_responses.stale_feedback,
            MotorFaultCause::SafePosition => self.spec.fault_responses.safe_position,
            MotorFaultCause::Temperature => self.spec.fault_responses.temperature,
            MotorFaultCause::ReportedFault => self.spec.fault_responses.reported_motor_fault,
            MotorFaultCause::ModeStatus => self.spec.fault_responses.mode_status,
        }
    }
    fn publish_command_policy_event(
        &self,
        motor: &crate::domain::MotorName,
        issue: &CommandPolicyIssue,
        response: FaultResponse,
    ) {
        self.telemetry.send(TelemetryEvent {
            monotonic_ns: 0,
            payload: TelemetryPayload::CommandPolicy(CommandPolicyTelemetry {
                group: self.spec.name.clone(),
                motor: motor.clone(),
                check: issue.check.as_str().into(),
                severity: match issue.severity {
                    CommandViolationSeverity::Warning => "warning",
                    CommandViolationSeverity::Fault => "fault",
                }
                .into(),
                detail: issue.detail.clone(),
                command_modified: issue.command_modified,
                command_rejected: issue.command_rejected,
                response: fault_response_name(response).into(),
            }),
        });
    }
    fn update_snapshot(&mut self, jitter_us: i64) {
        let state = self.fsm.state();
        let fault_reason = self.fsm.fault_reason().to_owned();
        let mut snapshot = self.snapshot.write();
        *snapshot = GroupSnapshot {
            state,
            fault_reason: fault_reason.clone(),
        };
        drop(snapshot);
        let changed =
            state != self.last_published_state || fault_reason != self.last_published_fault;
        if changed || self.last_publish.elapsed() >= GROUP_TELEMETRY_PERIOD {
            self.publish(jitter_us);
            self.last_publish = Instant::now();
            self.last_published_state = state;
            self.last_published_fault = fault_reason;
        }
    }
    fn publish(&self, jitter_us: i64) {
        self.telemetry.send(TelemetryEvent {
            monotonic_ns: 0,
            payload: TelemetryPayload::Group(GroupTelemetry {
                group: self.spec.name.clone(),
                state: self.fsm.state(),
                fault_reason: self.fsm.fault_reason().into(),
                lease_owner: None,
                command_count: self.command_count,
                control_tick: self.tick,
                jitter_us,
            }),
        });
    }
}

fn fault_response_name(response: FaultResponse) -> &'static str {
    match response {
        FaultResponse::None => "none",
        FaultResponse::Damping => "damping",
        FaultResponse::Disable => "disable",
    }
}
