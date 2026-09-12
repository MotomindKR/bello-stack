use std::fmt;
use std::time::Duration;

use tokio::sync::watch;
use tokio::task::JoinHandle;

use crate::{
    CommandFrame, CommandResult, CommandStream, CommandStreamOptions, CommandValidity, Error,
    GroupLease, GroupState, LeaseDuration, MotorGroup,
};

/// Policy owned by one ergonomic motor-group control session.
#[derive(Clone, Debug)]
pub struct ControlOptions {
    owner: String,
    lease_duration: LeaseDuration,
    command_validity: CommandValidity,
    acknowledgement_timeout: Duration,
    task_shutdown_timeout: Duration,
}

impl ControlOptions {
    pub fn new(owner: impl Into<String>) -> Self {
        Self {
            owner: owner.into(),
            lease_duration: LeaseDuration::ServerDefault,
            command_validity: CommandValidity::GroupDefault,
            acknowledgement_timeout: Duration::from_secs(1),
            task_shutdown_timeout: Duration::from_secs(1),
        }
    }

    pub fn lease_duration(mut self, duration: Duration) -> Self {
        self.lease_duration = LeaseDuration::For(duration);
        self
    }

    pub fn command_validity(mut self, duration: Duration) -> Self {
        self.command_validity = CommandValidity::For(duration);
        self
    }

    pub fn acknowledgement_timeout(mut self, timeout: Duration) -> Self {
        self.acknowledgement_timeout = timeout;
        self
    }

    pub fn task_shutdown_timeout(mut self, timeout: Duration) -> Self {
        self.task_shutdown_timeout = timeout;
        self
    }

    fn validate(&self) -> Result<(), Error> {
        if self.acknowledgement_timeout.is_zero() {
            return Err(Error::InvalidArgument(
                "acknowledgement timeout must be positive".into(),
            ));
        }
        if self.task_shutdown_timeout.is_zero() {
            return Err(Error::InvalidArgument(
                "task shutdown timeout must be positive".into(),
            ));
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ControlState {
    Ready,
    Damped,
    Running,
    Uncertain,
    Closed,
}

impl ControlState {
    fn description(self) -> &'static str {
        match self {
            Self::Ready => "ready",
            Self::Damped => "damped",
            Self::Running => "running",
            Self::Uncertain => "uncertain",
            Self::Closed => "closed",
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
enum RenewalStatus {
    Running,
    Failed(String),
}

/// Exclusive, automatically renewed control authority for one motor group.
pub struct ControlSession {
    group: MotorGroup,
    options: ControlOptions,
    lease: GroupLease,
    command_stream: Option<CommandStream>,
    state: ControlState,
    renewal_cancel: watch::Sender<bool>,
    renewal_status: watch::Receiver<RenewalStatus>,
    renewal_task: Option<JoinHandle<()>>,
}

impl MotorGroup {
    pub async fn control(&self, owner: impl Into<String>) -> Result<ControlSession, Error> {
        self.control_with(ControlOptions::new(owner)).await
    }

    pub async fn control_with(&self, options: ControlOptions) -> Result<ControlSession, Error> {
        options.validate()?;
        let lease = self
            .robot
            .inner
            .hardware
            .acquire_lease(self.name(), &options.owner, options.lease_duration)
            .await?;
        let renewer = lease.renewer();
        let renewal_duration = options.lease_duration;
        let (renewal_cancel, cancel_receiver) = watch::channel(false);
        let (renewal_status_sender, renewal_status) = watch::channel(RenewalStatus::Running);
        let renewal_task = tokio::spawn(async move {
            renewal_loop(
                renewer,
                renewal_duration,
                cancel_receiver,
                renewal_status_sender,
            )
            .await;
        });
        Ok(ControlSession {
            group: self.clone(),
            options,
            lease,
            command_stream: None,
            state: ControlState::Ready,
            renewal_cancel,
            renewal_status,
            renewal_task: Some(renewal_task),
        })
    }
}

impl ControlSession {
    pub fn group(&self) -> &MotorGroup {
        &self.group
    }

    pub fn state(&self) -> ControlState {
        self.state
    }

    /// Enables damping, then atomically starts control with the initial frame.
    pub async fn start(&mut self, initial: CommandFrame) -> Result<(), Error> {
        self.require_state(ControlState::Ready, "start")?;
        let initial = initial.into_commands(self.group.name())?;
        self.check_renewal()?;
        self.state = ControlState::Uncertain;
        let actual = GroupState::from_wire(self.lease.enable().await?);
        self.expect_state("enable", GroupState::Damped, actual)?;
        self.state = ControlState::Damped;

        let stream = self
            .lease
            .open_command_stream(CommandStreamOptions::default())
            .await;
        self.command_stream = match stream {
            Ok(stream) => Some(stream),
            Err(error) => return Err(self.rollback_start(error).await),
        };

        self.state = ControlState::Uncertain;
        let actual = match self
            .lease
            .start(self.options.command_validity, initial)
            .await
        {
            Ok(state) => GroupState::from_wire(state),
            Err(error) => return Err(self.rollback_start(error).await),
        };
        if let Err(error) = self.expect_state("start", GroupState::Running, actual) {
            return Err(self.rollback_start(error).await);
        }
        self.state = ControlState::Running;
        Ok(())
    }

    /// Atomically resumes a damped group with a fresh initial frame.
    pub async fn resume(&mut self, initial: CommandFrame) -> Result<(), Error> {
        self.require_state(ControlState::Damped, "resume")?;
        let initial = initial.into_commands(self.group.name())?;
        self.check_renewal()?;
        self.command_stream = Some(
            self.lease
                .open_command_stream(CommandStreamOptions::default())
                .await?,
        );
        self.state = ControlState::Uncertain;
        let actual = match self
            .lease
            .start(self.options.command_validity, initial)
            .await
        {
            Ok(state) => GroupState::from_wire(state),
            Err(error) => {
                return Err(self.rollback_resume(error).await);
            }
        };
        if let Err(error) = self.expect_state("resume", GroupState::Running, actual) {
            return Err(self.rollback_resume(error).await);
        }
        self.state = ControlState::Running;
        Ok(())
    }

    /// Submits one complete frame and waits until the daemon accepts or rejects it.
    pub async fn send(&mut self, frame: CommandFrame) -> Result<(), Error> {
        self.require_state(ControlState::Running, "send a command")?;
        let commands = frame.into_commands(self.group.name())?;
        if let Err(error) = self.check_renewal() {
            self.command_stream.take();
            self.state = ControlState::Uncertain;
            return Err(error);
        }
        match self.send_and_confirm(commands).await {
            Ok(()) => Ok(()),
            Err(error) => {
                self.command_stream.take();
                self.state = if matches!(
                    &error,
                    Error::RejectedCommand {
                        state: GroupState::Damped,
                        ..
                    }
                ) {
                    ControlState::Damped
                } else {
                    ControlState::Uncertain
                };
                Err(error)
            }
        }
    }

    /// Stops periodic control and leaves the group damped under the current lease.
    pub async fn stop(&mut self) -> Result<(), Error> {
        if !matches!(self.state, ControlState::Running | ControlState::Uncertain) {
            return Err(self.invalid_state("stop"));
        }
        self.command_stream.take();
        self.state = ControlState::Uncertain;
        let actual = GroupState::from_wire(self.lease.stop().await?);
        self.expect_state("stop", GroupState::Damped, actual)?;
        self.state = ControlState::Damped;
        Ok(())
    }

    /// Disables a damped or authoritatively discovered group and returns to `Ready`.
    pub async fn disable(&mut self) -> Result<(), Error> {
        if !matches!(self.state, ControlState::Ready | ControlState::Damped) {
            return Err(self.invalid_state("disable"));
        }
        self.command_stream.take();
        let actual = GroupState::from_wire(self.lease.disable().await?);
        self.expect_state("disable", GroupState::Idle, actual)?;
        self.state = ControlState::Ready;
        Ok(())
    }

    /// Recovers a faulted group and returns the session to `Ready`.
    pub async fn recover(&mut self) -> Result<(), Error> {
        if self.state == ControlState::Closed {
            return Err(self.invalid_state("recover"));
        }
        self.command_stream.take();
        self.check_renewal()?;
        let actual = GroupState::from_wire(self.lease.recover().await?);
        self.expect_state("recover", GroupState::Idle, actual)?;
        self.state = ControlState::Ready;
        Ok(())
    }

    /// Stops motion if needed, disables the group, stops renewal, and releases ownership.
    pub async fn shutdown(mut self) -> Result<(), Error> {
        self.command_stream.take();
        let mut errors = Vec::new();

        if self.state == ControlState::Running {
            match self.lease.stop().await {
                Ok(state) => {
                    let actual = GroupState::from_wire(state);
                    if let Err(error) = self.expect_state("stop", GroupState::Damped, actual) {
                        errors.push(error.to_string());
                    }
                }
                Err(error) => errors.push(error.to_string()),
            }
        }
        // Disable is valid from Idle, Damped, and Running. Always ask the
        // authoritative daemon, including after an ambiguous lifecycle error.
        match self.lease.disable().await {
            Ok(state) => {
                let actual = GroupState::from_wire(state);
                if let Err(error) = self.expect_state("disable", GroupState::Idle, actual) {
                    errors.push(error.to_string());
                }
            }
            Err(error) => errors.push(error.to_string()),
        }

        self.stop_renewal().await;
        match self.lease.release().await {
            Ok(state) => {
                let actual = GroupState::from_wire(state);
                if actual != GroupState::Idle {
                    errors.push(
                        Error::UnexpectedGroupState {
                            group: self.group.name().into(),
                            operation: "release",
                            expected: GroupState::Idle,
                            actual,
                        }
                        .to_string(),
                    );
                }
            }
            Err(error) => errors.push(error.to_string()),
        }
        self.state = ControlState::Closed;
        if errors.is_empty() {
            Ok(())
        } else {
            Err(Error::CleanupFailed(errors))
        }
    }

    async fn send_and_confirm(
        &mut self,
        commands: Vec<bello_hardware_api::MotorCommand>,
    ) -> Result<(), Error> {
        self.check_renewal()?;
        if self.command_stream.is_none() {
            return Err(self.invalid_state("send a command"));
        }
        let stream = self
            .command_stream
            .as_mut()
            .expect("command stream presence was checked above");
        let sequence = stream.send(self.options.command_validity, commands).await?;
        let group = self.group.name().to_owned();
        let timeout = self.options.acknowledgement_timeout;
        let renewal = &mut self.renewal_status;
        let acknowledgement = tokio::select! {
            response = tokio::time::timeout(timeout, stream.message()) => {
                response
                    .map_err(|_| Error::AcknowledgementTimeout { group: group.clone() })??
                    .ok_or_else(|| Error::AcknowledgementStreamClosed(group.clone()))?
            }
            changed = renewal.changed() => {
                let detail = if changed.is_err() {
                    "renewal task stopped unexpectedly".into()
                } else {
                    match renewal.borrow().clone() {
                        RenewalStatus::Running => "renewal state changed unexpectedly".into(),
                        RenewalStatus::Failed(detail) => detail,
                    }
                };
                return Err(Error::RenewalFailed { group, detail });
            }
        };
        if acknowledgement.sequence != sequence {
            return Err(Error::AcknowledgementMismatch {
                group,
                expected: sequence,
                actual: acknowledgement.sequence,
            });
        }
        let result = CommandResult::from_wire(acknowledgement.result);
        if result != CommandResult::Accepted {
            return Err(Error::RejectedCommand {
                group,
                sequence,
                result,
                detail: acknowledgement.detail,
                state: GroupState::from_wire(acknowledgement.state),
            });
        }
        self.check_renewal()
    }

    async fn rollback_start(&mut self, primary: Error) -> Error {
        self.command_stream.take();
        self.state = ControlState::Uncertain;
        match self.lease.disable().await {
            Ok(state) if GroupState::from_wire(state) == GroupState::Idle => {
                self.state = ControlState::Ready;
                primary
            }
            Ok(state) => Error::CleanupFailed(vec![
                primary.to_string(),
                format!(
                    "start rollback returned {:?} instead of Idle",
                    GroupState::from_wire(state)
                ),
            ]),
            Err(cleanup) => Error::CleanupFailed(vec![primary.to_string(), cleanup.to_string()]),
        }
    }

    async fn rollback_resume(&mut self, primary: Error) -> Error {
        self.command_stream.take();
        self.state = ControlState::Uncertain;
        match self.lease.stop().await {
            Ok(state) if GroupState::from_wire(state) == GroupState::Damped => {
                self.state = ControlState::Damped;
                primary
            }
            Ok(state) => Error::CleanupFailed(vec![
                primary.to_string(),
                format!(
                    "resume rollback returned {:?} instead of Damped",
                    GroupState::from_wire(state)
                ),
            ]),
            Err(cleanup) => Error::CleanupFailed(vec![primary.to_string(), cleanup.to_string()]),
        }
    }

    async fn stop_renewal(&mut self) {
        let _ = self.renewal_cancel.send(true);
        let Some(mut task) = self.renewal_task.take() else {
            return;
        };
        if tokio::time::timeout(self.options.task_shutdown_timeout, &mut task)
            .await
            .is_err()
        {
            task.abort();
            let _ = task.await;
        }
    }

    fn check_renewal(&self) -> Result<(), Error> {
        match self.renewal_status.borrow().clone() {
            RenewalStatus::Running if self.renewal_status.has_changed().is_ok() => Ok(()),
            RenewalStatus::Running => Err(Error::RenewalFailed {
                group: self.group.name().into(),
                detail: "renewal task stopped unexpectedly".into(),
            }),
            RenewalStatus::Failed(detail) => Err(Error::RenewalFailed {
                group: self.group.name().into(),
                detail,
            }),
        }
    }

    fn expect_state(
        &self,
        operation: &'static str,
        expected: GroupState,
        actual: GroupState,
    ) -> Result<(), Error> {
        if actual == expected {
            Ok(())
        } else {
            Err(Error::UnexpectedGroupState {
                group: self.group.name().into(),
                operation,
                expected,
                actual,
            })
        }
    }

    fn require_state(&self, expected: ControlState, operation: &'static str) -> Result<(), Error> {
        if self.state == expected {
            Ok(())
        } else {
            Err(self.invalid_state(operation))
        }
    }

    fn invalid_state(&self, operation: &'static str) -> Error {
        Error::InvalidSessionState {
            group: self.group.name().into(),
            operation,
            state: self.state.description(),
        }
    }
}

impl fmt::Debug for ControlSession {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter
            .debug_struct("ControlSession")
            .field("group", &self.group.name())
            .field("state", &self.state)
            .field("lease", &self.lease)
            .finish_non_exhaustive()
    }
}

impl Drop for ControlSession {
    fn drop(&mut self) {
        let _ = self.renewal_cancel.send(true);
        if let Some(task) = self.renewal_task.take() {
            task.abort();
        }
    }
}

async fn renewal_loop(
    renewer: crate::lease::LeaseRenewer,
    duration: LeaseDuration,
    mut cancel: watch::Receiver<bool>,
    status: watch::Sender<RenewalStatus>,
) {
    loop {
        let wait = (renewer.deadline().remaining() / 2).max(Duration::from_millis(1));
        tokio::select! {
            changed = cancel.changed() => {
                if changed.is_err() || *cancel.borrow() {
                    return;
                }
            }
            _ = tokio::time::sleep(wait) => {}
        }
        if let Err(error) = renewer.renew(duration).await {
            status.send_replace(RenewalStatus::Failed(error.to_string()));
            return;
        }
    }
}
