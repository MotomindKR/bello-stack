use crate::command::{CommandReceipt, CommandStream};
use crate::duration::requested_milliseconds;
use crate::lease::active_health;
use bello_hardware_api::{MotorCommand, StreamCommandsRequest};
use std::fmt;
use std::sync::Arc;
use std::time::Duration;
use tokio::sync::OwnedSemaphorePermit;

use tokio::sync::watch;
use tokio::task::JoinHandle;

use crate::{CommandFrame, EnumValue, Error, GroupLease, GroupState, MotorGroup};

#[derive(Clone, Debug)]
pub struct ControlOptions {
    owner: String,
    lease_duration: Option<Duration>,
    command_validity: Option<Duration>,
    max_pending: usize,
    acknowledgement_timeout: Duration,
    task_shutdown_timeout: Duration,
}

impl ControlOptions {
    pub fn new(owner: impl Into<String>) -> Self {
        Self {
            owner: owner.into(),
            lease_duration: None,
            command_validity: None,
            max_pending: 1,
            acknowledgement_timeout: Duration::from_secs(1),
            task_shutdown_timeout: Duration::from_secs(1),
        }
    }

    pub fn lease_duration(mut self, duration: Duration) -> Self {
        self.lease_duration = Some(duration);
        self
    }

    pub fn command_validity(mut self, duration: Duration) -> Self {
        self.command_validity = Some(duration);
        self
    }

    /// Bound on submitted commands whose acknowledgements have not arrived.
    pub fn max_pending(mut self, count: usize) -> Self {
        self.max_pending = count;
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
        if !(1..=tokio::sync::Semaphore::MAX_PERMITS).contains(&self.max_pending) {
            return Err(Error::InvalidArgument(
                "max_pending exceeds the command queue bounds".into(),
            ));
        }
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
pub(crate) enum RenewalStatus {
    Running,
    Failed(String),
}

/// Exclusive, automatically renewed control authority for one motor group.
pub struct ControlSession {
    group: MotorGroup,
    options: ControlOptions,
    lease: Arc<GroupLease>,
    command_stream: Option<CommandStream>,
    state: ControlState,
    next_sequence: Option<u64>,
    valid_for_ms: u64,
    renewal_cancel: watch::Sender<bool>,
    renewal_status: watch::Receiver<RenewalStatus>,
    renewal_task: Option<JoinHandle<()>>,
}

impl<T: Into<String>> From<T> for ControlOptions {
    fn from(owner: T) -> Self {
        Self::new(owner)
    }
}

impl MotorGroup {
    pub async fn control(
        &self,
        options: impl Into<ControlOptions>,
    ) -> Result<ControlSession, Error> {
        let options = options.into();
        options.validate()?;
        let valid_for_ms = requested_milliseconds(options.command_validity, "command validity")?;
        let requested_duration_ms =
            requested_milliseconds(options.lease_duration, "lease duration")?;
        let lease = Arc::new(
            GroupLease::acquire(
                self.robot.hardware.clone(),
                self.name(),
                &options.owner,
                requested_duration_ms,
            )
            .await?,
        );
        let (renewal_cancel, cancel_receiver) = watch::channel(false);
        let (renewal_status_sender, renewal_status) = watch::channel(RenewalStatus::Running);
        let renewal_task = tokio::spawn(renewal_loop(
            Arc::clone(&lease),
            cancel_receiver,
            renewal_status_sender,
        ));
        Ok(ControlSession {
            group: self.clone(),
            options,
            lease,
            command_stream: None,
            state: ControlState::Ready,
            next_sequence: Some(1),
            valid_for_ms,
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
        if self.state == ControlState::Running {
            self.command_stream
                .as_ref()
                .map_or(ControlState::Uncertain, CommandStream::state)
        } else {
            self.state
        }
    }

    /// Enables damping, then atomically starts control with the initial frame.
    pub async fn start(&mut self, initial: CommandFrame) -> Result<(), Error> {
        self.require_state(ControlState::Ready, "start")?;
        let initial = initial.into_commands(self.group.name())?;
        self.check_renewal()?;
        self.state = ControlState::Uncertain;
        self.lease.enable().await?;
        self.state = ControlState::Damped;

        if let Err(error) = self.open_stream().await {
            return Err(self.rollback("start", error).await);
        }
        self.start_control("start", initial).await
    }

    /// Atomically resumes a damped group with a fresh initial frame.
    pub async fn resume(&mut self, initial: CommandFrame) -> Result<(), Error> {
        self.require_state(ControlState::Damped, "resume")?;
        let initial = initial.into_commands(self.group.name())?;
        self.check_renewal()?;
        self.open_stream().await?;
        self.start_control("resume", initial).await
    }

    async fn open_stream(&mut self) -> Result<(), Error> {
        self.command_stream = Some(
            CommandStream::open(
                &self.lease,
                self.options.max_pending,
                self.renewal_status.clone(),
            )
            .await?,
        );
        Ok(())
    }

    async fn start_control(
        &mut self,
        operation: &'static str,
        initial: Vec<MotorCommand>,
    ) -> Result<(), Error> {
        self.state = ControlState::Uncertain;
        if let Err(error) = self
            .lease
            .start(operation, self.valid_for_ms, initial)
            .await
        {
            return Err(self.rollback(operation, error).await);
        }
        self.state = ControlState::Running;
        Ok(())
    }

    /// Waits for queue capacity, submits the frame, and waits for acknowledgement.
    pub async fn send(&mut self, frame: CommandFrame) -> Result<(), Error> {
        let slot = self.running_stream()?.slot().await?;
        self.submit(frame, slot)?.await
    }

    /// Submits immediately or returns `QueueFull`. The receipt resolves on acknowledgement.
    pub fn try_submit(&mut self, frame: CommandFrame) -> Result<CommandReceipt, Error> {
        let slot = self.running_stream()?.try_slot()?;
        self.submit(frame, slot)
    }

    fn running_stream(&self) -> Result<&CommandStream, Error> {
        self.require_state(ControlState::Running, "send a command")?;
        self.command_stream
            .as_ref()
            .ok_or(Error::CommandStreamClosed)
    }

    fn submit(
        &mut self,
        frame: CommandFrame,
        slot: OwnedSemaphorePermit,
    ) -> Result<CommandReceipt, Error> {
        self.check_renewal()?;
        active_health(&self.lease.health)?;
        let sequence = self.next_sequence.ok_or(Error::SequenceExhausted)?;
        let commands = frame.into_commands(self.group.name())?;
        let request = StreamCommandsRequest {
            group: self.group.name().into(),
            lease_token: self.lease.token.clone(),
            sequence,
            valid_for_ms: self.valid_for_ms,
            commands,
        };
        let receipt = self
            .command_stream
            .as_ref()
            .ok_or(Error::CommandStreamClosed)?
            .submit(slot, request, self.options.acknowledgement_timeout)?;
        self.next_sequence = sequence.checked_add(1);
        Ok(receipt)
    }

    /// Stops periodic control and leaves the group damped under the current lease.
    pub async fn stop(&mut self) -> Result<(), Error> {
        self.command_stream.take();
        self.state = ControlState::Uncertain;
        self.lease.stop().await?;
        self.state = ControlState::Damped;
        Ok(())
    }

    /// Disables the group directly and returns to `Ready` once the daemon confirms Idle.
    pub async fn disable(&mut self) -> Result<(), Error> {
        self.command_stream.take();
        self.state = ControlState::Uncertain;
        self.lease.disable().await?;
        self.state = ControlState::Ready;
        Ok(())
    }

    /// Recovers a faulted group and returns the session to `Ready`.
    pub async fn recover(&mut self) -> Result<(), Error> {
        self.command_stream.take();
        self.state = ControlState::Uncertain;
        self.check_renewal()?;
        self.lease.recover().await?;
        self.state = ControlState::Ready;
        Ok(())
    }

    /// Releases ownership with the daemon's damping reaction, without disabling motors.
    /// Running becomes Damped; Idle, Damped and a latched Fault are preserved.
    pub async fn release(mut self) -> Result<(), Error> {
        self.command_stream.take();
        self.stop_renewal().await;
        let result = self.lease.release().await;
        self.state = ControlState::Closed;
        let actual = result?;
        if matches!(
            actual,
            EnumValue::Known(GroupState::Idle | GroupState::Damped | GroupState::Fault)
        ) {
            Ok(())
        } else {
            Err(Error::UnexpectedGroupState {
                group: self.group.name().into(),
                operation: "release",
                expected: GroupState::Damped,
                actual,
            })
        }
    }

    /// Stops motion if needed, disables the group, stops renewal, and releases ownership.
    pub async fn shutdown(mut self) -> Result<(), Error> {
        self.state = self.state();
        self.command_stream.take();
        let mut errors = Vec::new();
        let mut faulted = false;

        if self.state == ControlState::Running
            && let Err(error) = self.lease.stop().await
        {
            faulted = self.confirm_fault(&error).await;
            if !faulted {
                errors.push(error.to_string());
            }
        }
        if !faulted && let Err(error) = self.lease.disable().await {
            faulted = self.confirm_fault(&error).await;
            if !faulted {
                errors.push(error.to_string());
            }
        }

        self.stop_renewal().await;
        match self.lease.release().await {
            Ok(actual) => {
                if actual != EnumValue::Known(GroupState::Idle)
                    && !(faulted && actual == EnumValue::Known(GroupState::Fault))
                {
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

    async fn confirm_fault(&self, error: &Error) -> bool {
        if !matches!(error, Error::PreconditionFailed(_)) {
            return false;
        }
        // Fault is already a latched daemon safety response. Releasing ownership
        // must preserve it; recovery is a separate, explicit operator action.
        self.group
            .robot
            .hardware
            .clone()
            .list_hardware(bello_hardware_api::ListHardwareRequest {})
            .await
            .is_ok_and(|inventory| {
                inventory.into_inner().groups.iter().any(|group| {
                    group.name == self.group.name()
                        && group.state == bello_hardware_api::GroupState::Fault as i32
                })
            })
    }

    async fn rollback(&mut self, operation: &str, primary: Error) -> Error {
        self.command_stream.take();
        self.state = ControlState::Uncertain;
        match self.lease.stop().await {
            Ok(()) => {
                self.state = ControlState::Damped;
                primary
            }
            Err(cleanup) => Error::CleanupFailed(vec![
                primary.to_string(),
                format!("{operation} rollback: {cleanup}"),
            ]),
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

    fn require_state(&self, expected: ControlState, operation: &'static str) -> Result<(), Error> {
        if self.state() == expected {
            Ok(())
        } else {
            Err(self.invalid_state(operation))
        }
    }

    fn invalid_state(&self, operation: &'static str) -> Error {
        Error::InvalidSessionState {
            group: self.group.name().into(),
            operation,
            state: self.state().description(),
        }
    }
}

impl fmt::Debug for ControlSession {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter
            .debug_struct("ControlSession")
            .field("group", &self.group.name())
            .field("state", &self.state())
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
    lease: Arc<GroupLease>,
    mut cancel: watch::Receiver<bool>,
    status: watch::Sender<RenewalStatus>,
) {
    loop {
        let wait = (lease.remaining() / 2).max(Duration::from_millis(1));
        tokio::select! {
            changed = cancel.changed() => {
                if changed.is_err() || *cancel.borrow() {
                    return;
                }
            }
            _ = tokio::time::sleep(wait) => {}
        }
        if let Err(error) = lease.renew().await {
            status.send_replace(RenewalStatus::Failed(error.to_string()));
            return;
        }
    }
}
