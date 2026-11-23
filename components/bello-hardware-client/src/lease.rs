use std::fmt;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU64};
use std::time::{Duration, Instant};

use bello_hardware_api::{
    AcquireLeaseResponse, DisableGroupRequest, EnableGroupRequest, GroupState, MotorCommand,
    RecoverGroupRequest, ReleaseLeaseRequest, RenewLeaseRequest, StartControlRequest,
    StopControlRequest,
};
use tokio::sync::{Mutex, watch};
use tonic::Code;

use crate::client::conservative_deadline;
use crate::command::{self, CommandStream, CommandStreamOptions};
use crate::duration::positive_milliseconds;
use crate::{CommandValidity, EnumValue, Error, HardwareClient};

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum LeaseDuration {
    /// Let the daemon select its configured default.
    #[default]
    ServerDefault,
    /// Ask for this duration. The daemon clamps it to its supported range.
    For(Duration),
}

impl LeaseDuration {
    pub(crate) fn to_milliseconds(self) -> Result<u64, Error> {
        match self {
            Self::ServerDefault => Ok(0),
            Self::For(duration) => positive_milliseconds(duration, "lease duration"),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LeaseStatus {
    Active,
    Releasing,
    Released,
    Expired,
    Lost,
    Dropped,
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct LeaseHealth {
    pub(crate) status: LeaseStatus,
    pub(crate) expires_at: Instant,
}

/// Conservative local estimate of the server lease deadline.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LeaseDeadline {
    expires_at: Instant,
}

impl LeaseDeadline {
    pub fn expires_at(self) -> Instant {
        self.expires_at
    }

    pub fn remaining(self) -> Duration {
        self.expires_at.saturating_duration_since(Instant::now())
    }
}

/// Exclusive write authority for one motor group.
///
/// The bearer token is intentionally private and redacted from `Debug`.
/// Dropping this value performs no asynchronous RPC; an unreleased server
/// lease expires according to daemon policy.
pub struct GroupLease {
    client: HardwareClient,
    group: String,
    token: String,
    health: watch::Sender<LeaseHealth>,
    renew_lock: Arc<Mutex<()>>,
    command_stream_open: Arc<AtomicBool>,
    next_command_sequence: Arc<AtomicU64>,
    command_sequence_exhausted: Arc<AtomicBool>,
}

impl GroupLease {
    pub(crate) fn from_acquired(
        client: HardwareClient,
        requested_group: String,
        request_started: Instant,
        response: AcquireLeaseResponse,
    ) -> Result<Self, Error> {
        if response.group != requested_group {
            return Err(Error::Protocol(format!(
                "lease response group {:?} does not match requested group {:?}",
                response.group, requested_group
            )));
        }
        if response.token.is_empty() {
            return Err(Error::Protocol(
                "lease response contains an empty token".into(),
            ));
        }
        let expires_at = conservative_deadline(request_started, response.expires_in_ms)?;
        let (health, _) = watch::channel(LeaseHealth {
            status: LeaseStatus::Active,
            expires_at,
        });
        Ok(Self {
            client,
            group: requested_group,
            token: response.token,
            health,
            renew_lock: Arc::new(Mutex::new(())),
            command_stream_open: Arc::new(AtomicBool::new(false)),
            next_command_sequence: Arc::new(AtomicU64::new(1)),
            command_sequence_exhausted: Arc::new(AtomicBool::new(false)),
        })
    }

    pub fn group(&self) -> &str {
        &self.group
    }

    pub fn status(&self) -> LeaseStatus {
        self.current_health().status
    }

    pub fn deadline(&self) -> LeaseDeadline {
        LeaseDeadline {
            expires_at: self.current_health().expires_at,
        }
    }

    pub fn remaining(&self) -> Duration {
        let health = self.current_health();
        if health.status == LeaseStatus::Active {
            health.expires_at.saturating_duration_since(Instant::now())
        } else {
            Duration::ZERO
        }
    }

    /// Renews the lease without monopolizing the underlying gRPC client, so it
    /// can run concurrently with a long enable or recover call.
    pub async fn renew(&self, duration: LeaseDuration) -> Result<LeaseDeadline, Error> {
        self.renewer().renew(duration).await
    }

    /// Returns a restricted, cloneable capability suitable for a dedicated
    /// renewal task. It can only renew this lease; it cannot move the group or
    /// submit commands.
    pub fn renewer(&self) -> LeaseRenewer {
        LeaseRenewer {
            client: self.client.clone(),
            group: self.group.clone(),
            token: self.token.clone(),
            health: self.health.clone(),
            renew_lock: Arc::clone(&self.renew_lock),
        }
    }

    pub async fn enable(&self) -> Result<EnumValue<GroupState>, Error> {
        self.ensure_active()?;
        let mut client = self.client.inner.clone();
        let response = client
            .enable_group(EnableGroupRequest {
                group: self.group.clone(),
                lease_token: self.token.clone(),
            })
            .await
            .map_err(|status| self.rpc_error(status))?
            .into_inner();
        Ok(EnumValue::from_raw(response.state))
    }

    pub async fn start(
        &self,
        validity: CommandValidity,
        commands: Vec<MotorCommand>,
    ) -> Result<EnumValue<GroupState>, Error> {
        self.ensure_active()?;
        let mut client = self.client.inner.clone();
        let response = client
            .start_control(StartControlRequest {
                group: self.group.clone(),
                lease_token: self.token.clone(),
                valid_for_ms: validity.to_milliseconds()?,
                commands,
            })
            .await
            .map_err(|status| self.rpc_error(status))?
            .into_inner();
        Ok(EnumValue::from_raw(response.state))
    }

    pub async fn stop(&self) -> Result<EnumValue<GroupState>, Error> {
        self.ensure_cleanup_authority()?;
        let mut client = self.client.inner.clone();
        let response = client
            .stop_control(StopControlRequest {
                group: self.group.clone(),
                lease_token: self.token.clone(),
            })
            .await
            .map_err(|status| self.rpc_error(status))?
            .into_inner();
        Ok(EnumValue::from_raw(response.state))
    }

    pub async fn disable(&self) -> Result<EnumValue<GroupState>, Error> {
        self.ensure_cleanup_authority()?;
        let mut client = self.client.inner.clone();
        let response = client
            .disable_group(DisableGroupRequest {
                group: self.group.clone(),
                lease_token: self.token.clone(),
            })
            .await
            .map_err(|status| self.rpc_error(status))?
            .into_inner();
        Ok(EnumValue::from_raw(response.state))
    }

    pub async fn recover(&self) -> Result<EnumValue<GroupState>, Error> {
        self.ensure_cleanup_authority()?;
        let mut client = self.client.inner.clone();
        let response = client
            .recover_group(RecoverGroupRequest {
                group: self.group.clone(),
                lease_token: self.token.clone(),
            })
            .await
            .map_err(|status| self.rpc_error(status))?
            .into_inner();
        Ok(EnumValue::from_raw(response.state))
    }

    /// Releases ownership and waits for the daemon's configured lease-release
    /// reaction. Normally call `stop`, then `disable`, before this operation.
    pub async fn release(&mut self) -> Result<EnumValue<GroupState>, Error> {
        let _renew = self.renew_lock.lock().await;
        self.ensure_cleanup_authority()?;
        self.health.send_if_modified(|health| {
            if matches!(health.status, LeaseStatus::Active | LeaseStatus::Expired) {
                health.status = LeaseStatus::Releasing;
                true
            } else {
                false
            }
        });
        let mut client = self.client.inner.clone();
        let response = match client
            .release_lease(ReleaseLeaseRequest {
                token: self.token.clone(),
            })
            .await
        {
            Ok(response) => response.into_inner(),
            Err(status) => {
                self.set_terminal(LeaseStatus::Lost);
                return Err(status.into());
            }
        };
        self.set_released();
        Ok(EnumValue::from_raw(response.state))
    }

    pub async fn open_command_stream(
        &self,
        options: CommandStreamOptions,
    ) -> Result<CommandStream, Error> {
        self.ensure_active()?;
        command::open(self, options).await
    }

    pub(crate) fn client(&self) -> &HardwareClient {
        &self.client
    }

    pub(crate) fn token(&self) -> &str {
        &self.token
    }

    pub(crate) fn health_sender(&self) -> watch::Sender<LeaseHealth> {
        self.health.clone()
    }

    pub(crate) fn command_stream_open_flag(&self) -> Arc<AtomicBool> {
        Arc::clone(&self.command_stream_open)
    }

    pub(crate) fn command_sequence(&self) -> (Arc<AtomicU64>, Arc<AtomicBool>) {
        (
            Arc::clone(&self.next_command_sequence),
            Arc::clone(&self.command_sequence_exhausted),
        )
    }

    fn current_health(&self) -> LeaseHealth {
        current_health(&self.health)
    }

    fn ensure_active(&self) -> Result<(), Error> {
        let status = self.current_health().status;
        if status == LeaseStatus::Active {
            Ok(())
        } else {
            Err(Error::LeaseInactive(status))
        }
    }

    fn ensure_cleanup_authority(&self) -> Result<(), Error> {
        let status = self.current_health().status;
        if matches!(status, LeaseStatus::Active | LeaseStatus::Expired) {
            Ok(())
        } else {
            Err(Error::LeaseInactive(status))
        }
    }

    fn record_rpc_failure(&self, status: &tonic::Status) {
        record_rpc_failure(&self.health, status);
    }

    fn rpc_error(&self, status: tonic::Status) -> Error {
        self.record_rpc_failure(&status);
        status.into()
    }

    fn set_terminal(&self, status: LeaseStatus) {
        set_terminal(&self.health, status);
    }

    fn set_released(&self) {
        self.health.send_if_modified(|health| {
            if health.status == LeaseStatus::Released {
                false
            } else {
                health.status = LeaseStatus::Released;
                true
            }
        });
    }
}

/// Restricted capability for renewing a [`GroupLease`] from another task.
#[derive(Clone)]
pub struct LeaseRenewer {
    client: HardwareClient,
    group: String,
    token: String,
    health: watch::Sender<LeaseHealth>,
    renew_lock: Arc<Mutex<()>>,
}

impl LeaseRenewer {
    pub fn group(&self) -> &str {
        &self.group
    }

    pub fn status(&self) -> LeaseStatus {
        current_health(&self.health).status
    }

    pub fn deadline(&self) -> LeaseDeadline {
        LeaseDeadline {
            expires_at: current_health(&self.health).expires_at,
        }
    }

    pub async fn renew(&self, duration: LeaseDuration) -> Result<LeaseDeadline, Error> {
        let _renew = self.renew_lock.lock().await;
        ensure_renewable(&self.health)?;
        let requested_duration_ms = duration.to_milliseconds()?;
        let request_started = Instant::now();
        let mut client = self.client.inner.clone();
        let response = match client
            .renew_lease(RenewLeaseRequest {
                token: self.token.clone(),
                requested_duration_ms,
            })
            .await
        {
            Ok(response) => response.into_inner(),
            Err(status) => {
                record_rpc_failure(&self.health, &status);
                return Err(status.into());
            }
        };
        if response.group != self.group {
            return Err(Error::Protocol(format!(
                "renew response group {:?} does not match lease group {:?}",
                response.group, self.group
            )));
        }
        if response.token != self.token {
            return Err(Error::Protocol(
                "renew response changed the lease token".into(),
            ));
        }
        let renewed = conservative_deadline(request_started, response.expires_in_ms)?;
        self.health.send_if_modified(|health| {
            if matches!(health.status, LeaseStatus::Active | LeaseStatus::Expired) {
                health.status = LeaseStatus::Active;
                health.expires_at = renewed;
                true
            } else {
                false
            }
        });
        ensure_active(&self.health)?;
        Ok(self.deadline())
    }
}

impl fmt::Debug for LeaseRenewer {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter
            .debug_struct("LeaseRenewer")
            .field("group", &self.group)
            .field("token", &"<redacted>")
            .field("status", &self.status())
            .field("deadline", &self.deadline())
            .finish()
    }
}

impl fmt::Debug for GroupLease {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter
            .debug_struct("GroupLease")
            .field("group", &self.group)
            .field("token", &"<redacted>")
            .field("status", &self.status())
            .field("deadline", &self.deadline())
            .finish()
    }
}

impl Drop for GroupLease {
    fn drop(&mut self) {
        self.set_terminal(LeaseStatus::Dropped);
    }
}

fn current_health(sender: &watch::Sender<LeaseHealth>) -> LeaseHealth {
    let health = *sender.borrow();
    if health.status == LeaseStatus::Active && Instant::now() >= health.expires_at {
        sender.send_if_modified(|current| {
            if current.status == LeaseStatus::Active && Instant::now() >= current.expires_at {
                current.status = LeaseStatus::Expired;
                true
            } else {
                false
            }
        });
        *sender.borrow()
    } else {
        health
    }
}

fn ensure_active(sender: &watch::Sender<LeaseHealth>) -> Result<(), Error> {
    let status = current_health(sender).status;
    if status == LeaseStatus::Active {
        Ok(())
    } else {
        Err(Error::LeaseInactive(status))
    }
}

fn ensure_renewable(sender: &watch::Sender<LeaseHealth>) -> Result<(), Error> {
    let status = current_health(sender).status;
    if matches!(status, LeaseStatus::Active | LeaseStatus::Expired) {
        Ok(())
    } else {
        Err(Error::LeaseInactive(status))
    }
}

fn record_rpc_failure(sender: &watch::Sender<LeaseHealth>, status: &tonic::Status) {
    if status.code() == Code::PermissionDenied {
        set_terminal(sender, LeaseStatus::Lost);
    }
}

fn set_terminal(sender: &watch::Sender<LeaseHealth>, status: LeaseStatus) {
    sender.send_if_modified(|health| {
        if matches!(
            health.status,
            LeaseStatus::Active | LeaseStatus::Expired | LeaseStatus::Releasing
        ) {
            health.status = status;
            true
        } else {
            false
        }
    });
}

pub(crate) fn mark_lease_lost(sender: &watch::Sender<LeaseHealth>) {
    set_terminal(sender, LeaseStatus::Lost);
}

pub(crate) fn observe_lease_health(sender: &watch::Sender<LeaseHealth>) -> LeaseHealth {
    current_health(sender)
}
