use std::fmt;
use std::sync::Arc;
use std::time::{Duration, Instant};

use bello_hardware_api::hardware_service_client::HardwareServiceClient;
use bello_hardware_api::{
    AcquireLeaseRequest, DisableGroupRequest, EnableGroupRequest, GroupState, MotorCommand,
    RecoverGroupRequest, ReleaseLeaseRequest, RenewLeaseRequest, StartControlRequest,
    StopControlRequest,
};
use tokio::sync::{Mutex, watch};
use tonic::Code;
use tonic::transport::Channel;

use crate::duration::positive_milliseconds;
use crate::{EnumValue, Error};

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

/// Exclusive write authority for one motor group.
///
/// The bearer token is intentionally private and redacted from `Debug`.
/// Dropping this value performs no asynchronous RPC; an unreleased server
/// lease expires according to daemon policy.
pub struct GroupLease {
    pub(crate) client: HardwareServiceClient<Channel>,
    pub(crate) group: String,
    pub(crate) token: String,
    pub(crate) health: watch::Sender<LeaseHealth>,
    renew_lock: Arc<Mutex<()>>,
}

impl GroupLease {
    pub(crate) async fn acquire(
        mut client: HardwareServiceClient<Channel>,
        group: &str,
        owner: &str,
        duration: LeaseDuration,
    ) -> Result<Self, Error> {
        let requested_duration_ms = duration.to_milliseconds()?;
        let request_started = Instant::now();
        let response = client
            .acquire_lease(AcquireLeaseRequest {
                group: group.into(),
                client_name: owner.into(),
                requested_duration_ms,
            })
            .await?
            .into_inner();
        if response.group != group {
            return Err(Error::Protocol(format!(
                "lease response group {:?} does not match requested group {:?}",
                response.group, group
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
            group: response.group,
            token: response.token,
            health,
            renew_lock: Arc::new(Mutex::new(())),
        })
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
        active_health(&self.health)?;
        let mut client = self.client.clone();
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
        valid_for_ms: u64,
        commands: Vec<MotorCommand>,
    ) -> Result<EnumValue<GroupState>, Error> {
        active_health(&self.health)?;
        let mut client = self.client.clone();
        let response = client
            .start_control(StartControlRequest {
                group: self.group.clone(),
                lease_token: self.token.clone(),
                valid_for_ms,
                commands,
            })
            .await
            .map_err(|status| self.rpc_error(status))?
            .into_inner();
        Ok(EnumValue::from_raw(response.state))
    }

    pub async fn stop(&self) -> Result<EnumValue<GroupState>, Error> {
        ensure_owned(&self.health)?;
        let mut client = self.client.clone();
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
        ensure_owned(&self.health)?;
        let mut client = self.client.clone();
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
        ensure_owned(&self.health)?;
        let mut client = self.client.clone();
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
        ensure_owned(&self.health)?;
        self.health.send_if_modified(|health| {
            if matches!(health.status, LeaseStatus::Active | LeaseStatus::Expired) {
                health.status = LeaseStatus::Releasing;
                true
            } else {
                false
            }
        });
        let mut client = self.client.clone();
        let response = match client
            .release_lease(ReleaseLeaseRequest {
                token: self.token.clone(),
            })
            .await
        {
            Ok(response) => response.into_inner(),
            Err(status) => {
                mark_lease_lost(&self.health);
                return Err(status.into());
            }
        };
        self.health
            .send_modify(|health| health.status = LeaseStatus::Released);
        Ok(EnumValue::from_raw(response.state))
    }

    fn rpc_error(&self, status: tonic::Status) -> Error {
        record_rpc_failure(&self.health, &status);
        status.into()
    }
}

/// Restricted capability for renewing a [`GroupLease`] from another task.
#[derive(Clone)]
pub struct LeaseRenewer {
    client: HardwareServiceClient<Channel>,
    group: String,
    token: String,
    health: watch::Sender<LeaseHealth>,
    renew_lock: Arc<Mutex<()>>,
}

impl LeaseRenewer {
    pub(crate) fn remaining(&self) -> Duration {
        current_health(&self.health)
            .expires_at
            .saturating_duration_since(Instant::now())
    }

    pub(crate) async fn renew(&self, duration: LeaseDuration) -> Result<(), Error> {
        let _renew = self.renew_lock.lock().await;
        ensure_owned(&self.health)?;
        let requested_duration_ms = duration.to_milliseconds()?;
        let request_started = Instant::now();
        let mut client = self.client.clone();
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
        active_health(&self.health)?;
        Ok(())
    }
}

impl fmt::Debug for GroupLease {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter
            .debug_struct("GroupLease")
            .field("group", &self.group)
            .field("token", &"<redacted>")
            .field("health", &current_health(&self.health))
            .finish()
    }
}

impl Drop for GroupLease {
    fn drop(&mut self) {
        set_terminal(&self.health, LeaseStatus::Dropped);
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

pub(crate) fn active_health(sender: &watch::Sender<LeaseHealth>) -> Result<LeaseHealth, Error> {
    let health = current_health(sender);
    if health.status == LeaseStatus::Active {
        Ok(health)
    } else {
        Err(Error::LeaseInactive(health.status))
    }
}

fn ensure_owned(sender: &watch::Sender<LeaseHealth>) -> Result<(), Error> {
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

fn conservative_deadline(request_started: Instant, expires_in_ms: u64) -> Result<Instant, Error> {
    if expires_in_ms == 0 {
        return Err(Error::Protocol(
            "lease response contains a zero remaining duration".into(),
        ));
    }
    request_started
        .checked_add(Duration::from_millis(expires_in_ms))
        .ok_or_else(|| Error::Protocol("lease deadline exceeds local Instant range".into()))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zero_server_lease_duration_is_a_protocol_error() {
        assert!(matches!(
            conservative_deadline(Instant::now(), 0),
            Err(Error::Protocol(_))
        ));
    }
}
