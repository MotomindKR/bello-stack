use std::fmt;
use std::time::{Duration, Instant};

use bello_hardware_api::hardware_service_client::HardwareServiceClient;
use bello_hardware_api::{
    AcquireLeaseRequest, DisableGroupRequest, EnableGroupRequest, GroupState, MotorCommand,
    RecoverGroupRequest, ReleaseLeaseRequest, RenewLeaseRequest, StartControlRequest,
    StopControlRequest,
};
use tokio::sync::watch;
use tonic::Code;
use tonic::transport::Channel;

use crate::{EnumValue, Error};

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

pub(crate) struct GroupLease {
    pub(crate) client: HardwareServiceClient<Channel>,
    pub(crate) group: String,
    pub(crate) token: String,
    pub(crate) health: watch::Sender<LeaseHealth>,
    requested_duration_ms: u64,
}

impl GroupLease {
    pub(crate) async fn acquire(
        mut client: HardwareServiceClient<Channel>,
        group: &str,
        owner: &str,
        requested_duration_ms: u64,
    ) -> Result<Self, Error> {
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
            requested_duration_ms,
        })
    }

    pub(crate) async fn enable(&self) -> Result<(), Error> {
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
        self.confirm_state("enable", GroupState::Damped, response.state)
    }

    pub(crate) async fn start(
        &self,
        operation: &'static str,
        valid_for_ms: u64,
        commands: Vec<MotorCommand>,
    ) -> Result<(), Error> {
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
        self.confirm_state(operation, GroupState::Running, response.state)
    }

    pub(crate) async fn stop(&self) -> Result<(), Error> {
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
        self.confirm_state("stop", GroupState::Damped, response.state)
    }

    pub(crate) async fn disable(&self) -> Result<(), Error> {
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
        self.confirm_state("disable", GroupState::Idle, response.state)
    }

    pub(crate) async fn recover(&self) -> Result<(), Error> {
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
        self.confirm_state("recover", GroupState::Idle, response.state)
    }

    pub(crate) async fn release(&self) -> Result<EnumValue<GroupState>, Error> {
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

    fn confirm_state(
        &self,
        operation: &'static str,
        expected: GroupState,
        state: i32,
    ) -> Result<(), Error> {
        if state == expected as i32 {
            Ok(())
        } else {
            Err(Error::UnexpectedGroupState {
                group: self.group.clone(),
                operation,
                expected,
                actual: EnumValue::from_raw(state),
            })
        }
    }

    fn rpc_error(&self, status: tonic::Status) -> Error {
        if status.code() == Code::PermissionDenied {
            mark_lease_lost(&self.health);
        }
        status.into()
    }

    pub(crate) fn remaining(&self) -> Duration {
        current_health(&self.health)
            .expires_at
            .saturating_duration_since(Instant::now())
    }

    pub(crate) async fn renew(&self) -> Result<(), Error> {
        ensure_owned(&self.health)?;
        let request_started = Instant::now();
        let mut client = self.client.clone();
        let response = client
            .renew_lease(RenewLeaseRequest {
                token: self.token.clone(),
                requested_duration_ms: self.requested_duration_ms,
            })
            .await
            .map_err(|status| self.rpc_error(status))?
            .into_inner();
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
