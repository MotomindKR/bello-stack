use std::sync::Arc;
use std::time::Instant;

use tokio::sync::watch;
use tonic::{Request, Response, Status};

use super::{command_stream, conversion, inventory, telemetry_stream};
use crate::app::HardwareDaemon;
use crate::control::{GroupQueueError, LifecycleAction, Transition};
use crate::domain::{GroupName, ImuName};
use crate::lease::LeaseError;
use bello_hardware_api::hardware_service_server::HardwareService;
use bello_hardware_api::*;

#[derive(Clone)]
pub struct HardwareGrpc {
    daemon: Arc<HardwareDaemon>,
    shutdown: watch::Receiver<bool>,
}

impl HardwareGrpc {
    pub fn new(daemon: Arc<HardwareDaemon>, shutdown: watch::Receiver<bool>) -> Self {
        Self { daemon, shutdown }
    }
}

#[tonic::async_trait]
impl HardwareService for HardwareGrpc {
    type WatchTelemetryStream = super::ResponseStream<WatchTelemetryResponse>;
    type StreamCommandsStream = super::ResponseStream<StreamCommandsResponse>;

    async fn list_hardware(
        &self,
        _: Request<ListHardwareRequest>,
    ) -> Result<Response<ListHardwareResponse>, Status> {
        Ok(Response::new(inventory::list(&self.daemon)))
    }

    async fn watch_telemetry(
        &self,
        request: Request<WatchTelemetryRequest>,
    ) -> Result<Response<Self::WatchTelemetryStream>, Status> {
        let request = request.into_inner();
        self.validate_telemetry_selectors(&request)?;
        Ok(Response::new(telemetry_stream::watch(
            self.daemon.clone(),
            request,
            self.shutdown.clone(),
        )))
    }

    async fn acquire_lease(
        &self,
        request: Request<AcquireLeaseRequest>,
    ) -> Result<Response<AcquireLeaseResponse>, Status> {
        let request = request.into_inner();
        if request.client_name.trim().is_empty() {
            return Err(Status::invalid_argument("client_name must be non-empty"));
        }
        let group = GroupName::from(request.group);
        self.daemon
            .group(&group)
            .map_err(|error| Status::not_found(error.to_string()))?;
        let lease = self
            .daemon
            .acquire_lease(&group, &request.client_name, request.requested_duration_ms)
            .map_err(lease_status)?;
        let (group, token, expires_in_ms) = lease_fields(lease);
        Ok(Response::new(AcquireLeaseResponse {
            group,
            token,
            expires_in_ms,
        }))
    }

    async fn renew_lease(
        &self,
        request: Request<RenewLeaseRequest>,
    ) -> Result<Response<RenewLeaseResponse>, Status> {
        let request = request.into_inner();
        let token = conversion::lease_token(&request.token)?;
        let lease = self
            .daemon
            .renew_lease(token, request.requested_duration_ms)
            .map_err(lease_status)?;
        let (group, token, expires_in_ms) = lease_fields(lease);
        Ok(Response::new(RenewLeaseResponse {
            group,
            token,
            expires_in_ms,
        }))
    }

    async fn release_lease(
        &self,
        request: Request<ReleaseLeaseRequest>,
    ) -> Result<Response<ReleaseLeaseResponse>, Status> {
        let token = conversion::lease_token(&request.into_inner().token)?;
        let (_, transition) = self
            .daemon
            .release_lease(token)
            .await
            .map_err(operation_status)?;
        if !transition.ok {
            return Err(Status::failed_precondition(transition.message));
        }
        Ok(Response::new(ReleaseLeaseResponse {
            state: conversion::group_state(transition.to),
        }))
    }

    async fn enable_group(
        &self,
        request: Request<EnableGroupRequest>,
    ) -> Result<Response<EnableGroupResponse>, Status> {
        let request = request.into_inner();
        let transition = self
            .group_operation(
                &request.group,
                &request.lease_token,
                LifecycleAction::Enable,
            )
            .await?;
        Ok(Response::new(EnableGroupResponse::from(transition)))
    }

    async fn start_control(
        &self,
        request: Request<StartControlRequest>,
    ) -> Result<Response<StartControlResponse>, Status> {
        let request = request.into_inner();
        let transition = self
            .group_operation(&request.group, &request.lease_token, LifecycleAction::Start)
            .await?;
        Ok(Response::new(StartControlResponse::from(transition)))
    }

    async fn stop_control(
        &self,
        request: Request<StopControlRequest>,
    ) -> Result<Response<StopControlResponse>, Status> {
        let request = request.into_inner();
        let transition = self
            .group_operation(&request.group, &request.lease_token, LifecycleAction::Stop)
            .await?;
        Ok(Response::new(StopControlResponse::from(transition)))
    }

    async fn disable_group(
        &self,
        request: Request<DisableGroupRequest>,
    ) -> Result<Response<DisableGroupResponse>, Status> {
        let request = request.into_inner();
        let transition = self
            .group_operation(
                &request.group,
                &request.lease_token,
                LifecycleAction::Disable,
            )
            .await?;
        Ok(Response::new(DisableGroupResponse::from(transition)))
    }

    async fn recover_group(
        &self,
        request: Request<RecoverGroupRequest>,
    ) -> Result<Response<RecoverGroupResponse>, Status> {
        let request = request.into_inner();
        let transition = self
            .group_operation(
                &request.group,
                &request.lease_token,
                LifecycleAction::Recover,
            )
            .await?;
        Ok(Response::new(RecoverGroupResponse::from(transition)))
    }

    async fn stream_commands(
        &self,
        request: Request<tonic::Streaming<StreamCommandsRequest>>,
    ) -> Result<Response<Self::StreamCommandsStream>, Status> {
        Ok(Response::new(command_stream::open(
            self.daemon.clone(),
            request.into_inner(),
            self.shutdown.clone(),
        )))
    }
}

impl HardwareGrpc {
    async fn group_operation(
        &self,
        group_name: &str,
        lease_token: &str,
        action: LifecycleAction,
    ) -> Result<Transition, Status> {
        let group_name = GroupName::from(group_name);
        let lease_token = conversion::lease_token(lease_token)?;
        self.daemon
            .group(&group_name)
            .map_err(|error| Status::not_found(error.to_string()))?;
        self.daemon
            .group_operation(&group_name, lease_token, action)
            .await
            .map_err(operation_status)
            .and_then(|transition| {
                if transition.ok {
                    Ok(transition)
                } else {
                    Err(Status::failed_precondition(transition.message))
                }
            })
    }

    fn validate_telemetry_selectors(&self, request: &WatchTelemetryRequest) -> Result<(), Status> {
        for group in &request.groups {
            if group.trim().is_empty() {
                return Err(Status::invalid_argument(
                    "telemetry group selectors must be non-empty",
                ));
            }
            let group = GroupName::from(group.as_str());
            if self
                .daemon
                .topology
                .groups()
                .all(|(name, _)| name != &group)
            {
                return Err(Status::invalid_argument(format!(
                    "unknown telemetry group {group}"
                )));
            }
        }
        if !request.include_imu && !request.imus.is_empty() {
            return Err(Status::invalid_argument(
                "imu selectors require include_imu=true",
            ));
        }
        for imu in &request.imus {
            if imu.trim().is_empty() {
                return Err(Status::invalid_argument(
                    "telemetry IMU selectors must be non-empty",
                ));
            }
            let imu = ImuName::from(imu.as_str());
            if self.daemon.topology.imus().all(|(name, _)| name != &imu) {
                return Err(Status::invalid_argument(format!(
                    "unknown telemetry IMU {imu}"
                )));
            }
        }
        Ok(())
    }
}

fn lease_status(error: LeaseError) -> Status {
    let message = error.to_string();
    match error {
        LeaseError::AlreadyHeld { .. } => Status::failed_precondition(message),
        LeaseError::Unavailable => Status::unavailable(message),
        LeaseError::NoLease(_)
        | LeaseError::UnknownToken
        | LeaseError::WrongOwner
        | LeaseError::Expired => Status::permission_denied(message),
    }
}

fn operation_status(error: anyhow::Error) -> Status {
    if let Some(error) = error.downcast_ref::<LeaseError>() {
        return lease_status(error.clone());
    }
    if let Some(error) = error.downcast_ref::<GroupQueueError>() {
        return match error {
            GroupQueueError::Stopped | GroupQueueError::Disconnected => {
                Status::unavailable(error.to_string())
            }
            GroupQueueError::Full => Status::resource_exhausted(error.to_string()),
        };
    }
    if error
        .downcast_ref::<tokio::sync::oneshot::error::RecvError>()
        .is_some()
    {
        return Status::unavailable("group controller stopped before replying");
    }
    Status::internal(error.to_string())
}

fn lease_fields(lease: crate::lease::Lease) -> (String, String, u64) {
    let expires_in_ms = lease
        .expires
        .saturating_duration_since(Instant::now())
        .as_millis() as u64;
    (
        lease.group.to_string(),
        lease.token.to_string(),
        expires_in_ms,
    )
}

macro_rules! impl_transition_response {
    ($($response:ty),+ $(,)?) => {
        $(
            impl From<Transition> for $response {
                fn from(transition: Transition) -> Self {
                    Self {
                        state: conversion::group_state(transition.to),
                    }
                }
            }
        )+
    };
}

impl_transition_response!(
    EnableGroupResponse,
    StartControlResponse,
    StopControlResponse,
    DisableGroupResponse,
    RecoverGroupResponse,
);
