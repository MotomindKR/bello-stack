use std::pin::Pin;
use std::sync::Arc;
use std::time::Instant;

use tonic::{Request, Response, Status};

use super::{command_stream, conversion, inventory, telemetry_stream};
use crate::app::HardwareDaemon;
use crate::control::Transition;
use crate::domain::{GroupName, LeaseToken};
use crate::proto::hardware_service_server::HardwareService;
use crate::proto::*;

#[derive(Clone)]
pub struct HardwareGrpc {
    daemon: Arc<HardwareDaemon>,
}

impl HardwareGrpc {
    pub fn new(daemon: Arc<HardwareDaemon>) -> Self {
        Self { daemon }
    }
}

type GroupFuture<'a> =
    Pin<Box<dyn std::future::Future<Output = anyhow::Result<Transition>> + Send + 'a>>;

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
        Ok(Response::new(telemetry_stream::watch(
            self.daemon.clone(),
            request.into_inner(),
        )))
    }

    async fn acquire_lease(
        &self,
        request: Request<AcquireLeaseRequest>,
    ) -> Result<Response<AcquireLeaseResponse>, Status> {
        let request = request.into_inner();
        let group = GroupName::from(request.group);
        self.daemon
            .group(&group)
            .map_err(|error| Status::not_found(error.to_string()))?;
        let lease = self
            .daemon
            .acquire_lease(&group, &request.client_name, request.requested_duration_ms)
            .map_err(|error| Status::failed_precondition(error.to_string()))?;
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
            .map_err(|error| Status::failed_precondition(error.to_string()))?;
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
        let lease = self
            .daemon
            .release_lease(token)
            .map_err(|error| Status::failed_precondition(error.to_string()))?;
        let group = self
            .daemon
            .group(&lease.group)
            .map_err(|error| Status::not_found(error.to_string()))?;
        let state = group.snapshot.read().state.as_str().into();
        Ok(Response::new(ReleaseLeaseResponse {
            success: true,
            message: "lease released".into(),
            state,
        }))
    }

    async fn enable_group(
        &self,
        request: Request<EnableGroupRequest>,
    ) -> Result<Response<EnableGroupResponse>, Status> {
        let request = request.into_inner();
        let transition = self
            .group_operation(&request.group, &request.lease_token, |group| {
                Box::pin(group.enable())
            })
            .await?;
        Ok(Response::new(EnableGroupResponse::from(transition)))
    }

    async fn start_control(
        &self,
        request: Request<StartControlRequest>,
    ) -> Result<Response<StartControlResponse>, Status> {
        let request = request.into_inner();
        let transition = self
            .group_operation(&request.group, &request.lease_token, |group| {
                Box::pin(group.start())
            })
            .await?;
        Ok(Response::new(StartControlResponse::from(transition)))
    }

    async fn stop_control(
        &self,
        request: Request<StopControlRequest>,
    ) -> Result<Response<StopControlResponse>, Status> {
        let request = request.into_inner();
        let transition = self
            .group_operation(&request.group, &request.lease_token, |group| {
                Box::pin(group.stop())
            })
            .await?;
        Ok(Response::new(StopControlResponse::from(transition)))
    }

    async fn disable_group(
        &self,
        request: Request<DisableGroupRequest>,
    ) -> Result<Response<DisableGroupResponse>, Status> {
        let request = request.into_inner();
        let transition = self
            .group_operation(&request.group, &request.lease_token, |group| {
                Box::pin(group.disable())
            })
            .await?;
        Ok(Response::new(DisableGroupResponse::from(transition)))
    }

    async fn recover_group(
        &self,
        request: Request<RecoverGroupRequest>,
    ) -> Result<Response<RecoverGroupResponse>, Status> {
        let request = request.into_inner();
        let transition = self
            .group_operation(&request.group, &request.lease_token, |group| {
                Box::pin(group.recover())
            })
            .await?;
        Ok(Response::new(RecoverGroupResponse::from(transition)))
    }

    async fn fault_group(
        &self,
        request: Request<FaultGroupRequest>,
    ) -> Result<Response<FaultGroupResponse>, Status> {
        let request = request.into_inner();
        let group_name = GroupName::from(request.group);
        let lease_token = conversion::lease_token(&request.lease_token)?;
        self.verify(&group_name, lease_token)?;
        let transition = self
            .daemon
            .group(&group_name)
            .map_err(|error| Status::not_found(error.to_string()))?
            .fault(request.reason)
            .await
            .map_err(|error| Status::internal(error.to_string()))?;
        Ok(Response::new(FaultGroupResponse::from(transition)))
    }

    async fn stream_commands(
        &self,
        request: Request<tonic::Streaming<StreamCommandsRequest>>,
    ) -> Result<Response<Self::StreamCommandsStream>, Status> {
        Ok(Response::new(command_stream::open(
            self.daemon.clone(),
            request.into_inner(),
        )))
    }
}

impl HardwareGrpc {
    fn verify(&self, group: &GroupName, token: LeaseToken) -> Result<(), Status> {
        self.daemon
            .group(group)
            .map_err(|error| Status::not_found(error.to_string()))?;
        self.daemon
            .verify_lease(group, token)
            .map_err(|error| Status::permission_denied(error.to_string()))
    }

    async fn group_operation<F>(
        &self,
        group_name: &str,
        lease_token: &str,
        operation: F,
    ) -> Result<Transition, Status>
    where
        F: for<'a> FnOnce(&'a crate::control::GroupHandle) -> GroupFuture<'a>,
    {
        let group_name = GroupName::from(group_name);
        let lease_token = conversion::lease_token(lease_token)?;
        self.verify(&group_name, lease_token)?;
        let group = self
            .daemon
            .group(&group_name)
            .map_err(|error| Status::not_found(error.to_string()))?;
        operation(group)
            .await
            .map_err(|error| Status::internal(error.to_string()))
    }
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
                        success: transition.ok,
                        message: transition.message,
                        state: transition.to.as_str().into(),
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
    FaultGroupResponse,
);
