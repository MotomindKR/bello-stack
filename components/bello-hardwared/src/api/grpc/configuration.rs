use bello_hardware_api::motor_configuration_service_server::MotorConfigurationService;
use bello_hardware_api::{
    CancelMotorConfigurationRequest, CancelMotorConfigurationResponse,
    CommitMotorConfigurationRequest, CommitMotorConfigurationResponse,
    GetMotorConfigurationRequest, GetMotorConfigurationResponse, MotorConfigurationKind,
    PrepareMotorConfigurationRequest, PrepareMotorConfigurationResponse,
};
use tonic::{Request, Response, Status};

use super::HardwareGrpc;

#[tonic::async_trait]
impl MotorConfigurationService for HardwareGrpc {
    async fn prepare_motor_configuration(
        &self,
        request: Request<PrepareMotorConfigurationRequest>,
    ) -> Result<Response<PrepareMotorConfigurationResponse>, Status> {
        let request = request.into_inner();
        let kind = MotorConfigurationKind::try_from(request.kind)
            .map_err(|_| Status::invalid_argument("unknown configuration kind"))?;
        self.daemon
            .prepare_configuration(request.motor, kind)
            .map(|id| Response::new(PrepareMotorConfigurationResponse { id }))
    }
    async fn get_motor_configuration(
        &self,
        request: Request<GetMotorConfigurationRequest>,
    ) -> Result<Response<GetMotorConfigurationResponse>, Status> {
        self.daemon
            .configuration_status(&request.into_inner().id)
            .map(Response::new)
    }
    async fn commit_motor_configuration(
        &self,
        request: Request<CommitMotorConfigurationRequest>,
    ) -> Result<Response<CommitMotorConfigurationResponse>, Status> {
        self.daemon
            .configuration_decision(&request.into_inner().id, true)?;
        Ok(Response::new(CommitMotorConfigurationResponse {}))
    }
    async fn cancel_motor_configuration(
        &self,
        request: Request<CancelMotorConfigurationRequest>,
    ) -> Result<Response<CancelMotorConfigurationResponse>, Status> {
        self.daemon
            .configuration_decision(&request.into_inner().id, false)?;
        Ok(Response::new(CancelMotorConfigurationResponse {}))
    }
}
