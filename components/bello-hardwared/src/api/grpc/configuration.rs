use bello_hardware_api::motor_configuration_service_server::MotorConfigurationService;
use bello_hardware_api::{
    MotorConfiguration, MotorConfigurationId, MotorConfigurationKind,
    PrepareMotorConfigurationRequest,
};
use tonic::{Request, Response, Status};

use super::HardwareGrpc;

#[tonic::async_trait]
impl MotorConfigurationService for HardwareGrpc {
    async fn prepare_motor_configuration(
        &self,
        request: Request<PrepareMotorConfigurationRequest>,
    ) -> Result<Response<MotorConfiguration>, Status> {
        let request = request.into_inner();
        let kind = MotorConfigurationKind::try_from(request.kind)
            .map_err(|_| Status::invalid_argument("unknown configuration kind"))?;
        self.daemon
            .prepare_configuration(request.motor, kind)
            .map(Response::new)
    }
    async fn get_motor_configuration(
        &self,
        request: Request<MotorConfigurationId>,
    ) -> Result<Response<MotorConfiguration>, Status> {
        self.daemon
            .configuration_status(&request.into_inner().id)
            .map(Response::new)
    }
    async fn commit_motor_configuration(
        &self,
        request: Request<MotorConfigurationId>,
    ) -> Result<Response<MotorConfiguration>, Status> {
        self.daemon
            .configuration_decision(&request.into_inner().id, true)
            .map(Response::new)
    }
    async fn cancel_motor_configuration(
        &self,
        request: Request<MotorConfigurationId>,
    ) -> Result<Response<MotorConfiguration>, Status> {
        self.daemon
            .configuration_decision(&request.into_inner().id, false)
            .map(Response::new)
    }
}
