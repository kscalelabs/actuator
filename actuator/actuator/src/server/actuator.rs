use crate::grpc::actuator::common::{
    actuator_controller_server::ActuatorController, ActuatorStatus, CommandActuatorsRequest,
    CommandActuatorsResponse, ResponseCode,
};
use tonic::{Request, Response, Status};

#[derive(Default, Clone, Copy)]
pub struct ActuatorService {}

#[tonic::async_trait]
impl ActuatorController for ActuatorService {
    async fn command_actuators(
        &self,
        request: Request<CommandActuatorsRequest>,
    ) -> Result<Response<CommandActuatorsResponse>, Status> {
        let cmd = request.into_inner();
        println!("Received CommandActuatorsRequest: {:?}", cmd);

        // Create a response for each actuator in the request
        let statuses: Vec<ActuatorStatus> = cmd
            .commands
            .iter()
            .map(|cmd| ActuatorStatus {
                actuator_id: cmd.actuator_id,
                position: cmd.position,
                velocity: cmd.velocity,
            })
            .collect();

        let response = CommandActuatorsResponse {
            statuses,
            response_code: ResponseCode::Ok as i32,
        };

        Ok(Response::new(response))
    }
}
