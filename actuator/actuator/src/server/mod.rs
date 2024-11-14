use crate::grpc::actuator::common::{
    actuator_controller_server::{ActuatorController, ActuatorControllerServer},
    ActuatorStatus, CommandActuatorsRequest, CommandActuatorsResponse, ResponseCode,
};
use crate::grpc::google::longrunning::{
    operations_server::{Operations, OperationsServer},
    CancelOperationRequest, DeleteOperationRequest, GetOperationRequest, ListOperationsRequest,
    ListOperationsResponse, Operation, WaitOperationRequest,
};
use std::net::SocketAddr;
use tonic::{transport::Server, Request, Response, Status};

#[derive(Default, Clone, Copy)]
pub struct ActuatorService {}

#[tonic::async_trait]
impl Operations for ActuatorService {
    async fn list_operations(
        &self,
        request: Request<ListOperationsRequest>,
    ) -> Result<Response<ListOperationsResponse>, Status> {
        // Implement your logic here
        Ok(Response::new(ListOperationsResponse {
            operations: vec![],
            next_page_token: String::new(),
        }))
    }

    async fn get_operation(
        &self,
        request: Request<GetOperationRequest>,
    ) -> Result<Response<Operation>, Status> {
        Err(Status::unimplemented("Not yet implemented"))
    }

    async fn delete_operation(
        &self,
        request: Request<DeleteOperationRequest>,
    ) -> Result<Response<()>, Status> {
        Err(Status::unimplemented("Not yet implemented"))
    }

    async fn cancel_operation(
        &self,
        request: Request<CancelOperationRequest>,
    ) -> Result<Response<()>, Status> {
        Err(Status::unimplemented("Not yet implemented"))
    }

    async fn wait_operation(
        &self,
        request: Request<WaitOperationRequest>,
    ) -> Result<Response<Operation>, Status> {
        Err(Status::unimplemented("Not yet implemented"))
    }
}

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

pub async fn run_server() -> Result<(), Box<dyn std::error::Error>> {
    let addr: SocketAddr = "[::1]:50051".parse()?;
    let service = ActuatorService::default();

    println!("Starting actuator server on {}", addr);

    Server::builder()
        .add_service(ActuatorControllerServer::new(service.clone()))
        .add_service(OperationsServer::new(service))
        .serve(addr)
        .await?;

    println!("Actuator server stopped");

    Ok(())
}
