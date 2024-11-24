mod actuator;
mod operations;

pub use actuator::ActuatorService;
pub use operations::OperationsService;

use crate::grpc::actuator::common::actuator_controller_server::ActuatorControllerServer;
use crate::grpc::google::longrunning::operations_server::OperationsServer;
use std::net::SocketAddr;
use tonic::transport::Server;

pub async fn run_server() -> Result<(), Box<dyn std::error::Error>> {
    let addr: SocketAddr = "[::1]:50051".parse()?;

    println!("Starting actuator server on {}", addr);

    Server::builder()
        .add_service(ActuatorControllerServer::new(ActuatorService::default()))
        .add_service(OperationsServer::new(OperationsService::default()))
        .serve(addr)
        .await?;

    println!("Actuator server stopped");

    Ok(())
}
