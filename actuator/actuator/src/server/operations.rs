use crate::grpc::google::longrunning::{
    operations_server::Operations, CancelOperationRequest, DeleteOperationRequest,
    GetOperationRequest, ListOperationsRequest, ListOperationsResponse, Operation,
    WaitOperationRequest,
};
use tonic::{Request, Response, Status};

#[derive(Default, Clone, Copy)]
pub struct OperationsService {}

#[tonic::async_trait]
impl Operations for OperationsService {
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
