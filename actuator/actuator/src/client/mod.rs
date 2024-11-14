use crate::grpc::google::longrunning::operations_client::OperationsClient;
use crate::grpc::google::longrunning::{
    CancelOperationRequest, DeleteOperationRequest, GetOperationRequest, ListOperationsRequest,
    ListOperationsResponse, Operation, WaitOperationRequest,
};
use prost_types::Duration as PbDuration;
use std::time::Duration;
use tonic::transport::Uri;
use tonic::{transport::Channel, Request};

pub struct Client {
    operations_client: OperationsClient<Channel>,
}

impl Client {
    pub async fn new(addr: Uri) -> Result<Self, Box<dyn std::error::Error>> {
        let channel = Channel::builder(addr).connect().await?;
        let operations_client = OperationsClient::new(channel);
        Ok(Self { operations_client })
    }

    pub async fn list_operations(
        &mut self,
        name: String,
        filter: String,
        page_size: i32,
        page_token: String,
    ) -> Result<tonic::Response<ListOperationsResponse>, tonic::Status> {
        let request = Request::new(ListOperationsRequest {
            name,
            filter,
            page_size,
            page_token,
        });

        self.operations_client.list_operations(request).await
    }

    pub async fn get_operation(
        &mut self,
        name: String,
    ) -> Result<tonic::Response<Operation>, tonic::Status> {
        let request = Request::new(GetOperationRequest { name });

        self.operations_client.get_operation(request).await
    }

    pub async fn delete_operation(
        &mut self,
        name: String,
    ) -> Result<tonic::Response<()>, tonic::Status> {
        let request = Request::new(DeleteOperationRequest { name });

        self.operations_client.delete_operation(request).await
    }

    pub async fn cancel_operation(
        &mut self,
        name: String,
    ) -> Result<tonic::Response<()>, tonic::Status> {
        let request = Request::new(CancelOperationRequest { name });

        self.operations_client.cancel_operation(request).await
    }

    pub async fn wait_operation(
        &mut self,
        name: String,
        timeout: Option<Duration>,
    ) -> Result<tonic::Response<Operation>, tonic::Status> {
        let timeout = timeout.map(|d| PbDuration {
            seconds: d.as_secs() as i64,
            nanos: d.subsec_nanos() as i32,
        });

        let request = Request::new(WaitOperationRequest { name, timeout });

        self.operations_client.wait_operation(request).await
    }
}

pub async fn run_dummy_client() -> Result<(), Box<dyn std::error::Error>> {
    let addr: Uri = "http://[::1]:50051".parse()?;
    let mut client = Client::new(addr).await?;

    // Send a request every second for 10 seconds
    for i in 0..10 {
        // List operations with some dummy parameters
        let response = client
            .list_operations("dummy_name".to_string(), "".to_string(), 10, "".to_string())
            .await?;

        println!("Request {}: Response received: {:?}", i + 1, response);

        // Wait for 1 second before the next request
        tokio::time::sleep(Duration::from_secs(1)).await;
    }

    Ok(())
}
