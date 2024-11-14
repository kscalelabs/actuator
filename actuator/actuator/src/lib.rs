mod proto {
    tonic::include_proto!("actuator.proto");
}

pub mod server;
pub mod client;
pub mod errors;
