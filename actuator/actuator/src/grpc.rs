pub mod actuator {
    pub mod common {
        tonic::include_proto!("actuator/actuator.common");
    }
}

pub mod google {
    pub mod api {
        tonic::include_proto!("actuator/google.api");
    }

    pub mod rpc {
        tonic::include_proto!("actuator/google.rpc");
    }

    pub mod longrunning {
        tonic::include_proto!("actuator/google.longrunning");
    }
}
