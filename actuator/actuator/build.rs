use std::env;
use std::path::PathBuf;

fn main() {
    let proto_root = "../proto";
    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());
    let protos = [
        "actuator/common.proto",
        "actuator/robstride.proto",
    ];

    tonic_build::configure()
        .out_dir(out_dir)
        .compile(&[proto_root], &[proto_root])
        .unwrap();

    std::fs::create_dir_all(out_dir.join("actuator")).expect("Failed to create actuator directory");

    tonic_build::configure()
        .build_server(true)
        .out_dir(out_dir.join("actuator"))
        .protoc_arg("--experimental_allow_proto3_optional")
        .compile(&protos, &includes)
        .expect("Failed to compile protos");

    for proto in protos {
        println!("cargo:rerun-if-changed={}/actuator/{}", proto_root, proto);
    }
    println!("cargo:rerun-if-changed={}", proto_root);
}
