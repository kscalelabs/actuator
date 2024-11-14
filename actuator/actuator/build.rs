use std::env;
use std::path::PathBuf;

fn main() {
    let proto_root = "proto";
    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());
    let protos = [
        "actuator/common.proto",
        "google/longrunning/operations.proto",
    ];

    let includes = [proto_root, &format!("{}/googleapis", proto_root)];

    std::fs::create_dir_all(out_dir.join("actuator")).expect("Failed to create actuator directory");

    tonic_build::configure()
        .build_server(true)
        .out_dir(out_dir.join("actuator"))
        .compile_protos(&protos, &includes)
        .expect("Failed to compile protos");

    for proto in protos {
        println!("cargo:rerun-if-changed={}/actuator/{}", proto_root, proto);
    }
    println!("cargo:rerun-if-changed={}", proto_root);
}
