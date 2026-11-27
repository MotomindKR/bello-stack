fn main() -> Result<(), Box<dyn std::error::Error>> {
    const PROTO: &str = "../../../proto/bello/hardware/v1/hardware.proto";
    tonic_prost_build::configure().compile_protos(
        &[PROTO, "../../../proto/bello/camera/v1/camera.proto"],
        &["../../../proto"],
    )?;
    println!("cargo:rerun-if-changed={PROTO}");
    println!("cargo:rerun-if-changed=../../../proto");
    Ok(())
}
