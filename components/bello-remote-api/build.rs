fn main() -> Result<(), Box<dyn std::error::Error>> {
    const PROTO: &str = "proto/bello/remote/v1/remote.proto";
    tonic_prost_build::configure()
        .file_descriptor_set_path(
            std::path::PathBuf::from(std::env::var_os("OUT_DIR").expect("OUT_DIR is set"))
                .join("bello_remote_v1_descriptor.bin"),
        )
        .compile_protos(&[PROTO], &["proto"])?;
    println!("cargo:rerun-if-changed={PROTO}");
    println!("cargo:rerun-if-changed=proto");
    Ok(())
}
