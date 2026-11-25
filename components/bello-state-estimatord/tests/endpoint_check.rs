use std::fs;
use std::net::TcpListener;
use std::process::Command;

#[test]
fn validates_hardware_endpoint_offline() {
    let temporary = tempfile::tempdir().unwrap();
    let config_path = temporary.path().join("bello.yaml");
    let reservation = TcpListener::bind("127.0.0.1:0").unwrap();
    let address = reservation.local_addr().unwrap();
    drop(reservation);

    let profile = include_str!("../config/bello.yaml")
        .replace(
            "../../../descriptions/",
            concat!(env!("CARGO_MANIFEST_DIR"), "/../../descriptions/"),
        )
        .replace("listen: 0.0.0.0:50053", "listen: 127.0.0.1:0");
    for (scheme, flags, exit_code) in [
        ("http", &["--check"][..], 0),
        ("tcp", &[][..], 1),
        ("tcp", &["--check"][..], 1),
    ] {
        let config = profile.replace("http://127.0.0.1:50051", &format!("{scheme}://{address}"));
        fs::write(&config_path, config).unwrap();
        let output = Command::new(env!("CARGO_BIN_EXE_bello-state-estimatord"))
            .args(flags)
            .arg("--config")
            .arg(&config_path)
            .output()
            .unwrap();
        let stderr = String::from_utf8_lossy(&output.stderr);
        assert_eq!(
            output.status.code(),
            Some(exit_code),
            "{scheme} {flags:?}: {stderr}"
        );
        if exit_code == 1 {
            assert!(stderr.contains("endpoint must start with"), "{stderr}");
        }
    }
}
