use std::{fs, process::Command};

#[test]
fn validates_hardware_endpoint_offline() {
    let temp = tempfile::tempdir().unwrap();
    let config = temp.path().join("estimator.yaml");
    let urdf = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../descriptions/bello/urdf/bello.urdf"
    );
    for (scheme, valid) in [("http", true), ("tcp", false)] {
        fs::write(&config, format!(
            "urdf: {urdf}\nhardware_endpoint: {scheme}://127.0.0.1:1\nlisten: 127.0.0.1:0\nimu_frame: pelvis_link\nplanar: true\nground_height_m: 0\n"
        )).unwrap();
        let result = Command::new(env!("CARGO_BIN_EXE_bello-state-estimatord"))
            .args(["--check", "--config"])
            .arg(&config)
            .output()
            .unwrap();
        assert_eq!(
            result.status.success(),
            valid,
            "{scheme}: {}",
            String::from_utf8_lossy(&result.stderr)
        );
    }
}
