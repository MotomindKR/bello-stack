use std::path::Path;
use std::process::{Command, Output};

fn check(filename: impl AsRef<Path>) -> Output {
    Command::new(env!("CARGO_BIN_EXE_bello-hardwared"))
        .arg("--check")
        .arg("--config")
        .arg(
            Path::new(env!("CARGO_MANIFEST_DIR"))
                .join("config")
                .join(filename),
        )
        .output()
        .expect("run hardwared configuration check")
}

#[test]
fn gsusb_deployment_validates_without_usb_hardware() {
    let result = check("milly-gsusb.yaml");
    assert!(
        result.status.success(),
        "{}",
        String::from_utf8_lossy(&result.stderr)
    );
}

#[test]
fn socketcan_is_available_only_on_linux() {
    let result = check("milly.yaml");
    if cfg!(target_os = "linux") {
        assert!(
            result.status.success(),
            "{}",
            String::from_utf8_lossy(&result.stderr)
        );
    } else {
        assert!(!result.status.success());
        assert!(String::from_utf8_lossy(&result.stderr).contains("SocketCAN requires Linux"));
    }
}

#[test]
fn check_reports_invalid_nested_runtime_field() {
    let config_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("config");
    let mut config: serde_yaml::Value =
        serde_yaml::from_str(include_str!("../config/bello-sim.yaml")).unwrap();
    let description = config_dir.join(config["description"].as_str().unwrap());
    config["description"] = serde_yaml::to_value(description).unwrap();
    config["runtime"] = serde_yaml::from_str("imu_filter: { gyro_cutoff_hz: .nan }").unwrap();
    let file = tempfile::NamedTempFile::new().unwrap();
    serde_yaml::to_writer(file.as_file(), &config).unwrap();

    let result = check(file.path());
    assert!(!result.status.success());
    let stderr = String::from_utf8_lossy(&result.stderr);
    assert!(
        stderr.contains("runtime.imu_filter.gyro_cutoff_hz"),
        "{stderr}"
    );
}
