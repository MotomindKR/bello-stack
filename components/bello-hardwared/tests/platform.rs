use std::path::Path;
use std::process::{Command, Output};

fn check(filename: &str) -> Output {
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
