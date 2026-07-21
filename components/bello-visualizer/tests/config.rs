use std::collections::BTreeSet;
use std::fs;

use bello_visualizer::Model;
use bello_visualizer::config::{ConfigError, RootConfig, VisualizerConfig};
use bello_visualizer::editor::GroupEditor;
use tempfile::tempdir;

#[test]
fn loads_one_yaml_and_resolves_local_paths_with_safe_defaults() {
    let directory = tempdir().unwrap();
    fs::write(directory.path().join("model.xml"), "<mujoco/>").unwrap();
    let path = directory.path().join("visualizer.yaml");
    fs::write(
        &path,
        r#"
mjcf: model.xml
hardware:
  endpoint: http://127.0.0.1:50051
control_profiles:
  legs:
    velocity: { min: -2.0, max: 2.0, default: 0.0 }
    torque_feedforward: { min: -3.0, max: 3.0, default: 0.0 }
    kp: { min: 0.0, max: 20.0, default: 10.0 }
    kd: { min: 0.0, max: 2.0, default: 1.0 }
"#,
    )
    .unwrap();

    let config = VisualizerConfig::load(&path).unwrap();
    assert_eq!(config.mjcf, directory.path().join("model.xml"));
    assert_eq!(config.refresh_hz, 60.0);
    assert!(matches!(config.root, RootConfig::Fixed { .. }));
    let profile = &config.control_profiles["legs"];
    assert_eq!(profile.command_rate_hz, 50.0);
    assert_eq!(profile.validity_ms, 100);
    assert_eq!(profile.telemetry_staleness_ms, 200);
    assert_eq!(profile.position_slew_rate, 0.5);
}

#[test]
fn rejects_partial_tls_identity_and_unsafe_gain_bounds() {
    let directory = tempdir().unwrap();
    let path = directory.path().join("visualizer.yaml");
    fs::write(
        &path,
        r#"
mjcf: model.xml
hardware:
  endpoint: https://robot.example
  tls:
    client_certificate: client.pem
control_profiles:
  legs:
    kp: { min: -1.0, max: 20.0, default: 10.0 }
"#,
    )
    .unwrap();

    let error = VisualizerConfig::load(&path).unwrap_err();
    assert!(matches!(error, ConfigError::Invalid(_)));
    assert!(error.to_string().contains("client_private_key"));

    let yaml = fs::read_to_string(&path)
        .unwrap()
        .replace("  tls:\n    client_certificate: client.pem\n", "");
    fs::write(&path, yaml).unwrap();
    let error = VisualizerConfig::load(&path).unwrap_err();
    assert!(
        error
            .to_string()
            .contains("gain bounds must be non-negative")
    );
}

#[test]
fn maintained_bello_vcan_profiles_cover_every_sphere_hand_coordinate() {
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("config/bello-vcan.yaml");
    let config = VisualizerConfig::load(path).unwrap();
    assert_eq!(config.hardware.endpoint, "http://127.0.0.1:50051");
    match &config.root {
        RootConfig::ExternalPose {
            endpoint,
            reference_frame,
            tracked_frame,
            staleness_ms,
            ..
        } => {
            assert_eq!(endpoint, "http://127.0.0.1:50052");
            assert_eq!(reference_frame, "odom");
            assert_eq!(tracked_frame, "pelvis_link");
            assert_eq!(*staleness_ms, 200);
        }
        RootConfig::Fixed { .. } => panic!("Bello vCAN must use runner PoseService"),
    }
    let model = Model::load(&config.mjcf).unwrap();
    let lower_names = BTreeSet::from([
        "waist_yaw_joint",
        "left_hip_pitch_joint",
        "left_hip_roll_joint",
        "left_hip_yaw_joint",
        "left_knee_joint",
        "left_ankle_motor_1",
        "left_ankle_motor_2",
        "right_hip_pitch_joint",
        "right_hip_roll_joint",
        "right_hip_yaw_joint",
        "right_knee_joint",
        "right_ankle_motor_1",
        "right_ankle_motor_2",
    ]);
    let upper_names = BTreeSet::from([
        "left_shoulder_pitch_joint",
        "left_shoulder_roll_joint",
        "left_shoulder_yaw_joint",
        "left_elbow_pitch_joint",
        "left_elbow_yaw_joint",
        "left_wrist_pitch_joint",
        "right_shoulder_pitch_joint",
        "right_shoulder_roll_joint",
        "right_shoulder_yaw_joint",
        "right_elbow_pitch_joint",
        "right_elbow_yaw_joint",
        "right_wrist_pitch_joint",
    ]);
    let lower = config.control_profiles["lower_body"]
        .resolve(&model, lower_names.iter())
        .unwrap();
    let upper = config.control_profiles["upper_body"]
        .resolve(&model, upper_names.iter())
        .unwrap();
    assert_eq!(
        lower
            .coordinates
            .keys()
            .map(String::as_str)
            .collect::<BTreeSet<_>>(),
        lower_names
    );
    assert_eq!(
        upper
            .coordinates
            .keys()
            .map(String::as_str)
            .collect::<BTreeSet<_>>(),
        upper_names
    );
    assert_eq!(
        lower.coordinates["left_ankle_motor_1"].position.min,
        -1.04723
    );
    assert_eq!(
        upper.coordinates["left_shoulder_yaw_joint"].position.max,
        2.0
    );
    GroupEditor::new(&model, lower).unwrap();
    GroupEditor::new(&model, upper).unwrap();
    assert!(config.control_profiles.values().all(|profile| {
        profile.command_rate_hz == 50.0
            && profile.validity_ms == 500
            && profile.telemetry_staleness_ms == 200
    }));
}
