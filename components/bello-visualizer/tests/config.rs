use std::fs::{self, File};
use std::io::BufReader;

use bello_robot_description::Geometry;
use bello_visualizer::RobotDescription;
use bello_visualizer::config::{
    BoundedDefault, ConfigError, CoordinateLimits, GroupControlProfile, RootConfig,
    VisualizerConfig,
};
use tempfile::tempdir;

#[test]
fn loads_one_yaml_and_resolves_local_paths_with_safe_defaults() {
    let directory = tempdir().unwrap();
    let path = directory.path().join("visualizer.yaml");
    fs::write(
        &path,
        r#"
urdf: model.urdf
hardware:
  endpoint: http://127.0.0.1:50051
remote: { listen: "127.0.0.1:0", controller_index: 0, poll_hz: 60, deadzone: 0.1 }
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
    assert_eq!(config.urdf, directory.path().join("model.urdf"));
    assert_eq!(config.refresh_hz, 60.0);
    assert!(matches!(config.root, RootConfig::Fixed { .. }));
    let profile = &config.control_profiles["legs"];
    assert_eq!(profile.command_rate_hz, 50.0);
    assert_eq!(profile.validity_ms, 100);
    assert_eq!(profile.telemetry_staleness_ms, 200);
    assert_eq!(profile.position_slew_rate, 0.5);

    let mut invalid = config.clone();
    invalid.remote.poll_hz = 0.0;
    assert!(
        invalid
            .validate()
            .unwrap_err()
            .to_string()
            .contains("remote.poll_hz")
    );
    invalid.remote.poll_hz = f64::MAX;
    assert!(
        invalid
            .validate()
            .unwrap_err()
            .to_string()
            .contains("system timer")
    );
    invalid.remote.poll_hz = 60.0;
    invalid.remote.deadzone = 1.0;
    assert!(
        invalid
            .validate()
            .unwrap_err()
            .to_string()
            .contains("remote.deadzone")
    );
}

#[test]
fn normalizes_external_pose_model_root_transform() {
    let directory = tempdir().unwrap();
    let path = directory.path().join("visualizer.yaml");
    fs::write(
        &path,
        r#"
urdf: model.urdf
hardware:
  endpoint: http://127.0.0.1:50051
remote: { listen: "127.0.0.1:0", controller_index: 0, poll_hz: 60, deadzone: 0.1 }
root:
  source: external_pose
  endpoint: http://127.0.0.1:50052
  reference_frame: odom
  tracked_frame: pelvis_link
  tracked_from_model_root:
    orientation_xyzw: [0.0, 0.0, 0.0, 2.0]
"#,
    )
    .unwrap();

    let config = VisualizerConfig::load(path).unwrap();
    let RootConfig::ExternalPose {
        tracked_from_model_root,
        ..
    } = config.root
    else {
        panic!("expected external pose root");
    };
    assert_eq!(
        tracked_from_model_root.orientation_xyzw,
        [0.0, 0.0, 0.0, 1.0]
    );
}

#[test]
fn rejects_partial_tls_identity_and_unsafe_gain_bounds() {
    let directory = tempdir().unwrap();
    let path = directory.path().join("visualizer.yaml");
    fs::write(
        &path,
        r#"
urdf: model.urdf
hardware:
  endpoint: https://robot.example
  tls:
    client_certificate: client.pem
remote: { listen: "127.0.0.1:0", controller_index: 0, poll_hz: 60, deadzone: 0.1 }
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
fn maintained_bello_example_configuration_loads_the_canonical_description() {
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("config/bello.example.yaml");
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
            assert_eq!(endpoint, "http://127.0.0.1:50053");
            assert_eq!(reference_frame, "odom");
            assert_eq!(tracked_frame, "pelvis_link");
            assert_eq!(*staleness_ms, 200);
        }
        RootConfig::Fixed { .. } => panic!("Bello vCAN must use runner PoseService"),
    }
    let description = RobotDescription::load(&config.urdf).unwrap();
    assert_eq!(description.name(), "bello");
    assert!(description.joint("left_hip_pitch_joint").is_some());
    assert!(
        description
            .transmission_for_coordinate("left_ankle_motor_1")
            .is_some()
    );
    assert_eq!(
        config
            .control_profiles
            .keys()
            .map(String::as_str)
            .collect::<Vec<_>>(),
        ["lower_body", "upper_body"]
    );
    assert!(config.control_profiles.values().all(|profile| {
        profile.command_rate_hz == 50.0
            && profile.validity_ms == 500
            && profile.telemetry_staleness_ms == 200
    }));
}

#[test]
fn maintained_bello_mini_example_configuration_loads_the_canonical_description() {
    let path =
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("config/bello-mini.example.yaml");
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
            assert_eq!(endpoint, "http://127.0.0.1:50053");
            assert_eq!(reference_frame, "odom");
            assert_eq!(tracked_frame, "pelvis_link");
            assert_eq!(*staleness_ms, 200);
        }
        RootConfig::Fixed { .. } => panic!("Bello Mini must use estimator PoseService"),
    }
    let description = RobotDescription::load(&config.urdf).unwrap();
    assert_eq!(description.name(), "bello_mini");
    assert!(description.joint("waist_roll_joint").is_some());
    assert!(
        description
            .transmission_for_coordinate("left_ankle_motor_1")
            .is_some()
    );
    assert_eq!(
        config
            .control_profiles
            .keys()
            .map(String::as_str)
            .collect::<Vec<_>>(),
        ["lower_body", "upper_body"]
    );
    assert!(config.control_profiles.values().all(|profile| {
        profile.command_rate_hz == 50.0
            && profile.validity_ms == 500
            && profile.telemetry_staleness_ms == 200
    }));
}

#[test]
fn maintained_milly_example_configuration_loads_the_canonical_description() {
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("config/milly.example.yaml");
    let config = VisualizerConfig::load(path).unwrap();
    assert_eq!(config.hardware.endpoint, "http://127.0.0.1:50051");
    assert!(matches!(config.root, RootConfig::Fixed { .. }));

    let description = RobotDescription::load(&config.urdf).unwrap();
    assert_eq!(description.name(), "milly");
    for coordinate in [
        "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "gripper",
    ] {
        assert!(description.joint(coordinate).is_some());
    }
    let visual_meshes = description
        .links()
        .flat_map(|link| &link.visuals)
        .filter_map(|visual| match &visual.geometry {
            Geometry::Mesh { path, .. } => Some(path),
            _ => None,
        })
        .collect::<Vec<_>>();
    assert_eq!(visual_meshes.len(), 139);
    for path in visual_meshes {
        let mesh = stl_io::read_stl(&mut BufReader::new(File::open(path).unwrap())).unwrap();
        assert!(
            !mesh.faces.is_empty(),
            "empty Milly visual mesh: {}",
            path.display()
        );
    }
    assert_eq!(
        config
            .control_profiles
            .keys()
            .map(String::as_str)
            .collect::<Vec<_>>(),
        ["arm"]
    );
}

#[test]
fn live_position_limits_are_intersected_with_direct_and_differential_urdf_limits() {
    let directory = tempdir().unwrap();
    let urdf = directory.path().join("limits.urdf");
    fs::write(
        &urdf,
        r#"<robot name="limits">
          <link name="base"/><link name="direct_link"/><link name="roll_link"/><link name="pitch_link"/>
          <joint name="direct" type="revolute">
            <parent link="base"/><child link="direct_link"/>
            <limit lower="-1" upper="2" effort="10" velocity="10"/>
          </joint>
          <joint name="roll" type="revolute">
            <parent link="direct_link"/><child link="roll_link"/>
            <limit lower="-0.5" upper="0.75" effort="10" velocity="10"/>
          </joint>
          <joint name="pitch" type="revolute">
            <parent link="roll_link"/><child link="pitch_link"/>
            <limit lower="-1" upper="1.5" effort="10" velocity="10"/>
          </joint>
          <transmission name="ankle">
            <type>transmission_interface/DifferentialTransmission</type>
            <actuator name="motor_1" role="actuator1"/>
            <actuator name="motor_2" role="actuator2"/>
            <joint name="roll" role="joint1"/>
            <joint name="pitch" role="joint2">
              <mechanical_reduction>-1</mechanical_reduction>
            </joint>
          </transmission>
        </robot>"#,
    )
    .unwrap();
    let description = RobotDescription::load(&urdf).unwrap();
    let bounds = BoundedDefault {
        min: -10.0,
        max: 10.0,
        default: 0.0,
    };
    let profile = GroupControlProfile {
        command_rate_hz: 50.0,
        validity_ms: 100,
        telemetry_staleness_ms: 200,
        position_slew_rate: 0.5,
        velocity: bounds,
        torque_feedforward: bounds,
        kp: BoundedDefault {
            min: 0.0,
            max: 10.0,
            default: 1.0,
        },
        kd: BoundedDefault {
            min: 0.0,
            max: 10.0,
            default: 1.0,
        },
    };
    let limits = |position_min, position_max| CoordinateLimits {
        position_min,
        position_max,
        velocity_min: -10.0,
        velocity_max: 10.0,
        torque_min: -10.0,
        torque_max: 10.0,
        kp_min: 0.0,
        kp_max: 10.0,
        kd_min: 0.0,
        kd_max: 10.0,
    };
    let resolved = profile
        .resolve_limits(
            &description,
            [
                ("direct".into(), limits(-0.8, 10.0)),
                ("motor_1".into(), limits(-1.8, 10.0)),
                ("motor_2".into(), limits(-10.0, 2.0)),
            ],
        )
        .unwrap();

    assert_eq!(
        resolved.coordinates["direct"].position,
        BoundedDefault {
            min: -0.8,
            max: 2.0,
            default: 0.0,
        }
    );
    assert_eq!(
        resolved.coordinates["motor_1"].position,
        BoundedDefault {
            min: -1.8,
            max: 1.75,
            default: 0.0,
        }
    );
    assert_eq!(
        resolved.coordinates["motor_2"].position,
        BoundedDefault {
            min: -1.5,
            max: 2.0,
            default: 0.0,
        }
    );
}
