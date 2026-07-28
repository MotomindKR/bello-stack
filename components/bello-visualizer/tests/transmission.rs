use std::collections::BTreeMap;
use std::fs;
use std::time::Duration;

use bello_visualizer::RobotDescription;
use bello_visualizer::transmission::{
    CoordinateFeedback, FeedbackReconstructor, Transmission, TransmissionError,
};
use tempfile::TempDir;

fn description(with_differential: bool) -> (TempDir, RobotDescription) {
    let directory = tempfile::tempdir().unwrap();
    let transmission = if with_differential {
        r#"
          <transmission name="ankle">
            <type>transmission_interface/DifferentialTransmission</type>
            <actuator name="motor_1" role="actuator1"/>
            <actuator name="motor_2" role="actuator2"/>
            <joint name="roll" role="joint1"/>
            <joint name="pitch" role="joint2"><mechanical_reduction>-1</mechanical_reduction></joint>
          </transmission>
        "#
    } else {
        ""
    };
    let path = directory.path().join("robot.urdf");
    fs::write(
        &path,
        format!(
            r#"<robot name="fixture">
              <link name="base"/><link name="pitch_link"/><link name="roll_link"/>
              <joint name="pitch" type="revolute">
                <parent link="base"/><child link="pitch_link"/>
                <limit lower="-10" upper="10" effort="10" velocity="10"/>
              </joint>
              <joint name="roll" type="revolute">
                <parent link="pitch_link"/><child link="roll_link"/>
                <limit lower="-10" upper="10" effort="10" velocity="10"/>
              </joint>
              {transmission}
            </robot>"#
        ),
    )
    .unwrap();
    let description = RobotDescription::load(&path).unwrap();
    (directory, description)
}

#[test]
fn maps_direct_coordinates_forward_and_reverse() {
    let (_directory, description) = description(false);
    let transmission = Transmission::from_description(&description, ["pitch", "roll"]).unwrap();
    assert_eq!(
        transmission.joint_to_coordinate(&[1.0, -2.0]).unwrap(),
        [1.0, -2.0]
    );
    assert_eq!(
        transmission.coordinate_to_joint(&[3.0, 4.0]).unwrap(),
        [3.0, 4.0]
    );
    assert_eq!(
        transmission
            .coordinate_effort_to_joint(&[5.0, 6.0])
            .unwrap(),
        [5.0, 6.0]
    );
}

#[test]
fn reconstructs_ankle_differential() {
    let (_directory, description) = description(true);
    let transmission =
        Transmission::from_description(&description, ["motor_1", "motor_2"]).unwrap();
    assert_eq!(
        transmission.joint_to_coordinate(&[2.0, 3.0]).unwrap(),
        [1.0, 5.0]
    );
    let reverse = transmission.coordinate_to_joint(&[1.0, 5.0]).unwrap();
    assert!((reverse[0] - 2.0).abs() < 1.0e-12);
    assert!((reverse[1] - 3.0).abs() < 1.0e-12);
    assert_eq!(
        transmission
            .coordinate_effort_to_joint(&[7.0, 11.0])
            .unwrap(),
        [4.0, 18.0]
    );
}

#[test]
fn rejects_an_unknown_coordinate() {
    let (_directory, description) = description(false);
    assert!(matches!(
        Transmission::from_description(&description, ["missing"]),
        Err(TransmissionError::UnknownCoordinate(_))
    ));
}

#[test]
fn stale_or_missing_input_holds_last_valid_pose() {
    let (_directory, description) = description(false);
    let transmission = Transmission::from_description(&description, ["pitch", "roll"]).unwrap();
    let mut reconstructor = FeedbackReconstructor::new(transmission, Duration::from_millis(200));
    let sample = |position, age| CoordinateFeedback {
        position,
        velocity: 0.0,
        effort: 0.0,
        age,
    };
    let mut feedback = BTreeMap::from([
        ("pitch".into(), sample(1.0, Duration::from_millis(10))),
        ("roll".into(), sample(2.0, Duration::from_millis(10))),
    ]);
    assert_eq!(reconstructor.update(&feedback).position, [1.0, 2.0]);
    feedback.get_mut("roll").unwrap().age = Duration::from_millis(300);
    let held = reconstructor.update(&feedback);
    assert_eq!(held.position, [1.0, 2.0]);
    assert!(!held.available);
    feedback.remove("roll");
    assert_eq!(reconstructor.update(&feedback).position, [1.0, 2.0]);
}
