use std::collections::BTreeMap;
use std::time::Duration;

use bello_visualizer::Model;
use bello_visualizer::transmission::{
    CoordinateFeedback, FeedbackReconstructor, Transmission, TransmissionError,
};

fn model(tendon: &str) -> Model {
    Model::parse(
        &format!(
            r#"<mujoco><worldbody><body name="b">
              <joint name="pitch" type="hinge"/><joint name="roll" type="hinge"/>
            </body></worldbody><tendon>{tendon}</tendon></mujoco>"#
        ),
        "/tmp/model.xml",
    )
    .unwrap()
}

#[test]
fn maps_direct_coordinates_forward_and_reverse() {
    let transmission = Transmission::from_model(&model(""), ["pitch", "roll"]).unwrap();
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
fn reconstructs_ankle_fixed_tendon() {
    let model = model(
        r#"<fixed name="motor_1"><joint joint="pitch" coef="-1"/><joint joint="roll" coef="1"/></fixed>
           <fixed name="motor_2"><joint joint="pitch" coef="1"/><joint joint="roll" coef="1"/></fixed>"#,
    );
    let transmission = Transmission::from_model(&model, ["motor_1", "motor_2"]).unwrap();
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
fn accepts_overdetermined_full_rank_mapping() {
    let model = model(
        r#"<fixed name="sum"><joint joint="pitch" coef="1"/><joint joint="roll" coef="1"/></fixed>"#,
    );
    let transmission = Transmission::from_model(&model, ["pitch", "roll", "sum"]).unwrap();
    let reconstructed = transmission.coordinate_to_joint(&[2.0, 3.0, 5.0]).unwrap();
    assert!((reconstructed[0] - 2.0).abs() < 1.0e-12);
    assert!((reconstructed[1] - 3.0).abs() < 1.0e-12);
}

#[test]
fn rejects_rank_deficient_mapping() {
    let model = model(
        r#"<fixed name="same_1"><joint joint="pitch" coef="1"/><joint joint="roll" coef="1"/></fixed>
           <fixed name="same_2"><joint joint="pitch" coef="2"/><joint joint="roll" coef="2"/></fixed>"#,
    );
    assert!(matches!(
        Transmission::from_model(&model, ["same_1", "same_2"]),
        Err(TransmissionError::Underdetermined { .. })
    ));
}

#[test]
fn stale_or_missing_input_holds_last_valid_pose() {
    let transmission = Transmission::from_model(&model(""), ["pitch", "roll"]).unwrap();
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
