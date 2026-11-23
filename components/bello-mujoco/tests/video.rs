#![cfg(target_os = "linux")]

use mujoco_rs::prelude::MjModel;
use std::io::Write;
use std::path::Path;
use std::process::Command;

fn render(directory: &Path) -> std::process::Output {
    Command::new(env!("CARGO_BIN_EXE_bello-mujoco"))
        .args(["render", "--recording"])
        .arg(directory)
        .output()
        .unwrap()
}

#[test]
#[ignore = "requires Linux software Mesa and ffmpeg; no GPU or CAN devices"]
fn software_video_is_visible_bounded_and_refuses_overwrites() {
    let dir = tempfile::tempdir().unwrap();
    let output = dir.path().join("test.mp4");
    let model = MjModel::from_xml_string(
        r#"<mujoco>
      <worldbody><geom type="plane" size="3 3 .1"/>
        <body name="body" pos="0 0 .4"><joint type="slide" axis="1 0 0"/>
          <geom type="box" size=".1 .15 .2" rgba=".9 .1 .1 1"/>
        </body></worldbody></mujoco>"#,
    )
    .unwrap();
    model.save_to_file(dir.path().join("model.mjb")).unwrap();
    let config = serde_json::json!({"output": output, "width": 160, "height": 120,
        "fps": 10, "max_seconds": 1., "render_timeout_seconds": 30.,
        "camera": {"body": "body", "distance_m": 2., "azimuth_degrees": 120.,
            "elevation_degrees": -20., "lookat_offset_m": [0., 0., 0.]}});
    serde_json::to_writer(
        std::fs::File::create(dir.path().join("video.json")).unwrap(),
        &config,
    )
    .unwrap();
    let mut poses = std::fs::File::create(dir.path().join("qpos.bin")).unwrap();
    for i in 0..10 {
        poses.write_all(&(i as f64 * 0.01).to_le_bytes()).unwrap();
    }
    drop(poses);
    let result = render(dir.path());
    assert!(
        result.status.success(),
        "{}",
        String::from_utf8_lossy(&result.stderr)
    );
    assert!(String::from_utf8_lossy(&result.stderr).contains("llvmpipe"));
    let frames = Command::new("ffmpeg")
        .args(["-v", "error", "-i"])
        .arg(&output)
        .args(["-f", "rawvideo", "-pix_fmt", "rgb24", "pipe:1"])
        .output()
        .unwrap();
    assert!(frames.status.success());
    assert_eq!(frames.stdout.len(), 10 * 160 * 120 * 3);
    let colors: std::collections::BTreeSet<_> = frames.stdout[..160 * 120 * 3]
        .as_chunks::<3>()
        .0
        .iter()
        .collect();
    assert!(colors.len() > 30, "rendered frame is blank");
    let original = std::fs::read(&output).unwrap();
    assert!(!render(dir.path()).status.success());
    assert_eq!(std::fs::read(&output).unwrap(), original);
    std::fs::write(dir.path().join("qpos.bin"), [0; 3]).unwrap();
    let result = render(dir.path());
    assert!(!result.status.success());
    assert!(String::from_utf8_lossy(&result.stderr).contains("invalid or oversized recording"));
}
