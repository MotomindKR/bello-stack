from pathlib import Path

import pytest

from bello_camera.config import load_config
from bello_camera.pipeline import receiver_pipeline, sender_pipeline

ROOT = Path(__file__).parents[1]


def test_bundled_configs_and_pipelines() -> None:
    for name in ("milly.example.yaml", "test.yaml"):
        camera = load_config(ROOT / "config" / name).camera("wrist")
        assert (camera.video.width, camera.video.height, camera.video.fps) == (320, 240, 30)
    camera = load_config(ROOT / "config" / "milly.example.yaml").camera("wrist")
    sender = sender_pipeline(camera)
    assert "x264enc tune=zerolatency" in sender
    assert "rtph264pay pt=96" in sender
    receiver = receiver_pipeline(camera)
    assert "video/x-raw,format=RGB,width=320,height=240" in receiver
    assert "appsink name=frames emit-signals=true max-buffers=1 drop=true" in receiver


def test_unknown_fields_are_rejected(tmp_path: Path) -> None:
    path = tmp_path / "invalid.yaml"
    path.write_text(
        """
cameras:
  - name: wrist
    typo: true
    source: { type: test }
    video: { width: 320, height: 240, fps: 30 }
    encoder: { type: x264 }
    rtp: { host: 127.0.0.1, port: 5600 }
""",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="unknown fields"):
        load_config(path)


def test_three_views_share_one_composed_rtp_stream(tmp_path: Path) -> None:
    path = tmp_path / "rig.yaml"
    path.write_text(
        """
cameras:
  - name: policy
    views:
      - { name: top, source: { type: test, pattern: smpte } }
      - { name: left_wrist, source: { type: test, pattern: ball } }
      - { name: right_wrist, source: { type: test, pattern: snow } }
    video: { width: 224, height: 224, fps: 10 }
    encoder: { type: x264, bitrate_kbps: 1500 }
    rtp: { host: 127.0.0.1, port: 5600 }
""",
        encoding="utf-8",
    )

    camera = load_config(path).camera("policy")
    sender = sender_pipeline(camera)
    receiver = receiver_pipeline(camera)

    assert camera.view_names == ("top", "left_wrist", "right_wrist")
    assert camera.composite_width == 672
    assert camera.compositor_latency_ms == 0
    assert "compositor name=compose background=black force-live=true latency=0" in sender
    assert "sink_2::xpos=448" in sender
    assert "video/x-raw,width=672,height=224,framerate=30/1" in sender
    assert "video/x-raw,width=672,height=224,framerate=10/1" in sender
    assert sender.count("udpsink") == 1
    assert "width=672,height=224" in receiver


def test_realsense_source_is_validated_and_exposed_as_rgb_appsrc(tmp_path: Path) -> None:
    path = tmp_path / "realsense.yaml"
    path.write_text(
        """
cameras:
  - name: policy
    source:
      type: realsense
      serial: "219323071273"
      capture_width: 640
      capture_height: 480
      capture_fps: 30
    video: { width: 224, height: 224, fps: 10 }
    encoder: { type: x264 }
    rtp: { host: 127.0.0.1, port: 5600 }
""",
        encoding="utf-8",
    )

    camera = load_config(path).camera("policy")
    sender = sender_pipeline(camera)

    assert camera.source.serial == "219323071273"
    assert "appsrc name=realsense_source_0" in sender
    assert "video/x-raw,format=RGB,width=640,height=480,framerate=30/1" in sender
    assert "video/x-raw,width=224,height=224,pixel-aspect-ratio=1/1,framerate=10/1" in sender


def test_realsense_capture_dimensions_are_validated(tmp_path: Path) -> None:
    path = tmp_path / "invalid-realsense.yaml"
    path.write_text(
        """
cameras:
  - name: policy
    source: { type: realsense, capture_width: 0 }
    video: { width: 224, height: 224, fps: 10 }
    encoder: { type: x264 }
    rtp: { host: 127.0.0.1, port: 5600 }
""",
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="capture width and height"):
        load_config(path)
