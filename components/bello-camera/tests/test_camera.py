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
