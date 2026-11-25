"""An occupied service socket must fail the preview before its requested duration."""

from pathlib import Path
import socket
import subprocess
import sys
import tempfile


def main(binary: str) -> None:
    configs = Path(__file__).resolve().parents[1] / "config"
    with (
        tempfile.TemporaryDirectory(dir="/tmp") as directory,
        socket.socket(socket.AF_UNIX) as owner,
    ):
        path = Path(directory) / "camera.sock"
        owner.bind(str(path))
        owner.listen()
        profile = Path(directory) / "cameras.yaml"
        profile.write_text(
            (configs / "cameras_mini_torso.yaml")
            .read_text()
            .replace("/run/bello/mujoco-cameras.sock", str(path))
        )
        result = subprocess.run(
            [
                binary,
                "preview",
                "--model-config",
                configs / "bello_mini_torso.yaml",
                "--camera-config",
                profile,
                "--seconds",
                "120",
            ],
            capture_output=True,
            text=True,
            timeout=45,
        )
        assert result.returncode != 0 and "already has a live owner" in result.stderr, (
            result.stderr
        )


if __name__ == "__main__":
    main(sys.argv[1])
