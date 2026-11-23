#!/usr/bin/env python3
"""Package the native hardware daemon and its configuration/model assets."""

import argparse
import os
import shutil
import subprocess
import zipfile
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--platform", required=True)
    parser.add_argument("--binary-dir", type=Path)
    args = parser.parse_args()
    if not args.platform or any(
        c not in "abcdefghijklmnopqrstuvwxyz0123456789-_" for c in args.platform
    ):
        parser.error(
            "platform must contain only lowercase letters, digits, '-' and '_'"
        )
    root = Path(__file__).resolve().parents[1]
    binary_dir = (
        args.binary_dir
        or Path(os.environ.get("CARGO_TARGET_DIR", root / "target")) / "release"
    )
    suffix = ".exe" if os.name == "nt" else ""
    bundle = root / "dist" / f"bello-desktop-{args.platform}"
    bundle.mkdir(parents=True, exist_ok=False)
    shutil.copy2(binary_dir / f"bello-hardwared{suffix}", bundle)
    tracked = subprocess.check_output(
        [
            "git",
            "ls-files",
            "-z",
            "--",
            "descriptions",
            "components/bello-hardwared/config",
        ],
        cwd=root,
    )
    for entry in tracked.split(b"\0"):
        if entry:
            relative = Path(os.fsdecode(entry))
            destination = bundle / relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(root / relative, destination)
    shutil.copy2(root / "LICENSE", bundle)
    (bundle / "docs").mkdir()
    shutil.copy2(root / "docs/desktop.md", bundle / "docs")
    (bundle / "README.md").write_text(
        "# Bello desktop\n\n"
        "Edit `components/bello-hardwared/config/milly-gsusb.yaml` to select your USB serial number.\n\n"
        "Start the daemon:\n\n"
        f"```\n./bello-hardwared{suffix} --config components/bello-hardwared/config/milly-gsusb.yaml\n```\n\n"
        "See [desktop setup](docs/desktop.md) for platform requirements and other robot profiles.\n",
        encoding="utf-8",
    )
    subprocess.run(
        [
            str(bundle / f"bello-hardwared{suffix}"),
            "--check",
            "--config",
            str(bundle / "components/bello-hardwared/config/milly-gsusb.yaml"),
        ],
        check=True,
    )
    archive = bundle.with_suffix(".zip")
    with zipfile.ZipFile(archive, "w", compression=zipfile.ZIP_DEFLATED) as output:
        for file in sorted(bundle.rglob("*")):
            if file.is_file():
                output.write(file, file.relative_to(bundle.parent))
    print(archive)


if __name__ == "__main__":
    main()
