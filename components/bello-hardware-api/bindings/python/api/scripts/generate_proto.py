#!/usr/bin/env python3
"""Regenerate the Python hardware API from the canonical proto."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

PACKAGE = Path(__file__).resolve().parents[1]
PROTO = PACKAGE.parents[2] / "proto" / "bello" / "hardware" / "v1" / "hardware.proto"
OUTPUT = PACKAGE / "src" / "bello_hardware_api"


def main() -> None:
    OUTPUT.mkdir(parents=True, exist_ok=True)
    subprocess.run(
        [
            sys.executable,
            "-m",
            "grpc_tools.protoc",
            f"--proto_path={PROTO.parent}",
            f"--python_out={OUTPUT}",
            f"--pyi_out={OUTPUT}",
            f"--grpc_python_out={OUTPUT}",
            PROTO.name,
        ],
        cwd=PROTO.parent,
        check=True,
    )

    protobuf_module = OUTPUT / "hardware_pb2.py"
    generated = protobuf_module.read_text()
    protobuf_module.write_text(
        generated.replace("'hardware_pb2', _globals", "__name__, _globals")
    )

    grpc_module = OUTPUT / "hardware_pb2_grpc.py"
    generated = grpc_module.read_text()
    generated = generated.replace(
        "import hardware_pb2 as hardware__pb2",
        "from . import hardware_pb2 as hardware__pb2",
    )
    grpc_module.write_text("# mypy: ignore-errors\n" + generated)

    protobuf_stubs = OUTPUT / "hardware_pb2.pyi"
    protobuf_stubs.write_text("# mypy: ignore-errors\n" + protobuf_stubs.read_text())


if __name__ == "__main__":
    main()
