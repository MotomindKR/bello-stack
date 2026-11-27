#!/usr/bin/env python3
"""Regenerate the Python hardware API from the canonical proto."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

PACKAGE = Path(__file__).resolve().parents[1]
SCHEMAS = PACKAGE.parents[2] / "proto" / "bello"
OUTPUT = PACKAGE / "src" / "bello_hardware_api"


def generate(name: str) -> None:
    proto = SCHEMAS / name / "v1" / f"{name}.proto"
    OUTPUT.mkdir(parents=True, exist_ok=True)
    subprocess.run(
        [
            sys.executable,
            "-m",
            "grpc_tools.protoc",
            f"--proto_path={proto.parent}",
            f"--python_out={OUTPUT}",
            f"--pyi_out={OUTPUT}",
            f"--grpc_python_out={OUTPUT}",
            proto.name,
        ],
        cwd=proto.parent,
        check=True,
    )

    protobuf_module = OUTPUT / f"{name}_pb2.py"
    generated = protobuf_module.read_text()
    protobuf_module.write_text(
        generated.replace(f"'{name}_pb2', _globals", "__name__, _globals")
    )

    grpc_module = OUTPUT / f"{name}_pb2_grpc.py"
    generated = grpc_module.read_text()
    generated = generated.replace(
        f"import {name}_pb2 as {name}__pb2",
        f"from . import {name}_pb2 as {name}__pb2",
    )
    grpc_module.write_text("# mypy: ignore-errors\n" + generated)

    protobuf_stubs = OUTPUT / f"{name}_pb2.pyi"
    protobuf_stubs.write_text("# mypy: ignore-errors\n" + protobuf_stubs.read_text())


if __name__ == "__main__":
    for service in ("hardware", "camera"):
        generate(service)
