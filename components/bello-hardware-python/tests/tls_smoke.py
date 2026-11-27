"""Process-level mTLS smoke entrypoint invoked by the Rust transport test."""

from __future__ import annotations

import sys

from bello_hardware import Robot, TlsCredentials


def main() -> None:
    if len(sys.argv) != 5:
        raise SystemExit("usage: tls_smoke.py ENDPOINT CA CLIENT_CERT CLIENT_KEY")
    endpoint, ca, client_certificate, client_private_key = sys.argv[1:]
    tls = TlsCredentials.from_files(
        ca_certificate=ca,
        client_certificate=client_certificate,
        client_private_key=client_private_key,
    )
    with Robot.connect(endpoint, timeout=2.0, tls=tls) as robot:
        if "e2e_group" not in robot.inventory.group_names:
            raise RuntimeError("Python mTLS inventory omitted e2e_group")


if __name__ == "__main__":
    main()
