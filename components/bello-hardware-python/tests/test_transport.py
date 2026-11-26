from pathlib import Path
from typing import Protocol

import pytest

from bello_hardware import Robot, TlsCredentials, proto


class _RunningServer(Protocol):
    endpoint: str


def test_unix_socket_connection(unix_hardware_server: _RunningServer) -> None:
    with Robot.connect(unix_hardware_server.endpoint) as robot:
        groups = robot.grpc_client().ListHardware(proto.ListHardwareRequest()).groups
        assert tuple(group.name for group in groups) == robot.inventory.group_names == ("leg",)


def test_tls_credentials_require_a_complete_client_identity() -> None:
    with pytest.raises(ValueError, match="provided together"):
        TlsCredentials(client_certificate=b"certificate")


def test_tls_credentials_load_files_without_exposing_the_private_key(tmp_path: Path) -> None:
    ca = tmp_path / "ca.pem"
    certificate = tmp_path / "client.pem"
    private_key = tmp_path / "client-key.pem"
    ca.write_bytes(b"ca")
    certificate.write_bytes(b"certificate")
    private_key.write_bytes(b"secret-private-key")

    credentials = TlsCredentials.from_files(
        ca_certificate=ca,
        client_certificate=certificate,
        client_private_key=private_key,
    )

    assert credentials.ca_certificate == b"ca"
    assert "secret-private-key" not in repr(credentials)


def test_tls_credentials_require_an_https_endpoint() -> None:
    credentials = TlsCredentials(ca_certificate=b"ca")
    with pytest.raises(ValueError, match="require an https"):
        Robot.connect("http://127.0.0.1:50051", tls=credentials)
