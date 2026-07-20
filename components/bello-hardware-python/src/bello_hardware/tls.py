"""TLS credentials for private CAs and mutual authentication."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True, slots=True, repr=False)
class TlsCredentials:
    """Optional private CA and client identity for an HTTPS connection."""

    ca_certificate: bytes | None = None
    client_certificate: bytes | None = None
    client_private_key: bytes | None = None

    def __post_init__(self) -> None:
        values = (
            ("CA certificate", self.ca_certificate),
            ("client certificate", self.client_certificate),
            ("client private key", self.client_private_key),
        )
        for description, value in values:
            if value is not None and not value:
                raise ValueError(f"TLS {description} must not be empty")
        if (self.client_certificate is None) != (self.client_private_key is None):
            raise ValueError(
                "TLS client certificate and client private key must be provided together"
            )

    @classmethod
    def from_files(
        cls,
        *,
        ca_certificate: str | Path | None = None,
        client_certificate: str | Path | None = None,
        client_private_key: str | Path | None = None,
    ) -> TlsCredentials:
        """Loads PEM credentials once while constructing the connection settings."""
        return cls(
            ca_certificate=_read(ca_certificate),
            client_certificate=_read(client_certificate),
            client_private_key=_read(client_private_key),
        )

    def __repr__(self) -> str:
        return (
            "TlsCredentials("
            f"custom_ca={self.ca_certificate is not None}, "
            f"client_identity={self.client_certificate is not None})"
        )

    def _grpc_arguments(self) -> dict[str, bytes | None]:
        return {
            "root_certificates": self.ca_certificate,
            "private_key": self.client_private_key,
            "certificate_chain": self.client_certificate,
        }


def _read(path: str | Path | None) -> bytes | None:
    if path is None:
        return None
    return Path(path).read_bytes()
