from __future__ import annotations

from pathlib import Path
from urllib.parse import urlsplit

import grpc  # type: ignore[import-untyped]

from .errors import ConnectionError
from .tls import TlsCredentials

_DEFAULT_OPTIONS: tuple[tuple[str, int | str], ...] = (
    ("grpc.keepalive_time_ms", 10_000),
    ("grpc.keepalive_timeout_ms", 3_000),
    ("grpc.keepalive_permit_without_calls", 1),
)


def create_channel(
    endpoint: str,
    *,
    timeout: float,
    tls: TlsCredentials | None,
) -> grpc.Channel:
    if endpoint != endpoint.strip() or not endpoint:
        raise ValueError("endpoint must be non-empty and contain no surrounding whitespace")
    if timeout <= 0.0:
        raise ValueError("connect timeout must be positive")

    options = _DEFAULT_OPTIONS
    if endpoint.startswith("unix:"):
        if not endpoint.startswith("unix:///"):
            raise ValueError("Unix endpoint must use unix:///absolute/path")
        socket_path = endpoint.removeprefix("unix://")
        if not Path(socket_path).is_absolute():
            raise ValueError("Unix endpoint path must be absolute")
        target = endpoint
        # The socket path is a transport address, not an HTTP authority.
        options += (("grpc.default_authority", "localhost"),)
        secure = False
    elif "://" not in endpoint:
        target = endpoint
        secure = False
    else:
        parsed = urlsplit(endpoint)
        if parsed.scheme not in {"http", "https"} or not parsed.netloc:
            raise ValueError("endpoint must use http://, https://, or unix:///absolute/path")
        if parsed.path not in {"", "/"} or parsed.query or parsed.fragment:
            raise ValueError("endpoint must not contain a path, query, or fragment")
        target = parsed.netloc
        secure = parsed.scheme == "https"

    if tls is not None and not secure:
        raise ValueError("TLS credentials require an https:// endpoint")

    if secure:
        arguments = tls._grpc_arguments() if tls is not None else {}
        channel = grpc.secure_channel(
            target,
            grpc.ssl_channel_credentials(**arguments),
            options=options,
        )
    else:
        channel = grpc.insecure_channel(target, options=options)

    try:
        grpc.channel_ready_future(channel).result(timeout=timeout)
    except grpc.FutureTimeoutError as error:
        channel.close()
        raise ConnectionError(f"timed out connecting to {endpoint!r}") from error
    return channel
