from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


@dataclass(frozen=True, slots=True)
class SourceConfig:
    type: str
    device: str | None = None
    format: str = "mjpeg"
    pattern: str = "smpte"
    launch: str | None = None


@dataclass(frozen=True, slots=True)
class VideoConfig:
    width: int
    height: int
    fps: int


@dataclass(frozen=True, slots=True)
class EncoderConfig:
    type: str
    bitrate_kbps: int = 1_500
    keyframe_interval: int | None = None
    speed_preset: str = "ultrafast"
    launch: str | None = None


@dataclass(frozen=True, slots=True)
class RtpConfig:
    host: str
    port: int
    bind_address: str = "0.0.0.0"
    payload_type: int = 96
    mtu: int = 1_200
    receiver_latency_ms: int = 30


@dataclass(frozen=True, slots=True)
class CameraConfig:
    name: str
    source: SourceConfig
    video: VideoConfig
    encoder: EncoderConfig
    rtp: RtpConfig


@dataclass(frozen=True, slots=True)
class StreamConfig:
    cameras: tuple[CameraConfig, ...]

    def camera(self, name: str) -> CameraConfig:
        for camera in self.cameras:
            if camera.name == name:
                return camera
        raise KeyError(f"unknown camera {name!r}")


def load_config(path: str | Path) -> StreamConfig:
    config_path = Path(path)
    root = _mapping(yaml.safe_load(config_path.read_text(encoding="utf-8")), "configuration")
    _only(root, {"cameras"}, "configuration")
    values = root.get("cameras")
    if not isinstance(values, list) or not values:
        raise ValueError("configuration.cameras must be a non-empty list")
    cameras = tuple(_camera(item, index) for index, item in enumerate(values))
    _validate_unique(cameras)
    return StreamConfig(cameras)


def _camera(value: object, index: int) -> CameraConfig:
    where = f"cameras[{index}]"
    raw = _mapping(value, where)
    _only(raw, {"name", "source", "video", "encoder", "rtp"}, where)
    name = _text(raw.get("name"), f"{where}.name")
    if not all(char.isascii() and (char.isalnum() or char in "_-") for char in name):
        raise ValueError(f"{where}.name must contain only ASCII letters, digits, '_' or '-'")
    return CameraConfig(
        name,
        _source(raw.get("source"), f"{where}.source"),
        _video(raw.get("video"), f"{where}.video"),
        _encoder(raw.get("encoder"), f"{where}.encoder"),
        _rtp(raw.get("rtp"), f"{where}.rtp"),
    )


def _source(value: object, where: str) -> SourceConfig:
    raw = _mapping(value, where)
    _only(raw, {"type", "device", "format", "pattern", "launch"}, where)
    kind = _text(raw.get("type"), f"{where}.type")
    if kind == "v4l2":
        device = _text(raw.get("device"), f"{where}.device")
        if not Path(device).is_absolute():
            raise ValueError(f"{where}.device must be an absolute path")
        media_format = _text(raw.get("format", "mjpeg"), f"{where}.format")
        if media_format not in {"mjpeg", "raw"}:
            raise ValueError(f"{where}.format must be 'mjpeg' or 'raw'")
        return SourceConfig(kind, device=device, format=media_format)
    if kind == "test":
        return SourceConfig(kind, pattern=_text(raw.get("pattern", "smpte"), f"{where}.pattern"))
    if kind == "custom":
        return SourceConfig(kind, launch=_text(raw.get("launch"), f"{where}.launch"))
    raise ValueError(f"{where}.type must be 'v4l2', 'test', or 'custom'")


def _video(value: object, where: str) -> VideoConfig:
    raw = _mapping(value, where)
    _only(raw, {"width", "height", "fps"}, where)
    config = VideoConfig(
        _integer(raw.get("width"), f"{where}.width"),
        _integer(raw.get("height"), f"{where}.height"),
        _integer(raw.get("fps"), f"{where}.fps"),
    )
    if not 16 <= config.width <= 8_192 or not 16 <= config.height <= 8_192:
        raise ValueError(f"{where} width and height must each be in 16..=8192")
    if not 1 <= config.fps <= 240:
        raise ValueError(f"{where}.fps must be in 1..=240")
    return config


def _encoder(value: object, where: str) -> EncoderConfig:
    raw = _mapping(value, where)
    _only(raw, {"type", "bitrate_kbps", "keyframe_interval", "speed_preset", "launch"}, where)
    kind = _text(raw.get("type"), f"{where}.type")
    if kind == "x264":
        bitrate = _integer(raw.get("bitrate_kbps", 1_500), f"{where}.bitrate_kbps")
        interval_raw = raw.get("keyframe_interval")
        interval = (
            None if interval_raw is None else _integer(interval_raw, f"{where}.keyframe_interval")
        )
        if not 1 <= bitrate <= 1_000_000:
            raise ValueError(f"{where}.bitrate_kbps must be in 1..=1000000")
        if interval is not None and interval <= 0:
            raise ValueError(f"{where}.keyframe_interval must be positive")
        return EncoderConfig(
            kind,
            bitrate_kbps=bitrate,
            keyframe_interval=interval,
            speed_preset=_text(raw.get("speed_preset", "ultrafast"), f"{where}.speed_preset"),
        )
    if kind == "custom":
        return EncoderConfig(kind, launch=_text(raw.get("launch"), f"{where}.launch"))
    raise ValueError(f"{where}.type must be 'x264' or 'custom'")


def _rtp(value: object, where: str) -> RtpConfig:
    raw = _mapping(value, where)
    _only(
        raw, {"host", "port", "bind_address", "payload_type", "mtu", "receiver_latency_ms"}, where
    )
    config = RtpConfig(
        _text(raw.get("host"), f"{where}.host"),
        _integer(raw.get("port"), f"{where}.port"),
        _text(raw.get("bind_address", "0.0.0.0"), f"{where}.bind_address"),
        _integer(raw.get("payload_type", 96), f"{where}.payload_type"),
        _integer(raw.get("mtu", 1_200), f"{where}.mtu"),
        _integer(raw.get("receiver_latency_ms", 30), f"{where}.receiver_latency_ms"),
    )
    if not 1 <= config.port <= 65_535:
        raise ValueError(f"{where}.port must be in 1..=65535")
    if not 96 <= config.payload_type <= 127:
        raise ValueError(f"{where}.payload_type must be in 96..=127")
    if not 256 <= config.mtu <= 9_000:
        raise ValueError(f"{where}.mtu must be in 256..=9000")
    if not 0 <= config.receiver_latency_ms <= 5_000:
        raise ValueError(f"{where}.receiver_latency_ms must be in 0..=5000")
    return config


def _validate_unique(cameras: tuple[CameraConfig, ...]) -> None:
    names: set[str] = set()
    destinations: set[tuple[str, int]] = set()
    for camera in cameras:
        if camera.name in names:
            raise ValueError(f"duplicate camera name {camera.name!r}")
        names.add(camera.name)
        destination = (camera.rtp.host, camera.rtp.port)
        if destination in destinations:
            raise ValueError(f"duplicate RTP destination {camera.rtp.host}:{camera.rtp.port}")
        destinations.add(destination)


def _mapping(value: object, where: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping) or not all(isinstance(key, str) for key in value):
        raise ValueError(f"{where} must be a mapping with string keys")
    return value


def _only(value: Mapping[str, Any], allowed: set[str], where: str) -> None:
    unknown = sorted(set(value) - allowed)
    if unknown:
        raise ValueError(f"{where} has unknown fields: {unknown}")


def _text(value: object, where: str) -> str:
    if not isinstance(value, str) or not value.strip() or value != value.strip() or "\0" in value:
        raise ValueError(f"{where} must be non-empty without surrounding whitespace or NUL")
    return value


def _integer(value: object, where: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{where} must be an integer")
    return value
