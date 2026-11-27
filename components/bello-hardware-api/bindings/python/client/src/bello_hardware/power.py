"""Read-only power board snapshots; independent of motor/IMU state frames."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, replace
from typing import TYPE_CHECKING

import grpc  # type: ignore[import-untyped]
from bello_hardware_api import hardware_pb2 as proto

from ._inventory import _optional_status
from .errors import ProtocolError, rpc_error
from .types import DeviceStatus

if TYPE_CHECKING:
    from .robot import Robot


@dataclass(frozen=True, slots=True)
class VoltageCurrent:
    voltage_v: float
    current_a: float


@dataclass(frozen=True, slots=True)
class PowerBattery:
    voltage_v: float
    current_a: float
    soc_percent: float | None


@dataclass(frozen=True, slots=True)
class PowerFlags:
    bits: int
    active_names: tuple[str, ...]


@dataclass(frozen=True, slots=True)
class PowerIwc:
    info: PowerFlags
    warning: PowerFlags
    critical: PowerFlags


@dataclass(frozen=True, slots=True)
class PowerLegacyFault:
    alarm: PowerFlags
    fault: PowerFlags


@dataclass(frozen=True, slots=True)
class PowerGpi:
    bits: int
    second_bits: int | None


@dataclass(frozen=True, slots=True)
class PowerEnvironment:
    temperature_c: float
    pressure_pa: float


@dataclass(frozen=True, slots=True)
class PowerLocalRails:
    rail_20v0: VoltageCurrent
    rail_5v0: VoltageCurrent
    rail_3v3: VoltageCurrent


@dataclass(frozen=True, slots=True)
class PowerEncoder:
    left_rpm: float
    right_rpm: float


PowerReading = (
    PowerBattery
    | PowerGpi
    | PowerIwc
    | PowerLegacyFault
    | PowerEnvironment
    | PowerLocalRails
    | VoltageCurrent
    | PowerEncoder
    | int
)


@dataclass(frozen=True, slots=True)
class PowerSample:
    kind: str
    sequence: int
    sample_age_ms: float
    stale: bool
    reading: PowerReading


@dataclass(frozen=True, slots=True)
class PowerBoardSnapshot:
    board_name: str
    connection_generation: int
    status: DeviceStatus | None
    samples: tuple[PowerSample, ...]
    invalid_frames: int
    unknown_frames: int
    received_monotonic: float
    stale_timeout_ms: int

    def age_ms(self, sample: PowerSample) -> float:
        return sample.sample_age_ms + (time.monotonic() - self.received_monotonic) * 1000

    def is_stale(self, sample: PowerSample) -> bool:
        return sample.stale or self.age_ms(sample) >= self.stale_timeout_ms


def _finite(*values: float) -> None:
    if not all(math.isfinite(v) for v in values):
        raise ProtocolError("nonfinite power board reading")


def _rail(value: proto.PowerVoltageCurrent) -> VoltageCurrent:
    _finite(value.voltage_v, value.current_a)
    return VoltageCurrent(value.voltage_v, value.current_a)


def _flags(value: proto.PowerFlags) -> PowerFlags:
    if len(value.active_names) > 32:
        raise ProtocolError("too many power flag names")
    return PowerFlags(value.bits, tuple(value.active_names))


def _decode_sample(sample: proto.PowerBoardSample) -> PowerSample:
    kind = sample.WhichOneof("reading")
    reading: PowerReading
    if kind == "battery":
        b = sample.battery
        _finite(b.voltage_v, b.current_a)
        soc = b.soc_percent if b.HasField("soc_percent") else None
        if soc is not None and (not math.isfinite(soc) or not 0 <= soc <= 100):
            raise ProtocolError("invalid power board charge percentage")
        reading = PowerBattery(b.voltage_v, b.current_a, soc)
    elif kind == "gpi":
        g = sample.gpi
        reading = PowerGpi(g.bits, g.second_bits if g.HasField("second_bits") else None)
    elif kind == "iwc":
        i = sample.iwc
        if not all(i.HasField(f) for f in ("info", "warning", "critical")):
            raise ProtocolError("incomplete IWC reading")
        reading = PowerIwc(_flags(i.info), _flags(i.warning), _flags(i.critical))
    elif kind == "legacy_fault":
        f = sample.legacy_fault
        if not f.HasField("alarm") or not f.HasField("fault"):
            raise ProtocolError("incomplete legacy fault reading")
        reading = PowerLegacyFault(_flags(f.alarm), _flags(f.fault))
    elif kind == "environment":
        e = sample.environment
        _finite(e.temperature_c, e.pressure_pa)
        reading = PowerEnvironment(e.temperature_c, e.pressure_pa)
    elif kind == "local_rails":
        r = sample.local_rails
        if not all(r.HasField(f) for f in ("rail_20v0", "rail_5v0", "rail_3v3")):
            raise ProtocolError("incomplete local rail reading")
        reading = PowerLocalRails(_rail(r.rail_20v0), _rail(r.rail_5v0), _rail(r.rail_3v3))
    elif kind == "motor_rail":
        reading = _rail(sample.motor_rail)
    elif kind == "tick_count":
        reading = sample.tick_count
    elif kind == "encoder":
        en = sample.encoder
        _finite(en.left_rpm, en.right_rpm)
        reading = PowerEncoder(en.left_rpm, en.right_rpm)
    else:
        raise ProtocolError("missing power board reading")
    if not 0 <= sample.sequence <= 255:
        raise ProtocolError("invalid COG sequence")
    return PowerSample(kind, sample.sequence, sample.sample_age_ms, sample.stale, reading)


class PowerBoardReader:
    """Fetch one snapshot per selected board. Each read opens a new stream.

    A board may have no samples yet.
    """

    def __init__(self, robot: Robot, boards: tuple[str, ...]) -> None:
        self._robot = robot
        known = {b.name: b for b in robot.inventory.power_boards}
        try:
            self._boards = {name: known[name] for name in boards} if boards else known
        except KeyError as error:
            raise ValueError(f"unknown power board {error}") from error

    def read(self, timeout: float | None = None) -> tuple[PowerBoardSnapshot, ...]:
        self._robot._ensure_open()
        timeout = self._robot._rpc_timeout if timeout is None else timeout
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("power telemetry timeout must be finite and positive")
        if not self._boards:
            return ()
        started = time.monotonic()
        stream = None
        results: dict[str, PowerBoardSnapshot] = {}
        try:
            stream = self._robot._stub.WatchTelemetry(
                proto.WatchTelemetryRequest(
                    include_power_boards=True,
                    power_boards=tuple(self._boards),
                ),
                timeout=timeout,
            )
            for event in stream:
                if event.WhichOneof("payload") != "power_board":
                    continue
                b = event.power_board
                if b.board_name not in self._boards:
                    raise ProtocolError("unexpected power board")
                samples = tuple(_decode_sample(s) for s in b.samples)
                if len({s.kind for s in samples}) != len(samples):
                    raise ProtocolError("duplicate power measurement")
                received = time.monotonic()
                # Conservatively include this request's complete elapsed time.
                samples = tuple(
                    replace(s, sample_age_ms=s.sample_age_ms + (received - started) * 1000)
                    for s in samples
                )
                results[b.board_name] = PowerBoardSnapshot(
                    b.board_name,
                    b.connection_generation,
                    _optional_status(b, "status"),
                    samples,
                    b.invalid_frames,
                    b.unknown_frames,
                    received,
                    self._boards[b.board_name].stale_timeout_ms,
                )
                if results.keys() == self._boards.keys():
                    return tuple(results[n] for n in sorted(results))
        except grpc.RpcError as error:
            raise rpc_error("power board telemetry", error) from error
        finally:
            if stream is not None:
                stream.cancel()
        raise ProtocolError("power telemetry stream ended before all boards responded")
