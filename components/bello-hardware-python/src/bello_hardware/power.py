"""Read-only power board snapshots; independent of motor/IMU state frames."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, replace
from typing import TYPE_CHECKING

import grpc  # type: ignore[import-untyped]

from ._generated import hardware_pb2 as proto
from ._inventory import _optional_status
from .errors import ProtocolError, rpc_error
from .types import DeviceStatus

if TYPE_CHECKING:
    from .robot import Robot


PowerReading = (
    proto.PowerBattery
    | proto.PowerGpi
    | proto.PowerIwc
    | proto.PowerLegacyFault
    | proto.PowerEnvironment
    | proto.PowerLocalRails
    | proto.PowerVoltageCurrent
    | proto.PowerEncoder
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


def _validate_rail(value: proto.PowerVoltageCurrent) -> None:
    _finite(value.voltage_v, value.current_a)


def _validate_flags(*values: proto.PowerFlags) -> None:
    if any(len(value.active_names) > 32 for value in values):
        raise ProtocolError("too many power flag names")


def _decode_sample(sample: proto.PowerBoardSample) -> PowerSample:
    kind = sample.WhichOneof("reading")
    reading: PowerReading
    if kind == "battery":
        reading = sample.battery
        _finite(reading.voltage_v, reading.current_a)
        if reading.HasField("soc_percent") and not 0 <= reading.soc_percent <= 100:
            raise ProtocolError("invalid power board charge percentage")
    elif kind == "gpi":
        reading = sample.gpi
    elif kind == "iwc":
        reading = sample.iwc
        if not all(reading.HasField(f) for f in ("info", "warning", "critical")):
            raise ProtocolError("incomplete IWC reading")
        _validate_flags(reading.info, reading.warning, reading.critical)
    elif kind == "legacy_fault":
        reading = sample.legacy_fault
        if not reading.HasField("alarm") or not reading.HasField("fault"):
            raise ProtocolError("incomplete legacy fault reading")
        _validate_flags(reading.alarm, reading.fault)
    elif kind == "environment":
        reading = sample.environment
        _finite(reading.temperature_c, reading.pressure_pa)
    elif kind == "local_rails":
        reading = sample.local_rails
        if not all(reading.HasField(f) for f in ("rail_20v0", "rail_5v0", "rail_3v3")):
            raise ProtocolError("incomplete local rail reading")
        _validate_rail(reading.rail_20v0)
        _validate_rail(reading.rail_5v0)
        _validate_rail(reading.rail_3v3)
    elif kind == "motor_rail":
        reading = sample.motor_rail
        _validate_rail(reading)
    elif kind == "tick_count":
        reading = sample.tick_count
    elif kind == "encoder":
        reading = sample.encoder
        _finite(reading.left_rpm, reading.right_rpm)
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
        if len(set(boards)) != len(boards) or set(boards) - known.keys():
            raise ValueError("unknown or duplicate power board selector")
        self._boards = {n: known[n] for n in boards} if boards else known

    def read(self, timeout: float | None = None) -> tuple[PowerBoardSnapshot, ...]:
        self._robot._ensure_open()
        timeout = self._robot._rpc_timeout if timeout is None else timeout
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("power telemetry timeout must be finite and positive")
        if not self._boards:
            return ()
        started = time.monotonic()
        stream = self._robot._stub.WatchTelemetry(
            proto.WatchTelemetryRequest(
                include_power_boards=True,
                power_boards=tuple(self._boards),
            ),
            timeout=timeout,
        )
        results: dict[str, PowerBoardSnapshot] = {}
        try:
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
            stream.cancel()
        raise ProtocolError("power telemetry stream ended before all boards responded")
