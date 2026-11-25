"""Blocking classic SocketCAN client for one Linker O6. Target values are raw SDK bytes."""

import socket
import struct
import sys
import threading
import time
from contextlib import contextmanager
from dataclasses import dataclass

from .config import Endpoint

AXES = (
    "thumb_cmc_pitch",
    "thumb_cmc_yaw",
    "index_mcp_pitch",
    "middle_mcp_pitch",
    "ring_mcp_pitch",
    "pinky_mcp_pitch",
)
POSITION = 0x01
FAULT = 0x35
CAN_FRAME = struct.Struct("=IB3x8s")
CAN_FILTER = struct.Struct("=II")
# Limit all host frames combined, including status requests and motion commands.
TX_INTERVAL = 0.020
RESPONSE_TIMEOUT = 0.15


@dataclass(frozen=True)
class Reading:
    values: tuple[int, ...]
    received_monotonic_ns: int


class O6Hand:
    """Single-owner hand session. Transport failures invalidate the session."""

    def __init__(self, endpoint: Endpoint):
        self.endpoint = endpoint
        self._owner = None
        self._operation = threading.Lock()
        self._next_tx = 0.0

    def __enter__(self):
        if sys.platform != "linux":
            raise OSError("SocketCAN hand control requires Linux")
        with self._operation:
            if self._owner is not None:
                raise RuntimeError("Hand session already open")
            owner = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
            try:
                # Abstract socket ownership follows the process and network namespace.
                owner.bind(f"\0bello-hand/{self.endpoint.interface}/{self.endpoint.can_id}")
            except OSError:
                owner.close()
                raise
            self._owner = owner
            self._next_tx = time.monotonic() + TX_INTERVAL
        return self

    def __exit__(self, *_):
        with self._operation:
            self._close()

    def _close(self):
        if self._owner is not None:
            self._owner.close()
            self._owner = None

    @contextmanager
    def _transaction(self):
        if not self._operation.acquire(blocking=False):
            raise RuntimeError("Hand session is busy")
        try:
            if self._owner is None:
                raise RuntimeError("Open a new hand session first")
            yield
        except OSError:
            self._close()
            raise
        finally:
            self._operation.release()

    def _exchange(self, register: int, target: bytes = b"") -> Reading | None:
        delay = self._next_tx - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        # A fresh receive queue excludes old state from previous transactions.
        with socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as bus:
            mask = socket.CAN_EFF_FLAG | socket.CAN_RTR_FLAG | socket.CAN_SFF_MASK
            bus.setsockopt(
                socket.SOL_CAN_RAW,
                socket.CAN_RAW_FILTER,
                CAN_FILTER.pack(self.endpoint.can_id, mask),
            )
            bus.bind((self.endpoint.interface,))
            bus.settimeout(RESPONSE_TIMEOUT)
            data = bytes([register]) + target
            frame = CAN_FRAME.pack(self.endpoint.can_id, len(data), data)
            if bus.send(frame) != CAN_FRAME.size:
                raise OSError("Incomplete SocketCAN write")
            self._next_tx = time.monotonic() + TX_INTERVAL
            if target:
                return None
            deadline = time.monotonic() + RESPONSE_TIMEOUT
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError("O6 response timed out")
                bus.settimeout(remaining)
                raw, _, flags, _ = bus.recvmsg(CAN_FRAME.size)
                # A target written by another local socket has the same bytes as a reply.
                if flags & socket.MSG_DONTROUTE or len(raw) != CAN_FRAME.size:
                    continue
                can_id, length, payload = CAN_FRAME.unpack(raw)
                if can_id == self.endpoint.can_id and length == 7 and payload[0] == register:
                    return Reading(tuple(payload[1:7]), time.monotonic_ns())

    def _read(self, register: int) -> Reading:
        reading = self._exchange(register)
        assert reading is not None
        return reading

    def status(self) -> tuple[Reading, Reading]:
        """Read positions and fault codes; each response has its own host receipt time."""
        with self._transaction():
            return self._read(POSITION), self._read(FAULT)

    def set_positions(self, values: tuple[int, ...]) -> None:
        """Queue one explicit position target after a fresh zero-fault response."""
        if len(values) != len(AXES) or any(type(v) is not int or not 0 <= v <= 255 for v in values):
            raise ValueError("Expected six integer O6 position bytes in 0..255")
        with self._transaction():
            faults = self._read(FAULT)
            if any(faults.values):
                raise ValueError(f"O6 reports active faults: {faults.values}")
            self._exchange(POSITION, bytes(values))
