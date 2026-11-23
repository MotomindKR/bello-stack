#!/usr/bin/env python3

from __future__ import annotations

import argparse
import math
import select
import socket
import struct
import sys
import time
from collections.abc import Callable


CAN_EFF_FLAG = 0x80000000
CAN_EFF_MASK = 0x1FFFFFFF
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000
CAN_FRAME = struct.Struct("=IB3x8s")

COMM_DEVICE_ID = 0
COMM_MOTOR_FEEDBACK = 2
COMM_MOTOR_STOP = 4
COMM_PARAM_READ = 17
COMM_PARAM_WRITE = 18
COMM_DATA_SAVE = 22

PARAM_MECHANICAL_POSITION = 0x7019
PARAM_POSITION_OFFSET = 0x702B

DEFAULT_HOST_ID = 0
DEFAULT_TIMEOUT_SECONDS = 0.5
DEFAULT_RETRIES = 3
POSITION_MIN_RADIANS = -12.57
POSITION_MAX_RADIANS = 12.57
VERIFY_TOLERANCE_RADIANS = 0.02


class ProtocolError(RuntimeError):
    pass


def build_can_id(comm_type: int, data_field: int, target_id: int) -> int:
    if not 0 <= comm_type <= 0x1F:
        raise ValueError(f"communication type is out of range: {comm_type}")
    if not 0 <= data_field <= 0xFFFF:
        raise ValueError(f"CAN data field is out of range: {data_field}")
    if not 0 <= target_id <= 0xFF:
        raise ValueError(f"CAN target ID is out of range: {target_id}")
    return (comm_type << 24) | (data_field << 8) | target_id


def comm_type(can_id: int) -> int:
    return (can_id >> 24) & 0x1F


def data_field(can_id: int) -> int:
    return (can_id >> 8) & 0xFFFF


def target_id(can_id: int) -> int:
    return can_id & 0xFF


def pack_frame(can_id: int, payload: bytes) -> bytes:
    if len(payload) > 8:
        raise ValueError("classic CAN payloads cannot exceed 8 bytes")
    return CAN_FRAME.pack(
        can_id | CAN_EFF_FLAG,
        len(payload),
        payload.ljust(8, b"\0"),
    )


def unpack_frame(raw: bytes) -> tuple[int, bytes] | None:
    if len(raw) != CAN_FRAME.size:
        return None
    raw_id, dlc, payload = CAN_FRAME.unpack(raw)
    if raw_id & CAN_EFF_FLAG == 0 or raw_id & (CAN_RTR_FLAG | CAN_ERR_FLAG) or dlc > 8:
        return None
    return raw_id & CAN_EFF_MASK, payload[:dlc]


class RobstrideConnection:
    def __init__(
        self,
        interface: str,
        motor_id: int,
        *,
        host_id: int = DEFAULT_HOST_ID,
        timeout_seconds: float = DEFAULT_TIMEOUT_SECONDS,
        retries: int = DEFAULT_RETRIES,
    ) -> None:
        if not hasattr(socket, "AF_CAN"):
            raise RuntimeError("SocketCAN is only available on Linux")
        if not interface.strip():
            raise ValueError("CAN interface must not be empty")
        if not 1 <= motor_id <= 0x7F:
            raise ValueError("motor ID must be in the range 1..127")
        if not 0 <= host_id <= 0xFF:
            raise ValueError("host ID must be in the range 0..255")
        if motor_id == host_id:
            raise ValueError("motor ID must differ from the host ID")
        if not math.isfinite(timeout_seconds) or timeout_seconds <= 0.0:
            raise ValueError("timeout must be positive and finite")
        if retries <= 0:
            raise ValueError("retries must be positive")

        self.interface = interface
        self.motor_id = motor_id
        self.host_id = host_id
        self.timeout_seconds = timeout_seconds
        self.retries = retries
        self.socket = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        try:
            self.socket.setblocking(False)
            self.socket.bind((interface,))
        except Exception:
            self.socket.close()
            raise

    def close(self) -> None:
        self.socket.close()

    def __enter__(self) -> RobstrideConnection:
        return self

    def __exit__(self, *args: object) -> None:
        self.close()

    def stop(self) -> None:
        payload = bytes(8)
        response = self._exchange(
            COMM_MOTOR_STOP,
            payload,
            lambda can_id, _payload: self._is_motor_response(
                can_id, COMM_MOTOR_FEEDBACK
            ),
            "stop acknowledgement",
        )
        if len(response) < 8:
            raise ProtocolError("motor returned a truncated stop acknowledgement")

    def read_float(self, parameter: int) -> float:
        payload = struct.pack("<H", parameter) + bytes(6)
        response = self._exchange(
            COMM_PARAM_READ,
            payload,
            lambda can_id, response_payload: (
                self._is_motor_response(can_id, COMM_PARAM_READ)
                and len(response_payload) >= 8
                and struct.unpack_from("<H", response_payload)[0] == parameter
            ),
            f"parameter read response for {parameter:#06x}",
        )
        value = struct.unpack_from("<f", response, 4)[0]
        if not math.isfinite(value):
            raise ProtocolError(
                f"motor returned a non-finite value for parameter {parameter:#06x}"
            )
        return value

    def write_float(self, parameter: int, value: float) -> None:
        if not math.isfinite(value):
            raise ValueError("parameter value must be finite")
        payload = bytearray(8)
        struct.pack_into("<H", payload, 0, parameter)
        struct.pack_into("<f", payload, 4, value)
        self._exchange(
            COMM_PARAM_WRITE,
            bytes(payload),
            lambda can_id, response_payload: (
                self._is_motor_response(can_id, COMM_MOTOR_FEEDBACK)
                or (
                    self._is_motor_response(can_id, COMM_PARAM_WRITE)
                    and len(response_payload) >= 2
                    and struct.unpack_from("<H", response_payload)[0] == parameter
                )
            ),
            f"parameter write response for {parameter:#06x}",
        )

    def save(self) -> None:
        self._exchange(
            COMM_DATA_SAVE,
            bytes(range(1, 9)),
            self._is_save_response,
            "save acknowledgement",
        )

    def _is_save_response(self, can_id: int, payload: bytes) -> bool:
        if len(payload) != 8:
            return False
        if self._is_motor_response(can_id, COMM_MOTOR_FEEDBACK):
            return True
        # The installed Mini firmware broadcasts its device ID after saving.
        return (
            comm_type(can_id) == COMM_DEVICE_ID
            and data_field(can_id) == self.motor_id
            and target_id(can_id) == 0xFE
        )

    def _is_motor_response(self, can_id: int, expected_type: int) -> bool:
        return (
            comm_type(can_id) == expected_type
            and target_id(can_id) == self.host_id
            and data_field(can_id) & 0xFF == self.motor_id
        )

    def _exchange(
        self,
        request_type: int,
        payload: bytes,
        matches: Callable[[int, bytes], bool],
        response_name: str,
    ) -> bytes:
        request_id = build_can_id(request_type, self.host_id, self.motor_id)
        for _attempt in range(1, self.retries + 1):
            self._drain_receive_queue()
            sent = self.socket.send(pack_frame(request_id, payload))
            if sent != CAN_FRAME.size:
                raise OSError(f"short SocketCAN write: {sent}/{CAN_FRAME.size}")

            deadline = time.monotonic() + self.timeout_seconds
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    break
                readable, _, _ = select.select([self.socket], [], [], remaining)
                if not readable:
                    break
                try:
                    raw = self.socket.recv(CAN_FRAME.size)
                except BlockingIOError:
                    continue
                frame = unpack_frame(raw)
                if frame is not None and matches(*frame):
                    return frame[1]
        raise ProtocolError(
            f"timed out waiting for {response_name} after {self.retries} attempts"
        )

    def _drain_receive_queue(self) -> None:
        while True:
            try:
                self.socket.recv(CAN_FRAME.size)
            except BlockingIOError:
                return


def parse_angle(value: str) -> float:
    text = value.strip().lower().replace(" ", "")
    if text.endswith("degrees"):
        number = text[: -len("degrees")]
        radians = math.radians(float(number))
    elif text.endswith("degree"):
        number = text[: -len("degree")]
        radians = math.radians(float(number))
    elif text.endswith("deg"):
        radians = math.radians(float(text[:-3]))
    elif text.endswith("°"):
        radians = math.radians(float(text[:-1]))
    elif text.endswith("radians"):
        radians = float(text[: -len("radians")])
    elif text.endswith("radian"):
        radians = float(text[: -len("radian")])
    elif text.endswith("rad"):
        radians = float(text[:-3])
    else:
        radians = float(text)
    if not math.isfinite(radians):
        raise ValueError("angle must be finite")
    if not POSITION_MIN_RADIANS <= radians <= POSITION_MAX_RADIANS:
        raise ValueError(
            "angle must be within the RobStride position range "
            f"[{POSITION_MIN_RADIANS}, {POSITION_MAX_RADIANS}] rad"
        )
    return radians


def parse_motor_id(value: str) -> int:
    motor_id = int(value.strip(), 0)
    if not 1 <= motor_id <= 0x7F:
        raise ValueError("motor ID must be in the range 1..127")
    return motor_id


def parse_host_id(value: str) -> int:
    host_id = int(value.strip(), 0)
    if not 0 <= host_id <= 0xFF:
        raise ValueError("host ID must be in the range 0..255")
    return host_id


def prompt_value(prompt: str, parser: Callable[[str], object]) -> object:
    while True:
        try:
            return parser(input(prompt))
        except ValueError as error:
            print(f"Invalid value: {error}", file=sys.stderr)


def requested_values(args: argparse.Namespace) -> tuple[str, int, float]:
    interface = args.canbus
    if interface is None:
        interface = input("CAN interface [can0]: ").strip() or "can0"
    if not interface.strip():
        raise ValueError("CAN interface must not be empty")

    motor_id = args.motor_id
    if motor_id is None:
        motor_id = prompt_value("Motor ID (decimal or 0xNN): ", parse_motor_id)

    angle = args.angle
    if angle is None:
        angle = prompt_value(
            "Angle to assign to the current position (rad; 'deg' suffix accepted): ",
            parse_angle,
        )
    return interface, motor_id, angle


def calibrate(
    connection: RobstrideConnection,
    desired_position: float,
    *,
    skip_confirmation: bool,
) -> tuple[float, float, float]:
    connection.stop()
    current_position = connection.read_float(PARAM_MECHANICAL_POSITION)
    current_offset = connection.read_float(PARAM_POSITION_OFFSET)
    new_offset = current_offset + current_position - desired_position

    print(f"Motor is disabled on {connection.interface}.")
    print(f"Current reported angle : {current_position:+.6f} rad")
    print(f"Requested angle        : {desired_position:+.6f} rad")
    print(f"Position offset        : {current_offset:+.6f} -> {new_offset:+.6f} rad")
    print("Keep the shaft at its current physical position during calibration.")

    if not skip_confirmation:
        confirmation = input("Type ZERO to write and save this calibration: ")
        if confirmation != "ZERO":
            raise ProtocolError("calibration cancelled; no values were changed")

    # Refresh after operator interaction in case the shaft moved while reading.
    current_position = connection.read_float(PARAM_MECHANICAL_POSITION)
    current_offset = connection.read_float(PARAM_POSITION_OFFSET)
    new_offset = current_offset + current_position - desired_position

    connection.write_float(PARAM_POSITION_OFFSET, new_offset)
    saved = False
    try:
        written_offset = connection.read_float(PARAM_POSITION_OFFSET)
        if not math.isclose(written_offset, new_offset, rel_tol=0.0, abs_tol=1e-4):
            raise ProtocolError(
                "motor did not retain the requested position offset: "
                f"expected {new_offset:+.6f}, read {written_offset:+.6f} rad"
            )

        verified_position = connection.read_float(PARAM_MECHANICAL_POSITION)
        if not math.isclose(
            verified_position,
            desired_position,
            rel_tol=0.0,
            abs_tol=VERIFY_TOLERANCE_RADIANS,
        ):
            raise ProtocolError(
                "live angle verification failed: "
                f"expected {desired_position:+.6f}, "
                f"read {verified_position:+.6f} rad"
            )

        # A lost acknowledgement can hide a successful save.
        saved = True
        connection.save()
    except Exception:
        if not saved:
            try:
                connection.write_float(PARAM_POSITION_OFFSET, current_offset)
            except Exception as rollback_error:
                raise ProtocolError(
                    "calibration failed and the previous runtime offset could not be "
                    f"restored: {rollback_error}"
                ) from rollback_error
        raise

    final_position = connection.read_float(PARAM_MECHANICAL_POSITION)
    return current_position, new_offset, final_position


def argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Assign an angle to a RobStride motor's current physical position. "
            "Missing values are prompted interactively."
        )
    )
    parser.add_argument("--canbus", help="SocketCAN interface, for example can0")
    parser.add_argument(
        "--motor-id",
        type=parse_motor_id,
        help="motor CAN ID in decimal or 0xNN form",
    )
    parser.add_argument(
        "--angle",
        type=parse_angle,
        help="angle to assign; bare values and 'rad' are radians, 'deg' is accepted",
    )
    parser.add_argument(
        "--host-id",
        type=parse_host_id,
        default=DEFAULT_HOST_ID,
        help=f"controller host ID (default: {DEFAULT_HOST_ID})",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=DEFAULT_TIMEOUT_SECONDS,
        help=f"response timeout in seconds (default: {DEFAULT_TIMEOUT_SECONDS})",
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="skip the final ZERO confirmation",
    )
    return parser


def main() -> int:
    args = argument_parser().parse_args()
    try:
        interface, motor_id, angle = requested_values(args)
        with RobstrideConnection(
            interface,
            motor_id,
            host_id=args.host_id,
            timeout_seconds=args.timeout,
        ) as connection:
            previous, offset, final = calibrate(
                connection,
                angle,
                skip_confirmation=args.yes,
            )
        print(
            f"Saved motor {motor_id} calibration on {interface}: "
            f"{previous:+.6f} -> {final:+.6f} rad "
            f"(add_offset={offset:+.6f} rad)."
        )
        return 0
    except (EOFError, KeyboardInterrupt):
        print("\nCancelled.", file=sys.stderr)
        return 130
    except (OSError, ProtocolError, RuntimeError, ValueError) as error:
        print(f"Error: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
