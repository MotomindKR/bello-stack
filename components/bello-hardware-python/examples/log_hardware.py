#!/usr/bin/env python3
"""Record complete Bello hardware state frames as newline-delimited JSON."""

from __future__ import annotations

import argparse
import json
import sys
import time
from collections.abc import Callable
from contextlib import nullcontext
from datetime import datetime, timezone
from pathlib import Path
from typing import IO, Any

from bello_hardware import (
    DeviceState,
    DeviceStatus,
    GroupCause,
    GroupState,
    HardwareStateFrame,
    Robot,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--endpoint",
        default="http://127.0.0.1:50051",
        help="bello-hardwared endpoint (default: %(default)s)",
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="JSONL destination (default: hardware-<UTC timestamp>.jsonl; use - for stdout)",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=50.0,
        help="maximum snapshot rate in Hz (default: %(default)s)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        help="stop after this many seconds (default: run until Ctrl-C)",
    )
    parser.add_argument(
        "--connect-timeout",
        type=float,
        default=5.0,
        help="initial connection timeout in seconds (default: %(default)s)",
    )
    args = parser.parse_args()
    if args.rate <= 0.0:
        parser.error("--rate must be positive")
    if args.duration is not None and args.duration <= 0.0:
        parser.error("--duration must be positive")
    if args.connect_timeout <= 0.0:
        parser.error("--connect-timeout must be positive")
    return args


def default_output_path() -> Path:
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    return Path(f"hardware-{stamp}.jsonl")


def enum_name(name: Callable[[int], str], value: int, prefix: str) -> str:
    try:
        return name(value).removeprefix(prefix).lower()
    except ValueError:
        return str(value)


def device_status(status: DeviceStatus | None) -> dict[str, Any] | None:
    if status is None:
        return None
    return {
        "state": enum_name(DeviceState.Name, status.state, "DEVICE_STATE_"),
        "detail": status.detail,
        "transition_count": status.transition_count,
    }


def inventory_record(robot: Robot, endpoint: str) -> dict[str, Any]:
    inventory = robot.inventory
    return {
        "record": "inventory",
        "schema_version": 1,
        "logged_at": datetime.now(timezone.utc).isoformat(),
        "endpoint": endpoint,
        "buses": [
            {
                "name": bus.name,
                "transport": bus.transport,
                "host_id": bus.host_id,
                "status": device_status(bus.status),
            }
            for bus in inventory.buses
        ],
        "serial_devices": [
            {
                "name": serial.name,
                "port": serial.port,
                "baud_rate": serial.baud_rate,
                "status": device_status(serial.status),
            }
            for serial in inventory.serial_devices
        ],
        "imus": [
            {
                "name": imu.name,
                "serial_device": imu.serial_device,
                "status": device_status(imu.status),
            }
            for imu in inventory.imus
        ],
        "groups": [
            {
                "name": name,
                "control_rate_hz": robot.group(name).control_rate_hz,
                "initial_state": enum_name(
                    GroupState.Name, robot.group(name).initial_state, "GROUP_STATE_"
                ),
                "motors": [motor.name for motor in robot.group(name).motors],
            }
            for name in robot.group_names
        ],
        "motors": [
            {
                "name": motor.name,
                "coordinate": motor.coordinate,
                "group": motor.group,
                "bus": motor.bus,
                "id": motor.id,
                "model": motor.model,
                "direction": motor.direction,
                "status": device_status(motor.status),
            }
            for motor in robot.motors
        ],
    }


def snapshot_record(frame: HardwareStateFrame) -> dict[str, Any]:
    return {
        "record": "state_frame",
        "logged_at": datetime.now(timezone.utc).isoformat(),
        "logged_unix_ns": time.time_ns(),
        "captured_monotonic_ns": frame.captured_monotonic_ns,
        "received_monotonic_ns": frame.received_monotonic_ns,
        "round_trip_ns": frame.round_trip_ns,
        "oldest_sample_age_ns": frame.oldest_sample_age_ns,
        "sample_span_ns": frame.sample_span_ns,
        "motors": [
            {
                "group": motor.group,
                "motor": motor.motor,
                "coordinate": motor.coordinate,
                "position": motor.position,
                "velocity": motor.velocity,
                "torque": motor.torque,
                "temperature": motor.temperature,
                "fault_bits": motor.fault_bits,
                "mode_status": motor.mode_status,
                "feedback_age_ms": motor.feedback_age_ms,
                "sample_monotonic_ns": motor.sample_monotonic_ns,
                "sample_age_ns": motor.sample_age_ns,
            }
            for motor in frame.motors
        ],
        "groups": [
            {
                "group": group.group,
                "state": enum_name(GroupState.Name, group.state, "GROUP_STATE_"),
                "detail": group.detail,
                "cause": enum_name(GroupCause.Name, group.cause, "GROUP_CAUSE_"),
                "command_count": group.command_count,
                "control_tick": group.control_tick,
                "jitter_us": group.jitter_us,
                "sample_monotonic_ns": group.sample_monotonic_ns,
                "sample_age_ns": group.sample_age_ns,
            }
            for group in frame.groups
        ],
        "imus": [
            {
                "imu": imu.imu,
                "orientation_xyzw": {
                    "x": imu.orientation.x,
                    "y": imu.orientation.y,
                    "z": imu.orientation.z,
                    "w": imu.orientation.w,
                },
                "angular_velocity": {
                    "x": imu.angular_velocity.x,
                    "y": imu.angular_velocity.y,
                    "z": imu.angular_velocity.z,
                },
                "linear_acceleration": {
                    "x": imu.linear_acceleration.x,
                    "y": imu.linear_acceleration.y,
                    "z": imu.linear_acceleration.z,
                },
                "device_timestamp_ms": imu.device_timestamp_ms,
                "sample_age_ms": imu.sample_age_ms,
                "sample_monotonic_ns": imu.sample_monotonic_ns,
                "sample_age_ns": imu.sample_age_ns,
            }
            for imu in frame.imus
        ],
    }


def write_record(output: IO[str], record: dict[str, Any]) -> None:
    json.dump(record, output, separators=(",", ":"), allow_nan=False)
    output.write("\n")
    output.flush()


def run(args: argparse.Namespace) -> int:
    output_path = args.output or default_output_path()
    output_context = (
        nullcontext(sys.stdout)
        if str(output_path) == "-"
        else output_path.open("x", encoding="utf-8", buffering=1)
    )
    if str(output_path) != "-":
        print(f"logging hardware telemetry to {output_path}", file=sys.stderr)

    with (
        output_context as output,
        Robot.connect(args.endpoint, timeout=args.connect_timeout) as robot,
    ):
        write_record(output, inventory_record(robot, args.endpoint))
        reader = robot.states()
        started = time.monotonic()
        sample_period = 1.0 / args.rate
        next_sample = started
        while args.duration is None or time.monotonic() - started < args.duration:
            now = time.monotonic()
            if now < next_sample:
                time.sleep(next_sample - now)
            remaining_duration = (
                None if args.duration is None else args.duration - (time.monotonic() - started)
            )
            timeout = 1.0 if remaining_duration is None else min(1.0, max(0.0, remaining_duration))
            if timeout <= 0.0:
                break
            write_record(output, snapshot_record(reader.read(timeout=timeout)))
            next_sample += sample_period
            if next_sample < time.monotonic():
                next_sample = time.monotonic()
    return 0


def main() -> int:
    args = parse_args()
    try:
        return run(args)
    except KeyboardInterrupt:
        print("stopped", file=sys.stderr)
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
