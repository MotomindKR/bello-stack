"""Command-line entry point for Bello upper-body teleoperation."""

from __future__ import annotations

import argparse
import json
import select
import sys
import termios
import time
from collections.abc import Sequence
from contextlib import suppress

from bello_hardware import BelloError, Robot

from .control import (
    OperatorConfirmationError,
    TeleopSettings,
    run_teleoperation,
)
from .dynamixel import (
    DynamixelError,
    DynamixelTeachingArms,
    MasterTorqueMode,
)


class _ConsoleConfirmationGate:
    """Require tokens typed after each prompt without blocking the control loop."""

    def __init__(self) -> None:
        self._expected_token: str | None = None

    def request(self, *, token: str, message: str) -> None:
        if self._expected_token is not None:
            raise RuntimeError("a confirmation request is already pending")
        if not sys.stdin.isatty():
            raise OperatorConfirmationError(
                "--enable requires an interactive terminal for HOLD and TELEOP confirmations"
            )
        with suppress(OSError, ValueError):
            termios.tcflush(sys.stdin.fileno(), termios.TCIFLUSH)
        self._expected_token = token
        print(message, file=sys.stderr)
        print(
            f"Type {token} and press Enter to continue; anything else cancels:",
            file=sys.stderr,
            flush=True,
        )

    def poll(self) -> bool | None:
        expected_token = self._expected_token
        if expected_token is None:
            raise RuntimeError("no confirmation request is pending")
        readable, _, _ = select.select((sys.stdin,), (), (), 0.0)
        if not readable:
            return None
        response = sys.stdin.readline()
        self._expected_token = None
        return response.strip() == expected_token


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Read Bello's Dynamixel teaching arms and control the daemon's upper_body group"
        )
    )
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument(
        "--dry-run",
        action="store_true",
        help="read and print teaching-arm positions without connecting to the daemon",
    )
    mode.add_argument(
        "--enable",
        action="store_true",
        help="prepare physical control; HOLD and TELEOP confirmations are still required",
    )
    parser.add_argument("--endpoint", default="http://127.0.0.1:50051")
    parser.add_argument("--group", default="upper_body")
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=1_000_000)
    parser.add_argument("--rate", type=float, default=30.0)
    parser.add_argument(
        "--max-velocity",
        type=float,
        default=0.5,
        help="maximum commanded position slew in radians per second (default: 0.5)",
    )
    parser.add_argument(
        "--gain-ramp",
        type=float,
        default=5.0,
        help="seconds to ramp robot kp from zero to configured gains (default: 5)",
    )
    parser.add_argument(
        "--transition-duration",
        type=float,
        default=5.0,
        help="minimum seconds to blend from held robot pose to teaching-arm pose (default: 5)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        help="stop after this many seconds; by default run until interrupted",
    )
    parser.add_argument(
        "--hold-wrist-yaw",
        action="store_true",
        help=("capture and hold left_arm_7/right_arm_7 through the daemon's wrist_yaw group"),
    )
    master_torque = parser.add_mutually_exclusive_group()
    master_torque.add_argument(
        "--enable-master-torque",
        "--master-torque",
        action="store_const",
        const="enable",
        dest="master_torque_mode",
        help=(
            "enable weak configured-pose teaching-arm centering; this is the default with --enable"
        ),
    )
    master_torque.add_argument(
        "--disable-master-torque",
        action="store_const",
        const="disable",
        dest="master_torque_mode",
        help=("force all teaching-arm Dynamixels torque-off; this is the default with --dry-run"),
    )
    master_torque.add_argument(
        "--leave-master-torque",
        action="store_const",
        const="unchanged",
        dest="master_torque_mode",
        help="leave all teaching-arm torque registers unchanged and unmanaged",
    )
    parser.set_defaults(master_torque_mode=None)
    return parser


def _dry_run(source: DynamixelTeachingArms, rate_hz: float, duration: float | None) -> None:
    if rate_hz <= 0.0:
        raise ValueError("rate must be positive")
    if duration is not None and duration <= 0.0:
        raise ValueError("duration must be positive")
    period = 1.0 / rate_hz
    started_at = time.monotonic()
    next_tick = started_at
    while duration is None or time.monotonic() - started_at < duration:
        sample = source.read()
        print(
            json.dumps(
                {
                    "positions": sample.positions,
                    "clipped_coordinates": sample.clipped_coordinates,
                },
                sort_keys=True,
            ),
            flush=True,
        )
        next_tick += period
        delay = next_tick - time.monotonic()
        if delay > 0.0:
            time.sleep(delay)
        else:
            next_tick = time.monotonic()


def _master_torque_mode(args: argparse.Namespace) -> MasterTorqueMode:
    requested = args.master_torque_mode
    if requested is None:
        return "enable" if args.enable else "disable"
    if requested == "disable":
        return "disable"
    if requested == "enable":
        return "enable"
    if requested == "unchanged":
        return "unchanged"
    raise ValueError(f"invalid master torque mode {requested!r}")


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.dry_run and args.hold_wrist_yaw:
            raise ValueError("--hold-wrist-yaw requires --enable")
        torque_mode = _master_torque_mode(args)
        if args.dry_run:
            with DynamixelTeachingArms(
                port=args.port,
                baudrate=args.baudrate,
                torque_mode=torque_mode,
            ) as source:
                _dry_run(source, args.rate, args.duration)
        else:
            if torque_mode == "enable":
                print(
                    "Preparing teaching-arm torque with current-position capture, "
                    "an 800-unit current cap, weak configured-pose centering, "
                    "and configured-position locks for waist/wrist auxiliaries.",
                    file=sys.stderr,
                    flush=True,
                )
            with (
                Robot.connect(args.endpoint) as robot,
                DynamixelTeachingArms(
                    port=args.port,
                    baudrate=args.baudrate,
                    torque_mode=torque_mode,
                ) as source,
            ):
                if source.master_torque_enabled:
                    print(
                        f"Teaching-rig torque enabled and verified on all "
                        f"{len(source.ids)} Dynamixels. Holding "
                        + ", ".join(source.held_names)
                        + " at their configured positions; all torque will be "
                        "disabled automatically on exit.",
                        file=sys.stderr,
                        flush=True,
                    )
                settings = TeleopSettings(
                    group=args.group,
                    wrist_yaw_group=("wrist_yaw" if args.hold_wrist_yaw else None),
                    rate_hz=args.rate,
                    maximum_velocity_radians_per_second=args.max_velocity,
                    gain_ramp_seconds=args.gain_ramp,
                    transition_duration_seconds=args.transition_duration,
                    duration_seconds=args.duration,
                )
                run_teleoperation(
                    robot,
                    source,
                    settings,
                    confirmation=_ConsoleConfirmationGate(),
                    status=print,
                )
    except KeyboardInterrupt:
        print("teleoperation stopped", file=sys.stderr)
        return 0
    except OperatorConfirmationError as error:
        print(f"teleoperation cancelled: {error}", file=sys.stderr)
        return 1
    except (BelloError, DynamixelError, TimeoutError, TypeError, ValueError) as error:
        print(f"teleoperation failed: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
