from __future__ import annotations

import argparse
import signal
import threading
import time
from collections.abc import Sequence

from .config import load_config
from .pipeline import receiver_pipeline, sender_pipeline
from .stream import CameraReceiver, CameraStreamer, validate_receiver, validate_sender


def stream_main(argv: Sequence[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Publish RTP/H.264 camera streams")
    parser.add_argument("--config", required=True)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--check", action="store_true")
    mode.add_argument("--print-sender", metavar="CAMERA")
    mode.add_argument("--print-receiver", metavar="CAMERA")
    args = parser.parse_args(argv)
    config = load_config(args.config)
    if args.print_sender:
        print(sender_pipeline(config.camera(args.print_sender)))
        return
    if args.print_receiver:
        print(receiver_pipeline(config.camera(args.print_receiver)))
        return
    if args.check:
        for camera in config.cameras:
            validate_sender(camera)
            validate_receiver(camera)
        print(f"{args.config}: configuration and pipelines are valid")
        return
    stopped = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stopped.set())
    signal.signal(signal.SIGTERM, lambda *_: stopped.set())
    with CameraStreamer(config) as streamer:
        while not stopped.is_set():
            streamer.poll()


def receive_main(argv: Sequence[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Validate an RTP/H.264 receiver")
    parser.add_argument("--config", required=True)
    parser.add_argument("--camera", required=True)
    parser.add_argument("--duration", type=float, default=0.0)
    args = parser.parse_args(argv)
    camera = load_config(args.config).camera(args.camera)
    deadline = None if args.duration == 0 else time.monotonic() + args.duration
    with CameraReceiver(camera) as receiver:
        frame = receiver.wait_for_frame(5.0)
        print(f"receiving {camera.name}: shape={frame.image.shape}")
        while deadline is None or time.monotonic() < deadline:
            time.sleep(1.0)
            frame = receiver.latest(max_age_seconds=1.0)
            print(f"sequence={frame.sequence} age_ms={frame.age_seconds() * 1000:.1f}")
