from __future__ import annotations

import logging
import threading
from collections.abc import Callable
from typing import Any

import numpy as np

from .config import CameraConfig, SourceConfig

LOGGER = logging.getLogger(__name__)
FRAME_TIMEOUT_MS = 2_000


class RealSenseError(RuntimeError):
    pass


def realsense_appsrc_name(view_index: int) -> str:
    return f"realsense_source_{view_index}"


def realsense_source_launch(source: SourceConfig, view_index: int) -> str:
    return (
        f"appsrc name={realsense_appsrc_name(view_index)} is-live=true format=time "
        "block=false max-buffers=1 leaky-type=downstream ! "
        f"video/x-raw,format=RGB,width={source.capture_width},"
        f"height={source.capture_height},framerate={source.capture_fps}/1,"
        "pixel-aspect-ratio=1/1"
    )


def create_realsense_sources(
    camera: CameraConfig,
    Gst: Any,
    pipeline: Any,
    error_callback: Callable[[BaseException], None],
) -> list[RealSenseAppSource]:
    sources: list[RealSenseAppSource] = []
    for index, view in enumerate(camera.views):
        if view.source.type != "realsense":
            continue
        appsrc = pipeline.get_by_name(realsense_appsrc_name(index))
        if appsrc is None:
            raise RealSenseError(
                f"camera {camera.name!r} is missing RealSense appsrc for view {view.name!r}"
            )
        sources.append(
            RealSenseAppSource(
                view.source,
                Gst=Gst,
                appsrc=appsrc,
                gst_pipeline=pipeline,
                label=f"{camera.name}.{view.name}",
                error_callback=error_callback,
            )
        )
    return sources


class RealSenseAppSource:
    """Capture librealsense RGB frames and timestamp them onto a GStreamer appsrc."""

    def __init__(
        self,
        source: SourceConfig,
        *,
        Gst: Any,
        appsrc: Any,
        gst_pipeline: Any,
        label: str,
        error_callback: Callable[[BaseException], None],
    ):
        if source.type != "realsense":
            raise ValueError("RealSenseAppSource requires a realsense source")
        self._source = source
        self._gst = Gst
        self._appsrc = appsrc
        self._gst_pipeline = gst_pipeline
        self._label = label
        self._error_callback = error_callback
        self._rs: Any | None = None
        self._rs_pipeline: Any | None = None
        self._thread: threading.Thread | None = None
        self._stopping = threading.Event()

    def start(self) -> None:
        if self._thread is not None:
            raise RealSenseError(f"RealSense source {self._label!r} is already started")
        try:
            import pyrealsense2 as rs
        except ImportError as error:
            raise RealSenseError(
                "pyrealsense2 is unavailable; install the robot dependencies with "
                "`uv sync --extra robot`"
            ) from error
        self._rs = rs
        self._open_device()
        self._thread = threading.Thread(
            target=self._run,
            name=f"realsense-{self._label}",
            daemon=True,
        )
        self._thread.start()

    def close(self) -> None:
        self._stopping.set()
        if self._thread is not None and self._thread is not threading.current_thread():
            self._thread.join(timeout=FRAME_TIMEOUT_MS / 1_000 + 1.0)
        self._thread = None
        self._close_device()

    def _open_device(self) -> None:
        assert self._rs is not None
        rs_pipeline = self._rs.pipeline()
        config = self._rs.config()
        if self._source.serial is not None:
            config.enable_device(self._source.serial)
        config.enable_stream(
            self._rs.stream.color,
            self._source.capture_width,
            self._source.capture_height,
            self._rs.format.rgb8,
            self._source.capture_fps,
        )
        try:
            rs_pipeline.start(config)
        except Exception as error:
            serial = self._source.serial or "first available device"
            raise RealSenseError(
                f"failed to start RealSense {self._label!r} ({serial}) at "
                f"{self._source.capture_width}x{self._source.capture_height}@"
                f"{self._source.capture_fps}: {error}"
            ) from error
        self._rs_pipeline = rs_pipeline

    def _close_device(self) -> None:
        rs_pipeline, self._rs_pipeline = self._rs_pipeline, None
        if rs_pipeline is not None:
            try:
                rs_pipeline.stop()
            except Exception:
                LOGGER.debug("failed to stop RealSense %s", self._label, exc_info=True)

    def _run(self) -> None:
        restarts_remaining = 1
        while not self._stopping.is_set():
            try:
                self._pump_frames()
                return
            except BaseException as error:
                if self._stopping.is_set():
                    return
                self._close_device()
                if restarts_remaining == 0:
                    self._error_callback(
                        RealSenseError(f"RealSense source {self._label!r} failed: {error}")
                    )
                    return
                restarts_remaining -= 1
                LOGGER.warning("restarting RealSense source %s after: %s", self._label, error)
                try:
                    self._open_device()
                except BaseException as restart_error:
                    self._error_callback(restart_error)
                    return

    def _pump_frames(self) -> None:
        assert self._rs_pipeline is not None
        expected_shape = (
            self._source.capture_height,
            self._source.capture_width,
            3,
        )
        duration_ns = self._gst.SECOND // self._source.capture_fps
        while not self._stopping.is_set():
            frames = self._rs_pipeline.wait_for_frames(FRAME_TIMEOUT_MS)
            color = frames.get_color_frame()
            if not color:
                raise RealSenseError("frameset did not contain a color frame")
            image = np.asanyarray(color.get_data())
            if image.shape != expected_shape or image.dtype != np.uint8:
                raise RealSenseError(
                    f"unexpected RGB frame shape or type: {image.shape}, {image.dtype}"
                )
            payload = image.tobytes()
            buffer = self._gst.Buffer.new_allocate(None, len(payload), None)
            buffer.fill(0, payload)
            buffer.pts = self._running_time_ns()
            buffer.dts = self._gst.CLOCK_TIME_NONE
            buffer.duration = duration_ns
            result = self._appsrc.emit("push-buffer", buffer)
            if result != self._gst.FlowReturn.OK:
                if self._stopping.is_set():
                    return
                raise RealSenseError(f"GStreamer appsrc returned {result}")

    def _running_time_ns(self) -> int:
        clock = self._gst_pipeline.get_clock()
        if clock is None:
            raise RealSenseError("GStreamer pipeline has no clock")
        base_time = self._gst_pipeline.get_base_time()
        if base_time == self._gst.CLOCK_TIME_NONE:
            raise RealSenseError("GStreamer pipeline has no base time")
        return max(0, int(clock.get_time()) - int(base_time))
