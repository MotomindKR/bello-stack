from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Any

import numpy as np
from numpy.typing import NDArray

from .config import CameraConfig, StreamConfig
from .pipeline import receiver_pipeline, sender_pipeline
from .realsense import RealSenseAppSource, RealSenseError, create_realsense_sources


class CameraError(RuntimeError):
    pass


class CameraStaleError(CameraError):
    pass


@dataclass(frozen=True, slots=True)
class CameraFrame:
    image: NDArray[np.uint8]
    sequence: int
    received_monotonic_ns: int

    def age_seconds(self) -> float:
        return max(0, time.monotonic_ns() - self.received_monotonic_ns) / 1_000_000_000


def _load_gst() -> Any:
    try:
        import gi

        gi.require_version("Gst", "1.0")
        from gi.repository import Gst
    except (ImportError, ValueError) as error:
        raise CameraError(
            "GStreamer Python bindings are unavailable; install PyGObject and "
            "GStreamer base/good/bad/ugly/libav plug-ins"
        ) from error
    Gst.init(None)
    return Gst


def _parse(description: str) -> tuple[Any, Any]:
    Gst = _load_gst()
    try:
        return Gst, Gst.parse_launch(description)
    except Exception as error:
        raise CameraError(f"failed to parse GStreamer pipeline: {error}") from error


def validate_sender(camera: CameraConfig) -> None:
    Gst, pipeline = _parse(sender_pipeline(camera))
    pipeline.set_state(Gst.State.NULL)


def validate_receiver(camera: CameraConfig) -> None:
    Gst, pipeline = _parse(receiver_pipeline(camera))
    pipeline.set_state(Gst.State.NULL)


class CameraStreamer:
    def __init__(self, config: StreamConfig):
        self._config = config
        self._gst: Any | None = None
        self._pipelines: list[tuple[str, Any, list[RealSenseAppSource]]] = []
        self._error: BaseException | None = None
        self._lock = threading.Lock()
        self._closed = False

    def start(self) -> None:
        if self._pipelines or self._closed:
            raise CameraError("camera streamer cannot be started in its current state")
        for camera in self._config.cameras:
            Gst, pipeline = _parse(sender_pipeline(camera))
            self._gst = Gst
            try:
                sources = create_realsense_sources(camera, Gst, pipeline, self._set_error)
            except RealSenseError as error:
                self.close()
                raise CameraError(str(error)) from error
            if pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
                self.close()
                raise CameraError(f"failed to start camera {camera.name!r}")
            self._pipelines.append((camera.name, pipeline, sources))
            try:
                for source in sources:
                    source.start()
            except RealSenseError as error:
                self.close()
                raise CameraError(str(error)) from error

    def poll(self, timeout_seconds: float = 0.1) -> None:
        if timeout_seconds < 0:
            raise ValueError("timeout_seconds must be non-negative")
        if not self._pipelines or self._gst is None:
            raise CameraError("camera streamer is not started")
        Gst = self._gst
        timeout_ns = int(timeout_seconds * 1_000_000_000) // len(self._pipelines)
        with self._lock:
            error = self._error
        if error is not None:
            raise CameraError(f"camera streamer failed: {error}") from error
        for name, pipeline, _ in self._pipelines:
            message = pipeline.get_bus().timed_pop_filtered(
                timeout_ns, Gst.MessageType.ERROR | Gst.MessageType.EOS
            )
            if message is None:
                continue
            if message.type == Gst.MessageType.ERROR:
                error, debug = message.parse_error()
                raise CameraError(f"camera {name!r} failed: {error} ({debug})")
            raise CameraError(f"camera {name!r} reached end-of-stream")

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        if self._gst is not None:
            for _, _, sources in reversed(self._pipelines):
                for source in reversed(sources):
                    source.close()
            for _, pipeline, _ in reversed(self._pipelines):
                pipeline.set_state(self._gst.State.NULL)
        self._pipelines.clear()

    def _set_error(self, error: BaseException) -> None:
        with self._lock:
            if self._error is None:
                self._error = error

    def __enter__(self) -> CameraStreamer:
        self.start()
        return self

    def __exit__(self, exc_type: object, exc: object, traceback: object) -> None:
        self.close()


class CameraReceiver:
    def __init__(self, camera: CameraConfig):
        self.camera = camera
        self._lock = threading.Lock()
        self._frame: CameraFrame | None = None
        self._error: CameraError | None = None
        self._sequence = 0
        self._gst: Any | None = None
        self._pipeline: Any | None = None
        self._watcher: threading.Thread | None = None
        self._stopping = threading.Event()

    def start(self) -> None:
        if self._pipeline is not None:
            raise CameraError("camera receiver is already started")
        Gst, pipeline = _parse(receiver_pipeline(self.camera))
        appsink = pipeline.get_by_name("frames")
        if appsink is None:
            raise CameraError("receiver pipeline has no 'frames' appsink")
        self._gst = Gst
        self._pipeline = pipeline
        appsink.connect("new-sample", self._on_sample)
        if pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
            self.close()
            raise CameraError(f"failed to start camera receiver {self.camera.name!r}")
        self._watcher = threading.Thread(target=self._watch_bus, daemon=True)
        self._watcher.start()

    def latest(self, *, max_age_seconds: float | None = None) -> CameraFrame:
        if max_age_seconds is not None and max_age_seconds <= 0:
            raise ValueError("max_age_seconds must be positive")
        with self._lock:
            error, frame = self._error, self._frame
        if error is not None:
            raise error
        if frame is None:
            raise CameraStaleError(f"camera {self.camera.name!r} has no frame")
        if max_age_seconds is not None and frame.age_seconds() > max_age_seconds:
            raise CameraStaleError(
                f"camera {self.camera.name!r} frame is {frame.age_seconds():.3f}s old"
            )
        return frame

    def wait_for_frame(self, timeout_seconds: float) -> CameraFrame:
        if timeout_seconds <= 0:
            raise ValueError("timeout_seconds must be positive")
        deadline = time.monotonic() + timeout_seconds
        while time.monotonic() < deadline:
            try:
                return self.latest()
            except CameraStaleError:
                time.sleep(0.01)
        raise CameraStaleError(f"camera {self.camera.name!r} timed out")

    def close(self) -> None:
        self._stopping.set()
        pipeline, self._pipeline = self._pipeline, None
        if self._gst is not None and pipeline is not None:
            pipeline.set_state(self._gst.State.NULL)
        if self._watcher is not None and self._watcher is not threading.current_thread():
            self._watcher.join(timeout=1.0)
        self._watcher = None

    def __enter__(self) -> CameraReceiver:
        self.start()
        return self

    def __exit__(self, exc_type: object, exc: object, traceback: object) -> None:
        self.close()

    def _on_sample(self, appsink: Any) -> Any:
        assert self._gst is not None
        Gst = self._gst
        try:
            sample = appsink.emit("pull-sample")
            structure = sample.get_caps().get_structure(0)
            width, height = int(structure.get_value("width")), int(structure.get_value("height"))
            if structure.get_value("format") != "RGB":
                raise CameraError("appsink did not produce RGB")
            buffer = sample.get_buffer()
            ok, info = buffer.map(Gst.MapFlags.READ)
            if not ok:
                raise CameraError("failed to map frame")
            try:
                raw = np.frombuffer(info.data, dtype=np.uint8)
                if raw.size != width * height * 3:
                    raise CameraError("decoded frame has unexpected stride or size")
                image = raw.reshape((height, width, 3)).copy()
            finally:
                buffer.unmap(info)
            with self._lock:
                self._sequence += 1
                self._frame = CameraFrame(image, self._sequence, time.monotonic_ns())
            return Gst.FlowReturn.OK
        except Exception as error:
            self._set_error(CameraError(f"failed to decode frame: {error}"))
            return Gst.FlowReturn.ERROR

    def _watch_bus(self) -> None:
        assert self._gst is not None and self._pipeline is not None
        Gst, bus = self._gst, self._pipeline.get_bus()
        while not self._stopping.is_set():
            message = bus.timed_pop_filtered(
                100 * Gst.MSECOND, Gst.MessageType.ERROR | Gst.MessageType.EOS
            )
            if message is None:
                continue
            if message.type == Gst.MessageType.ERROR:
                error, debug = message.parse_error()
                self._set_error(CameraError(f"receiver failed: {error} ({debug})"))
            else:
                self._set_error(CameraError("receiver reached end-of-stream"))
            return

    def _set_error(self, error: CameraError) -> None:
        with self._lock:
            if self._error is None:
                self._error = error
