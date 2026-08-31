from __future__ import annotations

from .config import CameraConfig, SourceConfig
from .realsense import realsense_source_launch


def sender_pipeline(camera: CameraConfig) -> str:
    video = camera.video
    if len(camera.views) == 1:
        source_launch = _source_launch(
            camera.views[0].source,
            camera,
            0,
            preserve_capture_rate=False,
        )
    else:
        compositor_fps = max(video.fps, *(view.source.capture_fps for view in camera.views))
        latency_ns = camera.compositor_latency_ms * 1_000_000
        pads = " ".join(
            f"sink_{index}::xpos={index * video.width} sink_{index}::ypos=0 "
            f"sink_{index}::width={video.width} sink_{index}::height={video.height}"
            for index in range(len(camera.views))
        )
        branches = " ".join(
            f"{_source_launch(view.source, camera, index, preserve_capture_rate=True)} ! "
            "queue leaky=downstream max-size-buffers=1 max-size-bytes=0 max-size-time=0 ! "
            "videoconvert ! "
            f"video/x-raw,width={video.width},height={video.height} ! "
            f"compose.sink_{index}"
            for index, view in enumerate(camera.views)
        )
        source_launch = (
            f"compositor name=compose background=black force-live=true latency={latency_ns} "
            f"{pads} "
            f"{branches} compose. ! video/x-raw,width={camera.composite_width},"
            f"height={video.height},framerate={compositor_fps}/1 ! "
            "videorate drop-only=true ! "
            f"video/x-raw,width={camera.composite_width},height={video.height},"
            f"framerate={video.fps}/1"
        )

    encoder = camera.encoder
    if encoder.type == "x264":
        encoder_launch = (
            f"x264enc tune=zerolatency speed-preset={_quote(encoder.speed_preset)} "
            f"bitrate={encoder.bitrate_kbps} key-int-max={encoder.keyframe_interval or video.fps} "
            "bframes=0"
        )
    else:
        assert encoder.launch is not None
        encoder_launch = encoder.launch

    return " ! ".join(
        [
            source_launch,
            "queue leaky=downstream max-size-buffers=1 max-size-bytes=0 max-size-time=0",
            "videoconvert",
            "video/x-raw,format=I420",
            encoder_launch,
            "h264parse config-interval=-1",
            "video/x-h264,stream-format=byte-stream,alignment=au",
            f"rtph264pay pt={camera.rtp.payload_type} config-interval=-1 mtu={camera.rtp.mtu}",
            f"udpsink host={_quote(camera.rtp.host)} port={camera.rtp.port} sync=false async=false",
        ]
    )


def _source_launch(
    source: SourceConfig,
    camera: CameraConfig,
    view_index: int,
    *,
    preserve_capture_rate: bool,
) -> str:
    video = camera.video
    if source.type == "v4l2":
        assert source.device is not None
        output_fps = source.capture_fps if preserve_capture_rate else video.fps
        caps = (
            f"image/jpeg,width={video.width},height={video.height},"
            f"framerate={output_fps}/1 ! jpegdec"
            if source.format == "mjpeg"
            else f"video/x-raw,width={video.width},height={video.height},framerate={output_fps}/1"
        )
        source_launch = f"v4l2src device={_quote(source.device)} do-timestamp=true ! {caps}"
    elif source.type == "test":
        output_fps = source.capture_fps if preserve_capture_rate else video.fps
        source_launch = (
            f"videotestsrc is-live=true pattern={_quote(source.pattern)} ! "
            f"video/x-raw,width={video.width},height={video.height},framerate={output_fps}/1"
        )
    elif source.type == "custom":
        assert source.launch is not None
        source_launch = source.launch
    else:
        output_fps = source.capture_fps if preserve_capture_rate else video.fps
        rate_launch = "" if preserve_capture_rate else "videorate drop-only=true ! "
        source_launch = (
            f"{realsense_source_launch(source, view_index)} ! "
            f"{rate_launch}videoscale method=bilinear add-borders=true ! "
            f"video/x-raw,width={video.width},height={video.height},"
            f"pixel-aspect-ratio=1/1,framerate={output_fps}/1"
        )
    return source_launch


def receiver_pipeline(camera: CameraConfig) -> str:
    caps = (
        "application/x-rtp,media=video,encoding-name=H264,clock-rate=90000,"
        f"payload={camera.rtp.payload_type}"
    )
    return " ! ".join(
        [
            f"udpsrc address={_quote(camera.rtp.bind_address)} "
            f"port={camera.rtp.port} caps={_quote(caps)}",
            f"rtpjitterbuffer latency={camera.rtp.receiver_latency_ms} drop-on-latency=true",
            "rtph264depay",
            "h264parse",
            "avdec_h264",
            "videoconvert",
            f"video/x-raw,format=RGB,width={camera.composite_width},height={camera.video.height}",
            "appsink name=frames emit-signals=true max-buffers=1 drop=true sync=false",
        ]
    )


def _quote(value: str) -> str:
    return '"' + value.replace("\\", "\\\\").replace('"', '\\"') + '"'
