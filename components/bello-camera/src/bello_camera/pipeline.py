from __future__ import annotations

from .config import CameraConfig


def sender_pipeline(camera: CameraConfig) -> str:
    video = camera.video
    source = camera.source
    if source.type == "v4l2":
        assert source.device is not None
        caps = (
            f"image/jpeg,width={video.width},height={video.height},"
            f"framerate={video.fps}/1 ! jpegdec"
            if source.format == "mjpeg"
            else f"video/x-raw,width={video.width},height={video.height},framerate={video.fps}/1"
        )
        source_launch = f"v4l2src device={_quote(source.device)} do-timestamp=true ! {caps}"
    elif source.type == "test":
        source_launch = (
            f"videotestsrc is-live=true pattern={_quote(source.pattern)} ! "
            f"video/x-raw,width={video.width},height={video.height},framerate={video.fps}/1"
        )
    else:
        assert source.launch is not None
        source_launch = source.launch

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
            f"video/x-raw,format=RGB,width={camera.video.width},height={camera.video.height}",
            "appsink name=frames emit-signals=true max-buffers=1 drop=true sync=false",
        ]
    )


def _quote(value: str) -> str:
    return '"' + value.replace("\\", "\\\\").replace('"', '\\"') + '"'
