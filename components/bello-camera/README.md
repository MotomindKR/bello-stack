# Bello camera transport

`bello-camera` is a ROS-free GStreamer camera transport. The robot-side CLI
captures cameras and publishes low-latency H.264 as RTP/UDP. The receiver
decodes the stream directly into a latest-frame RGB NumPy array for policy
input. It runs separately from `bello-hardwared` so media work cannot block
actuator leases, commands, or safety clocks.

The repository's Nix shell supplies PyGObject and GStreamer base, good, bad,
ugly, and libav plug-ins. On Debian/Ubuntu install the equivalent system
packages, including `python3-gi` and the GStreamer introspection package.

```sh
nix develop
python -m pip install -e components/bello-camera
```

Copy `config/milly.example.yaml`, set the physical wrist camera and remote GPU
address, then validate and stream:

```sh
bello-camera-stream --config camera.yaml --check
bello-camera-stream --config camera.yaml
```

On the GPU host:

```sh
bello-camera-receive --config camera.yaml --camera wrist
```

Use `config/test.yaml` for a camera-free SMPTE test stream. Policy consumers
must call `CameraReceiver.latest(max_age_seconds=...)` and fail closed on stale
video. RTP/UDP should run only on an isolated robot LAN or private VPN because
it has no authentication, encryption, or delivery guarantee.

A logical camera entry may replace `source` with an ordered `views` list containing one to
three named sources. The sender composes those per-view frames horizontally, encodes them
once, and the receiver reports a frame whose width is `video.width * len(views)`. View names
must be unique. `maximum_view_skew_ms` and `maximum_view_age_ms` default to 100 and 150 ms;
policy integrations should use the per-view capture identities to suppress metadata for an
incomplete, skewed, or stale composition.
