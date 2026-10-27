# bello-camera-api

Shared, read-only camera frames and WebRTC signaling contracts. No motor API or
GStreamer dependency is included.

| Service | RPCs |
| --- | --- |
| `bello.camera.v1.CameraSourceService` | `GetCameras`, `WatchFrames` |
| `bello.camera.v1.WebRtcService` | `CreateSession`, `CloseSession` |

Three named 640×360/30 FPS views share the same interface: `birds_eye`,
`right_gripper`, `left_gripper`. Each JPEG retains its source sequence, capture
and receipt time in host `CLOCK_MONOTONIC`, original V4L2 flags and optional
simulation time. Source descriptors carry physical mapping or simulated
intrinsics/mount metadata.

`Frames` provides a bounded producer implementation. Preview subscribers receive
the latest frame; ordered recording subscribers fail explicitly when they lag.
The shared UDS helper preserves live owners, recovers abandoned sockets and uses
0600 permissions. The crate exports generated tonic clients/servers and its
protobuf descriptor set.

The MuJoCo renderer implements the camera source service. The Rust GStreamer
media daemon in `bello_webxr_teleop` implements both services and owns physical
capture or subscribes to the HIL source. Dataset recording consumes original
frames independently of WebRTC transport or bitrate.
