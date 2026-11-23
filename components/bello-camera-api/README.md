# bello-camera-api

[camera.proto](proto/bello/camera/v1/camera.proto) defines the gRPC services for JPEG frames and WebRTC signaling.
[bello-mujoco](../bello-mujoco/) serves simulated frames.
[bello-camerad](https://github.com/MotomindKR/bello_webxr_teleop/tree/main/crates/cameras) captures physical cameras and serves WebRTC video from either source.
