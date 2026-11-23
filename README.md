# Bello stack

Bello stack is software for controlling and simulating robots. Each robot has a
URDF that defines its geometry, joints, and transmissions, and a hardware
description that assigns motors to buses and control groups. A deployment
profile selects the groups to run and the interfaces to use. See the
[included robot descriptions](descriptions) for examples.

On a physical robot, `bello-hardwared` talks to the motors over CAN and reads sensors over serial. A [Rust](components/bello-hardware-client) or [Python](components/bello-hardware-python) application can read telemetry and take control of a motor group over gRPC. The daemon enforces joint limits and handles faults.

The [MuJoCo simulator](components/bello-mujoco) emulates motors and sensors so the same daemon and application can run against simulated physics. By default, the simulator exchanges motor frames with the daemon over [loopback UDP](docs/operations.md#simulate-can-without-socketcan).

The [pose estimator](components/bello-state-estimatord) combines encoder and IMU readings to estimate the robot’s motion. MuJoCo can also publish ground-truth pose and rendered images through the [pose](components/bello-pose-api/proto/bello/pose/v1/pose.proto) and [camera](components/bello-camera-api/proto/bello/camera/v1/camera.proto) APIs.

## Documentation

- [Run on hardware or in simulation](docs/operations.md)
- [Configure a robot](docs/configuration.md)
- [Hardware API](docs/api.md) and [control and safety](docs/control-and-safety.md)
- [Desktop builds](docs/desktop.md) and [development setup](docs/development.md)
