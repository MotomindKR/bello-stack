# Bello stack

This repository contains the robot models, hardware control software, and MuJoCo simulator for Bello and Milly. The [robot descriptions](descriptions) cover Bello, Bello Mini, Mini Torso, and the single- and dual-arm Milly setups. The URDFs define geometry, joint limits, and transmissions. The hardware configurations name the motors, assign their CAN addresses, and group them for control.

On a physical robot, `bello-hardwared` talks to the motors over CAN and reads sensors over serial. A [Rust](components/bello-hardware-client) or [Python](components/bello-hardware-python) application can read telemetry and take control of a motor group over gRPC. The daemon enforces joint limits and handles faults.

On Linux, the [MuJoCo simulator](components/bello-mujoco) emulates the devices on virtual CAN and serial ports, so the same daemon and application can run against simulated physics.

The [pose estimator](components/bello-state-estimatord) combines encoder and IMU readings to estimate the robot’s motion. MuJoCo can also publish ground-truth pose and rendered images through the [pose](components/bello-pose-api/proto/bello/pose/v1/pose.proto) and [camera](components/bello-camera-api/proto/bello/camera/v1/camera.proto) APIs.

## Documentation

- [Run on hardware or in simulation](docs/operations.md)
- [Configure a robot](docs/configuration.md)
- [Hardware API](docs/api.md) and [control and safety](docs/control-and-safety.md)
- [Desktop builds](docs/desktop.md) and [development setup](docs/development.md)
