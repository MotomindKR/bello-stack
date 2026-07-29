from __future__ import annotations

import queue
import threading
import time
from collections.abc import Iterator
from concurrent.futures import ThreadPoolExecutor
from contextlib import suppress
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import grpc  # type: ignore[import-untyped]
import pytest

from bello_hardware._generated import hardware_pb2 as proto
from bello_hardware._generated import hardware_pb2_grpc


def valid_inventory_response() -> proto.ListHardwareResponse:
    """Return one internally consistent daemon inventory for client tests."""

    return proto.ListHardwareResponse(
        buses=[proto.BusInfo(name="main", interface="vcan0", host_id=0xFD)],
        motors=[
            proto.MotorInfo(
                name="hip_motor",
                coordinate="hip",
                group="leg",
                address=proto.MotorAddress(bus="main", id=1),
                model="ROBSTRIDE_03",
                direction=1,
                limits=proto.MotorLimits(
                    position_min=-12.57,
                    position_max=12.57,
                    velocity_min=-20.0,
                    velocity_max=20.0,
                    torque_min=-60.0,
                    torque_max=60.0,
                    kp_min=0.0,
                    kp_max=5000.0,
                    kd_min=0.0,
                    kd_max=100.0,
                ),
            ),
            proto.MotorInfo(
                name="knee_motor",
                coordinate="knee",
                group="leg",
                address=proto.MotorAddress(bus="main", id=2),
                model="ROBSTRIDE_03",
                direction=1,
                limits=proto.MotorLimits(
                    position_min=-12.57,
                    position_max=12.57,
                    velocity_min=-20.0,
                    velocity_max=20.0,
                    torque_min=-60.0,
                    torque_max=60.0,
                    kp_min=0.0,
                    kp_max=5000.0,
                    kd_min=0.0,
                    kd_max=100.0,
                ),
            ),
        ],
        groups=[
            proto.GroupInfo(
                name="leg",
                control_rate_hz=500.0,
                state=proto.GROUP_STATE_IDLE,
                motors=[
                    proto.MotorAddress(bus="main", id=1),
                    proto.MotorAddress(bus="main", id=2),
                ],
            )
        ],
        serial_devices=[
            proto.SerialDeviceInfo(
                name="body_serial",
                port="/dev/ttyUSB0",
                baud_rate=921_600,
            )
        ],
        imus=[proto.ImuInfo(name="body", serial_device="body_serial")],
    )


@pytest.fixture
def inventory_response() -> proto.ListHardwareResponse:
    return valid_inventory_response()


class FakeHardware:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.state = proto.GROUP_STATE_IDLE
        self.token = "test-token"
        self.command_count = 0
        self.renew_count = 0
        self.release_count = 0
        self.disable_count = 0
        self.reject_next = False
        self.command_delay = 0.0
        self.renew_failure: grpc.StatusCode | None = None
        self.enable_failure: grpc.StatusCode | None = None
        self.start_failure: grpc.StatusCode | None = None
        self.disable_failure: grpc.StatusCode | None = None
        self.telemetry_failures = 0
        self.telemetry_calls = 0
        self.telemetry_events: queue.Queue[proto.WatchTelemetryResponse] = queue.Queue()

    def emit_telemetry(self, response: proto.WatchTelemetryResponse) -> None:
        self.telemetry_events.put(response)

    def ListHardware(self, request: Any, context: Any) -> proto.ListHardwareResponse:
        del request, context
        return valid_inventory_response()

    def AcquireLease(self, request: Any, context: Any) -> proto.AcquireLeaseResponse:
        del context
        duration = request.requested_duration_ms or 200
        return proto.AcquireLeaseResponse(
            group=request.group,
            token=self.token,
            expires_in_ms=duration,
        )

    def RenewLease(self, request: Any, context: Any) -> proto.RenewLeaseResponse:
        with self.lock:
            self.renew_count += 1
        if self.renew_failure is not None:
            context.abort(self.renew_failure, "test renewal failure")
        duration = request.requested_duration_ms or 200
        return proto.RenewLeaseResponse(
            group="leg",
            token=self.token,
            expires_in_ms=duration,
        )

    def ReleaseLease(self, request: Any, context: Any) -> proto.ReleaseLeaseResponse:
        del request, context
        with self.lock:
            self.release_count += 1
            state = self.state
        return proto.ReleaseLeaseResponse(state=state)

    def EnableGroup(self, request: Any, context: Any) -> proto.EnableGroupResponse:
        del request
        if self.enable_failure is not None:
            context.abort(self.enable_failure, "test enable failure")
        with self.lock:
            self.state = proto.GROUP_STATE_DAMPED
        return proto.EnableGroupResponse(state=proto.GROUP_STATE_DAMPED)

    def StartControl(self, request: Any, context: Any) -> proto.StartControlResponse:
        del request
        with self.lock:
            self.command_count += 1
            self.state = proto.GROUP_STATE_RUNNING
        if self.start_failure is not None:
            context.abort(self.start_failure, "test start response failure")
        return proto.StartControlResponse(state=proto.GROUP_STATE_RUNNING)

    def StopControl(self, request: Any, context: Any) -> proto.StopControlResponse:
        del request, context
        with self.lock:
            self.state = proto.GROUP_STATE_DAMPED
        return proto.StopControlResponse(state=proto.GROUP_STATE_DAMPED)

    def DisableGroup(self, request: Any, context: Any) -> proto.DisableGroupResponse:
        del request
        if self.disable_failure is not None:
            context.abort(self.disable_failure, "test disable failure")
        with self.lock:
            self.disable_count += 1
            self.state = proto.GROUP_STATE_IDLE
        return proto.DisableGroupResponse(state=proto.GROUP_STATE_IDLE)

    def RecoverGroup(self, request: Any, context: Any) -> proto.RecoverGroupResponse:
        del request, context
        with self.lock:
            self.state = proto.GROUP_STATE_IDLE
        return proto.RecoverGroupResponse(state=proto.GROUP_STATE_IDLE)

    def StreamCommands(
        self, requests: Iterator[proto.StreamCommandsRequest], context: Any
    ) -> Iterator[proto.StreamCommandsResponse]:
        del context
        for request in requests:
            if self.command_delay:
                time.sleep(self.command_delay)
            with self.lock:
                self.command_count += 1
                reject = self.reject_next
                self.reject_next = False
                state = self.state
            names = {command.motor_name for command in request.commands}
            complete = names == {"hip_motor", "knee_motor"}
            result = proto.COMMAND_RESULT_ACCEPTED
            detail = ""
            if reject:
                result = proto.COMMAND_RESULT_UNSAFE
                detail = "test rejection"
            elif not complete:
                result = proto.COMMAND_RESULT_INCOMPLETE_GROUP
                detail = "incomplete"
            yield proto.StreamCommandsResponse(
                group=request.group,
                sequence=request.sequence,
                result=result,
                detail=detail,
                state=state,
            )

    def WatchTelemetry(self, request: Any, context: Any) -> Iterator[proto.WatchTelemetryResponse]:
        del request
        with self.lock:
            self.telemetry_calls += 1
            fail = self.telemetry_failures > 0
            if fail:
                self.telemetry_failures -= 1
        if fail:
            context.abort(grpc.StatusCode.UNAVAILABLE, "test telemetry interruption")
        yield proto.WatchTelemetryResponse(
            monotonic_ns=1_000,
            motor=proto.MotorTelemetry(
                group="leg",
                motor_name="hip_motor",
                position=0.25,
                velocity=0.5,
                torque=1.0,
                temperature=32.0,
                feedback_age_ms=2,
            ),
        )
        yield proto.WatchTelemetryResponse(
            monotonic_ns=2_000,
            group=proto.GroupTelemetry(
                group="leg",
                state=proto.GROUP_STATE_IDLE,
                command_count=self.command_count,
            ),
        )
        yield proto.WatchTelemetryResponse(
            monotonic_ns=3_000,
            health=proto.HealthTelemetry(
                component="imu/body",
                device_status=proto.DeviceStatus(
                    state=proto.DEVICE_STATE_ONLINE,
                    detail="streaming",
                    transition_count=1,
                ),
            ),
        )
        yield proto.WatchTelemetryResponse(
            monotonic_ns=4_000,
            imu=proto.ImuTelemetry(
                imu_name="body",
                orientation=proto.Quaternion(w=1.0),
                angular_velocity=proto.Vector3(),
                linear_acceleration=proto.Vector3(z=9.80665),
                device_timestamp_ms=100,
                sample_age_ms=3,
            ),
        )
        while context.is_active():
            with suppress(queue.Empty):
                yield self.telemetry_events.get(timeout=0.01)


@dataclass(frozen=True)
class RunningServer:
    endpoint: str
    service: FakeHardware


@pytest.fixture
def hardware_server() -> Iterator[RunningServer]:
    server = grpc.server(ThreadPoolExecutor(max_workers=8))
    service = FakeHardware()
    hardware_pb2_grpc.add_HardwareServiceServicer_to_server(  # type: ignore[no-untyped-call]
        service, server
    )
    port = server.add_insecure_port("127.0.0.1:0")
    server.start()
    try:
        yield RunningServer(f"http://127.0.0.1:{port}", service)
    finally:
        server.stop(grace=0).wait()


@pytest.fixture
def unix_hardware_server(tmp_path: Path) -> Iterator[RunningServer]:
    socket = tmp_path / "hardwared.sock"
    endpoint = f"unix://{socket}"
    server = grpc.server(ThreadPoolExecutor(max_workers=8))
    service = FakeHardware()
    hardware_pb2_grpc.add_HardwareServiceServicer_to_server(  # type: ignore[no-untyped-call]
        service, server
    )
    assert server.add_insecure_port(endpoint) != 0
    server.start()
    try:
        yield RunningServer(endpoint, service)
    finally:
        server.stop(grace=0).wait()
