from __future__ import annotations

import threading
import time
from collections.abc import Callable, Iterator
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import grpc  # type: ignore[import-untyped]
import pytest
from bello_hardware_api import hardware_pb2 as proto
from bello_hardware_api import hardware_pb2_grpc


def valid_inventory_response() -> proto.ListHardwareResponse:

    return proto.ListHardwareResponse(
        buses=[proto.BusInfo(name="main", transport="socketcan:vcan0", host_id=0xFD)],
        motors=[
            proto.MotorInfo(
                name="hip_motor",
                coordinate="hip",
                group="leg",
                address=proto.MotorAddress(bus="main", id=1),
                model="RS_03",
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
                model="RS_03",
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
        imus=[
            proto.ImuInfo(
                name="body",
                serial_device="body_serial",
                source_frame="imu_sensor_link",
                output_frame="imu_link",
            )
        ],
    )


@pytest.fixture
def inventory_response() -> proto.ListHardwareResponse:
    return valid_inventory_response()


class FakeHardware:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.inventory = valid_inventory_response()
        self.inventory_failure: grpc.StatusCode | None = None
        self.state = proto.GROUP_STATE_IDLE
        self.token = "test-token"
        self.start_valid_for_ms = 0
        self.command_count = 0
        self.renew_count = 0
        self.release_failure: grpc.StatusCode | None = None
        self.release_count = 0
        self.disable_count = 0
        self.next_command_result: proto.CommandResult | None = None
        self.command_delay = 0.0
        self.close_stream_gate: threading.Event | None = None
        self.command_ack_gate: threading.Event | None = None
        self.before_start_reply: Callable[[], None] | None = None
        self.renew_failure: grpc.StatusCode | None = None
        self.lose_enable_reply = False
        self.enable_failure: grpc.StatusCode | None = None
        self.start_failure: grpc.StatusCode | None = None
        self.disable_failure: grpc.StatusCode | None = None
        self.state_frame_calls = 0
        self.state_frame_incomplete_count = 0
        self.state_frame_failure: grpc.StatusCode | None = None

    def ListHardware(self, request: Any, context: Any) -> proto.ListHardwareResponse:
        del request
        if self.inventory_failure is not None:
            context.abort(self.inventory_failure, "test inventory failure")
        snapshot = proto.ListHardwareResponse()
        snapshot.CopyFrom(self.inventory)
        for group in snapshot.groups:
            if group.name == "leg":
                group.state = self.state
        return snapshot

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
        del request
        with self.lock:
            self.release_count += 1
            if self.release_failure is not None:
                context.abort(self.release_failure, "release failed")
            state = self.state
        return proto.ReleaseLeaseResponse(state=state)

    def EnableGroup(self, request: Any, context: Any) -> proto.EnableGroupResponse:
        del request
        if self.enable_failure is not None:
            context.abort(self.enable_failure, "test enable failure")
        with self.lock:
            self.state = proto.GROUP_STATE_DAMPED
        if self.lose_enable_reply:
            context.abort(grpc.StatusCode.UNAVAILABLE, "enable reply unavailable")
        return proto.EnableGroupResponse(state=proto.GROUP_STATE_DAMPED)

    def StartControl(self, request: Any, context: Any) -> proto.StartControlResponse:
        self.start_valid_for_ms = request.valid_for_ms
        with self.lock:
            self.command_count += 1
            self.state = proto.GROUP_STATE_RUNNING
        if self.before_start_reply is not None:
            self.before_start_reply()
        if self.start_failure is not None:
            context.abort(self.start_failure, "test start response failure")
        return proto.StartControlResponse(state=proto.GROUP_STATE_RUNNING)

    def StopControl(self, request: Any, context: Any) -> proto.StopControlResponse:
        del request
        with self.lock:
            if self.state == proto.GROUP_STATE_FAULT:
                context.abort(grpc.StatusCode.FAILED_PRECONDITION, "fault latched")
            self.state = proto.GROUP_STATE_DAMPED
        return proto.StopControlResponse(state=proto.GROUP_STATE_DAMPED)

    def DisableGroup(self, request: Any, context: Any) -> proto.DisableGroupResponse:
        del request
        if self.disable_failure is not None:
            context.abort(self.disable_failure, "test disable failure")
        with self.lock:
            self.disable_count += 1
            if self.state == proto.GROUP_STATE_FAULT:
                context.abort(grpc.StatusCode.FAILED_PRECONDITION, "fault latched")
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
        if self.close_stream_gate is not None:
            assert self.close_stream_gate.wait(5)
            return
        for request in requests:
            if self.command_ack_gate is not None:
                self.command_ack_gate.wait(2.0)
            if self.command_delay:
                time.sleep(self.command_delay)
            with self.lock:
                self.command_count += 1
                override = self.next_command_result
                self.next_command_result = None
                state = self.state
            names = {command.motor_name for command in request.commands}
            complete = names == {"hip_motor", "knee_motor"}
            result = proto.COMMAND_RESULT_ACCEPTED
            detail = ""
            if override is not None:
                result = override
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
        context.abort(grpc.StatusCode.UNIMPLEMENTED, "not used by Python client tests")
        if False:
            yield proto.WatchTelemetryResponse()

    def GetStateFrame(self, request: Any, context: Any) -> proto.GetStateFrameResponse:
        del request
        with self.lock:
            self.state_frame_calls += 1
            incomplete = self.state_frame_incomplete_count > 0
            self.state_frame_incomplete_count -= int(incomplete)
            failure = self.state_frame_failure
            state = self.state
            command_count = self.command_count
            captured = self.state_frame_calls * 10_000
        if failure is not None:
            context.abort(failure, "test state frame failure")
        if incomplete:
            context.abort(
                grpc.StatusCode.FAILED_PRECONDITION,
                "complete hardware state is not yet available",
            )
        return state_frame(
            captured=captured,
            state=state,
            command_count=command_count,
        )


def state_frame(
    *,
    captured: int = 10_000,
    state: Any = proto.GROUP_STATE_IDLE,
    command_count: int = 0,
) -> proto.GetStateFrameResponse:
    motor_sample = captured - 1_000
    group_sample = captured - 500
    return proto.GetStateFrameResponse(
        captured_monotonic_ns=captured,
        motors=[
            proto.StateMotor(
                group="leg",
                motor_name="hip_motor",
                position=0.25,
                velocity=0.5,
                torque=1.0,
                temperature=32.0,
                sample_monotonic_ns=motor_sample,
                sample_age_ns=1_000,
            ),
            proto.StateMotor(
                group="leg",
                motor_name="knee_motor",
                position=-0.5,
                velocity=-0.25,
                torque=-1.0,
                temperature=33.0,
                sample_monotonic_ns=motor_sample,
                sample_age_ns=1_000,
            ),
        ],
        imus=[
            proto.StateImu(
                imu_name="body",
                orientation=proto.Quaternion(w=1.0),
                angular_velocity=proto.Vector3(),
                linear_acceleration=proto.Vector3(z=9.80665),
                device_timestamp_ms=100,
                sample_monotonic_ns=motor_sample,
                sample_age_ns=1_000,
            )
        ],
        groups=[
            proto.StateGroup(
                group="leg",
                state=state,
                command_count=command_count,
                sample_monotonic_ns=group_sample,
                sample_age_ns=500,
            )
        ],
        oldest_sample_age_ns=1_000,
        sample_span_ns=500,
    )


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
