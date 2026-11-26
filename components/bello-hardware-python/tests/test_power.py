from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
from typing import Any

import grpc  # type: ignore[import-untyped]
import pytest
from bello_hardware_api import hardware_pb2 as proto
from bello_hardware_api import hardware_pb2_grpc

from bello_hardware import PowerBattery, PowerGpi, Robot
from bello_hardware.errors import ProtocolError
from bello_hardware.power import _decode_sample

from .conftest import FakeHardware


def test_power_reader_uses_telemetry_without_control_or_state_frame() -> None:
    class BoardHardware(FakeHardware):
        def ListHardware(self, request: Any, context: Any) -> proto.ListHardwareResponse:
            response = proto.ListHardwareResponse()
            response.CopyFrom(super().ListHardware(request, context))
            response.serial_devices.add(name="power_serial", port="/dev/ttyPOWER", baud_rate=921600)
            response.power_boards.add(
                name="power",
                serial_device="power_serial",
                profile="agx",
                stale_timeout_ms=3000,
                sync_period_ms=1000,
            )
            return response

        def WatchTelemetry(self, request: Any, context: Any) -> Any:
            assert request.include_power_boards and list(request.power_boards) == ["power"]
            yield proto.WatchTelemetryResponse(
                power_board=proto.PowerBoardTelemetry(
                    board_name="power",
                    connection_generation=2,
                    samples=[
                        proto.PowerBoardSample(
                            sequence=255,
                            sample_age_ms=4000,
                            battery=proto.PowerBattery(
                                voltage_v=48.5, current_a=-1.25, soc_percent=73
                            ),
                        )
                    ],
                )
            )

    service = BoardHardware()
    server = grpc.server(ThreadPoolExecutor(max_workers=2))
    hardware_pb2_grpc.add_HardwareServiceServicer_to_server(service, server)  # type: ignore[no-untyped-call]
    port = server.add_insecure_port("127.0.0.1:0")
    server.start()
    try:
        with Robot.connect(f"http://127.0.0.1:{port}") as robot:
            (snapshot,) = robot.power().read(timeout=1)
            assert snapshot.connection_generation == 2
            assert snapshot.samples[0].reading == PowerBattery(
                voltage_v=48.5, current_a=-1.25, soc_percent=73
            )
            assert snapshot.is_stale(snapshot.samples[0])
            assert service.command_count == 0 and service.state_frame_calls == 0
            with pytest.raises(ValueError, match="unknown"):
                robot.power(boards=("missing",))
    finally:
        server.stop(0).wait()


@pytest.mark.parametrize(
    "sample",
    [
        proto.PowerBoardSample(),
        proto.PowerBoardSample(sequence=256, tick_count=1),
        proto.PowerBoardSample(battery=proto.PowerBattery(soc_percent=101)),
        proto.PowerBoardSample(battery=proto.PowerBattery(voltage_v=float("nan"))),
        proto.PowerBoardSample(iwc=proto.PowerIwc()),
        proto.PowerBoardSample(local_rails=proto.PowerLocalRails()),
    ],
)
def test_invalid_power_samples_are_rejected(sample: proto.PowerBoardSample) -> None:
    with pytest.raises(ProtocolError):
        _decode_sample(sample)


@pytest.mark.parametrize("soc_percent", [None, 0])
def test_battery_charge_presence(soc_percent: int | None) -> None:
    sample = _decode_sample(
        proto.PowerBoardSample(
            battery=proto.PowerBattery(voltage_v=24, current_a=1, soc_percent=soc_percent),
        )
    )
    assert sample.reading == PowerBattery(voltage_v=24, current_a=1, soc_percent=soc_percent)


@pytest.mark.parametrize("second_bits", [None, 0])
def test_gpi_second_register_presence(second_bits: int | None) -> None:
    sample = _decode_sample(proto.PowerBoardSample(gpi=proto.PowerGpi(second_bits=second_bits)))
    assert sample.reading == PowerGpi(bits=0, second_bits=second_bits)
