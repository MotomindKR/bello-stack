from collections.abc import Callable

import pytest

from bello_hardware._generated import hardware_pb2 as proto
from bello_hardware._inventory import Inventory
from bello_hardware.errors import InventoryError

InventoryMutation = Callable[[proto.ListHardwareResponse], object]


@pytest.mark.parametrize(
    ("mutate", "message"),
    [
        pytest.param(
            lambda response: setattr(response.motors[0].address, "bus", "missing"),
            "unknown bus",
            id="unknown-motor-bus",
        ),
        pytest.param(
            lambda response: response.motors[1].address.CopyFrom(response.motors[0].address),
            "motor address .* assigned more than once",
            id="duplicate-motor-address",
        ),
        pytest.param(
            lambda response: setattr(response.motors[0], "model", ""),
            "has no model",
            id="empty-motor-model",
        ),
        pytest.param(
            lambda response: setattr(response.motors[0], "direction", 0),
            "direction must be -1 or 1",
            id="invalid-motor-direction",
        ),
        pytest.param(
            lambda response: setattr(response.buses[0], "transport", ""),
            "has no transport",
            id="empty-can-transport",
        ),
        pytest.param(
            lambda response: setattr(response.buses[0], "host_id", 256),
            "host ID must be in 0..=255",
            id="can-host-id-outside-protocol-range",
        ),
        pytest.param(
            lambda response: response.buses.add(
                name="duplicate_transport",
                transport="socketcan:vcan0",
                host_id=0xFC,
            ),
            "CAN transport .* assigned more than once",
            id="duplicate-can-transport",
        ),
        pytest.param(
            lambda response: response.buses.add(
                name="unused",
                transport="socketcan:vcan1",
                host_id=0xFC,
            ),
            "CAN buses have no motors",
            id="unused-can-bus",
        ),
        pytest.param(
            lambda response: setattr(response.motors[0].address, "id", 128),
            "ID must be in 0..=127",
            id="motor-id-outside-protocol-range",
        ),
        pytest.param(
            lambda response: setattr(response.buses[0], "host_id", 1),
            "ID conflicts with bus host ID",
            id="motor-id-conflicts-with-host",
        ),
        pytest.param(
            lambda response: setattr(response.motors[0], "model", "UNKNOWN"),
            "unsupported Robstride model",
            id="unsupported-motor-model",
        ),
        pytest.param(
            lambda response: setattr(response.serial_devices[0], "port", ""),
            "has no port",
            id="empty-serial-port",
        ),
        pytest.param(
            lambda response: setattr(response.serial_devices[0], "baud_rate", 0),
            "invalid baud rate",
            id="zero-serial-baud-rate",
        ),
        pytest.param(
            lambda response: response.serial_devices.add(
                name="duplicate_port",
                port="/dev/ttyUSB0",
                baud_rate=115_200,
            ),
            "serial port .* assigned more than once",
            id="duplicate-serial-port",
        ),
        pytest.param(
            lambda response: setattr(response.imus[0], "serial_device", "missing"),
            "unknown serial device",
            id="unknown-imu-serial-device",
        ),
        pytest.param(
            lambda response: setattr(response.imus[0], "source_frame", ""),
            "has no source frame",
            id="empty-imu-source-frame",
        ),
        pytest.param(
            lambda response: setattr(response.imus[0], "output_frame", ""),
            "has no output frame",
            id="empty-imu-output-frame",
        ),
        pytest.param(
            lambda response: response.imus.add(
                name="second_imu",
                serial_device="body_serial",
                source_frame="second_sensor_link",
                output_frame="second_imu_link",
            ),
            "assigned to more than one IMU",
            id="shared-imu-serial-device",
        ),
        pytest.param(
            lambda response: response.serial_devices.add(
                name="unused_serial",
                port="/dev/ttyUSB1",
                baud_rate=115_200,
            ),
            "not assigned to a sensor",
            id="unused-serial-device",
        ),
    ],
)
def test_inventory_rejects_inconsistent_physical_topology(
    inventory_response: proto.ListHardwareResponse,
    mutate: InventoryMutation,
    message: str,
) -> None:
    mutate(inventory_response)

    with pytest.raises(InventoryError, match=message):
        Inventory(inventory_response)


def test_inventory_exposes_imu_frame_contract(
    inventory_response: proto.ListHardwareResponse,
) -> None:
    inventory = Inventory(inventory_response)

    assert inventory.imus[0].source_frame == "imu_sensor_link"
    assert inventory.imus[0].output_frame == "imu_link"


@pytest.mark.parametrize("suffix", ["00", "01", "02", "03", "04", "05", "06", "10P"])
def test_inventory_requires_short_motor_model_names(
    inventory_response: proto.ListHardwareResponse, suffix: str
) -> None:
    model = f"RS_{suffix}"
    inventory_response.motors[0].model = model
    assert Inventory(inventory_response).motors[0].model == model

    inventory_response.motors[0].model = f"ROBSTRIDE_{suffix}"
    with pytest.raises(InventoryError, match="unsupported Robstride model"):
        Inventory(inventory_response)
