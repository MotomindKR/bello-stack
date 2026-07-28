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
            lambda response: setattr(response.buses[0], "interface", ""),
            "has no interface",
            id="empty-can-interface",
        ),
        pytest.param(
            lambda response: setattr(response.buses[0], "host_id", 256),
            "host ID must be in 0..=255",
            id="can-host-id-outside-protocol-range",
        ),
        pytest.param(
            lambda response: response.buses.add(
                name="duplicate_interface",
                interface="vcan0",
                host_id=0xFC,
            ),
            "CAN interface .* assigned more than once",
            id="duplicate-can-interface",
        ),
        pytest.param(
            lambda response: response.buses.add(
                name="unused",
                interface="vcan1",
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
            lambda response: response.imus.add(
                name="second_imu",
                serial_device="body_serial",
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
            "not assigned to an IMU",
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
