import pytest

from bello_hardware._generated import hardware_pb2 as proto
from bello_hardware._inventory import Inventory
from bello_hardware.errors import InventoryError


def test_inventory_rejects_ambiguous_command_coordinate(
    inventory_response: proto.ListHardwareResponse,
) -> None:
    inventory_response.motors[1].coordinate = inventory_response.motors[0].coordinate
    with pytest.raises(InventoryError, match="duplicate coordinate"):
        Inventory(inventory_response)


def test_inventory_exposes_new_motor_model(inventory_response: proto.ListHardwareResponse) -> None:
    inventory_response.motors[0].model = "future-model"
    assert Inventory(inventory_response).motors[0].model == "future-model"
