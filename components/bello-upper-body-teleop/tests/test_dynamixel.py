from __future__ import annotations

import math
from typing import Any

import pytest

from bello_upper_body_teleop.config import JointMapping
from bello_upper_body_teleop.dynamixel import (
    AUXILIARY_HELD_DYNAMIXELS,
    DynamixelCommunicationError,
    DynamixelTeachingArms,
    HeldDynamixel,
)


class FakePortHandler:
    def __init__(self, port: str) -> None:
        self.port = port
        self.opened = False
        self.closed = False
        self.baudrate = 0

    def openPort(self) -> bool:
        self.opened = True
        return True

    def setBaudRate(self, baudrate: int) -> bool:
        self.baudrate = baudrate
        return True

    def closePort(self) -> None:
        self.closed = True


class FakePacketHandler:
    def __init__(
        self,
        protocol: float,
        ids: tuple[int, ...],
        *,
        read_result: int = 0,
        read_error: int = 0,
        torque_stays_enabled: bool = False,
        operating_modes: dict[int, int] | None = None,
        current_limits: dict[int, int] | None = None,
    ) -> None:
        self.protocol = protocol
        self.torque_writes: list[tuple[int, int, int]] = []
        self.sync_writes: list[tuple[int, int, dict[int, tuple[int, ...]]]] = []
        self.read_result = read_result
        self.read_error = read_error
        self.torque_stays_enabled = torque_stays_enabled
        modes = operating_modes or {}
        limits = current_limits or {}
        self.registers = {
            (dxl_id, address): value
            for dxl_id in ids
            for address, value in (
                (11, modes.get(dxl_id, 5)),
                (38, limits.get(dxl_id, 1_750)),
                (64, 0),
            )
        }

    def read1ByteTxRx(
        self,
        _port: FakePortHandler,
        dxl_id: int,
        address: int,
    ) -> tuple[int, int, int]:
        value = (
            1
            if address == 64 and self.torque_stays_enabled
            else self.registers.get((dxl_id, address), 0)
        )
        return value, self.read_result, self.read_error

    def read2ByteTxRx(
        self,
        _port: FakePortHandler,
        dxl_id: int,
        address: int,
    ) -> tuple[int, int, int]:
        value = self.registers.get((dxl_id, address), 0)
        return value, self.read_result, self.read_error

    def getTxRxResult(self, result: int) -> str:
        return f"result={result}"


class FakeGroupSyncWrite:
    def __init__(
        self,
        packet: FakePacketHandler,
        address: int,
        length: int,
        result: int,
    ) -> None:
        self.packet = packet
        self.address = address
        self.length = length
        self.result = result
        self.params: dict[int, list[int]] = {}
        self.cleared = False

    def addParam(self, dxl_id: int, values: list[int]) -> bool:
        self.params[dxl_id] = values
        return True

    def txPacket(self) -> int:
        if self.result == 0:
            self.packet.sync_writes.append(
                (
                    self.address,
                    self.length,
                    {dxl_id: tuple(values) for dxl_id, values in self.params.items()},
                )
            )
            for dxl_id, values in self.params.items():
                value = sum(byte << (index * 8) for index, byte in enumerate(values))
                self.packet.registers[(dxl_id, self.address)] = value
                if self.address == 64:
                    self.packet.torque_writes.append((dxl_id, self.address, values[0]))
        return self.result

    def clearParam(self) -> None:
        self.cleared = True


class FakeGroupSyncRead:
    def __init__(
        self,
        _port: FakePortHandler,
        _packet: FakePacketHandler,
        _address: int,
        _length: int,
        values: dict[int, int],
    ) -> None:
        self.values = values
        self.ids: list[int] = []
        self.result = 0

    def addParam(self, dxl_id: int) -> bool:
        self.ids.append(dxl_id)
        return True

    def txRxPacket(self) -> int:
        return self.result

    def isAvailable(self, dxl_id: int, _address: int, _length: int) -> bool:
        return dxl_id in self.values

    def getData(self, dxl_id: int, _address: int, _length: int) -> int:
        return self.values[dxl_id]


class FakeSdk:
    COMM_SUCCESS = 0

    def __init__(
        self,
        values: dict[int, int],
        *,
        write_result: int = 0,
        torque_read_result: int = 0,
        torque_read_error: int = 0,
        torque_stays_enabled: bool = False,
        operating_modes: dict[int, int] | None = None,
        current_limits: dict[int, int] | None = None,
    ) -> None:
        self.values = values
        self.write_result = write_result
        self.torque_read_result = torque_read_result
        self.torque_read_error = torque_read_error
        self.torque_stays_enabled = torque_stays_enabled
        self.operating_modes = operating_modes
        self.current_limits = current_limits
        self.port_handler: FakePortHandler | None = None
        self.packet_handler: FakePacketHandler | None = None
        self.group_write: FakeGroupSyncWrite | None = None
        self.group_writes: list[FakeGroupSyncWrite] = []
        self.group_read: FakeGroupSyncRead | None = None

    def PortHandler(self, port: str) -> FakePortHandler:
        self.port_handler = FakePortHandler(port)
        return self.port_handler

    def PacketHandler(self, protocol: float) -> FakePacketHandler:
        self.packet_handler = FakePacketHandler(
            protocol,
            tuple(self.values),
            read_result=self.torque_read_result,
            read_error=self.torque_read_error,
            torque_stays_enabled=self.torque_stays_enabled,
            operating_modes=self.operating_modes,
            current_limits=self.current_limits,
        )
        return self.packet_handler

    def GroupSyncWrite(
        self,
        _port: FakePortHandler,
        packet: FakePacketHandler,
        address: int,
        length: int,
    ) -> FakeGroupSyncWrite:
        self.group_write = FakeGroupSyncWrite(
            packet,
            address,
            length,
            self.write_result,
        )
        self.group_writes.append(self.group_write)
        return self.group_write

    def GroupSyncRead(
        self,
        port: FakePortHandler,
        packet: FakePacketHandler,
        address: int,
        length: int,
    ) -> FakeGroupSyncRead:
        self.group_read = FakeGroupSyncRead(port, packet, address, length, self.values)
        return self.group_read


def _mapping(dxl_id: int, coordinate: str) -> JointMapping:
    return JointMapping(
        dxl_id=dxl_id,
        coordinate=coordinate,
        position_offset_radians=0.0,
        action_gain=1.0,
        minimum_radians=-10.0,
        maximum_radians=10.0,
        kp=1.0,
        kd=0.1,
    )


def _require(value: Any | None) -> Any:
    assert value is not None
    return value


def _decoded_write(
    packet: FakePacketHandler,
    address: int,
    dxl_id: int,
    *,
    occurrence: int = -1,
) -> int:
    matching = [
        values[dxl_id]
        for written_address, _length, values in packet.sync_writes
        if written_address == address
    ]
    encoded = matching[occurrence]
    return sum(byte << (index * 8) for index, byte in enumerate(encoded))


def test_reader_opens_port_disables_torque_and_decodes_signed_positions() -> None:
    sdk = FakeSdk({1: 2048, 2: 0xFFFFFFFF})
    joints = (_mapping(1, "one"), _mapping(2, "two"))

    with DynamixelTeachingArms(
        port="/dev/test",
        baudrate=1_000_000,
        joints=joints,
        held_dynamixels=(),
        sdk=sdk,
    ) as reader:
        sample = reader.read()

    port = _require(sdk.port_handler)
    packet = _require(sdk.packet_handler)
    group_write = _require(sdk.group_write)
    assert port.opened
    assert port.closed
    assert port.baudrate == 1_000_000
    assert packet.torque_writes == [(1, 64, 0), (2, 64, 0)]
    assert group_write.cleared
    assert sample.positions["one"] == pytest.approx(3.141592653589793)
    assert sample.positions["two"] == pytest.approx(-3.141592653589793 / 2048.0)


def test_reader_surfaces_group_communication_failure() -> None:
    sdk = FakeSdk({1: 0})
    reader = DynamixelTeachingArms(
        joints=(_mapping(1, "one"),),
        held_dynamixels=(),
        sdk=sdk,
    )
    group_read = _require(sdk.group_read)
    group_read.result = -1001

    with pytest.raises(DynamixelCommunicationError, match="result=-1001"):
        reader.read()

    reader.close()


def test_reader_requires_torque_disable_readback() -> None:
    sdk = FakeSdk({1: 0}, torque_read_result=-1001)

    with pytest.raises(DynamixelCommunicationError, match="torque-disable"):
        DynamixelTeachingArms(
            joints=(_mapping(1, "one"),),
            held_dynamixels=(),
            sdk=sdk,
        )


def test_reader_surfaces_torque_sync_write_failure() -> None:
    sdk = FakeSdk({1: 0}, write_result=-1001)

    with pytest.raises(DynamixelCommunicationError, match="synchronous write"):
        DynamixelTeachingArms(
            joints=(_mapping(1, "one"),),
            held_dynamixels=(),
            sdk=sdk,
        )


def test_reader_rejects_torque_that_remains_enabled() -> None:
    sdk = FakeSdk({1: 0}, torque_stays_enabled=True)

    with pytest.raises(DynamixelCommunicationError, match="still reports torque enabled"):
        DynamixelTeachingArms(
            joints=(_mapping(1, "one"),),
            held_dynamixels=(),
            sdk=sdk,
        )


def test_reader_safely_enables_then_disables_owned_master_torque() -> None:
    sdk = FakeSdk({1: 2_048, 2: 0xFFFFFFFF})
    joints = (_mapping(1, "one"), _mapping(2, "two"))

    reader = DynamixelTeachingArms(
        joints=joints,
        held_dynamixels=(),
        torque_mode="enable",
        sdk=sdk,
    )

    packet = _require(sdk.packet_handler)
    assert reader.master_torque_enabled
    assert [address for address, _length, _values in packet.sync_writes] == [
        64,
        116,
        80,
        82,
        84,
        102,
        64,
        116,
        84,
    ]
    assert _decoded_write(packet, 116, 1, occurrence=0) == 2_048
    assert _decoded_write(packet, 116, 2, occurrence=0) == 0xFFFFFFFF
    assert _decoded_write(packet, 116, 1) == 0
    assert _decoded_write(packet, 116, 2) == 0
    assert _decoded_write(packet, 80, 1) == 0
    assert _decoded_write(packet, 82, 1) == 0
    assert _decoded_write(packet, 84, 1) == 10
    assert _decoded_write(packet, 102, 1) == 800
    assert packet.torque_writes[-2:] == [(1, 64, 1), (2, 64, 1)]

    reader.close()

    assert not reader.master_torque_enabled
    assert packet.torque_writes[-2:] == [(1, 64, 0), (2, 64, 0)]


def test_master_torque_requires_current_based_position_mode() -> None:
    sdk = FakeSdk({1: 0}, operating_modes={1: 3})

    with pytest.raises(DynamixelCommunicationError, match="expected current-based"):
        DynamixelTeachingArms(
            joints=(_mapping(1, "one"),),
            held_dynamixels=(),
            torque_mode="enable",
            sdk=sdk,
        )

    packet = _require(sdk.packet_handler)
    assert all(value == 0 for _dxl_id, _address, value in packet.torque_writes)


def test_master_torque_requires_sufficient_configured_current_limit() -> None:
    sdk = FakeSdk({1: 0}, current_limits={1: 799})

    with pytest.raises(DynamixelCommunicationError, match="below the requested 800"):
        DynamixelTeachingArms(
            joints=(_mapping(1, "one"),),
            held_dynamixels=(),
            torque_mode="enable",
            sdk=sdk,
        )


def test_torque_feedback_constantly_writes_configured_default_at_weak_gain() -> None:
    joint = JointMapping(
        dxl_id=1,
        coordinate="one",
        position_offset_radians=1.5,
        action_gain=-1.0,
        minimum_radians=-0.5,
        maximum_radians=0.5,
        kp=1.0,
        kd=0.1,
        default_position_radians=-0.25,
    )
    sdk = FakeSdk({1: 1_000})

    with DynamixelTeachingArms(
        joints=(joint,),
        held_dynamixels=(),
        torque_mode="enable",
        sdk=sdk,
    ) as reader:
        packet = _require(sdk.packet_handler)
        default_target = joint.raw_position_for_robot_position(
            joint.default_position_radians,
        )
        assert _decoded_write(packet, 116, 1) == default_target
        assert _decoded_write(packet, 84, 1) == 10

        sdk.values[1] = 2_000
        reader.read()
        assert _decoded_write(packet, 116, 1) == default_target
        assert _decoded_write(packet, 84, 1) == 100

        sdk.values[1] = 1_000
        reader.read()
        assert _decoded_write(packet, 116, 1) == default_target
        assert _decoded_write(packet, 84, 1) == 10


def test_limit_stop_gain_activates_at_both_joint_boundaries() -> None:
    joint = JointMapping(
        dxl_id=1,
        coordinate="one",
        position_offset_radians=0.0,
        action_gain=1.0,
        minimum_radians=-math.pi / 2.0,
        maximum_radians=math.pi / 2.0,
        kp=1.0,
        kd=0.1,
    )
    sdk = FakeSdk({1: 0})

    with DynamixelTeachingArms(
        joints=(joint,),
        held_dynamixels=(),
        torque_mode="enable",
        sdk=sdk,
    ) as reader:
        packet = _require(sdk.packet_handler)
        assert _decoded_write(packet, 84, 1) == 10

        sdk.values[1] = 1_024
        reader.read()
        assert _decoded_write(packet, 84, 1) == 100

        sdk.values[1] = -1_024
        reader.read()
        assert _decoded_write(packet, 84, 1) == 100


def test_auxiliary_waist_and_wrists_lock_to_configured_positions() -> None:
    joint = _mapping(1, "mapped")
    held = (
        HeldDynamixel(9, "waist", 512),
        HeldDynamixel(27, "left_wrist", 4_096),
        HeldDynamixel(17, "right_wrist", 4_096),
    )
    captured = {1: 1_000, 9: 436, 27: 2_135, 17: 6_049}
    hold_targets = {9: 512, 27: 4_096, 17: 4_096}
    sdk = FakeSdk(dict(captured))

    with DynamixelTeachingArms(
        joints=(joint,),
        held_dynamixels=held,
        torque_mode="enable",
        sdk=sdk,
    ) as reader:
        packet = _require(sdk.packet_handler)
        assert reader.ids == (1, 9, 27, 17)
        assert reader.held_ids == (9, 27, 17)
        assert reader.held_names == ("waist", "left_wrist", "right_wrist")
        assert _decoded_write(packet, 116, 1) == 0
        for dxl_id in (9, 27, 17):
            assert _decoded_write(packet, 116, dxl_id) == hold_targets[dxl_id]
            assert _decoded_write(packet, 84, dxl_id) == 250
            assert _decoded_write(packet, 102, dxl_id) == 800

        sdk.values.update({9: 500, 27: 2_200, 17: 6_100})
        sample = reader.read()
        assert sample.raw_positions == {1: 1_000}
        for dxl_id in (9, 27, 17):
            assert _decoded_write(packet, 116, dxl_id) == hold_targets[dxl_id]

    assert packet.torque_writes[-4:] == [
        (1, 64, 0),
        (9, 64, 0),
        (27, 64, 0),
        (17, 64, 0),
    ]


def test_default_auxiliary_hold_ids_match_bello_teaching_rig() -> None:
    assert tuple(motor.dxl_id for motor in AUXILIARY_HELD_DYNAMIXELS) == (9, 27, 17)
    assert tuple(motor.goal_position for motor in AUXILIARY_HELD_DYNAMIXELS) == (
        512,
        4_096,
        4_096,
    )
