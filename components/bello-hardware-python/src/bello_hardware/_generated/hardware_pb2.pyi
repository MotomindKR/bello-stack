# mypy: ignore-errors
from google.protobuf.internal import containers as _containers
from google.protobuf.internal import enum_type_wrapper as _enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from collections.abc import Iterable as _Iterable, Mapping as _Mapping
from typing import ClassVar as _ClassVar, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class DeviceState(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = ()
    DEVICE_STATE_UNSPECIFIED: _ClassVar[DeviceState]
    DEVICE_STATE_STARTING: _ClassVar[DeviceState]
    DEVICE_STATE_CONNECTING: _ClassVar[DeviceState]
    DEVICE_STATE_ONLINE: _ClassVar[DeviceState]
    DEVICE_STATE_DEGRADED: _ClassVar[DeviceState]
    DEVICE_STATE_RECONNECTING: _ClassVar[DeviceState]
    DEVICE_STATE_FAULT: _ClassVar[DeviceState]
    DEVICE_STATE_SHUTDOWN: _ClassVar[DeviceState]

class GroupState(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = ()
    GROUP_STATE_UNSPECIFIED: _ClassVar[GroupState]
    GROUP_STATE_IDLE: _ClassVar[GroupState]
    GROUP_STATE_RUNNING: _ClassVar[GroupState]
    GROUP_STATE_FAULT: _ClassVar[GroupState]
    GROUP_STATE_SHUTDOWN: _ClassVar[GroupState]
    GROUP_STATE_DAMPED: _ClassVar[GroupState]

class CommandPolicyCheck(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = ()
    COMMAND_POLICY_CHECK_UNSPECIFIED: _ClassVar[CommandPolicyCheck]
    COMMAND_POLICY_CHECK_LIMIT: _ClassVar[CommandPolicyCheck]
    COMMAND_POLICY_CHECK_POSITION_STEP: _ClassVar[CommandPolicyCheck]

class CommandBehavior(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = ()
    COMMAND_BEHAVIOR_UNSPECIFIED: _ClassVar[CommandBehavior]
    COMMAND_BEHAVIOR_CLAMP: _ClassVar[CommandBehavior]
    COMMAND_BEHAVIOR_REJECT: _ClassVar[CommandBehavior]
    COMMAND_BEHAVIOR_FAULT: _ClassVar[CommandBehavior]
    COMMAND_BEHAVIOR_WARN: _ClassVar[CommandBehavior]

class FaultResponse(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = ()
    FAULT_RESPONSE_UNSPECIFIED: _ClassVar[FaultResponse]
    FAULT_RESPONSE_DAMPING: _ClassVar[FaultResponse]
    FAULT_RESPONSE_DISABLE: _ClassVar[FaultResponse]

class GroupCause(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = ()
    GROUP_CAUSE_UNSPECIFIED: _ClassVar[GroupCause]
    GROUP_CAUSE_ENABLE: _ClassVar[GroupCause]
    GROUP_CAUSE_START: _ClassVar[GroupCause]
    GROUP_CAUSE_STOP: _ClassVar[GroupCause]
    GROUP_CAUSE_COMMAND_TIMEOUT: _ClassVar[GroupCause]
    GROUP_CAUSE_DISABLE: _ClassVar[GroupCause]
    GROUP_CAUSE_RECOVER: _ClassVar[GroupCause]
    GROUP_CAUSE_COMMAND_LIMIT: _ClassVar[GroupCause]
    GROUP_CAUSE_COMMAND_POSITION_STEP: _ClassVar[GroupCause]
    GROUP_CAUSE_SAFE_POSITION: _ClassVar[GroupCause]
    GROUP_CAUSE_STALE_FEEDBACK: _ClassVar[GroupCause]
    GROUP_CAUSE_TEMPERATURE: _ClassVar[GroupCause]
    GROUP_CAUSE_REPORTED_MOTOR_FAULT: _ClassVar[GroupCause]
    GROUP_CAUSE_MODE_STATUS: _ClassVar[GroupCause]
    GROUP_CAUSE_COMMUNICATION: _ClassVar[GroupCause]
    GROUP_CAUSE_TRANSMISSION: _ClassVar[GroupCause]
    GROUP_CAUSE_LEASE_LOSS: _ClassVar[GroupCause]
    GROUP_CAUSE_EXIT: _ClassVar[GroupCause]
    GROUP_CAUSE_INTERNAL: _ClassVar[GroupCause]

class CommandResult(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = ()
    COMMAND_RESULT_UNSPECIFIED: _ClassVar[CommandResult]
    COMMAND_RESULT_ACCEPTED: _ClassVar[CommandResult]
    COMMAND_RESULT_UNKNOWN_GROUP: _ClassVar[CommandResult]
    COMMAND_RESULT_LEASE_LOST: _ClassVar[CommandResult]
    COMMAND_RESULT_INVALID_STATE: _ClassVar[CommandResult]
    COMMAND_RESULT_INCOMPLETE_GROUP: _ClassVar[CommandResult]
    COMMAND_RESULT_STALE_SEQUENCE: _ClassVar[CommandResult]
    COMMAND_RESULT_UNSAFE: _ClassVar[CommandResult]
    COMMAND_RESULT_SUPERSEDED: _ClassVar[CommandResult]
    COMMAND_RESULT_CONTROLLER_STOPPED: _ClassVar[CommandResult]
    COMMAND_RESULT_EXPIRED: _ClassVar[CommandResult]
DEVICE_STATE_UNSPECIFIED: DeviceState
DEVICE_STATE_STARTING: DeviceState
DEVICE_STATE_CONNECTING: DeviceState
DEVICE_STATE_ONLINE: DeviceState
DEVICE_STATE_DEGRADED: DeviceState
DEVICE_STATE_RECONNECTING: DeviceState
DEVICE_STATE_FAULT: DeviceState
DEVICE_STATE_SHUTDOWN: DeviceState
GROUP_STATE_UNSPECIFIED: GroupState
GROUP_STATE_IDLE: GroupState
GROUP_STATE_RUNNING: GroupState
GROUP_STATE_FAULT: GroupState
GROUP_STATE_SHUTDOWN: GroupState
GROUP_STATE_DAMPED: GroupState
COMMAND_POLICY_CHECK_UNSPECIFIED: CommandPolicyCheck
COMMAND_POLICY_CHECK_LIMIT: CommandPolicyCheck
COMMAND_POLICY_CHECK_POSITION_STEP: CommandPolicyCheck
COMMAND_BEHAVIOR_UNSPECIFIED: CommandBehavior
COMMAND_BEHAVIOR_CLAMP: CommandBehavior
COMMAND_BEHAVIOR_REJECT: CommandBehavior
COMMAND_BEHAVIOR_FAULT: CommandBehavior
COMMAND_BEHAVIOR_WARN: CommandBehavior
FAULT_RESPONSE_UNSPECIFIED: FaultResponse
FAULT_RESPONSE_DAMPING: FaultResponse
FAULT_RESPONSE_DISABLE: FaultResponse
GROUP_CAUSE_UNSPECIFIED: GroupCause
GROUP_CAUSE_ENABLE: GroupCause
GROUP_CAUSE_START: GroupCause
GROUP_CAUSE_STOP: GroupCause
GROUP_CAUSE_COMMAND_TIMEOUT: GroupCause
GROUP_CAUSE_DISABLE: GroupCause
GROUP_CAUSE_RECOVER: GroupCause
GROUP_CAUSE_COMMAND_LIMIT: GroupCause
GROUP_CAUSE_COMMAND_POSITION_STEP: GroupCause
GROUP_CAUSE_SAFE_POSITION: GroupCause
GROUP_CAUSE_STALE_FEEDBACK: GroupCause
GROUP_CAUSE_TEMPERATURE: GroupCause
GROUP_CAUSE_REPORTED_MOTOR_FAULT: GroupCause
GROUP_CAUSE_MODE_STATUS: GroupCause
GROUP_CAUSE_COMMUNICATION: GroupCause
GROUP_CAUSE_TRANSMISSION: GroupCause
GROUP_CAUSE_LEASE_LOSS: GroupCause
GROUP_CAUSE_EXIT: GroupCause
GROUP_CAUSE_INTERNAL: GroupCause
COMMAND_RESULT_UNSPECIFIED: CommandResult
COMMAND_RESULT_ACCEPTED: CommandResult
COMMAND_RESULT_UNKNOWN_GROUP: CommandResult
COMMAND_RESULT_LEASE_LOST: CommandResult
COMMAND_RESULT_INVALID_STATE: CommandResult
COMMAND_RESULT_INCOMPLETE_GROUP: CommandResult
COMMAND_RESULT_STALE_SEQUENCE: CommandResult
COMMAND_RESULT_UNSAFE: CommandResult
COMMAND_RESULT_SUPERSEDED: CommandResult
COMMAND_RESULT_CONTROLLER_STOPPED: CommandResult
COMMAND_RESULT_EXPIRED: CommandResult

class DeviceStatus(_message.Message):
    __slots__ = ("state", "detail", "transition_count")
    STATE_FIELD_NUMBER: _ClassVar[int]
    DETAIL_FIELD_NUMBER: _ClassVar[int]
    TRANSITION_COUNT_FIELD_NUMBER: _ClassVar[int]
    state: DeviceState
    detail: str
    transition_count: int
    def __init__(self, state: _Optional[_Union[DeviceState, str]] = ..., detail: _Optional[str] = ..., transition_count: _Optional[int] = ...) -> None: ...

class MotorAddress(_message.Message):
    __slots__ = ("bus", "id")
    BUS_FIELD_NUMBER: _ClassVar[int]
    ID_FIELD_NUMBER: _ClassVar[int]
    bus: str
    id: int
    def __init__(self, bus: _Optional[str] = ..., id: _Optional[int] = ...) -> None: ...

class BusInfo(_message.Message):
    __slots__ = ("name", "interface", "host_id", "status")
    NAME_FIELD_NUMBER: _ClassVar[int]
    INTERFACE_FIELD_NUMBER: _ClassVar[int]
    HOST_ID_FIELD_NUMBER: _ClassVar[int]
    STATUS_FIELD_NUMBER: _ClassVar[int]
    name: str
    interface: str
    host_id: int
    status: DeviceStatus
    def __init__(self, name: _Optional[str] = ..., interface: _Optional[str] = ..., host_id: _Optional[int] = ..., status: _Optional[_Union[DeviceStatus, _Mapping]] = ...) -> None: ...

class SerialDeviceInfo(_message.Message):
    __slots__ = ("name", "port", "baud_rate", "status")
    NAME_FIELD_NUMBER: _ClassVar[int]
    PORT_FIELD_NUMBER: _ClassVar[int]
    BAUD_RATE_FIELD_NUMBER: _ClassVar[int]
    STATUS_FIELD_NUMBER: _ClassVar[int]
    name: str
    port: str
    baud_rate: int
    status: DeviceStatus
    def __init__(self, name: _Optional[str] = ..., port: _Optional[str] = ..., baud_rate: _Optional[int] = ..., status: _Optional[_Union[DeviceStatus, _Mapping]] = ...) -> None: ...

class ImuInfo(_message.Message):
    __slots__ = ("name", "serial_device", "status")
    NAME_FIELD_NUMBER: _ClassVar[int]
    SERIAL_DEVICE_FIELD_NUMBER: _ClassVar[int]
    STATUS_FIELD_NUMBER: _ClassVar[int]
    name: str
    serial_device: str
    status: DeviceStatus
    def __init__(self, name: _Optional[str] = ..., serial_device: _Optional[str] = ..., status: _Optional[_Union[DeviceStatus, _Mapping]] = ...) -> None: ...

class MotorLimits(_message.Message):
    __slots__ = ("position_min", "position_max", "velocity_min", "velocity_max", "torque_min", "torque_max", "kp_min", "kp_max", "kd_min", "kd_max")
    POSITION_MIN_FIELD_NUMBER: _ClassVar[int]
    POSITION_MAX_FIELD_NUMBER: _ClassVar[int]
    VELOCITY_MIN_FIELD_NUMBER: _ClassVar[int]
    VELOCITY_MAX_FIELD_NUMBER: _ClassVar[int]
    TORQUE_MIN_FIELD_NUMBER: _ClassVar[int]
    TORQUE_MAX_FIELD_NUMBER: _ClassVar[int]
    KP_MIN_FIELD_NUMBER: _ClassVar[int]
    KP_MAX_FIELD_NUMBER: _ClassVar[int]
    KD_MIN_FIELD_NUMBER: _ClassVar[int]
    KD_MAX_FIELD_NUMBER: _ClassVar[int]
    position_min: float
    position_max: float
    velocity_min: float
    velocity_max: float
    torque_min: float
    torque_max: float
    kp_min: float
    kp_max: float
    kd_min: float
    kd_max: float
    def __init__(self, position_min: _Optional[float] = ..., position_max: _Optional[float] = ..., velocity_min: _Optional[float] = ..., velocity_max: _Optional[float] = ..., torque_min: _Optional[float] = ..., torque_max: _Optional[float] = ..., kp_min: _Optional[float] = ..., kp_max: _Optional[float] = ..., kd_min: _Optional[float] = ..., kd_max: _Optional[float] = ...) -> None: ...

class MotorInfo(_message.Message):
    __slots__ = ("name", "coordinate", "group", "address", "model", "direction", "status", "limits")
    NAME_FIELD_NUMBER: _ClassVar[int]
    COORDINATE_FIELD_NUMBER: _ClassVar[int]
    GROUP_FIELD_NUMBER: _ClassVar[int]
    ADDRESS_FIELD_NUMBER: _ClassVar[int]
    MODEL_FIELD_NUMBER: _ClassVar[int]
    DIRECTION_FIELD_NUMBER: _ClassVar[int]
    STATUS_FIELD_NUMBER: _ClassVar[int]
    LIMITS_FIELD_NUMBER: _ClassVar[int]
    name: str
    coordinate: str
    group: str
    address: MotorAddress
    model: str
    direction: int
    status: DeviceStatus
    limits: MotorLimits
    def __init__(self, name: _Optional[str] = ..., coordinate: _Optional[str] = ..., group: _Optional[str] = ..., address: _Optional[_Union[MotorAddress, _Mapping]] = ..., model: _Optional[str] = ..., direction: _Optional[int] = ..., status: _Optional[_Union[DeviceStatus, _Mapping]] = ..., limits: _Optional[_Union[MotorLimits, _Mapping]] = ...) -> None: ...

class GroupInfo(_message.Message):
    __slots__ = ("name", "control_rate_hz", "state", "motors")
    NAME_FIELD_NUMBER: _ClassVar[int]
    CONTROL_RATE_HZ_FIELD_NUMBER: _ClassVar[int]
    STATE_FIELD_NUMBER: _ClassVar[int]
    MOTORS_FIELD_NUMBER: _ClassVar[int]
    name: str
    control_rate_hz: float
    state: GroupState
    motors: _containers.RepeatedCompositeFieldContainer[MotorAddress]
    def __init__(self, name: _Optional[str] = ..., control_rate_hz: _Optional[float] = ..., state: _Optional[_Union[GroupState, str]] = ..., motors: _Optional[_Iterable[_Union[MotorAddress, _Mapping]]] = ...) -> None: ...

class ListHardwareRequest(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class ListHardwareResponse(_message.Message):
    __slots__ = ("buses", "motors", "groups", "serial_devices", "imus")
    BUSES_FIELD_NUMBER: _ClassVar[int]
    MOTORS_FIELD_NUMBER: _ClassVar[int]
    GROUPS_FIELD_NUMBER: _ClassVar[int]
    SERIAL_DEVICES_FIELD_NUMBER: _ClassVar[int]
    IMUS_FIELD_NUMBER: _ClassVar[int]
    buses: _containers.RepeatedCompositeFieldContainer[BusInfo]
    motors: _containers.RepeatedCompositeFieldContainer[MotorInfo]
    groups: _containers.RepeatedCompositeFieldContainer[GroupInfo]
    serial_devices: _containers.RepeatedCompositeFieldContainer[SerialDeviceInfo]
    imus: _containers.RepeatedCompositeFieldContainer[ImuInfo]
    def __init__(self, buses: _Optional[_Iterable[_Union[BusInfo, _Mapping]]] = ..., motors: _Optional[_Iterable[_Union[MotorInfo, _Mapping]]] = ..., groups: _Optional[_Iterable[_Union[GroupInfo, _Mapping]]] = ..., serial_devices: _Optional[_Iterable[_Union[SerialDeviceInfo, _Mapping]]] = ..., imus: _Optional[_Iterable[_Union[ImuInfo, _Mapping]]] = ...) -> None: ...

class AcquireLeaseRequest(_message.Message):
    __slots__ = ("group", "client_name", "requested_duration_ms")
    GROUP_FIELD_NUMBER: _ClassVar[int]
    CLIENT_NAME_FIELD_NUMBER: _ClassVar[int]
    REQUESTED_DURATION_MS_FIELD_NUMBER: _ClassVar[int]
    group: str
    client_name: str
    requested_duration_ms: int
    def __init__(self, group: _Optional[str] = ..., client_name: _Optional[str] = ..., requested_duration_ms: _Optional[int] = ...) -> None: ...

class AcquireLeaseResponse(_message.Message):
    __slots__ = ("group", "token", "expires_in_ms")
    GROUP_FIELD_NUMBER: _ClassVar[int]
    TOKEN_FIELD_NUMBER: _ClassVar[int]
    EXPIRES_IN_MS_FIELD_NUMBER: _ClassVar[int]
    group: str
    token: str
    expires_in_ms: int
    def __init__(self, group: _Optional[str] = ..., token: _Optional[str] = ..., expires_in_ms: _Optional[int] = ...) -> None: ...

class RenewLeaseRequest(_message.Message):
    __slots__ = ("token", "requested_duration_ms")
    TOKEN_FIELD_NUMBER: _ClassVar[int]
    REQUESTED_DURATION_MS_FIELD_NUMBER: _ClassVar[int]
    token: str
    requested_duration_ms: int
    def __init__(self, token: _Optional[str] = ..., requested_duration_ms: _Optional[int] = ...) -> None: ...

class RenewLeaseResponse(_message.Message):
    __slots__ = ("group", "token", "expires_in_ms")
    GROUP_FIELD_NUMBER: _ClassVar[int]
    TOKEN_FIELD_NUMBER: _ClassVar[int]
    EXPIRES_IN_MS_FIELD_NUMBER: _ClassVar[int]
    group: str
    token: str
    expires_in_ms: int
    def __init__(self, group: _Optional[str] = ..., token: _Optional[str] = ..., expires_in_ms: _Optional[int] = ...) -> None: ...

class ReleaseLeaseRequest(_message.Message):
    __slots__ = ("token",)
    TOKEN_FIELD_NUMBER: _ClassVar[int]
    token: str
    def __init__(self, token: _Optional[str] = ...) -> None: ...

class ReleaseLeaseResponse(_message.Message):
    __slots__ = ("state",)
    STATE_FIELD_NUMBER: _ClassVar[int]
    state: GroupState
    def __init__(self, state: _Optional[_Union[GroupState, str]] = ...) -> None: ...

class EnableGroupRequest(_message.Message):
    __slots__ = ("group", "lease_token")
    GROUP_FIELD_NUMBER: _ClassVar[int]
    LEASE_TOKEN_FIELD_NUMBER: _ClassVar[int]
    group: str
    lease_token: str
    def __init__(self, group: _Optional[str] = ..., lease_token: _Optional[str] = ...) -> None: ...

class EnableGroupResponse(_message.Message):
    __slots__ = ("state",)
    STATE_FIELD_NUMBER: _ClassVar[int]
    state: GroupState
    def __init__(self, state: _Optional[_Union[GroupState, str]] = ...) -> None: ...

class StartControlRequest(_message.Message):
    __slots__ = ("group", "lease_token", "valid_for_ms", "commands")
    GROUP_FIELD_NUMBER: _ClassVar[int]
    LEASE_TOKEN_FIELD_NUMBER: _ClassVar[int]
    VALID_FOR_MS_FIELD_NUMBER: _ClassVar[int]
    COMMANDS_FIELD_NUMBER: _ClassVar[int]
    group: str
    lease_token: str
    valid_for_ms: int
    commands: _containers.RepeatedCompositeFieldContainer[MotorCommand]
    def __init__(self, group: _Optional[str] = ..., lease_token: _Optional[str] = ..., valid_for_ms: _Optional[int] = ..., commands: _Optional[_Iterable[_Union[MotorCommand, _Mapping]]] = ...) -> None: ...

class StartControlResponse(_message.Message):
    __slots__ = ("state",)
    STATE_FIELD_NUMBER: _ClassVar[int]
    state: GroupState
    def __init__(self, state: _Optional[_Union[GroupState, str]] = ...) -> None: ...

class StopControlRequest(_message.Message):
    __slots__ = ("group", "lease_token")
    GROUP_FIELD_NUMBER: _ClassVar[int]
    LEASE_TOKEN_FIELD_NUMBER: _ClassVar[int]
    group: str
    lease_token: str
    def __init__(self, group: _Optional[str] = ..., lease_token: _Optional[str] = ...) -> None: ...

class StopControlResponse(_message.Message):
    __slots__ = ("state",)
    STATE_FIELD_NUMBER: _ClassVar[int]
    state: GroupState
    def __init__(self, state: _Optional[_Union[GroupState, str]] = ...) -> None: ...

class DisableGroupRequest(_message.Message):
    __slots__ = ("group", "lease_token")
    GROUP_FIELD_NUMBER: _ClassVar[int]
    LEASE_TOKEN_FIELD_NUMBER: _ClassVar[int]
    group: str
    lease_token: str
    def __init__(self, group: _Optional[str] = ..., lease_token: _Optional[str] = ...) -> None: ...

class DisableGroupResponse(_message.Message):
    __slots__ = ("state",)
    STATE_FIELD_NUMBER: _ClassVar[int]
    state: GroupState
    def __init__(self, state: _Optional[_Union[GroupState, str]] = ...) -> None: ...

class RecoverGroupRequest(_message.Message):
    __slots__ = ("group", "lease_token")
    GROUP_FIELD_NUMBER: _ClassVar[int]
    LEASE_TOKEN_FIELD_NUMBER: _ClassVar[int]
    group: str
    lease_token: str
    def __init__(self, group: _Optional[str] = ..., lease_token: _Optional[str] = ...) -> None: ...

class RecoverGroupResponse(_message.Message):
    __slots__ = ("state",)
    STATE_FIELD_NUMBER: _ClassVar[int]
    state: GroupState
    def __init__(self, state: _Optional[_Union[GroupState, str]] = ...) -> None: ...

class MotorCommand(_message.Message):
    __slots__ = ("motor_name", "position", "velocity", "torque_feedforward", "kp", "kd")
    MOTOR_NAME_FIELD_NUMBER: _ClassVar[int]
    POSITION_FIELD_NUMBER: _ClassVar[int]
    VELOCITY_FIELD_NUMBER: _ClassVar[int]
    TORQUE_FEEDFORWARD_FIELD_NUMBER: _ClassVar[int]
    KP_FIELD_NUMBER: _ClassVar[int]
    KD_FIELD_NUMBER: _ClassVar[int]
    motor_name: str
    position: float
    velocity: float
    torque_feedforward: float
    kp: float
    kd: float
    def __init__(self, motor_name: _Optional[str] = ..., position: _Optional[float] = ..., velocity: _Optional[float] = ..., torque_feedforward: _Optional[float] = ..., kp: _Optional[float] = ..., kd: _Optional[float] = ...) -> None: ...

class StreamCommandsRequest(_message.Message):
    __slots__ = ("group", "lease_token", "sequence", "valid_for_ms", "commands")
    GROUP_FIELD_NUMBER: _ClassVar[int]
    LEASE_TOKEN_FIELD_NUMBER: _ClassVar[int]
    SEQUENCE_FIELD_NUMBER: _ClassVar[int]
    VALID_FOR_MS_FIELD_NUMBER: _ClassVar[int]
    COMMANDS_FIELD_NUMBER: _ClassVar[int]
    group: str
    lease_token: str
    sequence: int
    valid_for_ms: int
    commands: _containers.RepeatedCompositeFieldContainer[MotorCommand]
    def __init__(self, group: _Optional[str] = ..., lease_token: _Optional[str] = ..., sequence: _Optional[int] = ..., valid_for_ms: _Optional[int] = ..., commands: _Optional[_Iterable[_Union[MotorCommand, _Mapping]]] = ...) -> None: ...

class StreamCommandsResponse(_message.Message):
    __slots__ = ("group", "sequence", "result", "detail", "state")
    GROUP_FIELD_NUMBER: _ClassVar[int]
    SEQUENCE_FIELD_NUMBER: _ClassVar[int]
    RESULT_FIELD_NUMBER: _ClassVar[int]
    DETAIL_FIELD_NUMBER: _ClassVar[int]
    STATE_FIELD_NUMBER: _ClassVar[int]
    group: str
    sequence: int
    result: CommandResult
    detail: str
    state: GroupState
    def __init__(self, group: _Optional[str] = ..., sequence: _Optional[int] = ..., result: _Optional[_Union[CommandResult, str]] = ..., detail: _Optional[str] = ..., state: _Optional[_Union[GroupState, str]] = ...) -> None: ...

class WatchTelemetryRequest(_message.Message):
    __slots__ = ("groups", "include_motors", "include_imu", "include_health", "imus", "include_command_policy_events")
    GROUPS_FIELD_NUMBER: _ClassVar[int]
    INCLUDE_MOTORS_FIELD_NUMBER: _ClassVar[int]
    INCLUDE_IMU_FIELD_NUMBER: _ClassVar[int]
    INCLUDE_HEALTH_FIELD_NUMBER: _ClassVar[int]
    IMUS_FIELD_NUMBER: _ClassVar[int]
    INCLUDE_COMMAND_POLICY_EVENTS_FIELD_NUMBER: _ClassVar[int]
    groups: _containers.RepeatedScalarFieldContainer[str]
    include_motors: bool
    include_imu: bool
    include_health: bool
    imus: _containers.RepeatedScalarFieldContainer[str]
    include_command_policy_events: bool
    def __init__(self, groups: _Optional[_Iterable[str]] = ..., include_motors: _Optional[bool] = ..., include_imu: _Optional[bool] = ..., include_health: _Optional[bool] = ..., imus: _Optional[_Iterable[str]] = ..., include_command_policy_events: _Optional[bool] = ...) -> None: ...

class MotorTelemetry(_message.Message):
    __slots__ = ("group", "motor_name", "position", "velocity", "torque", "temperature", "fault_bits", "mode_status", "feedback_age_ms")
    GROUP_FIELD_NUMBER: _ClassVar[int]
    MOTOR_NAME_FIELD_NUMBER: _ClassVar[int]
    POSITION_FIELD_NUMBER: _ClassVar[int]
    VELOCITY_FIELD_NUMBER: _ClassVar[int]
    TORQUE_FIELD_NUMBER: _ClassVar[int]
    TEMPERATURE_FIELD_NUMBER: _ClassVar[int]
    FAULT_BITS_FIELD_NUMBER: _ClassVar[int]
    MODE_STATUS_FIELD_NUMBER: _ClassVar[int]
    FEEDBACK_AGE_MS_FIELD_NUMBER: _ClassVar[int]
    group: str
    motor_name: str
    position: float
    velocity: float
    torque: float
    temperature: float
    fault_bits: int
    mode_status: int
    feedback_age_ms: int
    def __init__(self, group: _Optional[str] = ..., motor_name: _Optional[str] = ..., position: _Optional[float] = ..., velocity: _Optional[float] = ..., torque: _Optional[float] = ..., temperature: _Optional[float] = ..., fault_bits: _Optional[int] = ..., mode_status: _Optional[int] = ..., feedback_age_ms: _Optional[int] = ...) -> None: ...

class ImuTelemetry(_message.Message):
    __slots__ = ("imu_name", "orientation", "angular_velocity", "linear_acceleration", "device_timestamp_ms", "sample_age_ms")
    IMU_NAME_FIELD_NUMBER: _ClassVar[int]
    ORIENTATION_FIELD_NUMBER: _ClassVar[int]
    ANGULAR_VELOCITY_FIELD_NUMBER: _ClassVar[int]
    LINEAR_ACCELERATION_FIELD_NUMBER: _ClassVar[int]
    DEVICE_TIMESTAMP_MS_FIELD_NUMBER: _ClassVar[int]
    SAMPLE_AGE_MS_FIELD_NUMBER: _ClassVar[int]
    imu_name: str
    orientation: Quaternion
    angular_velocity: Vector3
    linear_acceleration: Vector3
    device_timestamp_ms: int
    sample_age_ms: int
    def __init__(self, imu_name: _Optional[str] = ..., orientation: _Optional[_Union[Quaternion, _Mapping]] = ..., angular_velocity: _Optional[_Union[Vector3, _Mapping]] = ..., linear_acceleration: _Optional[_Union[Vector3, _Mapping]] = ..., device_timestamp_ms: _Optional[int] = ..., sample_age_ms: _Optional[int] = ...) -> None: ...

class Quaternion(_message.Message):
    __slots__ = ("x", "y", "z", "w")
    X_FIELD_NUMBER: _ClassVar[int]
    Y_FIELD_NUMBER: _ClassVar[int]
    Z_FIELD_NUMBER: _ClassVar[int]
    W_FIELD_NUMBER: _ClassVar[int]
    x: float
    y: float
    z: float
    w: float
    def __init__(self, x: _Optional[float] = ..., y: _Optional[float] = ..., z: _Optional[float] = ..., w: _Optional[float] = ...) -> None: ...

class Vector3(_message.Message):
    __slots__ = ("x", "y", "z")
    X_FIELD_NUMBER: _ClassVar[int]
    Y_FIELD_NUMBER: _ClassVar[int]
    Z_FIELD_NUMBER: _ClassVar[int]
    x: float
    y: float
    z: float
    def __init__(self, x: _Optional[float] = ..., y: _Optional[float] = ..., z: _Optional[float] = ...) -> None: ...

class GroupTelemetry(_message.Message):
    __slots__ = ("group", "state", "command_count", "control_tick", "jitter_us", "detail", "cause")
    GROUP_FIELD_NUMBER: _ClassVar[int]
    STATE_FIELD_NUMBER: _ClassVar[int]
    COMMAND_COUNT_FIELD_NUMBER: _ClassVar[int]
    CONTROL_TICK_FIELD_NUMBER: _ClassVar[int]
    JITTER_US_FIELD_NUMBER: _ClassVar[int]
    DETAIL_FIELD_NUMBER: _ClassVar[int]
    CAUSE_FIELD_NUMBER: _ClassVar[int]
    group: str
    state: GroupState
    command_count: int
    control_tick: int
    jitter_us: int
    detail: str
    cause: GroupCause
    def __init__(self, group: _Optional[str] = ..., state: _Optional[_Union[GroupState, str]] = ..., command_count: _Optional[int] = ..., control_tick: _Optional[int] = ..., jitter_us: _Optional[int] = ..., detail: _Optional[str] = ..., cause: _Optional[_Union[GroupCause, str]] = ...) -> None: ...

class CommandPolicyTelemetry(_message.Message):
    __slots__ = ("group", "motor_name", "check", "behavior", "detail", "command_modified", "command_rejected", "fault_response")
    GROUP_FIELD_NUMBER: _ClassVar[int]
    MOTOR_NAME_FIELD_NUMBER: _ClassVar[int]
    CHECK_FIELD_NUMBER: _ClassVar[int]
    BEHAVIOR_FIELD_NUMBER: _ClassVar[int]
    DETAIL_FIELD_NUMBER: _ClassVar[int]
    COMMAND_MODIFIED_FIELD_NUMBER: _ClassVar[int]
    COMMAND_REJECTED_FIELD_NUMBER: _ClassVar[int]
    FAULT_RESPONSE_FIELD_NUMBER: _ClassVar[int]
    group: str
    motor_name: str
    check: CommandPolicyCheck
    behavior: CommandBehavior
    detail: str
    command_modified: bool
    command_rejected: bool
    fault_response: FaultResponse
    def __init__(self, group: _Optional[str] = ..., motor_name: _Optional[str] = ..., check: _Optional[_Union[CommandPolicyCheck, str]] = ..., behavior: _Optional[_Union[CommandBehavior, str]] = ..., detail: _Optional[str] = ..., command_modified: _Optional[bool] = ..., command_rejected: _Optional[bool] = ..., fault_response: _Optional[_Union[FaultResponse, str]] = ...) -> None: ...

class HealthTelemetry(_message.Message):
    __slots__ = ("component", "device_status", "dropped_events")
    COMPONENT_FIELD_NUMBER: _ClassVar[int]
    DEVICE_STATUS_FIELD_NUMBER: _ClassVar[int]
    DROPPED_EVENTS_FIELD_NUMBER: _ClassVar[int]
    component: str
    device_status: DeviceStatus
    dropped_events: int
    def __init__(self, component: _Optional[str] = ..., device_status: _Optional[_Union[DeviceStatus, _Mapping]] = ..., dropped_events: _Optional[int] = ...) -> None: ...

class WatchTelemetryResponse(_message.Message):
    __slots__ = ("monotonic_ns", "motor", "imu", "group", "health", "command_policy")
    MONOTONIC_NS_FIELD_NUMBER: _ClassVar[int]
    MOTOR_FIELD_NUMBER: _ClassVar[int]
    IMU_FIELD_NUMBER: _ClassVar[int]
    GROUP_FIELD_NUMBER: _ClassVar[int]
    HEALTH_FIELD_NUMBER: _ClassVar[int]
    COMMAND_POLICY_FIELD_NUMBER: _ClassVar[int]
    monotonic_ns: int
    motor: MotorTelemetry
    imu: ImuTelemetry
    group: GroupTelemetry
    health: HealthTelemetry
    command_policy: CommandPolicyTelemetry
    def __init__(self, monotonic_ns: _Optional[int] = ..., motor: _Optional[_Union[MotorTelemetry, _Mapping]] = ..., imu: _Optional[_Union[ImuTelemetry, _Mapping]] = ..., group: _Optional[_Union[GroupTelemetry, _Mapping]] = ..., health: _Optional[_Union[HealthTelemetry, _Mapping]] = ..., command_policy: _Optional[_Union[CommandPolicyTelemetry, _Mapping]] = ...) -> None: ...
