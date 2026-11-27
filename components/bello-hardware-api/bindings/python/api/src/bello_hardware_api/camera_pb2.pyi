# mypy: ignore-errors
from google.protobuf.internal import containers as _containers
from google.protobuf.internal import enum_type_wrapper as _enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from collections.abc import Iterable as _Iterable, Mapping as _Mapping
from typing import ClassVar as _ClassVar, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class CameraLayout(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = ()
    CAMERA_LAYOUT_UNSPECIFIED: _ClassVar[CameraLayout]
    CAMERA_LAYOUT_MONO: _ClassVar[CameraLayout]
    CAMERA_LAYOUT_STEREO_SIDE_BY_SIDE: _ClassVar[CameraLayout]
CAMERA_LAYOUT_UNSPECIFIED: CameraLayout
CAMERA_LAYOUT_MONO: CameraLayout
CAMERA_LAYOUT_STEREO_SIDE_BY_SIDE: CameraLayout

class GetCamerasRequest(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class Camera(_message.Message):
    __slots__ = ("name", "width", "height", "fps", "source", "metadata_json")
    NAME_FIELD_NUMBER: _ClassVar[int]
    WIDTH_FIELD_NUMBER: _ClassVar[int]
    HEIGHT_FIELD_NUMBER: _ClassVar[int]
    FPS_FIELD_NUMBER: _ClassVar[int]
    SOURCE_FIELD_NUMBER: _ClassVar[int]
    METADATA_JSON_FIELD_NUMBER: _ClassVar[int]
    name: str
    width: int
    height: int
    fps: int
    source: str
    metadata_json: str
    def __init__(self, name: _Optional[str] = ..., width: _Optional[int] = ..., height: _Optional[int] = ..., fps: _Optional[int] = ..., source: _Optional[str] = ..., metadata_json: _Optional[str] = ...) -> None: ...

class LocalCameraSource(_message.Message):
    __slots__ = ("layout", "unixfd_socket")
    LAYOUT_FIELD_NUMBER: _ClassVar[int]
    UNIXFD_SOCKET_FIELD_NUMBER: _ClassVar[int]
    layout: CameraLayout
    unixfd_socket: str
    def __init__(self, layout: _Optional[_Union[CameraLayout, str]] = ..., unixfd_socket: _Optional[str] = ...) -> None: ...

class GetCamerasResponse(_message.Message):
    __slots__ = ("cameras", "local_sources")
    class LocalSourcesEntry(_message.Message):
        __slots__ = ("key", "value")
        KEY_FIELD_NUMBER: _ClassVar[int]
        VALUE_FIELD_NUMBER: _ClassVar[int]
        key: str
        value: LocalCameraSource
        def __init__(self, key: _Optional[str] = ..., value: _Optional[_Union[LocalCameraSource, _Mapping]] = ...) -> None: ...
    CAMERAS_FIELD_NUMBER: _ClassVar[int]
    LOCAL_SOURCES_FIELD_NUMBER: _ClassVar[int]
    cameras: _containers.RepeatedCompositeFieldContainer[Camera]
    local_sources: _containers.MessageMap[str, LocalCameraSource]
    def __init__(self, cameras: _Optional[_Iterable[_Union[Camera, _Mapping]]] = ..., local_sources: _Optional[_Mapping[str, LocalCameraSource]] = ...) -> None: ...

class WatchFramesRequest(_message.Message):
    __slots__ = ("name", "latest_only")
    NAME_FIELD_NUMBER: _ClassVar[int]
    LATEST_ONLY_FIELD_NUMBER: _ClassVar[int]
    name: str
    latest_only: bool
    def __init__(self, name: _Optional[str] = ..., latest_only: _Optional[bool] = ...) -> None: ...

class Frame(_message.Message):
    __slots__ = ("name", "sequence", "capture_ns", "received_ns", "jpeg", "flags", "simulation_time_ns")
    NAME_FIELD_NUMBER: _ClassVar[int]
    SEQUENCE_FIELD_NUMBER: _ClassVar[int]
    CAPTURE_NS_FIELD_NUMBER: _ClassVar[int]
    RECEIVED_NS_FIELD_NUMBER: _ClassVar[int]
    JPEG_FIELD_NUMBER: _ClassVar[int]
    FLAGS_FIELD_NUMBER: _ClassVar[int]
    SIMULATION_TIME_NS_FIELD_NUMBER: _ClassVar[int]
    name: str
    sequence: int
    capture_ns: int
    received_ns: int
    jpeg: bytes
    flags: int
    simulation_time_ns: int
    def __init__(self, name: _Optional[str] = ..., sequence: _Optional[int] = ..., capture_ns: _Optional[int] = ..., received_ns: _Optional[int] = ..., jpeg: _Optional[bytes] = ..., flags: _Optional[int] = ..., simulation_time_ns: _Optional[int] = ...) -> None: ...

class WatchFramesResponse(_message.Message):
    __slots__ = ("frame",)
    FRAME_FIELD_NUMBER: _ClassVar[int]
    frame: Frame
    def __init__(self, frame: _Optional[_Union[Frame, _Mapping]] = ...) -> None: ...

class CreateSessionRequest(_message.Message):
    __slots__ = ("sdp",)
    SDP_FIELD_NUMBER: _ClassVar[int]
    sdp: str
    def __init__(self, sdp: _Optional[str] = ...) -> None: ...

class CreateSessionResponse(_message.Message):
    __slots__ = ("session", "sdp")
    SESSION_FIELD_NUMBER: _ClassVar[int]
    SDP_FIELD_NUMBER: _ClassVar[int]
    session: str
    sdp: str
    def __init__(self, session: _Optional[str] = ..., sdp: _Optional[str] = ...) -> None: ...

class CloseSessionRequest(_message.Message):
    __slots__ = ("session",)
    SESSION_FIELD_NUMBER: _ClassVar[int]
    session: str
    def __init__(self, session: _Optional[str] = ...) -> None: ...

class CloseSessionResponse(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...
