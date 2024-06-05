from google.protobuf import timestamp_pb2 as _timestamp_pb2
from google.protobuf.internal import containers as _containers
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Iterable as _Iterable, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class Pose2D(_message.Message):
    __slots__ = ("x", "y", "theta")
    X_FIELD_NUMBER: _ClassVar[int]
    Y_FIELD_NUMBER: _ClassVar[int]
    THETA_FIELD_NUMBER: _ClassVar[int]
    x: float
    y: float
    theta: float
    def __init__(self, x: _Optional[float] = ..., y: _Optional[float] = ..., theta: _Optional[float] = ...) -> None: ...

class PoseStamped(_message.Message):
    __slots__ = ("header", "pose")
    HEADER_FIELD_NUMBER: _ClassVar[int]
    POSE_FIELD_NUMBER: _ClassVar[int]
    header: Header
    pose: Pose2D
    def __init__(self, header: _Optional[_Union[Header, _Mapping]] = ..., pose: _Optional[_Union[Pose2D, _Mapping]] = ...) -> None: ...

class Header(_message.Message):
    __slots__ = ("seq", "stamp", "frame_id")
    SEQ_FIELD_NUMBER: _ClassVar[int]
    STAMP_FIELD_NUMBER: _ClassVar[int]
    FRAME_ID_FIELD_NUMBER: _ClassVar[int]
    seq: int
    stamp: _timestamp_pb2.Timestamp
    frame_id: str
    def __init__(self, seq: _Optional[int] = ..., stamp: _Optional[_Union[_timestamp_pb2.Timestamp, _Mapping]] = ..., frame_id: _Optional[str] = ...) -> None: ...

class Path2D(_message.Message):
    __slots__ = ("poses",)
    POSES_FIELD_NUMBER: _ClassVar[int]
    poses: _containers.RepeatedCompositeFieldContainer[PoseStamped]
    def __init__(self, poses: _Optional[_Iterable[_Union[PoseStamped, _Mapping]]] = ...) -> None: ...

class Acknowledge(_message.Message):
    __slots__ = ("success", "message")
    SUCCESS_FIELD_NUMBER: _ClassVar[int]
    MESSAGE_FIELD_NUMBER: _ClassVar[int]
    success: bool
    message: str
    def __init__(self, success: bool = ..., message: _Optional[str] = ...) -> None: ...
