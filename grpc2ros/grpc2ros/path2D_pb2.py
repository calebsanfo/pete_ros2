# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: path2D.proto
# Protobuf Python Version: 5.26.1
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import symbol_database as _symbol_database
from google.protobuf.internal import builder as _builder
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from google.protobuf import timestamp_pb2 as google_dot_protobuf_dot_timestamp__pb2


DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n\x0cpath2D.proto\x12\x08ros2grpc\x1a\x1fgoogle/protobuf/timestamp.proto\"-\n\x06Pose2D\x12\t\n\x01x\x18\x01 \x01(\x01\x12\t\n\x01y\x18\x02 \x01(\x01\x12\r\n\x05theta\x18\x03 \x01(\x01\"O\n\x0bPoseStamped\x12 \n\x06header\x18\x01 \x01(\x0b\x32\x10.ros2grpc.Header\x12\x1e\n\x04pose\x18\x02 \x01(\x0b\x32\x10.ros2grpc.Pose2D\"R\n\x06Header\x12\x0b\n\x03seq\x18\x01 \x01(\r\x12)\n\x05stamp\x18\x02 \x01(\x0b\x32\x1a.google.protobuf.Timestamp\x12\x10\n\x08\x66rame_id\x18\x03 \x01(\t\".\n\x06Path2D\x12$\n\x05poses\x18\x01 \x03(\x0b\x32\x15.ros2grpc.PoseStamped\"/\n\x0b\x41\x63knowledge\x12\x0f\n\x07success\x18\x01 \x01(\x08\x12\x0f\n\x07message\x18\x02 \x01(\t2B\n\x0bPathService\x12\x33\n\x08SendPath\x12\x10.ros2grpc.Path2D\x1a\x15.ros2grpc.Acknowledgeb\x06proto3')

_globals = globals()
_builder.BuildMessageAndEnumDescriptors(DESCRIPTOR, _globals)
_builder.BuildTopDescriptorsAndMessages(DESCRIPTOR, 'path2D_pb2', _globals)
if not _descriptor._USE_C_DESCRIPTORS:
  DESCRIPTOR._loaded_options = None
  _globals['_POSE2D']._serialized_start=59
  _globals['_POSE2D']._serialized_end=104
  _globals['_POSESTAMPED']._serialized_start=106
  _globals['_POSESTAMPED']._serialized_end=185
  _globals['_HEADER']._serialized_start=187
  _globals['_HEADER']._serialized_end=269
  _globals['_PATH2D']._serialized_start=271
  _globals['_PATH2D']._serialized_end=317
  _globals['_ACKNOWLEDGE']._serialized_start=319
  _globals['_ACKNOWLEDGE']._serialized_end=366
  _globals['_PATHSERVICE']._serialized_start=368
  _globals['_PATHSERVICE']._serialized_end=434
# @@protoc_insertion_point(module_scope)