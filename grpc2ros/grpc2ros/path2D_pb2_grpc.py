# Generated by the gRPC Python protocol compiler plugin. DO NOT EDIT!
"""Client and server classes corresponding to protobuf-defined services."""
import grpc
import warnings

import path2D_pb2 as path2D__pb2

GRPC_GENERATED_VERSION = '1.64.0'
GRPC_VERSION = grpc.__version__
EXPECTED_ERROR_RELEASE = '1.65.0'
SCHEDULED_RELEASE_DATE = 'June 25, 2024'
_version_not_supported = False

try:
    from grpc._utilities import first_version_is_lower
    _version_not_supported = first_version_is_lower(GRPC_VERSION, GRPC_GENERATED_VERSION)
except ImportError:
    _version_not_supported = True

if _version_not_supported:
    warnings.warn(
        f'The grpc package installed is at version {GRPC_VERSION},'
        + f' but the generated code in path2D_pb2_grpc.py depends on'
        + f' grpcio>={GRPC_GENERATED_VERSION}.'
        + f' Please upgrade your grpc module to grpcio>={GRPC_GENERATED_VERSION}'
        + f' or downgrade your generated code using grpcio-tools<={GRPC_VERSION}.'
        + f' This warning will become an error in {EXPECTED_ERROR_RELEASE},'
        + f' scheduled for release on {SCHEDULED_RELEASE_DATE}.',
        RuntimeWarning
    )


class PathServiceStub(object):
    """The gRPC service definition
    """

    def __init__(self, channel):
        """Constructor.

        Args:
            channel: A grpc.Channel.
        """
        self.SendPath = channel.unary_unary(
                '/ros2grpc.PathService/SendPath',
                request_serializer=path2D__pb2.Path2D.SerializeToString,
                response_deserializer=path2D__pb2.Acknowledge.FromString,
                _registered_method=True)


class PathServiceServicer(object):
    """The gRPC service definition
    """

    def SendPath(self, request, context):
        """RPC method to send a Path2D
        """
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')


def add_PathServiceServicer_to_server(servicer, server):
    rpc_method_handlers = {
            'SendPath': grpc.unary_unary_rpc_method_handler(
                    servicer.SendPath,
                    request_deserializer=path2D__pb2.Path2D.FromString,
                    response_serializer=path2D__pb2.Acknowledge.SerializeToString,
            ),
    }
    generic_handler = grpc.method_handlers_generic_handler(
            'ros2grpc.PathService', rpc_method_handlers)
    server.add_generic_rpc_handlers((generic_handler,))
    server.add_registered_method_handlers('ros2grpc.PathService', rpc_method_handlers)


 # This class is part of an EXPERIMENTAL API.
class PathService(object):
    """The gRPC service definition
    """

    @staticmethod
    def SendPath(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(
            request,
            target,
            '/ros2grpc.PathService/SendPath',
            path2D__pb2.Path2D.SerializeToString,
            path2D__pb2.Acknowledge.FromString,
            options,
            channel_credentials,
            insecure,
            call_credentials,
            compression,
            wait_for_ready,
            timeout,
            metadata,
            _registered_method=True)
