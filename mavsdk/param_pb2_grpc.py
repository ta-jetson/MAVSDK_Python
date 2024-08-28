# Generated by the gRPC Python protocol compiler plugin. DO NOT EDIT!
"""Client and server classes corresponding to protobuf-defined services."""
import grpc

from . import param_pb2 as param_dot_param__pb2


class ParamServiceStub(object):
    """Provide raw access to get and set parameters.
    """

    def __init__(self, channel):
        """Constructor.

        Args:
            channel: A grpc.Channel.
        """
        self.GetParamInt = channel.unary_unary(
                '/mavsdk.rpc.param.ParamService/GetParamInt',
                request_serializer=param_dot_param__pb2.GetParamIntRequest.SerializeToString,
                response_deserializer=param_dot_param__pb2.GetParamIntResponse.FromString,
                )
        self.SetParamInt = channel.unary_unary(
                '/mavsdk.rpc.param.ParamService/SetParamInt',
                request_serializer=param_dot_param__pb2.SetParamIntRequest.SerializeToString,
                response_deserializer=param_dot_param__pb2.SetParamIntResponse.FromString,
                )
        self.GetParamFloat = channel.unary_unary(
                '/mavsdk.rpc.param.ParamService/GetParamFloat',
                request_serializer=param_dot_param__pb2.GetParamFloatRequest.SerializeToString,
                response_deserializer=param_dot_param__pb2.GetParamFloatResponse.FromString,
                )
        self.SetParamFloat = channel.unary_unary(
                '/mavsdk.rpc.param.ParamService/SetParamFloat',
                request_serializer=param_dot_param__pb2.SetParamFloatRequest.SerializeToString,
                response_deserializer=param_dot_param__pb2.SetParamFloatResponse.FromString,
                )
        self.GetParamCustom = channel.unary_unary(
                '/mavsdk.rpc.param.ParamService/GetParamCustom',
                request_serializer=param_dot_param__pb2.GetParamCustomRequest.SerializeToString,
                response_deserializer=param_dot_param__pb2.GetParamCustomResponse.FromString,
                )
        self.SetParamCustom = channel.unary_unary(
                '/mavsdk.rpc.param.ParamService/SetParamCustom',
                request_serializer=param_dot_param__pb2.SetParamCustomRequest.SerializeToString,
                response_deserializer=param_dot_param__pb2.SetParamCustomResponse.FromString,
                )
        self.GetAllParams = channel.unary_unary(
                '/mavsdk.rpc.param.ParamService/GetAllParams',
                request_serializer=param_dot_param__pb2.GetAllParamsRequest.SerializeToString,
                response_deserializer=param_dot_param__pb2.GetAllParamsResponse.FromString,
                )
        self.SelectComponent = channel.unary_unary(
                '/mavsdk.rpc.param.ParamService/SelectComponent',
                request_serializer=param_dot_param__pb2.SelectComponentRequest.SerializeToString,
                response_deserializer=param_dot_param__pb2.SelectComponentResponse.FromString,
                )


class ParamServiceServicer(object):
    """Provide raw access to get and set parameters.
    """

    def GetParamInt(self, request, context):
        """
        Get an int parameter.

        If the type is wrong, the result will be `WRONG_TYPE`.
        """
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def SetParamInt(self, request, context):
        """
        Set an int parameter.

        If the type is wrong, the result will be `WRONG_TYPE`.
        """
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def GetParamFloat(self, request, context):
        """
        Get a float parameter.

        If the type is wrong, the result will be `WRONG_TYPE`.
        """
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def SetParamFloat(self, request, context):
        """
        Set a float parameter.

        If the type is wrong, the result will be `WRONG_TYPE`.
        """
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def GetParamCustom(self, request, context):
        """
        Get a custom parameter.

        If the type is wrong, the result will be `WRONG_TYPE`.
        """
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def SetParamCustom(self, request, context):
        """
        Set a custom parameter.

        If the type is wrong, the result will be `WRONG_TYPE`.
        """
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def GetAllParams(self, request, context):
        """
        Get all parameters.
        """
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def SelectComponent(self, request, context):
        """
        Select component ID of parameter component to talk to and param protocol version.

        Default is the autopilot component (1), and Version (0).
        """
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')


def add_ParamServiceServicer_to_server(servicer, server):
    rpc_method_handlers = {
            'GetParamInt': grpc.unary_unary_rpc_method_handler(
                    servicer.GetParamInt,
                    request_deserializer=param_dot_param__pb2.GetParamIntRequest.FromString,
                    response_serializer=param_dot_param__pb2.GetParamIntResponse.SerializeToString,
            ),
            'SetParamInt': grpc.unary_unary_rpc_method_handler(
                    servicer.SetParamInt,
                    request_deserializer=param_dot_param__pb2.SetParamIntRequest.FromString,
                    response_serializer=param_dot_param__pb2.SetParamIntResponse.SerializeToString,
            ),
            'GetParamFloat': grpc.unary_unary_rpc_method_handler(
                    servicer.GetParamFloat,
                    request_deserializer=param_dot_param__pb2.GetParamFloatRequest.FromString,
                    response_serializer=param_dot_param__pb2.GetParamFloatResponse.SerializeToString,
            ),
            'SetParamFloat': grpc.unary_unary_rpc_method_handler(
                    servicer.SetParamFloat,
                    request_deserializer=param_dot_param__pb2.SetParamFloatRequest.FromString,
                    response_serializer=param_dot_param__pb2.SetParamFloatResponse.SerializeToString,
            ),
            'GetParamCustom': grpc.unary_unary_rpc_method_handler(
                    servicer.GetParamCustom,
                    request_deserializer=param_dot_param__pb2.GetParamCustomRequest.FromString,
                    response_serializer=param_dot_param__pb2.GetParamCustomResponse.SerializeToString,
            ),
            'SetParamCustom': grpc.unary_unary_rpc_method_handler(
                    servicer.SetParamCustom,
                    request_deserializer=param_dot_param__pb2.SetParamCustomRequest.FromString,
                    response_serializer=param_dot_param__pb2.SetParamCustomResponse.SerializeToString,
            ),
            'GetAllParams': grpc.unary_unary_rpc_method_handler(
                    servicer.GetAllParams,
                    request_deserializer=param_dot_param__pb2.GetAllParamsRequest.FromString,
                    response_serializer=param_dot_param__pb2.GetAllParamsResponse.SerializeToString,
            ),
            'SelectComponent': grpc.unary_unary_rpc_method_handler(
                    servicer.SelectComponent,
                    request_deserializer=param_dot_param__pb2.SelectComponentRequest.FromString,
                    response_serializer=param_dot_param__pb2.SelectComponentResponse.SerializeToString,
            ),
    }
    generic_handler = grpc.method_handlers_generic_handler(
            'mavsdk.rpc.param.ParamService', rpc_method_handlers)
    server.add_generic_rpc_handlers((generic_handler,))


 # This class is part of an EXPERIMENTAL API.
class ParamService(object):
    """Provide raw access to get and set parameters.
    """

    @staticmethod
    def GetParamInt(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/mavsdk.rpc.param.ParamService/GetParamInt',
            param_dot_param__pb2.GetParamIntRequest.SerializeToString,
            param_dot_param__pb2.GetParamIntResponse.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)

    @staticmethod
    def SetParamInt(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/mavsdk.rpc.param.ParamService/SetParamInt',
            param_dot_param__pb2.SetParamIntRequest.SerializeToString,
            param_dot_param__pb2.SetParamIntResponse.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)

    @staticmethod
    def GetParamFloat(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/mavsdk.rpc.param.ParamService/GetParamFloat',
            param_dot_param__pb2.GetParamFloatRequest.SerializeToString,
            param_dot_param__pb2.GetParamFloatResponse.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)

    @staticmethod
    def SetParamFloat(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/mavsdk.rpc.param.ParamService/SetParamFloat',
            param_dot_param__pb2.SetParamFloatRequest.SerializeToString,
            param_dot_param__pb2.SetParamFloatResponse.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)

    @staticmethod
    def GetParamCustom(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/mavsdk.rpc.param.ParamService/GetParamCustom',
            param_dot_param__pb2.GetParamCustomRequest.SerializeToString,
            param_dot_param__pb2.GetParamCustomResponse.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)

    @staticmethod
    def SetParamCustom(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/mavsdk.rpc.param.ParamService/SetParamCustom',
            param_dot_param__pb2.SetParamCustomRequest.SerializeToString,
            param_dot_param__pb2.SetParamCustomResponse.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)

    @staticmethod
    def GetAllParams(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/mavsdk.rpc.param.ParamService/GetAllParams',
            param_dot_param__pb2.GetAllParamsRequest.SerializeToString,
            param_dot_param__pb2.GetAllParamsResponse.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)

    @staticmethod
    def SelectComponent(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/mavsdk.rpc.param.ParamService/SelectComponent',
            param_dot_param__pb2.SelectComponentRequest.SerializeToString,
            param_dot_param__pb2.SelectComponentResponse.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)
