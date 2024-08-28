# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: component_metadata/component_metadata.proto
"""Generated protocol buffer code."""
from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from . import mavsdk_options_pb2 as mavsdk__options__pb2


DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n+component_metadata/component_metadata.proto\x12\x1dmavsdk.rpc.component_metadata\x1a\x14mavsdk_options.proto\")\n\x17RequestComponentRequest\x12\x0e\n\x06\x63ompid\x18\x01 \x01(\r\"h\n\x12GetMetadataRequest\x12\x0e\n\x06\x63ompid\x18\x01 \x01(\r\x12\x42\n\rmetadata_type\x18\x02 \x01(\x0e\x32+.mavsdk.rpc.component_metadata.MetadataType\"\xaf\x01\n\x13GetMetadataResponse\x12Y\n\x19\x63omponent_metadata_result\x18\x01 \x01(\x0b\x32\x36.mavsdk.rpc.component_metadata.ComponentMetadataResult\x12=\n\x08response\x18\x02 \x01(\x0b\x32+.mavsdk.rpc.component_metadata.MetadataData\"%\n\x0cMetadataData\x12\x15\n\rjson_metadata\x18\x01 \x01(\t\"\xd4\x02\n\x17\x43omponentMetadataResult\x12M\n\x06result\x18\x01 \x01(\x0e\x32=.mavsdk.rpc.component_metadata.ComponentMetadataResult.Result\x12\x12\n\nresult_str\x18\x02 \x01(\t\"\xd5\x01\n\x06Result\x12\x12\n\x0eRESULT_SUCCESS\x10\x00\x12\x18\n\x14RESULT_NOT_AVAILABLE\x10\x01\x12\x1b\n\x17RESULT_CONNECTION_ERROR\x10\x02\x12\x16\n\x12RESULT_UNSUPPORTED\x10\x03\x12\x11\n\rRESULT_DENIED\x10\x04\x12\x11\n\rRESULT_FAILED\x10\x05\x12\x12\n\x0eRESULT_TIMEOUT\x10\x06\x12\x14\n\x10RESULT_NO_SYSTEM\x10\x07\x12\x18\n\x14RESULT_NOT_REQUESTED\x10\x08\"\x1a\n\x18RequestComponentResponse\"\"\n RequestAutopilotComponentRequest\"#\n!RequestAutopilotComponentResponse\"#\n!SubscribeMetadataAvailableRequest\"X\n\x19MetadataAvailableResponse\x12;\n\x04\x64\x61ta\x18\x01 \x01(\x0b\x32-.mavsdk.rpc.component_metadata.MetadataUpdate\"r\n\x0eMetadataUpdate\x12\x0e\n\x06\x63ompid\x18\x01 \x01(\r\x12\x39\n\x04type\x18\x02 \x01(\x0e\x32+.mavsdk.rpc.component_metadata.MetadataType\x12\x15\n\rjson_metadata\x18\x03 \x01(\t*\x83\x01\n\x0cMetadataType\x12\x1f\n\x1bMETADATA_TYPE_ALL_COMPLETED\x10\x00\x12\x1b\n\x17METADATA_TYPE_PARAMETER\x10\x01\x12\x18\n\x14METADATA_TYPE_EVENTS\x10\x02\x12\x1b\n\x17METADATA_TYPE_ACTUATORS\x10\x03\x32\xec\x04\n\x18\x43omponentMetadataService\x12\x89\x01\n\x10RequestComponent\x12\x36.mavsdk.rpc.component_metadata.RequestComponentRequest\x1a\x37.mavsdk.rpc.component_metadata.RequestComponentResponse\"\x04\x80\xb5\x18\x01\x12\xa4\x01\n\x19RequestAutopilotComponent\x12?.mavsdk.rpc.component_metadata.RequestAutopilotComponentRequest\x1a@.mavsdk.rpc.component_metadata.RequestAutopilotComponentResponse\"\x04\x80\xb5\x18\x01\x12\xa0\x01\n\x1aSubscribeMetadataAvailable\x12@.mavsdk.rpc.component_metadata.SubscribeMetadataAvailableRequest\x1a\x38.mavsdk.rpc.component_metadata.MetadataAvailableResponse\"\x04\x80\xb5\x18\x00\x30\x01\x12z\n\x0bGetMetadata\x12\x31.mavsdk.rpc.component_metadata.GetMetadataRequest\x1a\x32.mavsdk.rpc.component_metadata.GetMetadataResponse\"\x04\x80\xb5\x18\x01\x42\x36\n\x1cio.mavsdk.component_metadataB\x16\x43omponentMetadataProtob\x06proto3')

_METADATATYPE = DESCRIPTOR.enum_types_by_name['MetadataType']
MetadataType = enum_type_wrapper.EnumTypeWrapper(_METADATATYPE)
METADATA_TYPE_ALL_COMPLETED = 0
METADATA_TYPE_PARAMETER = 1
METADATA_TYPE_EVENTS = 2
METADATA_TYPE_ACTUATORS = 3


_REQUESTCOMPONENTREQUEST = DESCRIPTOR.message_types_by_name['RequestComponentRequest']
_GETMETADATAREQUEST = DESCRIPTOR.message_types_by_name['GetMetadataRequest']
_GETMETADATARESPONSE = DESCRIPTOR.message_types_by_name['GetMetadataResponse']
_METADATADATA = DESCRIPTOR.message_types_by_name['MetadataData']
_COMPONENTMETADATARESULT = DESCRIPTOR.message_types_by_name['ComponentMetadataResult']
_REQUESTCOMPONENTRESPONSE = DESCRIPTOR.message_types_by_name['RequestComponentResponse']
_REQUESTAUTOPILOTCOMPONENTREQUEST = DESCRIPTOR.message_types_by_name['RequestAutopilotComponentRequest']
_REQUESTAUTOPILOTCOMPONENTRESPONSE = DESCRIPTOR.message_types_by_name['RequestAutopilotComponentResponse']
_SUBSCRIBEMETADATAAVAILABLEREQUEST = DESCRIPTOR.message_types_by_name['SubscribeMetadataAvailableRequest']
_METADATAAVAILABLERESPONSE = DESCRIPTOR.message_types_by_name['MetadataAvailableResponse']
_METADATAUPDATE = DESCRIPTOR.message_types_by_name['MetadataUpdate']
_COMPONENTMETADATARESULT_RESULT = _COMPONENTMETADATARESULT.enum_types_by_name['Result']
RequestComponentRequest = _reflection.GeneratedProtocolMessageType('RequestComponentRequest', (_message.Message,), {
  'DESCRIPTOR' : _REQUESTCOMPONENTREQUEST,
  '__module__' : 'component_metadata.component_metadata_pb2'
  # @@protoc_insertion_point(class_scope:mavsdk.rpc.component_metadata.RequestComponentRequest)
  })
_sym_db.RegisterMessage(RequestComponentRequest)

GetMetadataRequest = _reflection.GeneratedProtocolMessageType('GetMetadataRequest', (_message.Message,), {
  'DESCRIPTOR' : _GETMETADATAREQUEST,
  '__module__' : 'component_metadata.component_metadata_pb2'
  # @@protoc_insertion_point(class_scope:mavsdk.rpc.component_metadata.GetMetadataRequest)
  })
_sym_db.RegisterMessage(GetMetadataRequest)

GetMetadataResponse = _reflection.GeneratedProtocolMessageType('GetMetadataResponse', (_message.Message,), {
  'DESCRIPTOR' : _GETMETADATARESPONSE,
  '__module__' : 'component_metadata.component_metadata_pb2'
  # @@protoc_insertion_point(class_scope:mavsdk.rpc.component_metadata.GetMetadataResponse)
  })
_sym_db.RegisterMessage(GetMetadataResponse)

MetadataData = _reflection.GeneratedProtocolMessageType('MetadataData', (_message.Message,), {
  'DESCRIPTOR' : _METADATADATA,
  '__module__' : 'component_metadata.component_metadata_pb2'
  # @@protoc_insertion_point(class_scope:mavsdk.rpc.component_metadata.MetadataData)
  })
_sym_db.RegisterMessage(MetadataData)

ComponentMetadataResult = _reflection.GeneratedProtocolMessageType('ComponentMetadataResult', (_message.Message,), {
  'DESCRIPTOR' : _COMPONENTMETADATARESULT,
  '__module__' : 'component_metadata.component_metadata_pb2'
  # @@protoc_insertion_point(class_scope:mavsdk.rpc.component_metadata.ComponentMetadataResult)
  })
_sym_db.RegisterMessage(ComponentMetadataResult)

RequestComponentResponse = _reflection.GeneratedProtocolMessageType('RequestComponentResponse', (_message.Message,), {
  'DESCRIPTOR' : _REQUESTCOMPONENTRESPONSE,
  '__module__' : 'component_metadata.component_metadata_pb2'
  # @@protoc_insertion_point(class_scope:mavsdk.rpc.component_metadata.RequestComponentResponse)
  })
_sym_db.RegisterMessage(RequestComponentResponse)

RequestAutopilotComponentRequest = _reflection.GeneratedProtocolMessageType('RequestAutopilotComponentRequest', (_message.Message,), {
  'DESCRIPTOR' : _REQUESTAUTOPILOTCOMPONENTREQUEST,
  '__module__' : 'component_metadata.component_metadata_pb2'
  # @@protoc_insertion_point(class_scope:mavsdk.rpc.component_metadata.RequestAutopilotComponentRequest)
  })
_sym_db.RegisterMessage(RequestAutopilotComponentRequest)

RequestAutopilotComponentResponse = _reflection.GeneratedProtocolMessageType('RequestAutopilotComponentResponse', (_message.Message,), {
  'DESCRIPTOR' : _REQUESTAUTOPILOTCOMPONENTRESPONSE,
  '__module__' : 'component_metadata.component_metadata_pb2'
  # @@protoc_insertion_point(class_scope:mavsdk.rpc.component_metadata.RequestAutopilotComponentResponse)
  })
_sym_db.RegisterMessage(RequestAutopilotComponentResponse)

SubscribeMetadataAvailableRequest = _reflection.GeneratedProtocolMessageType('SubscribeMetadataAvailableRequest', (_message.Message,), {
  'DESCRIPTOR' : _SUBSCRIBEMETADATAAVAILABLEREQUEST,
  '__module__' : 'component_metadata.component_metadata_pb2'
  # @@protoc_insertion_point(class_scope:mavsdk.rpc.component_metadata.SubscribeMetadataAvailableRequest)
  })
_sym_db.RegisterMessage(SubscribeMetadataAvailableRequest)

MetadataAvailableResponse = _reflection.GeneratedProtocolMessageType('MetadataAvailableResponse', (_message.Message,), {
  'DESCRIPTOR' : _METADATAAVAILABLERESPONSE,
  '__module__' : 'component_metadata.component_metadata_pb2'
  # @@protoc_insertion_point(class_scope:mavsdk.rpc.component_metadata.MetadataAvailableResponse)
  })
_sym_db.RegisterMessage(MetadataAvailableResponse)

MetadataUpdate = _reflection.GeneratedProtocolMessageType('MetadataUpdate', (_message.Message,), {
  'DESCRIPTOR' : _METADATAUPDATE,
  '__module__' : 'component_metadata.component_metadata_pb2'
  # @@protoc_insertion_point(class_scope:mavsdk.rpc.component_metadata.MetadataUpdate)
  })
_sym_db.RegisterMessage(MetadataUpdate)

_COMPONENTMETADATASERVICE = DESCRIPTOR.services_by_name['ComponentMetadataService']
if _descriptor._USE_C_DESCRIPTORS == False:

  DESCRIPTOR._options = None
  DESCRIPTOR._serialized_options = b'\n\034io.mavsdk.component_metadataB\026ComponentMetadataProto'
  _COMPONENTMETADATASERVICE.methods_by_name['RequestComponent']._options = None
  _COMPONENTMETADATASERVICE.methods_by_name['RequestComponent']._serialized_options = b'\200\265\030\001'
  _COMPONENTMETADATASERVICE.methods_by_name['RequestAutopilotComponent']._options = None
  _COMPONENTMETADATASERVICE.methods_by_name['RequestAutopilotComponent']._serialized_options = b'\200\265\030\001'
  _COMPONENTMETADATASERVICE.methods_by_name['SubscribeMetadataAvailable']._options = None
  _COMPONENTMETADATASERVICE.methods_by_name['SubscribeMetadataAvailable']._serialized_options = b'\200\265\030\000'
  _COMPONENTMETADATASERVICE.methods_by_name['GetMetadata']._options = None
  _COMPONENTMETADATASERVICE.methods_by_name['GetMetadata']._serialized_options = b'\200\265\030\001'
  _METADATATYPE._serialized_start=1154
  _METADATATYPE._serialized_end=1285
  _REQUESTCOMPONENTREQUEST._serialized_start=100
  _REQUESTCOMPONENTREQUEST._serialized_end=141
  _GETMETADATAREQUEST._serialized_start=143
  _GETMETADATAREQUEST._serialized_end=247
  _GETMETADATARESPONSE._serialized_start=250
  _GETMETADATARESPONSE._serialized_end=425
  _METADATADATA._serialized_start=427
  _METADATADATA._serialized_end=464
  _COMPONENTMETADATARESULT._serialized_start=467
  _COMPONENTMETADATARESULT._serialized_end=807
  _COMPONENTMETADATARESULT_RESULT._serialized_start=594
  _COMPONENTMETADATARESULT_RESULT._serialized_end=807
  _REQUESTCOMPONENTRESPONSE._serialized_start=809
  _REQUESTCOMPONENTRESPONSE._serialized_end=835
  _REQUESTAUTOPILOTCOMPONENTREQUEST._serialized_start=837
  _REQUESTAUTOPILOTCOMPONENTREQUEST._serialized_end=871
  _REQUESTAUTOPILOTCOMPONENTRESPONSE._serialized_start=873
  _REQUESTAUTOPILOTCOMPONENTRESPONSE._serialized_end=908
  _SUBSCRIBEMETADATAAVAILABLEREQUEST._serialized_start=910
  _SUBSCRIBEMETADATAAVAILABLEREQUEST._serialized_end=945
  _METADATAAVAILABLERESPONSE._serialized_start=947
  _METADATAAVAILABLERESPONSE._serialized_end=1035
  _METADATAUPDATE._serialized_start=1037
  _METADATAUPDATE._serialized_end=1151
  _COMPONENTMETADATASERVICE._serialized_start=1288
  _COMPONENTMETADATASERVICE._serialized_end=1908
# @@protoc_insertion_point(module_scope)
