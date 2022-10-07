# -*- coding: utf-8 -*-
#
# Copyright (c) 2019-2022, Martin Günther (DFKI GmbH) and others
# Copyright (c) 2017-2019 Open Source Robotics Foundation, Inc.
# Copyright (c) 2013-2016, Brandon Alexander
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import array
import base64
from collections import OrderedDict
from typing import Any, Dict, Text

import numpy as np
import rosidl_parser.definition
from rosidl_parser.definition import AbstractNestedType, NamespacedType, Array, UnboundedSequence, BasicType
from rosidl_runtime_py.convert import get_message_slot_types
from rosidl_runtime_py.import_message import import_message_from_namespaced_type
from rosidl_runtime_py.utilities import get_message, get_service


def convert_dictionary_to_ros_message(
    message_type: Any,
    dictionary: Dict[str, Any],
    kind: str = 'message',
    strict_mode: bool = True,
    check_missing_fields: bool = False,
) -> Any:
    """
    Takes in the message type and a Python dictionary and returns a ROS message.

    :param message_type: Either the type name of the ROS message to return (as str), or the message class.
    :param dictionary: The values to set in the ROS message. The keys of the dictionary represent
        fields of the message.
    :param kind: Whether to create a message, service request or service response (valid values: "message",
        "request", "response").
    :param strict_mode: If strict_mode is set, an AttributeError will be raised if the input dictionary contains
        extra fields that are not present in the ROS message.
    :param check_missing_fields: If this parameter is set, a ValueError will be raised if the input dictionary
        is missing an entry for some field of the ROS message.
    :raises AttributeError: If the message does not have a field provided in the input dictionary.
    :raises TypeError: If a message value does not match its field type.
    :raises ValueError: If the input dictionary is incomplete (i.e., is missing an entry for some field
        of the ROS message) or if an invalid value was passed to `kind`.

    Example:
        >>> msg_type = "std_msgs/msg/String"
        >>> dict_msg = { "data": "Hello, Robot" }
        >>> convert_dictionary_to_ros_message(msg_type, dict_msg)
        std_msgs.msg.String(data='Hello, Robot')

        >>> import std_msgs.msg
        >>> msg_type = std_msgs.msg.String
        >>> dict_msg = { "data": "Hello, Robot" }
        >>> convert_dictionary_to_ros_message(msg_type, dict_msg)
        std_msgs.msg.String(data='Hello, Robot')

        >>> msg_type = "std_srvs/srv/SetBool"
        >>> dict_msg = { "data": True }
        >>> kind = "request"
        >>> convert_dictionary_to_ros_message(msg_type, dict_msg, kind)
        std_srvs.srv.SetBool_Request(data=True)
    """
    if isinstance(message_type, Text):
        # message_type = type name as string (e.g., "std_msgs/msg/String")
        if kind == 'message':
            message_class = get_message(message_type)
            message = message_class()
        elif kind == 'request':
            service_class = get_service(message_type)
            message = service_class.Request()
        elif kind == 'response':
            service_class = get_service(message_type)
            message = service_class.Response()
        else:
            raise ValueError('Unknown kind "%s".' % kind)
    else:
        # message_type = message class (e.g., std_msgs.msg.String)
        message = message_type()

    set_message_fields(message, dictionary, strict_mode, check_missing_fields)
    return message


def set_message_fields(
    msg: Any, values: Dict[str, Any], strict_mode: bool = True, check_missing_fields: bool = False
) -> None:
    """
    Set the fields of a ROS message.

    This method was copied and modified from rosidl_runtime_py.set_message.set_message_fields .

    :param msg: The ROS message to populate.
    :param values: The values to set in the ROS message. The keys of the dictionary represent
        fields of the message.
    :param strict_mode: If strict_mode is set, an AttributeError will be raised if the input dictionary contains
        extra fields that are not present in the ROS message.
    :param check_missing_fields: If this parameter is set, a ValueError will be raised if the input dictionary
        is missing an entry for some field of the ROS message.
    :raises AttributeError: If the message does not have a field provided in the input dictionary.
    :raises TypeError: If a message value does not match its field type.
    :raises ValueError: If the input dictionary is incomplete (i.e., is missing an entry for some field
        of the ROS message).
    """
    if values is None:
        values = {}
    try:
        items = values.items()
    except AttributeError:
        raise TypeError("Value '%s' is expected to be a dictionary but is a %s" % (values, type(values).__name__))

    remaining_message_fields = dict(_get_message_fields(msg))

    for field_name, field_value in items:
        if field_value is None:
            continue
        try:
            field = getattr(msg, field_name)
            del remaining_message_fields[field_name]
        except AttributeError as e:
            if strict_mode:
                raise e
            else:
                continue
        field_type = type(field)
        if field_type is array.array:
            if isinstance(field_value, (str, bytes)):
                # If field_value is not properly base64 encoded and there are non-base64-alphabet characters in the
                # input, a binascii.Error will be raised.
                field_value = list(base64.b64decode(field_value, validate=True))
            value = field_type(field.typecode, field_value)
        elif field_type is np.ndarray:
            if isinstance(field_value, (str, bytes)):
                # If field_value is not properly base64 encoded and there are non-base64-alphabet characters in the
                # input, a binascii.Error will be raised.
                field_value = list(base64.b64decode(field_value, validate=True))
            value = np.array(field_value, dtype=field.dtype)
        elif type(field_value) is field_type:
            value = field_value
        else:
            try:
                value = field_type(field_value)
            except TypeError:
                value = field_type()
                set_message_fields(value, field_value, strict_mode, check_missing_fields)
        rosidl_type = get_message_slot_types(msg)[field_name]
        # Check if field is an array of ROS messages
        if isinstance(rosidl_type, AbstractNestedType):
            if isinstance(rosidl_type.value_type, NamespacedType):
                field_elem_type = import_message_from_namespaced_type(rosidl_type.value_type)
                for n in range(len(value)):
                    submsg = field_elem_type()
                    set_message_fields(submsg, value[n], strict_mode, check_missing_fields)
                    value[n] = submsg
        setattr(msg, field_name, value)

    if check_missing_fields and remaining_message_fields:
        error_message = 'fields in dictionary missing from ROS message: "{0}"'.format(
            list(remaining_message_fields.keys())
        )
        raise ValueError(error_message)


def convert_ros_message_to_dictionary(message: Any, base64_encoding: bool = True) -> OrderedDict:
    """
    Takes in a ROS message and returns an OrderedDict.

    Example:
        >>> import std_msgs.msg
        >>> ros_message = std_msgs.msg.UInt32(data=42)
        >>> convert_ros_message_to_dictionary(ros_message)
        OrderedDict([('data', 42)])
    """

    return message_to_ordereddict(message, base64_encoding=base64_encoding)


def message_to_ordereddict(
    msg: Any,
    *,
    base64_encoding: bool = True,
    truncate_length: int = None,
    no_arr: bool = False,
    no_str: bool = False,
) -> OrderedDict:
    """
    Convert a ROS message to an OrderedDict.

    This method was copied and modified from rosidl_runtime_py.convert.message_to_ordereddict.

    :param msg: The ROS message to convert.
    :param base64_encoding: If this parameter is true, encode all variable-size `uint8[]` or fixed-size `uint8[n]`
           fields using Base64 encoding. This saves a lot of space when converting large messages
           (such as sensor_msgs/Image) to json format.
           If this parameter is `False`, these fields will be converted to `array.array` (for variable-size fields) or
           `numpy.ndarray` (for fixed-size fields) instead.
    :param truncate_length: Truncate values for all message fields to this length.
        This does not truncate the list of fields (ie. the dictionary keys).
    :param no_arr: Exclude array fields of the message.
    :param no_str: Exclude string fields of the message.
    :returns: An OrderedDict where the keys are the ROS message fields and the values are
        set to the values of the input message.
    """
    d = OrderedDict()

    # We rely on __slots__ retaining the order of the fields in the .msg file.
    for field_name, field_type in zip(msg.__slots__, msg.SLOT_TYPES):
        value = getattr(msg, field_name, None)

        value = _convert_value(
            value,
            base64_encoding=base64_encoding,
            field_type=field_type,
            truncate_length=truncate_length,
            no_arr=no_arr,
            no_str=no_str,
        )
        # Remove leading underscore from field name
        d[field_name[1:]] = value
    return d


def _convert_value(
    value, *, base64_encoding: bool = True, field_type=None, truncate_length=None, no_arr=False, no_str=False
):
    if isinstance(value, bytes):
        if truncate_length is not None and len(value) > truncate_length:
            value = bytearray(''.join([chr(c) for c in value[:truncate_length]]) + '...', 'utf-8')
    elif isinstance(value, str):
        if no_str is True:
            value = '<string length: <{0}>>'.format(len(value))
        elif truncate_length is not None and len(value) > truncate_length:
            value = value[:truncate_length] + '...'
    elif isinstance(value, (list, tuple, array.array, np.ndarray)):
        # Since arrays and ndarrays can't contain mixed types convert to list
        typename = tuple if isinstance(value, tuple) else list

        if (
            base64_encoding
            and isinstance(field_type, (Array, UnboundedSequence))
            and type(field_type.value_type) is BasicType
            and field_type.value_type.typename == 'uint8'
        ):
            value = base64.b64encode(value).decode('utf-8')
        elif no_arr is True and field_type is not None:
            value = __abbreviate_array_info(value, field_type)
        elif truncate_length is not None and len(value) > truncate_length:
            # Truncate the sequence
            value = value[:truncate_length]
            # Truncate every item in the sequence
            value = typename(
                [
                    _convert_value(
                        v,
                        base64_encoding=base64_encoding,
                        truncate_length=truncate_length,
                        no_arr=no_arr,
                        no_str=no_str,
                    )
                    for v in value
                ]
                + ['...']
            )
        else:
            # Convert every item in the list
            value = typename(
                [
                    _convert_value(
                        v,
                        base64_encoding=base64_encoding,
                        truncate_length=truncate_length,
                        no_arr=no_arr,
                        no_str=no_str,
                    )
                    for v in value
                ]
            )
    elif isinstance(value, dict) or isinstance(value, OrderedDict):
        # Convert each key and value in the mapping
        new_value = {} if isinstance(value, dict) else OrderedDict()
        for k, v in value.items():
            # Don't truncate keys because that could result in key collisions and data loss
            new_value[_convert_value(k)] = _convert_value(
                v,
                base64_encoding=base64_encoding,
                truncate_length=truncate_length,
                no_arr=no_arr,
                no_str=no_str,
            )
        value = new_value
    elif isinstance(value, np.number):
        value = value.item()
    elif not isinstance(value, (bool, float, int)):
        # Assuming value is a message since it is neither a collection nor a primitive type
        value = message_to_ordereddict(
            value,
            base64_encoding=base64_encoding,
            truncate_length=truncate_length,
            no_arr=no_arr,
            no_str=no_str,
        )
    return value


def __abbreviate_array_info(value, field_type):
    value_type_name = __get_type_name(field_type.value_type)
    if isinstance(field_type, rosidl_parser.definition.Array):
        return '<array type: {0}[{1}]>'.format(value_type_name, field_type.size)
    elif isinstance(field_type, rosidl_parser.definition.BoundedSequence):
        return '<sequence type: {0}[{1}], length: {2}>'.format(value_type_name, field_type.maximum_size, len(value))
    elif isinstance(field_type, rosidl_parser.definition.UnboundedSequence):
        return '<sequence type: {0}, length: {1}>'.format(value_type_name, len(value))
    return 'unknown'


def __get_type_name(value_type):
    if isinstance(value_type, rosidl_parser.definition.BasicType):
        return value_type.typename
    elif isinstance(value_type, rosidl_parser.definition.AbstractString):
        return 'string'
    elif isinstance(value_type, rosidl_parser.definition.AbstractWString):
        return 'wstring'
    elif isinstance(value_type, rosidl_parser.definition.NamedType):
        return value_type.name
    elif isinstance(value_type, rosidl_parser.definition.NamespacedType):
        return '/'.join(value_type.namespaced_name())
    else:
        return 'unknown'


def _get_message_fields(message):
    # Workaround due to new python API https://github.com/ros2/rosidl_python/issues/99
    slots = [slot[1:] for slot in message.__slots__]  # remove leading underscore
    return zip(slots, message.SLOT_TYPES)


if __name__ == "__main__":
    import doctest

    doctest.testmod()
