# -*- coding: utf-8 -*-
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2019-2022, Martin Günther (DFKI GmbH) and others
# Copyright (c) 2013-2016, Brandon Alexander
#
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of this project nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import logging
import roslib.message
import rospy
import base64
import sys
import copy
import collections

python3 = sys.hexversion > 0x03000000

python_list_types = [list, tuple]

if python3:
    python_string_types = [str, bytes]
    python_int_types = [int]
else:
    python_string_types = [str, unicode]  # noqa
    python_int_types = [int, long]  # noqa

python_float_types = [float]

ros_to_python_type_map = {
    'bool': [bool],
    'float32': copy.deepcopy(python_float_types + python_int_types),
    'float64': copy.deepcopy(python_float_types + python_int_types),
    'int8': copy.deepcopy(python_int_types),
    'int16': copy.deepcopy(python_int_types),
    'int32': copy.deepcopy(python_int_types),
    'int64': copy.deepcopy(python_int_types),
    'uint8': copy.deepcopy(python_int_types),
    'uint16': copy.deepcopy(python_int_types),
    'uint32': copy.deepcopy(python_int_types),
    'uint64': copy.deepcopy(python_int_types),
    'byte': copy.deepcopy(python_int_types),
    'char': copy.deepcopy(python_int_types),
    'string': copy.deepcopy(python_string_types),
}

try:
    import numpy as np

    _ros_to_numpy_type_map = {
        'float32': [np.float32, np.int8, np.int16, np.uint8, np.uint16],
        # don't include int32, because conversion to float may change value:
        # v = np.iinfo(np.int32).max; np.float32(v) != v
        'float64': [np.float32, np.float64, np.int8, np.int16, np.int32, np.uint8, np.uint16, np.uint32],
        'int8': [np.int8],
        'int16': [np.int8, np.int16, np.uint8],
        'int32': [np.int8, np.int16, np.int32, np.uint8, np.uint16],
        'int64': [np.int8, np.int16, np.int32, np.int64, np.uint8, np.uint16, np.uint32],
        'uint8': [np.uint8],
        'uint16': [np.uint8, np.uint16],
        'uint32': [np.uint8, np.uint16, np.uint32],
        'uint64': [np.uint8, np.uint16, np.uint32, np.uint64],
        'byte': [np.int8],
        'char': [np.uint8],
    }

    # merge type_maps
    merged = collections.defaultdict(list, ros_to_python_type_map)
    for k, v in _ros_to_numpy_type_map.items():
        merged[k].extend(v)
    ros_to_python_type_map = dict(merged)
except ImportError:
    pass


ros_time_types = ['time', 'duration']
ros_primitive_types = [
    'bool',
    'byte',
    'char',
    'int8',
    'uint8',
    'int16',
    'uint16',
    'int32',
    'uint32',
    'int64',
    'uint64',
    'float32',
    'float64',
    'string',
]
ros_header_types = ['Header', 'std_msgs/Header', 'roslib/Header']


def convert_dictionary_to_ros_message(
    message_type,
    dictionary,
    kind='message',
    strict_mode=True,
    check_missing_fields=False,
    check_types=True,
    log_level='error',
):
    """
    Takes in the message type and a Python dictionary and returns a ROS message.

    Example:
        >>> msg_type = "std_msgs/String"
        >>> dict_msg = { "data": "Hello, Robot" }
        >>> convert_dictionary_to_ros_message(msg_type, dict_msg)
        data: "Hello, Robot"

        >>> msg_type = "std_srvs/SetBool"
        >>> dict_msg = { "data": True }
        >>> kind = "request"
        >>> convert_dictionary_to_ros_message(msg_type, dict_msg, kind)
        data: True
    """
    if kind == 'message':
        message_class = roslib.message.get_message_class(message_type)
        message = message_class()
    elif kind == 'request':
        service_class = roslib.message.get_service_class(message_type)
        message = service_class._request_class()
    elif kind == 'response':
        service_class = roslib.message.get_service_class(message_type)
        message = service_class._response_class()
    else:
        raise ValueError('Unknown kind "%s".' % kind)
    message_fields = dict(_get_message_fields(message))

    remaining_message_fields = copy.deepcopy(message_fields)

    if dictionary is None:
        dictionary = {}
    for field_name, field_value in dictionary.items():
        if field_name in message_fields:
            field_type = message_fields[field_name]
            if field_value is not None:
                field_value = _convert_to_ros_type(
                    field_name, field_type, field_value, strict_mode, check_missing_fields, check_types, log_level
                )
                setattr(message, field_name, field_value)
            del remaining_message_fields[field_name]
        else:
            error_message = 'ROS message type "{0}" has no field named "{1}"'.format(message_type, field_name)
            if strict_mode:
                raise ValueError(error_message)
            else:
                if log_level not in ["debug", "info", "warning", "error", "critical"]:
                    log_level = "error"
                logger = logging.getLogger('rosout')
                log_func = getattr(logger, log_level)

                log_func('{}! It will be ignored.'.format(error_message))

    if check_missing_fields and remaining_message_fields:
        error_message = 'Missing fields "{0}"'.format(remaining_message_fields)
        raise ValueError(error_message)

    return message


def _convert_to_ros_type(
    field_name,
    field_type,
    field_value,
    strict_mode=True,
    check_missing_fields=False,
    check_types=True,
    log_level='error',
):
    if _is_ros_binary_type(field_type):
        field_value = _convert_to_ros_binary(field_type, field_value)
    elif field_type in ros_time_types:
        field_value = _convert_to_ros_time(field_type, field_value)
    elif field_type in ros_primitive_types:
        # Note: one could also use genpy.message.check_type() here, but:
        # 1. check_type is "not designed to run fast and is meant only for error diagnosis"
        # 2. it doesn't check floats (see ros/genpy#130)
        # 3. it rejects numpy types, although they can be serialized
        if check_types and type(field_value) not in ros_to_python_type_map[field_type]:
            raise TypeError(
                "Field '{0}' has wrong type {1} (valid types: {2})".format(
                    field_name, type(field_value), ros_to_python_type_map[field_type]
                )
            )
        field_value = _convert_to_ros_primitive(field_type, field_value)
    elif _is_field_type_a_primitive_array(field_type):
        field_value = field_value
    elif _is_field_type_an_array(field_type):
        field_value = _convert_to_ros_array(
            field_name, field_type, field_value, strict_mode, check_missing_fields, check_types, log_level
        )
    else:
        field_value = convert_dictionary_to_ros_message(
            field_type,
            field_value,
            strict_mode=strict_mode,
            check_missing_fields=check_missing_fields,
            check_types=check_types,
            log_level=log_level,
        )
    return field_value


def _convert_to_ros_binary(field_type, field_value):
    if type(field_value) in python_string_types:
        if python3:
            # base64 in python3 added the `validate` arg:
            # If field_value is not properly base64 encoded and there are non-base64-alphabet characters in the input,
            # a binascii.Error will be raised.
            binary_value_as_string = base64.b64decode(field_value, validate=True)
        else:
            # base64 in python2 doesn't have the `validate` arg: characters that are not in the base-64 alphabet are
            # silently discarded, resulting in garbage output.
            binary_value_as_string = base64.b64decode(field_value)
    else:
        binary_value_as_string = bytes(bytearray(field_value))
    return binary_value_as_string


def _convert_to_ros_time(field_type, field_value):
    time = None

    if field_type == 'time' and field_value == 'now':
        time = rospy.get_rostime()
    else:
        if field_type == 'time':
            time = rospy.rostime.Time()
        elif field_type == 'duration':
            time = rospy.rostime.Duration()
        if 'secs' in field_value and field_value['secs'] is not None:
            setattr(time, 'secs', field_value['secs'])
        if 'nsecs' in field_value and field_value['nsecs'] is not None:
            setattr(time, 'nsecs', field_value['nsecs'])

    return time


def _convert_to_ros_primitive(field_type, field_value):
    # std_msgs/msg/_String.py always calls encode() on python3, so don't do it here
    if field_type == "string" and not python3:
        field_value = field_value.encode('utf-8')
    return field_value


def _convert_to_ros_array(
    field_name,
    field_type,
    list_value,
    strict_mode=True,
    check_missing_fields=False,
    check_types=True,
    log_level='error',
):
    # use index to raise ValueError if '[' not present
    list_type = field_type[: field_type.index('[')]
    return [
        _convert_to_ros_type(field_name, list_type, value, strict_mode, check_missing_fields, check_types, log_level)
        for value in list_value
    ]


def convert_ros_message_to_dictionary(message, binary_array_as_bytes=True):
    """
    Takes in a ROS message and returns a Python dictionary.

    Example:
        >>> import std_msgs.msg
        >>> ros_message = std_msgs.msg.UInt32(data=42)
        >>> convert_ros_message_to_dictionary(ros_message)
        {'data': 42}
    """
    dictionary = {}
    message_fields = _get_message_fields(message)
    for field_name, field_type in message_fields:
        field_value = getattr(message, field_name)
        dictionary[field_name] = _convert_from_ros_type(field_type, field_value, binary_array_as_bytes)

    return dictionary


def _convert_from_ros_type(field_type, field_value, binary_array_as_bytes=True):
    if field_type in ros_primitive_types:
        field_value = _convert_from_ros_primitive(field_type, field_value)
    elif field_type in ros_time_types:
        field_value = _convert_from_ros_time(field_type, field_value)
    elif _is_ros_binary_type(field_type):
        if binary_array_as_bytes:
            field_value = _convert_from_ros_binary(field_type, field_value)
        elif type(field_value) == str:
            field_value = [ord(v) for v in field_value]
        else:
            field_value = list(field_value)
    elif _is_field_type_a_primitive_array(field_type):
        field_value = list(field_value)
    elif _is_field_type_an_array(field_type):
        field_value = _convert_from_ros_array(field_type, field_value, binary_array_as_bytes)
    else:
        field_value = convert_ros_message_to_dictionary(field_value, binary_array_as_bytes)

    return field_value


def _is_ros_binary_type(field_type):
    """Checks if the field is a binary array one, fixed size or not

    >>> _is_ros_binary_type("uint8")
    False
    >>> _is_ros_binary_type("uint8[]")
    True
    >>> _is_ros_binary_type("uint8[3]")
    True
    >>> _is_ros_binary_type("char")
    False
    >>> _is_ros_binary_type("char[]")
    True
    >>> _is_ros_binary_type("char[3]")
    True
    """
    return field_type.startswith('uint8[') or field_type.startswith('char[')


def _convert_from_ros_binary(field_type, field_value):
    field_value = base64.b64encode(field_value).decode('utf-8')
    return field_value


def _convert_from_ros_time(field_type, field_value):
    field_value = {'secs': field_value.secs, 'nsecs': field_value.nsecs}
    return field_value


def _convert_from_ros_primitive(field_type, field_value):
    # std_msgs/msg/_String.py always calls decode() on python3, so don't do it here
    if field_type == "string" and not python3:
        field_value = field_value.decode('utf-8')
    return field_value


def _convert_from_ros_array(field_type, field_value, binary_array_as_bytes=True):
    # use index to raise ValueError if '[' not present
    list_type = field_type[: field_type.index('[')]
    return [_convert_from_ros_type(list_type, value, binary_array_as_bytes) for value in field_value]


def _get_message_fields(message):
    return zip(message.__slots__, message._slot_types)


def _is_field_type_an_array(field_type):
    return field_type.find('[') >= 0


def _is_field_type_a_primitive_array(field_type):
    bracket_index = field_type.find('[')
    if bracket_index < 0:
        return False
    else:
        list_type = field_type[:bracket_index]
        return list_type in ros_primitive_types


if __name__ == "__main__":
    import doctest

    doctest.testmod()
