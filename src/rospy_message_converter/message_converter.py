#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Willow Garage, Inc.
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
#  * Neither the name of Willow Garage, Inc. nor the names of its
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

import roslib.message
import rospy
import base64
import sys
import copy
import collections

python3 = (sys.hexversion > 0x03000000)

python_list_types = [list, tuple]

if python3:
    python_string_types = [str]
    python_int_types = [int]
else:
    python_string_types = [str, unicode]
    python_int_types = [int, long]

python_float_types = [float]

ros_to_python_type_map = {
    'bool'    : [bool],
    'float32' : copy.deepcopy(python_float_types + python_int_types),
    'float64' : copy.deepcopy(python_float_types + python_int_types),
    'int8'    : copy.deepcopy(python_int_types),
    'int16'   : copy.deepcopy(python_int_types),
    'int32'   : copy.deepcopy(python_int_types),
    'int64'   : copy.deepcopy(python_int_types),
    'uint8'   : copy.deepcopy(python_int_types),
    'uint16'  : copy.deepcopy(python_int_types),
    'uint32'  : copy.deepcopy(python_int_types),
    'uint64'  : copy.deepcopy(python_int_types),
    'byte'    : copy.deepcopy(python_int_types),
    'char'    : copy.deepcopy(python_int_types),
    'string'  : copy.deepcopy(python_string_types)
}

try:
    import numpy as np
    _ros_to_numpy_type_map = {
        'float32' : [np.float32, np.int8, np.int16, np.uint8, np.uint16],
        # don't include int32, because conversion to float may change value: v = np.iinfo(np.int32).max; np.float32(v) != v
        'float64' : [np.float32, np.float64, np.int8, np.int16, np.int32, np.uint8, np.uint16, np.uint32],
        'int8'    : [np.int8],
        'int16'   : [np.int8, np.int16, np.uint8],
        'int32'   : [np.int8, np.int16, np.int32, np.uint8, np.uint16],
        'int64'   : [np.int8, np.int16, np.int32, np.int64, np.uint8, np.uint16, np.uint32],
        'uint8'   : [np.uint8],
        'uint16'  : [np.uint8, np.uint16],
        'uint32'  : [np.uint8, np.uint16, np.uint32],
        'uint64'  : [np.uint8, np.uint16, np.uint32, np.uint64],
        'byte'    : [np.int8],
        'char'    : [np.uint8],
    }

    # merge type_maps
    merged = collections.defaultdict(list, ros_to_python_type_map)
    for k, v in _ros_to_numpy_type_map.items():
        merged[k].extend(v)
    ros_to_python_type_map = dict(merged)
except ImportError:
    pass


ros_time_types = ['time', 'duration']
ros_primitive_types = ['bool', 'byte', 'char', 'int8', 'uint8', 'int16',
                       'uint16', 'int32', 'uint32', 'int64', 'uint64',
                       'float32', 'float64', 'string']
ros_header_types = ['Header', 'std_msgs/Header', 'roslib/Header']

def convert_dictionary_to_ros_message(message_type, dictionary, kind='message', strict_mode=True, check_missing_fields=False, check_types=True):
    """
    Takes in the message type and a Python dictionary and returns a ROS message.

    Example:
        message_type = "std_msgs/String"
        dict_message = { "data": "Hello, Robot" }
        ros_message = convert_dictionary_to_ros_message(message_type, dict_message)

        message_type = "std_srvs/SetBool"
        dict_message = { "data": True }
        kind = "request"
        ros_message = convert_dictionary_to_ros_message(message_type, dict_message, kind)
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

    for field_name, field_value in dictionary.items():
        if field_name in message_fields:
            field_type = message_fields[field_name]
            field_value = _convert_to_ros_type(field_name, field_type, field_value, check_types)
            setattr(message, field_name, field_value)
            del remaining_message_fields[field_name]
        else:
            error_message = 'ROS message type "{0}" has no field named "{1}"'\
                .format(message_type, field_name)
            if strict_mode:
                raise ValueError(error_message)
            else:
                rospy.logerr('{}! It will be ignored.'.format(error_message))

    if check_missing_fields and remaining_message_fields:
        error_message = 'Missing fields "{0}"'.format(remaining_message_fields)
        raise ValueError(error_message)

    return message

def _convert_to_ros_type(field_name, field_type, field_value, check_types=True):
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
            raise TypeError("Field '{0}' has wrong type {1} (valid types: {2})".format(field_name, type(field_value), ros_to_python_type_map[field_type]))
        field_value = _convert_to_ros_primitive(field_type, field_value)
    elif _is_field_type_a_primitive_array(field_type):
        field_value = field_value
    elif _is_field_type_an_array(field_type):
        field_value = _convert_to_ros_array(field_name, field_type, field_value, check_types)
    else:
        field_value = convert_dictionary_to_ros_message(field_type, field_value)

    return field_value

def _convert_to_ros_binary(field_type, field_value):
    if type(field_value) in python_string_types:
        binary_value_as_string = base64.standard_b64decode(field_value)
    elif python3:
        binary_value_as_string = bytes(bytearray(field_value))
    else:
        binary_value_as_string = str(bytearray(field_value))
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
        if 'secs' in field_value:
            setattr(time, 'secs', field_value['secs'])
        if 'nsecs' in field_value:
            setattr(time, 'nsecs', field_value['nsecs'])

    return time

def _convert_to_ros_primitive(field_type, field_value):
    # std_msgs/msg/_String.py always calls encode() on python3, so don't do it here
    if field_type == "string" and not python3:
        field_value = field_value.encode('utf-8')
    return field_value

def _convert_to_ros_array(field_name, field_type, list_value, check_types=True):
    # use index to raise ValueError if '[' not present
    list_type = field_type[:field_type.index('[')]
    return [_convert_to_ros_type(field_name, list_type, value, check_types) for value in list_value]

def convert_ros_message_to_dictionary(message):
    """
    Takes in a ROS message and returns a Python dictionary.

    Example:
        ros_message = std_msgs.msg.String(data="Hello, Robot")
        dict_message = convert_ros_message_to_dictionary(ros_message)
    """
    dictionary = {}
    message_fields = _get_message_fields(message)
    for field_name, field_type in message_fields:
        field_value = getattr(message, field_name)
        dictionary[field_name] = _convert_from_ros_type(field_type, field_value)

    return dictionary

def _convert_from_ros_type(field_type, field_value):
    if field_type in ros_primitive_types:
        field_value = field_value
    elif field_type in ros_time_types:
        field_value = _convert_from_ros_time(field_type, field_value)
    elif _is_ros_binary_type(field_type):
        field_value = _convert_from_ros_binary(field_type, field_value)
    elif _is_field_type_a_primitive_array(field_type):
        field_value = list(field_value)
    elif _is_field_type_an_array(field_type):
        field_value = _convert_from_ros_array(field_type, field_value)
    else:
        field_value = convert_ros_message_to_dictionary(field_value)

    return field_value

def _is_ros_binary_type(field_type):
    """ Checks if the field is a binary array one, fixed size or not

    _is_ros_binary_type("uint8")
    >>> False
    _is_ros_binary_type("uint8[]")
    >>> True
    _is_ros_binary_type("uint8[3]")
    >>> True
    _is_ros_binary_type("char")
    >>> False
    _is_ros_binary_type("char[]")
    >>> True
    _is_ros_binary_type("char[3]")
    >>> True
    """
    return field_type.startswith('uint8[') or field_type.startswith('char[')

def _convert_from_ros_binary(field_type, field_value):
    field_value = base64.standard_b64encode(field_value).decode('utf-8')
    return field_value

def _convert_from_ros_time(field_type, field_value):
    field_value = {
        'secs'  : field_value.secs,
        'nsecs' : field_value.nsecs
    }
    return field_value

def _convert_from_ros_array(field_type, field_value):
    # use index to raise ValueError if '[' not present
    list_type = field_type[:field_type.index('[')]
    return [_convert_from_ros_type(list_type, value) for value in field_value]

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
