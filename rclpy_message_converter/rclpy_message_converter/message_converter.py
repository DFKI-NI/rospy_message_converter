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


import base64
import sys
import copy
import collections
import rclpy
import std_msgs
import geometry_msgs

from time import time

from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py.utilities import get_service

from rosidl_parser.definition import BasicType
from rosidl_parser.definition import NamedType
from rosidl_parser.definition import NamespacedType
from rosidl_parser.definition import AbstractGenericString
from rosidl_parser.definition import Array
from rosidl_parser.definition import AbstractSequence

from builtin_interfaces.msg import Time
from builtin_interfaces.msg import Duration

python3 = (sys.hexversion > 0x03000000)
python_list_types = [list, tuple]
if python3:
    python_string_types = [str, bytes]
    python_int_types = [int]
else:
    python_string_types = [str, unicode]
    python_int_types = [int, long]
python_float_types = [float]

ros_to_python_type_map = {
    'boolean' : [bool],  # msg type interface returns c++ type value
    'double'  : [float], # msg type interface returns c++ type value
    'float'   : [float], # msg type interface returns c++ type value
    'octet'   : [bytes], # msg type interface returns c++ type value
    'float32' : [float],
    'float64' : [float],
    'int8'    : [int],
    'int16'   : [int],
    'int32'   : [int],
    'int64'   : [int],
    'uint8'   : [int],
    'uint16'  : [int],
    'uint32'  : [int],
    'uint64'  : [int],
    'byte'    : [bytes],
    'char'    : [str],
    'string'  : [str],
    'wstring' : [str]
}

ros_time_types = ['Time', 'Duration']

# static array, unbounded dynamic array, bounded dynamic array, bounded string
ros_header_types = ['Header', 'std_msgs/msg/Header', 'roslib/Header']

def convert_dictionary_to_ros_message(message_type, dictionary, kind='message', strict_mode=True,
                                      check_missing_fields=False, check_types=True):
    """
    Takes in the message type and a Python dictionary and returns a ROS message.

    Example:
        >>> msg_type = "std_msgs/msg/String"
        >>> dict_msg = { "data": "Hello, Robot" }
        >>> convert_dictionary_to_ros_message(msg_type, dict_msg)
        data: "Hello, Robot"

        >>> msg_type = "std_srvs/srv/SetBool"
        >>> dict_msg = { "data": True }
        >>> kind = "request"
        >>> convert_dictionary_to_ros_message(msg_type, dict_msg, kind)
        data: True
    """

    """ If message type is manually given as string """
    if type(message_type) in python_string_types:
        if kind == 'message':
            message_class =  get_message(message_type)
            message = message_class()
        elif kind == 'request':
            service_class = get_service(message_type)
            message = service_class.Request()
        elif kind == 'response':
            service_class = get_service(message_type)
            message = service_class.Response()
        else:
            raise ValueError('Unknown kind "%s".' % kind)
    else:   # If message type is handed as object
    # if (isinstance Message) #from rosidl definition import Message / Service 
        message = message_type()
    ## TODO: Add error msg and test

    message_fields = _get_message_fields_and_types(message)
    remaining_message_fields = copy.deepcopy(message_fields)

    for field_name, field_value in dictionary.items():
        if field_name in message_fields:
            field_type = message_fields[field_name]
            field_value = _convert_to_ros_type(field_name, field_type, field_value, strict_mode, check_missing_fields,
                                               check_types)
            setattr(message, field_name, field_value)
            del remaining_message_fields[field_name]
        else:
            error_message = 'ROS message type "{0}" has no field named "{1}"'\
                .format(message_type, field_name)
            if strict_mode:
                raise ValueError(error_message)
            else:
                #rospy.logerr('{}! It will be ignored.'.format(error_message))
                pass

    if check_missing_fields and remaining_message_fields:
        error_message = 'Missing fields: {0}'.format(list(remaining_message_fields.keys()))
        raise ValueError(error_message)

    return message

def _convert_to_ros_type(field_name, field_type, field_value, strict_mode=True, check_missing_fields=False,
                         check_types=True):
    
    # Check all Types according to rosidl parser definition
    # https://github.com/ros2/rosidl/blob/master/rosidl_parser/rosidl_parser/definition.py

    # BasicTypes / Builtin Types / Primitive Types
    if isinstance(field_type, BasicType):
        # check if field_value fits field_type
        # Note: one could also use genpy.message.check_type() here, but:
        # 1. check_type is "not designed to run fast and is meant only for error diagnosis"
        # 2. it doesn't check floats (see ros/genpy#130)
        # 3. it rejects numpy types, although they can be serialized
        if check_types and type(field_value) not in ros_to_python_type_map[field_type.typename]:
            raise TypeError("Field '{0}' has wrong type {1} (valid types: {2})".format(
                field_name, type(field_value), ros_to_python_type_map[field_type.typename]))
        pass
    elif isinstance(field_type, NamedType):
        pass
    elif isinstance(field_type, NamespacedType):
        if field_type.name in ros_time_types:
            field_value = _convert_to_ros_time(field_type, field_value)
        else:
            field_type = _parse_rosidl_type_to_string(field_type)
            field_value = convert_dictionary_to_ros_message(field_type, field_value, strict_mode=strict_mode,
                check_missing_fields=check_missing_fields,
                check_types=check_types)
    elif isinstance(field_type, AbstractGenericString):
        # includes UnboundedString, BoundedString, UnboundedWString and BoundedWString
        # field_value = field_value
        pass
    elif isinstance(field_type, Array):
        # field_value = field_value
        pass
    elif isinstance(field_type, AbstractSequence):
        # includes BoundedSequence and UnboundedSequence
        field_value = _convert_to_ros_array(field_name, field_type, field_value, strict_mode, check_missing_fields, check_types)
    else: 
        field_type = _parse_rosidl_type_to_string(field_type)
        field_value = convert_dictionary_to_ros_message(field_type, field_value, strict_mode=strict_mode,
            check_missing_fields=check_missing_fields,
            check_types=check_types)

    return field_value

def _convert_to_ros_time(field_type, field_value):
    time = None

    if field_type.name == 'Time' and field_value == 'now':
        time = _get_now_time()
    else:
        if field_type.name == 'Time':
            time = Time()
        elif field_type.name == 'Duration':
            time = Duration()
        if 'sec' in field_value:
            setattr(time, 'sec', field_value['sec'])
        if 'nanosec' in field_value:
            setattr(time, 'nanosec', field_value['nanosec'])

    return time

def _convert_to_ros_array(field_name, field_type, list_value, strict_mode=True, check_missing_fields=False,
                          check_types=True):
    # use index to raise ValueError if '[' not present
    list_type = field_type.value_type
    return [_convert_to_ros_type(field_name, list_type, value, strict_mode, check_missing_fields, check_types) for value
            in list_value]

def convert_ros_message_to_dictionary(message):

    """
    Takes in a ROS message and returns a Python dictionary.

    Example:
        >>> import std_msgs.msg
        >>> ros_message = std_msgs.msg.UInt32(data=42)
        >>> convert_ros_message_to_dictionary(ros_message)
        {'data': 42}
    """
    dictionary = {}
    message_fields = _get_message_fields_and_types(message)
    for field_name, field_type in message_fields.items() :
       field_value = getattr(message, field_name)
       dictionary[field_name] = _convert_from_ros_type(field_type, field_value)

    return dictionary


def _convert_from_ros_type(field_type, field_value):

    if isinstance(field_type, BasicType):
        # field_value = field_value
        pass
    elif isinstance(field_type, NamedType):
        pass
    elif isinstance(field_type, NamespacedType):
        field_type = _parse_rosidl_type_to_string(field_type)
        field_value = convert_ros_message_to_dictionary(field_value)
    elif isinstance(field_type, AbstractGenericString):
        # includes UnboundedString, BoundedString, UnboundedWString and BoundedWString
        # field_value = field_value
        pass
    elif isinstance(field_type, Array):
        field_value = list(field_value) 
        pass
    elif isinstance(field_type, AbstractSequence):
        # includes BoundedSequence and UnboundedSequence
        field_value = _convert_from_ros_array(field_type, field_value)
    else: 
        pass

    return field_value

def _convert_from_ros_time(field_type, field_value):
    field_value = {
        'secs'  : field_value.secs,
        'nsecs' : field_value.nsecs
    }
    return field_value

def _convert_from_ros_array(field_type, field_value):
    # use index to raise ValueError if '[' not present
    list_type = field_type.value_type
    return [_convert_from_ros_type(list_type, value) for value in field_value]

def _get_message_fields_and_types(message):
    dict = {}
    # Workaround due to new python API https://github.com/ros2/rosidl_python/issues/99
    for slot, slot_type in zip(message.__slots__, message.SLOT_TYPES):
        dict[slot[1:]] = slot_type #remove trailing underscore
    return dict

def _parse_rosidl_type_to_string(rosidl_type):
    # TODO: find more elegant solution to this
    if not type(rosidl_type) in python_string_types:
        field_type = ""
        for ns in rosidl_type.namespaces:
            field_type += ns 
            field_type += '/'
        field_type+= rosidl_type.name
    
    return field_type

def _get_now_time():
    # TODO: look for more elegant ways
    now = time()
    now_time = Time()
    now_time.sec = (int)(now/60)
    now_time.nanosec = (int)(now%60)
    return now_time

if __name__ == "__main__":
    import doctest
    doctest.testmod()
