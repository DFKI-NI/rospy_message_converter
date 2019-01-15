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
import re
import base64
import sys
python3 = True if sys.hexversion > 0x03000000 else False
python_to_ros_type_map = {
    'bool'    : ['bool'],
    'int'     : ['int8', 'byte', 'uint8', 'char',
                 'int16', 'uint16', 'int32', 'uint32',
                 'int64', 'uint64', 'float32', 'float64'],
    'float'   : ['float32', 'float64'],
    'str'     : ['string'],
    'unicode' : ['string'],
    'long'    : ['uint64']
}
if python3:
    python_string_types = [str]
else:
    python_string_types = [str, unicode]
python_list_types = [list, tuple]

ros_time_types = ['time', 'duration']
ros_primitive_types = ['bool', 'byte', 'char', 'int8', 'uint8', 'int16',
                       'uint16', 'int32', 'uint32', 'int64', 'uint64',
                       'float32', 'float64', 'string']
ros_header_types = ['Header', 'std_msgs/Header', 'roslib/Header']
ros_binary_types_regexp = re.compile(r'(uint8|char)\[[^\]]*\]')

list_brackets = re.compile(r'\[[^\]]*\]')

def convert_dictionary_to_ros_message(message_type, dictionary, kind='message'):
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

    for field_name, field_value in dictionary.items():
        if field_name in message_fields:
            field_type = message_fields[field_name]
            field_value = _convert_to_ros_type(field_type, field_value)
            setattr(message, field_name, field_value)
        else:
            error_message = 'ROS message type "{0}" has no field named "{1}"'\
                .format(message_type, field_name)
            raise ValueError(error_message)

    return message

def _convert_to_ros_type(field_type, field_value):
    if is_ros_binary_type(field_type, field_value):
        field_value = _convert_to_ros_binary(field_type, field_value)
    elif field_type in ros_time_types:
        field_value = _convert_to_ros_time(field_type, field_value)
    elif field_type in ros_primitive_types:
        field_value = _convert_to_ros_primitive(field_type, field_value)
    elif _is_field_type_an_array(field_type):
        field_value = _convert_to_ros_array(field_type, field_value)
    else:
        field_value = convert_dictionary_to_ros_message(field_type, field_value)

    return field_value

def _convert_to_ros_binary(field_type, field_value):
    if type(field_value) in python_string_types:
        binary_value_as_string = base64.standard_b64decode(field_value)
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
    if field_type == "string":
        field_value = field_value.encode('utf-8')
    return field_value

def _convert_to_ros_array(field_type, list_value):
    list_type = list_brackets.sub('', field_type)
    return [_convert_to_ros_type(list_type, value) for value in list_value]

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
    if is_ros_binary_type(field_type, field_value):
        field_value = _convert_from_ros_binary(field_type, field_value)
    elif field_type in ros_time_types:
        field_value = _convert_from_ros_time(field_type, field_value)
    elif field_type in ros_primitive_types:
        field_value = field_value
    elif _is_field_type_an_array(field_type):
        field_value = _convert_from_ros_array(field_type, field_value)
    else:
        field_value = convert_ros_message_to_dictionary(field_value)

    return field_value


def is_ros_binary_type(field_type, field_value):
    """ Checks if the field is a binary array one, fixed size or not

    is_ros_binary_type("uint8", 42)
    >>> False
    is_ros_binary_type("uint8[]", [42, 18])
    >>> True
    is_ros_binary_type("uint8[3]", [42, 18, 21]
    >>> True
    is_ros_binary_type("char", 42)
    >>> False
    is_ros_binary_type("char[]", [42, 18])
    >>> True
    is_ros_binary_type("char[3]", [42, 18, 21]
    >>> True
    """
    return re.search(ros_binary_types_regexp, field_type) is not None

def _convert_from_ros_binary(field_type, field_value):
    field_value = base64.standard_b64encode(field_value)
    return field_value

def _convert_from_ros_time(field_type, field_value):
    field_value = {
        'secs'  : field_value.secs,
        'nsecs' : field_value.nsecs
    }
    return field_value

def _convert_from_ros_primitive(field_type, field_value):
    return field_value

def _convert_from_ros_array(field_type, field_value):
    list_type = list_brackets.sub('', field_type)
    return [_convert_from_ros_type(list_type, value) for value in field_value]

def _get_message_fields(message):
    return zip(message.__slots__, message._slot_types)

def _is_field_type_an_array(field_type):
    return list_brackets.search(field_type) is not None
