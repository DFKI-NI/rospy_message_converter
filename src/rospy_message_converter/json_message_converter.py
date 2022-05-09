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

import json

from rospy_message_converter import message_converter


def convert_json_to_ros_message(message_type, json_message, strict_mode=True, log_level='error'):
    """
    Takes in the message type and a JSON-formatted string and returns a ROS
    message.

    :param message_type: The desired ROS message type of the result
    :type  message_type: str
    :param json_message: A JSON-formatted string
    :type  json_message: str
    :param strict_mode: If strict_mode is set, an exception will be thrown if the json message contains extra fields.
    :type  strict_mode: bool, optional
    :param log_level: The log level to be used. Available levels: debug, info, warning, error, critical
    :type  log_level: str, optional
    :return: A ROS message
    :rtype: class:`genpy.Message`

    Example:
        >>> msg_type = "std_msgs/String"
        >>> json_msg = '{"data": "Hello, Robot"}'
        >>> convert_json_to_ros_message(msg_type, json_msg)
        data: "Hello, Robot"
    """
    dictionary = json.loads(json_message)
    return message_converter.convert_dictionary_to_ros_message(
        message_type, dictionary, strict_mode=strict_mode, log_level=log_level
    )


def convert_ros_message_to_json(message, binary_array_as_bytes=True):
    """
    Takes in a ROS message and returns a JSON-formatted string.

    :param message: A ROS message to convert
    :type  message: class:`genpy.Message`
    :param binary_array_as_bytes: rospy treats `uint8[]` data as a `bytes`, which is the Python representation for byte
           data. In Python 2, this is the same as `str`. If this parameter is `False`, all `uint8[]` fields will be
           converted to `list(int)` instead.
    :type  binary_array_as_bytes: bool, optional
    :return: A JSON-formatted string
    :rtype:  str

    Example:
        >>> import std_msgs.msg
        >>> ros_message = std_msgs.msg.String(data="Hello, Robot")
        >>> convert_ros_message_to_json(ros_message)
        '{"data": "Hello, Robot"}'
    """
    dictionary = message_converter.convert_ros_message_to_dictionary(message, binary_array_as_bytes)
    json_message = json.dumps(dictionary)
    return json_message


if __name__ == "__main__":
    import doctest

    doctest.testmod()
