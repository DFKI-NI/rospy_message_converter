# -*- coding: utf-8 -*-
#
# Copyright (c) 2019-2022, Martin Günther (DFKI GmbH) and others
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

import json

from rclpy_message_converter import message_converter


def convert_json_to_ros_message(message_type, json_message, strict_mode=True):
    """
    Takes in the message type and a JSON-formatted string and returns a ROS
    message.

    :param message_type: The desired ROS message type of the result
    :type  message_type: str
    :param json_message: A JSON-formatted string
    :type  json_message: str
    :param strict_mode: If strict_mode is set, an exception will be thrown if the json message contains extra fields.
    :type  strict_mode: bool, optional
    :return: A ROS message
    :rtype: class:`genpy.Message`

    Example:
        >>> msg_type = "std_msgs/msg/String"
        >>> json_msg = '{"data": "Hello, Robot"}'
        >>> convert_json_to_ros_message(msg_type, json_msg)
        std_msgs.msg.String(data='Hello, Robot')
    """
    dictionary = json.loads(json_message)
    return message_converter.convert_dictionary_to_ros_message(message_type, dictionary, strict_mode=strict_mode)


def convert_ros_message_to_json(message, base64_encoding=True):
    """
    Takes in a ROS message and returns a JSON-formatted string.

    :param message: A ROS message to convert
    :type  message: class:`genpy.Message`
    :param base64_encoding: If this parameter is true, encode all variable-size `uint8[]` or fixed-size `uint8[n]`
           fields using Base64 encoding. This saves a lot of space when converting large messages
           (such as sensor_msgs/Image) to json format.
           If this parameter is `False`, these fields will be converted to `array.array` (for variable-size fields) or
           `numpy.ndarray` (for fixed-size fields) instead.
    :type  base64_encoding: bool, optional
    :return: A JSON-formatted string
    :rtype:  str

    Example:
        >>> import std_msgs.msg
        >>> ros_message = std_msgs.msg.String(data="Hello, Robot")
        >>> convert_ros_message_to_json(ros_message)
        '{"data": "Hello, Robot"}'
    """
    dictionary = message_converter.convert_ros_message_to_dictionary(message, base64_encoding)
    json_message = json.dumps(dictionary)
    return json_message


if __name__ == "__main__":
    import doctest

    doctest.testmod()
