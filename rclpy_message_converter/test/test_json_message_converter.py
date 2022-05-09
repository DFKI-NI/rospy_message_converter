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

import unittest
import rospy
from rospy_message_converter import json_message_converter


class TestJsonMessageConverter(unittest.TestCase):
    def test_ros_message_with_string(self):
        from std_msgs.msg import String

        expected_json = '{"data": "Hello"}'
        message = String(data='Hello')
        message = serialize_deserialize(message)
        returned_json = json_message_converter.convert_ros_message_to_json(message)
        self.assertEqual(returned_json, expected_json)

    def test_ros_message_with_string_unicode(self):
        from std_msgs.msg import String

        expected_json = '{"data": "Hello \\u00dcnicode"}'
        message = String(data=u'Hello \u00dcnicode')
        message = serialize_deserialize(message)
        returned_json = json_message_converter.convert_ros_message_to_json(message)
        self.assertEqual(returned_json, expected_json)

    def test_ros_message_with_header(self):
        from std_msgs.msg import Header
        from time import time

        now_time = rospy.Time(time())
        expected_json1 = '{{"stamp": {{"secs": {0}, "nsecs": {1}}}, "frame_id": "my_frame", "seq": 3}}'.format(
            now_time.secs, now_time.nsecs
        )
        expected_json2 = '{{"seq": 3, "stamp": {{"secs": {0}, "nsecs": {1}}}, "frame_id": "my_frame"}}'.format(
            now_time.secs, now_time.nsecs
        )
        expected_json3 = '{{"frame_id": "my_frame", "seq": 3, "stamp": {{"secs": {0}, "nsecs": {1}}}}}'.format(
            now_time.secs, now_time.nsecs
        )
        message = Header(stamp=now_time, frame_id='my_frame', seq=3)
        message = serialize_deserialize(message)
        returned_json = json_message_converter.convert_ros_message_to_json(message)
        self.assertTrue(
            returned_json == expected_json1 or returned_json == expected_json2 or returned_json == expected_json3
        )

    def test_ros_message_with_uint8_array(self):
        from rospy_message_converter.msg import Uint8ArrayTestMessage

        input_data = [97, 98, 99, 100]
        expected_json = '{"data": "YWJjZA=="}'  # base64.b64encode("abcd") is "YWJjZA=="
        message = Uint8ArrayTestMessage(data=input_data)
        message = serialize_deserialize(message)
        returned_json = json_message_converter.convert_ros_message_to_json(message)
        self.assertEqual(returned_json, expected_json)

    def test_ros_message_with_3uint8_array(self):
        from rospy_message_converter.msg import Uint8Array3TestMessage

        input_data = [97, 98, 99]
        expected_json = '{"data": "YWJj"}'  # base64.b64encode("abc") is "YWJj"
        message = Uint8Array3TestMessage(data=input_data)
        message = serialize_deserialize(message)
        returned_json = json_message_converter.convert_ros_message_to_json(message)
        self.assertEqual(returned_json, expected_json)

    def test_json_with_string(self):
        from std_msgs.msg import String

        expected_message = String(data='Hello')
        json_str = '{"data": "Hello"}'
        message = json_message_converter.convert_json_to_ros_message('std_msgs/String', json_str)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_json_with_string_unicode(self):
        from std_msgs.msg import String

        expected_message = String(data=u'Hello \u00dcnicode')
        json_str = '{"data": "Hello \\u00dcnicode"}'
        message = json_message_converter.convert_json_to_ros_message('std_msgs/String', json_str)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_json_with_header(self):
        from std_msgs.msg import Header
        from time import time

        now_time = rospy.Time(time())
        expected_message = Header(stamp=now_time, frame_id='my_frame', seq=12)
        json_str = '{{"stamp": {{"secs": {0}, "nsecs": {1}}}, "frame_id": "my_frame", "seq": 12}}'.format(
            now_time.secs, now_time.nsecs
        )
        message = json_message_converter.convert_json_to_ros_message('std_msgs/Header', json_str)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_json_with_string_null(self):
        from std_msgs.msg import String

        expected_message = String(data='')
        json_str = '{"data": null}'
        message = json_message_converter.convert_json_to_ros_message('std_msgs/String', json_str)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_json_with_invalid_message_fields(self):
        self.assertRaises(
            ValueError, json_message_converter.convert_json_to_ros_message, 'std_msgs/String', '{"not_data": "Hello"}'
        )


def serialize_deserialize(message):
    """
    Serialize and then deserialize a message. This simulates sending a message
    between ROS nodes and makes sure that the ROS messages being tested are
    actually serializable, and are in the same format as they would be received
    over the network. In rospy, it is possible to assign an illegal data type
    to a message field (for example, `message = String(data=42)`), but trying
    to publish this message will throw `SerializationError: field data must be
    of type str`. This method will expose such bugs.
    """
    from io import BytesIO

    buff = BytesIO()
    message.serialize(buff)
    result = message.__class__()  # create new instance of same class as message
    result.deserialize(buff.getvalue())
    return result


PKG = 'rospy_message_converter'
NAME = 'test_json_message_converter'
if __name__ == '__main__':
    import rosunit

    rosunit.unitrun(PKG, NAME, TestJsonMessageConverter)
