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

import unittest
import logging

from rclpy_message_converter import json_message_converter
from rclpy.serialization import deserialize_message
from rclpy.serialization import serialize_message

logging.basicConfig(level=logging.DEBUG)


class TestJsonMessageConverter(unittest.TestCase):
    def test_ros_message_with_string(self):
        from std_msgs.msg import String

        expected_json = '{"data": "Hello"}'
        message = String(data='Hello')
        message = serialize_deserialize(message)
        returned_json = json_message_converter.convert_ros_message_to_json(message)
        self.assertEqual(expected_json, returned_json)

    def test_ros_message_with_string_unicode(self):
        from std_msgs.msg import String

        expected_json = '{"data": "Hello \\u00dcnicode"}'
        message = String(data=u'Hello \u00dcnicode')
        message = serialize_deserialize(message)
        returned_json = json_message_converter.convert_ros_message_to_json(message)
        self.assertEqual(expected_json, returned_json)

    def test_ros_message_with_header(self):
        from std_msgs.msg import Header

        now_time = _get_now_time()
        expected_json1 = '{{"stamp": {{"sec": {0}, "nanosec": {1}}}, "frame_id": "my_frame"}}'.format(
            now_time.sec, now_time.nanosec
        )
        expected_json2 = '{{"frame_id": "my_frame", "stamp": {{"sec": {0}, "nanosec": {1}}}}}'.format(
            now_time.sec, now_time.nanosec
        )
        message = Header(stamp=now_time, frame_id='my_frame')
        message = serialize_deserialize(message)
        returned_json = json_message_converter.convert_ros_message_to_json(message)
        self.assertTrue(expected_json1 == returned_json or expected_json2 == returned_json)

    def test_ros_message_with_tfmessage(self):
        from tf2_msgs.msg import TFMessage
        from geometry_msgs.msg import TransformStamped

        t = TransformStamped()
        t.header.stamp.sec = 12345
        t.header.stamp.nanosec = 67890
        t.header.frame_id = 'dummy_frame_id'
        t.child_frame_id = 'dummy_child_frame_id'
        t.transform.translation.x = 1.0
        t.transform.translation.y = 2.0
        t.transform.translation.z = 3.0
        t.transform.rotation.x = -0.5
        t.transform.rotation.y = 0.5
        t.transform.rotation.z = -0.5
        t.transform.rotation.w = 0.5
        message = TFMessage(transforms=[t])
        expected_json = f'''{{
            "transforms": [
                {{
                    "header": {{
                        "stamp": {{"sec": {t.header.stamp.sec}, "nanosec": {t.header.stamp.nanosec}}},
                        "frame_id": "{t.header.frame_id}"
                    }},
                    "child_frame_id": "{t.child_frame_id}",
                    "transform": {{
                        "translation": {{
                            "x": {t.transform.translation.x},
                            "y": {t.transform.translation.y},
                            "z": {t.transform.translation.z}
                        }},
                        "rotation": {{
                            "x": {t.transform.rotation.x},
                            "y": {t.transform.rotation.y},
                            "z": {t.transform.rotation.z},
                            "w": {t.transform.rotation.w}
                        }}
                    }}
                }}
            ]
        }}'''
        message = serialize_deserialize(message)
        returned_json = json_message_converter.convert_ros_message_to_json(message)
        # strip whitespace:
        expected_json = ''.join(expected_json.split())
        returned_json = ''.join((returned_json.split()))
        self.assertEqual(expected_json, returned_json)

    def test_ros_message_with_uint8_array(self):
        from rclpy_message_converter_msgs.msg import Uint8ArrayTestMessage

        input_data = [97, 98, 99, 100]
        expected_json = '{"data": "YWJjZA=="}'  # base64.b64encode("abcd") is "YWJjZA=="
        message = Uint8ArrayTestMessage(data=input_data)
        message = serialize_deserialize(message)
        returned_json = json_message_converter.convert_ros_message_to_json(message)
        self.assertEqual(expected_json, returned_json)

    def test_ros_message_with_3uint8_array(self):
        from rclpy_message_converter_msgs.msg import Uint8Array3TestMessage

        input_data = [97, 98, 99]
        expected_json = '{"data": "YWJj"}'  # base64.b64encode("abc") is "YWJj"
        message = Uint8Array3TestMessage(data=input_data)
        message = serialize_deserialize(message)
        returned_json = json_message_converter.convert_ros_message_to_json(message)
        self.assertEqual(expected_json, returned_json)

    def test_json_with_string(self):
        from std_msgs.msg import String

        expected_message = String(data='Hello')
        json_str = '{"data": "Hello"}'
        message = json_message_converter.convert_json_to_ros_message('std_msgs/msg/String', json_str)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(expected_message, message)

    def test_json_with_string_unicode(self):
        from std_msgs.msg import String

        expected_message = String(data=u'Hello \u00dcnicode')
        json_str = '{"data": "Hello \\u00dcnicode"}'
        message = json_message_converter.convert_json_to_ros_message('std_msgs/msg/String', json_str)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(expected_message, message)

    def test_json_with_header(self):
        from std_msgs.msg import Header

        now_time = _get_now_time()
        expected_message = Header(
            stamp=now_time,
            frame_id='my_frame',
        )
        json_str = '{{"stamp": {{"sec": {0}, "nanosec": {1}}}, "frame_id": "my_frame"}}'.format(
            now_time.sec, now_time.nanosec
        )
        message = json_message_converter.convert_json_to_ros_message('std_msgs/msg/Header', json_str)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(expected_message, message)

    def test_json_with_tfmessage(self):
        from tf2_msgs.msg import TFMessage
        from geometry_msgs.msg import TransformStamped

        t = TransformStamped()
        t.header.stamp.sec = 12345
        t.header.stamp.nanosec = 67890
        t.header.frame_id = 'dummy_frame_id'
        t.child_frame_id = 'dummy_child_frame_id'
        t.transform.translation.x = 1.0
        t.transform.translation.y = 2.0
        t.transform.translation.z = 3.0
        t.transform.rotation.x = -0.5
        t.transform.rotation.y = 0.5
        t.transform.rotation.z = -0.5
        t.transform.rotation.w = 0.5
        expected_message = TFMessage(transforms=[t])
        json_str = f'''{{
            "transforms": [
                {{
                    "header": {{
                        "stamp": {{"sec": {t.header.stamp.sec}, "nanosec": {t.header.stamp.nanosec}}},
                        "frame_id": "{t.header.frame_id}"
                    }},
                    "child_frame_id": "{t.child_frame_id}",
                    "transform": {{
                        "translation": {{
                            "x": {t.transform.translation.x},
                            "y": {t.transform.translation.y},
                            "z": {t.transform.translation.z}
                        }},
                        "rotation": {{
                            "x": {t.transform.rotation.x},
                            "y": {t.transform.rotation.y},
                            "z": {t.transform.rotation.z},
                            "w": {t.transform.rotation.w}
                        }}
                    }}
                }}
            ]
        }}'''
        message = json_message_converter.convert_json_to_ros_message('tf2_msgs/msg/TFMessage', json_str)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(expected_message, message)

    def test_json_with_string_null(self):
        from std_msgs.msg import String

        expected_message = String(data='')
        json_str = '{"data": null}'
        message = json_message_converter.convert_json_to_ros_message('std_msgs/msg/String', json_str)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(expected_message, message)

    def test_json_with_invalid_message_fields(self):
        self.assertRaises(
            AttributeError,
            json_message_converter.convert_json_to_ros_message,
            'std_msgs/msg/String',
            '{"not_data": "Hello"}',
        )


def _get_now_time():
    from builtin_interfaces.msg import Time
    import time

    now = time.time()
    now_time = Time()
    now_time.sec = int(now)
    now_time.nanosec = int((now % 1) * 1e9)
    return now_time


def serialize_deserialize(msg):
    """
    Serialize and then deserialize a message.

    This simulates sending a message between ROS nodes and makes sure that the ROS messages
    being tested are actually serializable, and are in the same format as they would be received
    over the network. In rospy, it is possible to assign an illegal data type
    to a message field (for example, `message = String(data=42)`), but trying
    to publish this message will throw `SerializationError: field data must be
    of type str`. This method will expose such bugs.
    """
    msg_serialized = serialize_message(msg)
    msg_deserialized = deserialize_message(msg_serialized, type(msg))
    assert msg == msg_deserialized
    return msg_deserialized
