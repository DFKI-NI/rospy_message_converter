#!/usr/bin/env python
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
        now_time = json_message_converter.get_now_time()
        expected_json1 = '{{"stamp": {{"sec": {0}, "nanosec": {1}}}, "frame_id": "my_frame"}}'\
            .format(now_time.sec, now_time.nanosec)
        expected_json2 = '{{"stamp": {{"sec": {0}, "nanosec": {1}}}, "frame_id": "my_frame"}}'\
            .format(now_time.sec, now_time.nanosec)
        expected_json3 = '{{"frame_id": "my_frame", "stamp": {{"sec": {0}, "nanosec": {1}}}}}'\
            .format(now_time.sec, now_time.nanosec)
        message = Header(stamp=now_time, frame_id='my_frame')
        message = serialize_deserialize(message)
        returned_json = json_message_converter.convert_ros_message_to_json(message)
        self.assertTrue(returned_json == expected_json1 or
                        returned_json == expected_json2 or
                        returned_json == expected_json3)

    def test_ros_message_with_uint8_array(self):
        from rclpy_message_converter_msgs.msg import Uint8ArrayTestMessage
        input_data = [97, 98, 99, 100]
        expected_json = '{"data": "YWJjZA=="}'  # base64.b64encode("abcd") is "YWJjZA=="
        message = Uint8ArrayTestMessage(data=input_data)
        message = serialize_deserialize(message)
        returned_json = json_message_converter.convert_ros_message_to_json(message)
        self.assertEqual(returned_json, expected_json)

    def test_ros_message_with_3uint8_array(self):
        from rclpy_message_converter_msgs.msg import Uint8Array3TestMessage
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
        message = json_message_converter.convert_json_to_ros_message(
            'std_msgs/msg/String', json_str)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_json_with_string_unicode(self):
        from std_msgs.msg import String
        expected_message = String(data=u'Hello \u00dcnicode')
        json_str = '{"data": "Hello \\u00dcnicode"}'
        message = json_message_converter.convert_json_to_ros_message(
            'std_msgs/msg/String', json_str)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_json_with_header(self):
        from std_msgs.msg import Header
        now_time = json_message_converter.get_now_time()
        expected_message = Header(
            stamp=now_time,
            frame_id='my_frame',
        )
        json_str = '{{"stamp": {{"sec": {0}, "nanosec": {1}}}, "frame_id": "my_frame"}}'\
            .format(now_time.sec, now_time.nanosec)
        message = json_message_converter.convert_json_to_ros_message(
            'std_msgs/msg/Header', json_str)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_json_with_invalid_message_fields(self):
        self.assertRaises(ValueError,
                          json_message_converter.convert_json_to_ros_message,
                          'std_msgs/msg/String',
                          '{"not_data": "Hello"}')


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
