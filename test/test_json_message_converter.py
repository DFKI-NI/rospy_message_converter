#!/usr/bin/env python
import unittest
import rospy
import rostest
from rospy_message_converter import json_message_converter

class TestJsonMessageConverter(unittest.TestCase):

    def test_ros_message_with_string(self):
        from std_msgs.msg import String
        expected_json = '{"data": "Hello"}'
        message = String(data = 'Hello')
        returned_json = json_message_converter.convert_ros_message_to_json(message)
        self.assertEqual(returned_json, expected_json)

    def test_ros_message_with_header(self):
        from std_msgs.msg import Header
        rospy.init_node('time_node')
        now_time = rospy.Time.now()
        expected_json = '{{"stamp": {{"secs": {0}, "nsecs": {1}}}, "frame_id": "my_frame", "seq": 3}}'\
            .format(now_time.secs, now_time.nsecs)
        message = Header(stamp = now_time, frame_id = 'my_frame', seq = 3)
        returned_json = json_message_converter.convert_ros_message_to_json(message)
        self.assertEqual(returned_json, expected_json)

    def test_ros_message_with_uint8_array(self):
        from rospy_message_converter.msg import Uint8ArrayTestMessage
        input_data = "".join([chr(i) for i in [97, 98, 99, 100]])
        expected_json = '{"data": "YWJjZA=="}'  # base64encode("abcd") is YWJjZA==
        message = Uint8ArrayTestMessage(data=input_data)
        returned_json = json_message_converter.convert_ros_message_to_json(message)
        self.assertEqual(returned_json, expected_json)

    def test_ros_message_with_uint8_array(self):
        from rospy_message_converter.msg import Uint8Array3TestMessage
        input_data = "".join([chr(i) for i in [97, 98, 99, 100]])
        expected_json = '{"data": "YWJjZA=="}'  # base64encode("abcd") YWJjZA==
        message = Uint8Array3TestMessage(data=input_data)
        returned_json = json_message_converter.convert_ros_message_to_json(message)
        self.assertEqual(returned_json, expected_json)

    def test_json_with_string(self):
        from std_msgs.msg import String
        expected_message = String(data = 'Hello')
        json_str = '{"data": "Hello"}'
        message = json_message_converter.convert_json_to_ros_message('std_msgs/String', json_str)
        self.assertEqual(message, expected_message)

    def test_json_with_header(self):
        from std_msgs.msg import Header
        rospy.init_node('time_node')
        now_time = rospy.Time.now()
        expected_message = Header(
            stamp = now_time,
            frame_id = 'my_frame',
            seq = 12
        )
        json_str = '{{"stamp": {{"secs": {0}, "nsecs": {1}}}, "frame_id": "my_frame", "seq": 12}}'\
            .format(now_time.secs, now_time.nsecs)
        message = json_message_converter.convert_json_to_ros_message('std_msgs/Header', json_str)
        self.assertEqual(message, expected_message)

    def test_json_with_invalid_message_fields(self):
        self.assertRaises(ValueError,
                          json_message_converter.convert_json_to_ros_message,
                          'std_msgs/String',
                          '{"not_data": "Hello"}')


PKG = 'rospy_message_converter'
NAME = 'test_json_message_converter'
if __name__ == '__main__':
    rostest.unitrun(PKG, NAME, TestJsonMessageConverter)
