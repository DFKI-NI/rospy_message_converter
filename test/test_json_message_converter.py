#!/usr/bin/env python
import unittest
import rospy
from rospy_message_converter import json_message_converter

class TestJsonMessageConverter(unittest.TestCase):

    def test_ros_message_with_string(self):
        from std_msgs.msg import String
        expected_json = '{"data": "Hello"}'
        message = String(data = 'Hello')
        message = serialize_deserialize(message)
        returned_json = json_message_converter.convert_ros_message_to_json(message)
        self.assertEqual(returned_json, expected_json)

    def test_ros_message_with_header(self):
        from std_msgs.msg import Header
        from time import time
        now_time = rospy.Time(time())
        expected_json = '{{"stamp": {{"secs": {0}, "nsecs": {1}}}, "frame_id": "my_frame", "seq": 3}}'\
            .format(now_time.secs, now_time.nsecs)
        message = Header(stamp = now_time, frame_id = 'my_frame', seq = 3)
        message = serialize_deserialize(message)
        returned_json = json_message_converter.convert_ros_message_to_json(message)
        self.assertEqual(returned_json, expected_json)

    def test_ros_message_with_uint8_array(self):
        from rospy_message_converter.msg import Uint8ArrayTestMessage
        input_data = "".join([chr(i) for i in [97, 98, 99, 100]])
        expected_json = '{"data": "YWJjZA=="}'  # base64.standard_b64encode("abcd") is "YWJjZA=="
        message = Uint8ArrayTestMessage(data=input_data)
        message = serialize_deserialize(message)
        returned_json = json_message_converter.convert_ros_message_to_json(message)
        self.assertEqual(returned_json, expected_json)

    def test_ros_message_with_3uint8_array(self):
        from rospy_message_converter.msg import Uint8Array3TestMessage
        input_data = "".join([chr(i) for i in [97, 98, 99]])
        expected_json = '{"data": "YWJj"}'  # base64.standard_b64encode("abc") is "YWJj"
        message = Uint8Array3TestMessage(data=input_data)
        message = serialize_deserialize(message)
        returned_json = json_message_converter.convert_ros_message_to_json(message)
        self.assertEqual(returned_json, expected_json)

    def test_json_with_string(self):
        from std_msgs.msg import String
        expected_message = String(data = 'Hello')
        json_str = '{"data": "Hello"}'
        message = json_message_converter.convert_json_to_ros_message('std_msgs/String', json_str)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_json_with_header(self):
        from std_msgs.msg import Header
        from time import time
        now_time = rospy.Time(time())
        expected_message = Header(
            stamp = now_time,
            frame_id = 'my_frame',
            seq = 12
        )
        json_str = '{{"stamp": {{"secs": {0}, "nsecs": {1}}}, "frame_id": "my_frame", "seq": 12}}'\
            .format(now_time.secs, now_time.nsecs)
        message = json_message_converter.convert_json_to_ros_message('std_msgs/Header', json_str)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_json_with_invalid_message_fields(self):
        self.assertRaises(ValueError,
                          json_message_converter.convert_json_to_ros_message,
                          'std_msgs/String',
                          '{"not_data": "Hello"}')


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
    from StringIO import StringIO
    buff = StringIO()
    message.serialize(buff)
    result = message.__class__()   # create new instance of same class as message
    result.deserialize(buff.getvalue())
    return result


PKG = 'rospy_message_converter'
NAME = 'test_json_message_converter'
if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, NAME, TestJsonMessageConverter)
