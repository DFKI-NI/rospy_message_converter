#!/usr/bin/env python
import struct
import sys
import unittest
import logging

from rclpy.serialization import deserialize_message
from rclpy.serialization import serialize_message

from rclpy_message_converter import message_converter

python3 = (sys.hexversion > 0x03000000)

class TestMessageConverter(unittest.TestCase):

    def test_ros_message_with_array(self):
        from rclpy_message_converter_msgs.msg import TestArray
        expected_dictionary = {
            'data': [1.1, 2.2, 3.3]
        }
        message = TestArray(data = expected_dictionary['data'])
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(dictionary, expected_dictionary)

    def test_ros_message_with_bool(self):
        from std_msgs.msg import Bool
        expected_dictionary = { 'data': True }
        message = Bool(data=expected_dictionary['data'])
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(dictionary, expected_dictionary)

    def test_ros_message_with_byte(self):
        from std_msgs.msg import Byte
        expected_dictionary = { 'data': bytes([5]) }
        message = Byte(data=expected_dictionary['data'])
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(dictionary, expected_dictionary)

    def test_ros_message_with_char(self):
        from std_msgs.msg import Char
        expected_dictionary = { 'data': 99 }
        message = Char(data=expected_dictionary['data'])
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(dictionary, expected_dictionary)

    def test_ros_message_with_duration(self):
        from builtin_interfaces.msg import Duration
        duration = Duration(sec= 33, nanosec =25)
        expected_dictionary = {
            'sec'  : duration.sec,
            'nanosec' : duration.nanosec
        }
        message = Duration(sec = duration.sec, nanosec = duration.nanosec)
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(dictionary, expected_dictionary)

    def test_ros_message_with_empty(self):
        from std_msgs.msg import Empty
        expected_dictionary = {}
        message = Empty()
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(dictionary, expected_dictionary)

    def test_ros_message_with_float32(self):
        from std_msgs.msg import Float32
        expected_dictionary = { 'data': struct.unpack('<f', b'\x7F\x7F\xFF\xFD')[0] }
        message = Float32(data=expected_dictionary['data'])
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(dictionary, expected_dictionary)

    def test_ros_message_with_float64(self):
        from std_msgs.msg import Float64
        expected_dictionary = { 'data': struct.unpack('<d', b'\x7F\xEF\xFF\xFF\xFF\xFF\xFF\xFD')[0] }
        message = Float64(data=expected_dictionary['data'])
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(dictionary, expected_dictionary)

    def test_ros_message_with_header(self):
        from std_msgs.msg import Header
        # TODO: more elegant way for now-time
        now_time = message_converter.get_now_time()
        expected_dictionary = {
            'stamp': { 'sec': now_time.sec, 'nanosec': now_time.nanosec },
            'frame_id' : 'my_frame',
        }
        message = Header(
            stamp = now_time,
            frame_id = expected_dictionary['frame_id'],
        )
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(dictionary, expected_dictionary)

    def test_ros_message_with_int8(self):
        from std_msgs.msg import Int8
        expected_dictionary = { 'data': -0x7F }
        message = Int8(data=expected_dictionary['data'])
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(dictionary, expected_dictionary)

    def test_ros_message_with_uint8(self):
        from std_msgs.msg import UInt8
        expected_dictionary = { 'data': 0xFF }
        message = UInt8(data=expected_dictionary['data'])
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(dictionary, expected_dictionary)

    def test_ros_message_with_uint8_array(self):
        from rclpy_message_converter_msgs.msg import Uint8ArrayTestMessage
        expected_data = [97, 98, 99, 100]
        message = Uint8ArrayTestMessage(data=expected_data)
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        expected_data = expected_data
        self.assertEqual(dictionary["data"], expected_data)

    def test_ros_message_with_3uint8_array(self):
        from rclpy_message_converter_msgs.msg import Uint8Array3TestMessage
        expected_data = [97, 98, 99]
        message = Uint8Array3TestMessage(data=expected_data)
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(dictionary["data"], expected_data)

    def test_ros_message_with_nested_uint8_array(self):
        from rclpy_message_converter_msgs.msg import NestedUint8ArrayTestMessage, Uint8ArrayTestMessage
        expected_data = [97, 98, 99]
        message = NestedUint8ArrayTestMessage(arrays=[Uint8ArrayTestMessage(data=expected_data)])
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(dictionary["arrays"][0]["data"], expected_data)

    def test_ros_message_with_int16(self):
        from std_msgs.msg import Int16
        expected_dictionary = { 'data': -0x7FFF }
        message = Int16(data=expected_dictionary['data'])
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(dictionary, expected_dictionary)

    def test_ros_message_with_uint16(self):
        from std_msgs.msg import UInt16
        expected_dictionary = { 'data': 0xFFFF }
        message = UInt16(data=expected_dictionary['data'])
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(dictionary, expected_dictionary)

    def test_ros_message_with_int32(self):
        from std_msgs.msg import Int32
        expected_dictionary = { 'data': -0x7FFFFFFF }
        message = Int32(data=expected_dictionary['data'])
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(dictionary, expected_dictionary)

    def test_ros_message_with_uint32(self):
        from std_msgs.msg import UInt32
        expected_dictionary = { 'data': 0xFFFFFFFF }
        message = UInt32(data=expected_dictionary['data'])
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(dictionary, expected_dictionary)

    def test_ros_message_with_int64(self):
        from std_msgs.msg import Int64
        expected_dictionary = { 'data': -0x7FFFFFFFFFFFFFFF }
        message = Int64(data=expected_dictionary['data'])
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(dictionary, expected_dictionary)

    def test_ros_message_with_uint64(self):
        from std_msgs.msg import UInt64
        expected_dictionary = { 'data': 0xFFFFFFFFFFFFFFFF }
        message = UInt64(data=expected_dictionary['data'])
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(dictionary, expected_dictionary)

    def test_ros_message_with_string(self):
        from std_msgs.msg import String
        expected_dictionary = { 'data': 'Hello' }
        message = String(data=expected_dictionary['data'])
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(dictionary, expected_dictionary)

    def test_ros_message_with_unicode(self):
        """
        Test that strings are encoded as utf8
        """
        from std_msgs.msg import String
        expected_dictionary = { 'data': u'Hello \u00dcnicode' }
        message = String(data=expected_dictionary['data'])
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(dictionary, expected_dictionary)

    def test_ros_message_with_time(self):
        from builtin_interfaces.msg import Time
        now_time = message_converter.get_now_time()
        expected_dictionary = { 
            'sec': now_time.sec, 'nanosec': now_time.nanosec 
        }
        message = Time(sec = now_time.sec, nanosec= now_time.nanosec)
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(dictionary, expected_dictionary)

    def test_ros_message_with_child_message(self):
        from std_msgs.msg import Float64MultiArray, MultiArrayLayout, MultiArrayDimension
        expected_dictionary = {
            'layout': {
                'dim': [
                    { 'label': 'Dimension1', 'size': 12, 'stride': 7 },
                    { 'label': 'Dimension2', 'size': 24, 'stride': 14 }
                ],
                'data_offset': 0
            },
            'data': [1.1, 2.2, 3.3]
        }
        dimension1 = MultiArrayDimension(
            label  = expected_dictionary['layout']['dim'][0]['label'],
            size   = expected_dictionary['layout']['dim'][0]['size'],
            stride = expected_dictionary['layout']['dim'][0]['stride']
        )
        dimension2 = MultiArrayDimension(
            label  = expected_dictionary['layout']['dim'][1]['label'],
            size   = expected_dictionary['layout']['dim'][1]['size'],
            stride = expected_dictionary['layout']['dim'][1]['stride']
        )
        layout = MultiArrayLayout(
            dim = [dimension1, dimension2],
            data_offset = expected_dictionary['layout']['data_offset']
        )
        message = Float64MultiArray(
            layout = layout,
            data   = expected_dictionary['data']
        )
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(dictionary, expected_dictionary)

    def test_ros_message_with_empty_service(self):
        from std_srvs.srv import Empty
        expected_dictionary_req = {}
        expected_dictionary_res = {}
        request = Empty.Request()
        request = serialize_deserialize(request)
        response = Empty.Response()
        response = serialize_deserialize(response)
        dictionary_req = message_converter.convert_ros_message_to_dictionary(request)
        self.assertEqual(dictionary_req, expected_dictionary_req)
        dictionary_res = message_converter.convert_ros_message_to_dictionary(response)
        self.assertEqual(dictionary_res, expected_dictionary_res)

    def test_ros_message_with_nested_service(self):
        from rclpy_message_converter_msgs.srv import NestedUint8ArrayTestService, NestedUint8ArrayTestService
        from rclpy_message_converter_msgs.msg import NestedUint8ArrayTestMessage, Uint8ArrayTestMessage
        expected_data = [97, 98, 99]

        expected_dictionary_req = {"input": {"arrays": [{"data": expected_data}]}}
        expected_dictionary_res = {"output": {"arrays": [{"data": expected_data}]}}
        request = NestedUint8ArrayTestService.Request(
            input=NestedUint8ArrayTestMessage(arrays=[Uint8ArrayTestMessage(data=expected_data)]))
        request = serialize_deserialize(request)
        response = NestedUint8ArrayTestService.Response(
            output=NestedUint8ArrayTestMessage(arrays=[Uint8ArrayTestMessage(data=expected_data)]))
        response = serialize_deserialize(response)

        dictionary_req = message_converter.convert_ros_message_to_dictionary(request)
        self.assertEqual(dictionary_req, expected_dictionary_req)
        dictionary_res = message_converter.convert_ros_message_to_dictionary(response)
        self.assertEqual(dictionary_res, expected_dictionary_res)

    def test_dictionary_with_array(self):
        from rclpy_message_converter_msgs.msg import TestArray
        expected_message = TestArray(data = [1.1, 2.2, 3.3, 4.4])
        dictionary = { 'data': expected_message.data }
        message = message_converter.convert_dictionary_to_ros_message('rclpy_message_converter_msgs/msg/TestArray', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_uint8_array_list(self):
        from rclpy_message_converter_msgs.msg import Uint8ArrayTestMessage
        expected_message = Uint8ArrayTestMessage(data=[1, 2, 3, 4])
        dictionary = {'data': expected_message.data}
        message = message_converter.convert_dictionary_to_ros_message('rclpy_message_converter_msgs/msg/Uint8ArrayTestMessage',
                                                                      dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_uint8_array_list_invalid(self):
        dictionary = {'data': [1, 2, 3, 4000]}
        with self.assertRaises(AssertionError) as context:
            message_converter.convert_dictionary_to_ros_message('rclpy_message_converter_msgs/msg/Uint8ArrayTestMessage',
                                                                dictionary)
        self.assertEqual("The 'data' field must be a set or sequence and each value of type 'int' and each unsigned integer in [0, 255]", context.exception.args[0])

    def test_dictionary_with_3uint8_array_list(self):
        from rclpy_message_converter_msgs.msg import Uint8Array3TestMessage
        expected_message = Uint8Array3TestMessage(data=[97, 98, 99])
        dictionary = {'data': expected_message.data}
        message = message_converter.convert_dictionary_to_ros_message('rclpy_message_converter_msgs/msg/Uint8Array3TestMessage', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_bool(self):
        from std_msgs.msg import Bool
        expected_message = Bool(data = True)
        dictionary = { 'data': expected_message.data }
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/msg/Bool', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_byte(self):
        from std_msgs.msg import Byte
        expected_message = Byte(data = bytes([3]))
        dictionary = { 'data': expected_message.data }
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/msg/Byte', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_char(self):
        from std_msgs.msg import Char
        expected_message = Char(data = 99)
        dictionary = { 'data': expected_message.data }
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/msg/Char', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_duration(self):
        from builtin_interfaces.msg import Duration
        duration = Duration(sec= 33, nanosec = 25)
        expected_message = Duration(sec = duration.sec, nanosec = duration.nanosec)
        dictionary = {
            'sec'  : duration.sec,
            'nanosec' : duration.nanosec
        }
        message = message_converter.convert_dictionary_to_ros_message('builtin_interfaces/msg/Duration', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_empty(self):
        from std_msgs.msg import Empty
        expected_message = Empty()
        dictionary = {}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/msg/Empty', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_empty_additional_args_strict_mode(self):
        dictionary = {"additional_args": "should raise value error"}
        with self.assertRaises(ValueError) as context:
            message_converter.convert_dictionary_to_ros_message('std_msgs/msg/Empty', dictionary)
        self.assertEqual('''ROS message type "std_msgs/msg/Empty" has no field named "additional_args"''',
                         context.exception.args[0])

    def test_dictionary_with_empty_additional_args_forgiving(self):
        from std_msgs.msg import Empty
        expected_message = Empty()
        dictionary = {"additional_args": "should be ignored"}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/msg/Empty', dictionary, strict_mode=False)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_nested_additional_args_strict_mode(self):
        expected_data = [97, 98, 99]
        dictionary = {"arrays": [{"data": expected_data, "additional_args": "should raise value error"}]}
        with self.assertRaises(ValueError) as context:
            message_converter.convert_dictionary_to_ros_message('rclpy_message_converter_msgs/msg/NestedUint8ArrayTestMessage',
                                                                dictionary)
        self.assertEqual(
            'ROS message type "rclpy_message_converter_msgs/msg/Uint8ArrayTestMessage" has no field named "additional_args"',
            context.exception.args[0])

    def test_dictionary_with_nested_additional_args_forgiving(self):
        from rclpy_message_converter_msgs.msg import NestedUint8ArrayTestMessage, Uint8ArrayTestMessage
        from base64 import b64encode
        expected_data = [97, 98, 99]
        expected_message = NestedUint8ArrayTestMessage(arrays=[Uint8ArrayTestMessage(data=expected_data)])
        dictionary = {"arrays": [{"data": expected_data, "additional_args": "should be ignored"}]}
        message = message_converter.convert_dictionary_to_ros_message(
            'rclpy_message_converter_msgs/msg/NestedUint8ArrayTestMessage', dictionary, strict_mode=False)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_missing_field_unchecked(self):
        from std_msgs.msg import Bool
        expected_message = Bool(data=False)
        dictionary = {}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/msg/Bool', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_missing_field_checked(self):
        dictionary = {}
        with self.assertRaises(ValueError) as context:
            message_converter.convert_dictionary_to_ros_message('std_msgs/msg/Bool', dictionary, check_missing_fields=True)
        self.assertEqual('''Missing fields: ['data']''',
                         context.exception.args[0])

    def test_dictionary_with_nested_missing_field_unchecked(self):
        from rclpy_message_converter_msgs.msg import NestedUint8ArrayTestMessage, Uint8ArrayTestMessage
        expected_message = NestedUint8ArrayTestMessage(arrays=[Uint8ArrayTestMessage(data=[])])
        dictionary = {"arrays": [{}]}
        message = message_converter.convert_dictionary_to_ros_message(
            'rclpy_message_converter_msgs/msg/NestedUint8ArrayTestMessage', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_nested_missing_field_checked(self):
        dictionary = {"arrays": [{}]}
        with self.assertRaises(ValueError) as context:
            message_converter.convert_dictionary_to_ros_message('rclpy_message_converter_msgs/msg/NestedUint8ArrayTestMessage',
                                                                dictionary, check_missing_fields=True)
        self.assertEqual('''Missing fields: ['data']''',
                         context.exception.args[0])

    def test_dictionary_with_wrong_type(self):
        dictionary = {"data": "should_be_a_bool"}
        with self.assertRaises(TypeError) as context:
            message_converter.convert_dictionary_to_ros_message('std_msgs/msg/Bool', dictionary)
        self.assertTrue("Field 'data' has wrong type" in context.exception.args[0])

    def test_dictionary_with_float32(self):
        from std_msgs.msg import Float32
        expected_message = Float32(data = struct.unpack('<f', b'\x7F\x7F\xFF\xFD')[0])
        dictionary = { 'data': expected_message.data }
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/msg/Float32', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_float64(self):
        from std_msgs.msg import Float64
        expected_message = Float64(data = struct.unpack('<d', b'\x7F\xEF\xFF\xFF\xFF\xFF\xFF\xFD')[0])
        dictionary = { 'data': expected_message.data }
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/msg/Float64', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_header(self):
        from std_msgs.msg import Header
        from time import time
        now_time = message_converter.get_now_time()
        expected_message = Header(
            stamp = now_time,
            frame_id = 'my_frame',
        )
        dictionary = {
            'stamp': { 'sec': now_time.sec, 'nanosec': now_time.nanosec },
            'frame_id' : expected_message.frame_id,
        }
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/msg/Header', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_header_with_no_prefix(self):
        from std_msgs.msg import Header
        from time import time
        now_time = message_converter.get_now_time()
        expected_message = Header(
            stamp = now_time,
            frame_id = 'my_frame',
        )
        dictionary = {
            'stamp': { 'sec': now_time.sec, 'nanosec': now_time.nanosec },
            'frame_id' : expected_message.frame_id,
        }
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/msg/Header', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_int8(self):
        from std_msgs.msg import Int8
        expected_message = Int8(data = -0x7F)
        dictionary = { 'data': expected_message.data }
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/msg/Int8', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_uint8(self):
        from std_msgs.msg import UInt8
        expected_message = UInt8(data = 0xFF)
        dictionary = { 'data': expected_message.data }
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/msg/UInt8', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_int16(self):
        from std_msgs.msg import Int16
        expected_message = Int16(data = -0x7FFF)
        dictionary = { 'data': expected_message.data }
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/msg/Int16', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_uint16(self):
        from std_msgs.msg import UInt16
        expected_message = UInt16(data = 0xFFFF)
        dictionary = { 'data': expected_message.data }
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/msg/UInt16', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_int32(self):
        from std_msgs.msg import Int32
        expected_message = Int32(data = -0x7FFFFFFF)
        dictionary = { 'data': expected_message.data }
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/msg/Int32', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_uint32(self):
        from std_msgs.msg import UInt32
        expected_message = UInt32(data = 0xFFFFFFFF)
        dictionary = { 'data': expected_message.data }
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/msg/UInt32', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_int64(self):
        from std_msgs.msg import Int64
        expected_message = Int64(data = -0x7FFFFFFFFFFFFFFF)
        dictionary = { 'data': expected_message.data }
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/msg/Int64', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_uint64(self):
        from std_msgs.msg import UInt64
        expected_message = UInt64(data = 0xFFFFFFFFFFFFFFFF)
        dictionary = { 'data': expected_message.data }
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/msg/UInt64', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_string(self):
        from std_msgs.msg import String
        expected_message = String(data = 'Hello')
        dictionary = { 'data': expected_message.data }
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/msg/String', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_unicode(self):
        from std_msgs.msg import String
        expected_message = String(data = u'Hello \u00dcnicode')
        dictionary = { 'data': expected_message.data }
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/msg/String', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message.data,expected_message.data)
        self.assertEqual(type(message.data),type(expected_message.data))

    def test_dictionary_with_time(self):
        from builtin_interfaces.msg import Time
        now_time = message_converter.get_now_time()
        expected_message = Time(sec = now_time.sec, nanosec= now_time.nanosec)
        dictionary = {
            'sec'  : now_time.sec,
            'nanosec' : now_time.nanosec
        }
        message = message_converter.convert_dictionary_to_ros_message('builtin_interfaces/msg/Time', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    # # TODO: Rewrite test once get_time is defined
    # # def test_dictionary_with_time_now(self):
    # #     dictionary = {
    # #         'data': 'now'
    # #     }
    # #     with self.assertRaises(ROSInitException) as context:
    # #         message_converter.convert_dictionary_to_ros_message('builtin_interfaces/msg/Time', dictionary)
    # #     self.assertEqual('time is not initialized. Have you called init_node()?', context.exception.args[0])

    def test_dictionary_with_child_message(self):
        from std_msgs.msg import Float64MultiArray, MultiArrayLayout, MultiArrayDimension
        expected_message = Float64MultiArray(
            layout = MultiArrayLayout(
                dim = [
                    MultiArrayDimension(label = 'Dimension1', size = 12, stride = 7),
                    MultiArrayDimension(label = 'Dimension2', size = 90, stride = 8)
                ],
                data_offset = 1
            ),
            data = [1.1, 2.2, 3.3]
        )
        dictionary = {
            'layout': {
                'dim': [
                    {
                        'label'  : expected_message.layout.dim[0].label,
                        'size'   : expected_message.layout.dim[0].size,
                        'stride' : expected_message.layout.dim[0].stride
                    },
                    {
                        'label'  : expected_message.layout.dim[1].label,
                        'size'   : expected_message.layout.dim[1].size,
                        'stride' : expected_message.layout.dim[1].stride
                    }
                ],
                'data_offset': expected_message.layout.data_offset
            },
            'data': expected_message.data
        }
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/msg/Float64MultiArray', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_invalid_message_fields(self):
        self.assertRaises(ValueError,
                          message_converter.convert_dictionary_to_ros_message,
                          'std_msgs/msg/Empty',
                          {'invalid_field': 1})

    def test_dictionary_with_empty_service(self):
        from std_srvs.srv import Empty
        expected_req = Empty.Request()
        expected_res = Empty.Response()
        dictionary_req = {}
        dictionary_res = {}
        message = message_converter.convert_dictionary_to_ros_message('std_srvs/srv/Empty', dictionary_req,
                                                                      'request')
        expected_req = serialize_deserialize(expected_req)
        self.assertEqual(message, expected_req)
        message = message_converter.convert_dictionary_to_ros_message('std_srvs/srv/Empty', dictionary_res,
                                                                      'response')
        expected_res = serialize_deserialize(expected_res)
        self.assertEqual(message, expected_res)

    def test_dictionary_with_nested_service(self):
        from rclpy_message_converter_msgs.srv import NestedUint8ArrayTestService
        from rclpy_message_converter_msgs.msg import NestedUint8ArrayTestMessage, Uint8ArrayTestMessage
        expected_data = [97, 98, 99]
        expected_req = NestedUint8ArrayTestService.Request(
            input=NestedUint8ArrayTestMessage(arrays=[Uint8ArrayTestMessage(data=expected_data)]))
        expected_req = serialize_deserialize(expected_req)
        expected_res = NestedUint8ArrayTestService.Response(
            output=NestedUint8ArrayTestMessage(arrays=[Uint8ArrayTestMessage(data=expected_data)]))
        expected_res = serialize_deserialize(expected_res)

        dictionary_req = {"input": {"arrays": [{"data": expected_data}]}}
        dictionary_res = {"output": {"arrays": [{"data": expected_data}]}}
        message = message_converter.convert_dictionary_to_ros_message(
            'rclpy_message_converter_msgs/srv/NestedUint8ArrayTestService', dictionary_req, 'request')
        self.assertEqual(message, expected_req)
        message = message_converter.convert_dictionary_to_ros_message(
            'rclpy_message_converter_msgs/srv/NestedUint8ArrayTestService', dictionary_res, 'response')
        self.assertEqual(message, expected_res)

    def test_dictionary_with_setbool_service(self):
        from std_srvs.srv import SetBool
        expected_req = SetBool.Request(data=True)
        expected_res = SetBool.Response(success=True, message='Success!')
        dictionary_req = { 'data': True }
        dictionary_res = { 'success': True, 'message': 'Success!' }
        message = message_converter.convert_dictionary_to_ros_message('std_srvs/srv/SetBool', dictionary_req,
                                                                      'request')
        expected_req = serialize_deserialize(expected_req)
        self.assertEqual(message, expected_req)
        message = message_converter.convert_dictionary_to_ros_message('std_srvs/srv/SetBool', dictionary_res,
                                                                      'response')
        expected_res = serialize_deserialize(expected_res)
        self.assertEqual(message, expected_res)

    def test_dictionary_with_trigger_service(self):
        from std_srvs.srv import Trigger
        expected_req = Trigger.Request()
        expected_res = Trigger.Response(success=True, message='Success!')
        dictionary_req = {}
        dictionary_res = { 'success': True, 'message': 'Success!' }
        message = message_converter.convert_dictionary_to_ros_message('std_srvs/srv/Trigger', dictionary_req,
                                                                      'request')
        expected_req = serialize_deserialize(expected_req)
        self.assertEqual(message, expected_req)
        message = message_converter.convert_dictionary_to_ros_message('std_srvs/srv/Trigger', dictionary_res,
                                                                      'response')
        expected_res = serialize_deserialize(expected_res)
        self.assertEqual(message, expected_res)

    def test_dictionary_with_invalid_kind(self):
        with self.assertRaises(ValueError) as context:
            message_converter.convert_dictionary_to_ros_message('std_msgs/msg/Empty', {}, kind='invalid')
        self.assertEqual('Unknown kind "invalid".', context.exception.args[0])


def serialize_deserialize(msg):
    """
    Serialize and then deserialize a message. This simulates sending a message
    between ROS nodes and makes sure that the ROS messages being tested are
    actually serializable, and are in the same format as they would be received
    over the network. In rospy, it is possible to assign an illegal data type
    to a message field (for example, `message = String(data=42)`), but trying
    to publish this message will throw `SerializationError: field data must be
    of type str`. This method will expose such bugs.
    """
    msg_serialized = serialize_message(msg)
    msg_deserialized = deserialize_message(msg_serialized, type(msg))
    assert msg == msg_deserialized
    return msg_deserialized


PKG = 'rclpy_message_converter'
NAME = 'test_message_converter'
if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, NAME, TestMessageConverter)
