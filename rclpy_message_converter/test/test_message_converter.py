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

import struct
import unittest
import logging

from rclpy.serialization import deserialize_message
from rclpy.serialization import serialize_message

from rclpy_message_converter import message_converter

logging.basicConfig(level=logging.DEBUG)


class TestMessageConverter(unittest.TestCase):
    def test_ros_message_with_array(self):
        from rclpy_message_converter_msgs.msg import TestArray

        expected_dictionary = {'data': [1.1, 2.2, 3.3]}
        message = TestArray(data=expected_dictionary['data'])
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(expected_dictionary, dictionary)

    def test_ros_message_with_bool(self):
        from std_msgs.msg import Bool

        expected_dictionary = {'data': True}
        message = Bool(data=expected_dictionary['data'])
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(expected_dictionary, dictionary)

    def test_ros_message_with_byte(self):
        from std_msgs.msg import Byte

        expected_dictionary = {'data': bytes([5])}
        message = Byte(data=expected_dictionary['data'])
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(expected_dictionary, dictionary)

    def test_ros_message_with_char(self):
        from std_msgs.msg import Char

        expected_dictionary = {'data': 99}
        message = Char(data=expected_dictionary['data'])
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(expected_dictionary, dictionary)

    def test_ros_message_with_duration(self):
        from builtin_interfaces.msg import Duration

        duration = Duration(sec=33, nanosec=25)
        expected_dictionary = {'sec': duration.sec, 'nanosec': duration.nanosec}
        message = Duration(sec=duration.sec, nanosec=duration.nanosec)
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(expected_dictionary, dictionary)

    def test_ros_message_with_empty(self):
        from std_msgs.msg import Empty

        expected_dictionary = {}
        message = Empty()
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(expected_dictionary, dictionary)

    def test_ros_message_with_float32(self):
        from std_msgs.msg import Float32

        expected_dictionary = {'data': struct.unpack('<f', b'\x7F\x7F\xFF\xFD')[0]}
        message = Float32(data=expected_dictionary['data'])
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(expected_dictionary, dictionary)

    def test_ros_message_with_float64(self):
        from std_msgs.msg import Float64

        expected_dictionary = {'data': struct.unpack('<d', b'\x7F\xEF\xFF\xFF\xFF\xFF\xFF\xFD')[0]}
        message = Float64(data=expected_dictionary['data'])
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(expected_dictionary, dictionary)

    def test_ros_message_with_header(self):
        from std_msgs.msg import Header

        now_time = _get_now_time()
        expected_dictionary = {
            'stamp': {'sec': now_time.sec, 'nanosec': now_time.nanosec},
            'frame_id': 'my_frame',
        }
        message = Header(stamp=now_time, frame_id=expected_dictionary['frame_id'])
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(expected_dictionary, dictionary)

    def test_ros_message_with_int8(self):
        from std_msgs.msg import Int8

        expected_dictionary = {'data': -0x7F}
        message = Int8(data=expected_dictionary['data'])
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(expected_dictionary, dictionary)

    def test_ros_message_with_uint8(self):
        from std_msgs.msg import UInt8

        expected_dictionary = {'data': 0xFF}
        message = UInt8(data=expected_dictionary['data'])
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(expected_dictionary, dictionary)

    def test_ros_message_with_uint8_array(self):
        from rclpy_message_converter_msgs.msg import Uint8ArrayTestMessage
        from base64 import b64encode

        expected_data = [97, 98, 99, 100]
        message = Uint8ArrayTestMessage(data=expected_data)
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        expected_data = b64encode(bytearray(expected_data)).decode('utf-8')
        self.assertEqual(expected_data, dictionary["data"])

    def test_ros_message_with_3uint8_array(self):
        from rclpy_message_converter_msgs.msg import Uint8Array3TestMessage
        from base64 import b64encode

        expected_data = [97, 98, 99]
        message = Uint8Array3TestMessage(data=expected_data)
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        expected_data = b64encode(bytearray(expected_data)).decode('utf-8')
        self.assertEqual(expected_data, dictionary["data"])

    def test_ros_message_with_3uint8_array_binary_array_as_array(self):
        from rclpy_message_converter_msgs.msg import Uint8Array3TestMessage

        expected_data = [97, 98, 99]
        message = Uint8Array3TestMessage(data=expected_data)
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message, base64_encoding=False)
        self.assertEqual(expected_data, dictionary["data"])

    def test_ros_message_with_nested_uint8_array_binary_array_as_array(self):
        from rclpy_message_converter_msgs.msg import NestedUint8ArrayTestMessage, Uint8ArrayTestMessage

        expected_data = [97, 98, 99]
        message = NestedUint8ArrayTestMessage(arrays=[Uint8ArrayTestMessage(data=expected_data)])
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message, base64_encoding=False)
        self.assertEqual(expected_data, dictionary["arrays"][0]["data"])

    def test_ros_message_with_nested_uint8_array(self):
        from rclpy_message_converter_msgs.msg import NestedUint8ArrayTestMessage, Uint8ArrayTestMessage
        from base64 import b64encode

        expected_data = bytes(bytearray([97, 98, 99]))
        message = NestedUint8ArrayTestMessage(arrays=[Uint8ArrayTestMessage(data=expected_data)])
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(b64encode(expected_data).decode('utf-8'), dictionary["arrays"][0]["data"])

    def test_ros_message_with_int16(self):
        from std_msgs.msg import Int16

        expected_dictionary = {'data': -0x7FFF}
        message = Int16(data=expected_dictionary['data'])
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(expected_dictionary, dictionary)

    def test_ros_message_with_uint16(self):
        from std_msgs.msg import UInt16

        expected_dictionary = {'data': 0xFFFF}
        message = UInt16(data=expected_dictionary['data'])
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(expected_dictionary, dictionary)

    def test_ros_message_with_int32(self):
        from std_msgs.msg import Int32

        expected_dictionary = {'data': -0x7FFFFFFF}
        message = Int32(data=expected_dictionary['data'])
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(expected_dictionary, dictionary)

    def test_ros_message_with_uint32(self):
        from std_msgs.msg import UInt32

        expected_dictionary = {'data': 0xFFFFFFFF}
        message = UInt32(data=expected_dictionary['data'])
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(expected_dictionary, dictionary)

    def test_ros_message_with_int64(self):
        from std_msgs.msg import Int64

        expected_dictionary = {'data': -0x7FFFFFFFFFFFFFFF}
        message = Int64(data=expected_dictionary['data'])
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(expected_dictionary, dictionary)

    def test_ros_message_with_uint64(self):
        from std_msgs.msg import UInt64

        expected_dictionary = {'data': 0xFFFFFFFFFFFFFFFF}
        message = UInt64(data=expected_dictionary['data'])
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(expected_dictionary, dictionary)

    def test_ros_message_with_string(self):
        from std_msgs.msg import String

        expected_dictionary = {'data': 'Hello'}
        message = String(data=expected_dictionary['data'])
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(expected_dictionary, dictionary)

    def test_ros_message_with_unicode(self):
        """
        Test that strings are encoded as utf8
        """
        from std_msgs.msg import String

        expected_dictionary = {'data': u'Hello \u00dcnicode'}
        message = String(data=expected_dictionary['data'])
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(expected_dictionary, dictionary)

    def test_ros_message_with_time(self):
        from builtin_interfaces.msg import Time

        now_time = _get_now_time()
        expected_dictionary = {'sec': now_time.sec, 'nanosec': now_time.nanosec}
        message = Time(sec=now_time.sec, nanosec=now_time.nanosec)
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(expected_dictionary, dictionary)

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
        expected_dictionary = {
            'transforms': [
                {
                    'header': {
                        'stamp': {'sec': t.header.stamp.sec, 'nanosec': t.header.stamp.nanosec},
                        'frame_id': t.header.frame_id,
                    },
                    'child_frame_id': t.child_frame_id,
                    'transform': {
                        'translation': {
                            'x': t.transform.translation.x,
                            'y': t.transform.translation.y,
                            'z': t.transform.translation.z,
                        },
                        'rotation': {
                            'x': t.transform.rotation.x,
                            'y': t.transform.rotation.y,
                            'z': t.transform.rotation.z,
                            'w': t.transform.rotation.w,
                        },
                    },
                }
            ]
        }
        message = TFMessage(transforms=[t])
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(expected_dictionary, dictionary)

    def test_ros_message_with_child_message(self):
        from std_msgs.msg import Float64MultiArray, MultiArrayLayout, MultiArrayDimension

        expected_dictionary = {
            'layout': {
                'dim': [
                    {'label': 'Dimension1', 'size': 12, 'stride': 7},
                    {'label': 'Dimension2', 'size': 24, 'stride': 14},
                ],
                'data_offset': 0,
            },
            'data': [1.1, 2.2, 3.3],
        }
        dimension1 = MultiArrayDimension(
            label=expected_dictionary['layout']['dim'][0]['label'],
            size=expected_dictionary['layout']['dim'][0]['size'],
            stride=expected_dictionary['layout']['dim'][0]['stride'],
        )
        dimension2 = MultiArrayDimension(
            label=expected_dictionary['layout']['dim'][1]['label'],
            size=expected_dictionary['layout']['dim'][1]['size'],
            stride=expected_dictionary['layout']['dim'][1]['stride'],
        )
        layout = MultiArrayLayout(
            dim=[dimension1, dimension2], data_offset=expected_dictionary['layout']['data_offset']
        )
        message = Float64MultiArray(layout=layout, data=expected_dictionary['data'])
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(expected_dictionary, dictionary)

    def test_ros_message_with_empty_service(self):
        from std_srvs.srv import Empty

        expected_dictionary_req = {}
        expected_dictionary_res = {}
        request = Empty.Request()
        request = serialize_deserialize(request)
        response = Empty.Response()
        response = serialize_deserialize(response)
        dictionary_req = message_converter.convert_ros_message_to_dictionary(request)
        self.assertEqual(expected_dictionary_req, dictionary_req)
        dictionary_res = message_converter.convert_ros_message_to_dictionary(response)
        self.assertEqual(expected_dictionary_res, dictionary_res)

    def test_ros_message_with_nested_service(self):
        from rclpy_message_converter_msgs.srv import NestedUint8ArrayTestService
        from rclpy_message_converter_msgs.msg import NestedUint8ArrayTestMessage, Uint8ArrayTestMessage
        from base64 import b64encode

        expected_data = bytes(bytearray([97, 98, 99]))

        expected_dictionary_req = {"input": {"arrays": [{"data": b64encode(expected_data).decode('utf-8')}]}}
        expected_dictionary_res = {"output": {"arrays": [{"data": b64encode(expected_data).decode('utf-8')}]}}
        request = NestedUint8ArrayTestService.Request(
            input=NestedUint8ArrayTestMessage(arrays=[Uint8ArrayTestMessage(data=expected_data)])
        )
        request = serialize_deserialize(request)
        response = NestedUint8ArrayTestService.Response(
            output=NestedUint8ArrayTestMessage(arrays=[Uint8ArrayTestMessage(data=expected_data)])
        )
        response = serialize_deserialize(response)

        dictionary_req = message_converter.convert_ros_message_to_dictionary(request)
        self.assertEqual(expected_dictionary_req, dictionary_req)
        dictionary_res = message_converter.convert_ros_message_to_dictionary(response)
        self.assertEqual(expected_dictionary_res, dictionary_res)

    def test_dictionary_with_array(self):
        from rclpy_message_converter_msgs.msg import TestArray

        expected_message = TestArray(data=[1.1, 2.2, 3.3, 4.4])
        dictionary = {'data': expected_message.data}
        message = message_converter.convert_dictionary_to_ros_message(
            'rclpy_message_converter_msgs/msg/TestArray', dictionary
        )
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(expected_message, message)

    def test_dictionary_as_class_with_array(self):
        from rclpy_message_converter_msgs.msg import TestArray

        expected_message = TestArray(data=[1.1, 2.2, 3.3, 4.4])
        dictionary = {'data': expected_message.data}
        message = message_converter.convert_dictionary_to_ros_message(TestArray, dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(expected_message, message)

    def test_dictionary_with_uint8_array_bytes(self):
        """
        rospy treats uint8[] data as `bytes`, which is the Python representation for byte data. In Python 2, this is
        the same as `str`. The `bytes` value must be base64-encoded.
        """
        from rclpy_message_converter_msgs.msg import Uint8ArrayTestMessage
        from base64 import b64encode

        expected_message = Uint8ArrayTestMessage(data=bytes(bytearray([97, 98, 99])))
        dictionary = {'data': b64encode(expected_message.data)}  # base64 encoding
        message = message_converter.convert_dictionary_to_ros_message(
            'rclpy_message_converter_msgs/msg/Uint8ArrayTestMessage', dictionary
        )
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(expected_message, message)

    def test_dictionary_with_uint8_array_list(self):
        """
        Even though rospy treats uint8[] data as `bytes`, rclpy_message_converter also handles lists of int. In that
        case, the input data must *not* be base64-encoded.
        """
        from rclpy_message_converter_msgs.msg import Uint8ArrayTestMessage

        expected_message = Uint8ArrayTestMessage(data=[1, 2, 3, 4])
        dictionary = {'data': expected_message.data}  # no base64 encoding
        message = message_converter.convert_dictionary_to_ros_message(
            'rclpy_message_converter_msgs/msg/Uint8ArrayTestMessage', dictionary
        )
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(expected_message, message)

    def test_dictionary_with_uint8_array_list_invalid(self):
        dictionary = {'data': [1, 2, 3, 4000]}
        with self.assertRaises(OverflowError) as context:
            message_converter.convert_dictionary_to_ros_message(
                'rclpy_message_converter_msgs/msg/Uint8ArrayTestMessage', dictionary
            )
        self.assertEqual('unsigned byte integer is greater than maximum', context.exception.args[0])

    def test_dictionary_with_uint8_array_bytes_unencoded(self):
        """
        If the value of a uint8[] field has type `bytes`, rclpy_message_converter expects that data to be
        base64-encoded and runs b64decode on it. This test documents what happens if the value is
        not base64-encoded.
        """
        import binascii

        # this raises a TypeError, because:
        # * b64decode removes all characters that are not in the standard alphabet ([A-Za-Z0-9+/])
        # * this only leaves 97 (= 'a')
        # * the length of a base64 string must be a multiple of 4 characters (if necessary, padded at the end with '=')
        # * since the length of 'a' is not a multiple of 4, a TypeError is thrown
        dictionary = {'data': bytes(bytearray([1, 2, 97, 4]))}
        with self.assertRaises((TypeError, binascii.Error)) as context:
            message_converter.convert_dictionary_to_ros_message(
                'rclpy_message_converter_msgs/msg/Uint8ArrayTestMessage', dictionary
            )
        error_msg = context.exception.args[0]
        self.assertIn(error_msg, ['Incorrect padding', 'Non-base64 digit found'])

        dictionary = {'data': bytes(bytearray([1, 97, 97, 2, 3, 97, 4, 97]))}
        with self.assertRaises(binascii.Error) as context:
            message_converter.convert_dictionary_to_ros_message(
                'rclpy_message_converter_msgs/msg/Uint8ArrayTestMessage', dictionary
            )
        self.assertEqual('Non-base64 digit found', context.exception.args[0])

    def test_dictionary_with_3uint8_array_bytes(self):
        from rclpy_message_converter_msgs.msg import Uint8Array3TestMessage
        from base64 import b64encode

        expected_message = Uint8Array3TestMessage(data=[97, 98, 99])
        dictionary = {'data': b64encode(expected_message.data)}
        message = message_converter.convert_dictionary_to_ros_message(
            'rclpy_message_converter_msgs/msg/Uint8Array3TestMessage', dictionary
        )
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(expected_message, message)

    def test_dictionary_with_3uint8_array_list(self):
        from rclpy_message_converter_msgs.msg import Uint8Array3TestMessage

        expected_message = Uint8Array3TestMessage(data=[97, 98, 99])
        dictionary = {'data': expected_message.data}
        message = message_converter.convert_dictionary_to_ros_message(
            'rclpy_message_converter_msgs/msg/Uint8Array3TestMessage', dictionary
        )
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(expected_message, message)

    def test_dictionary_with_bool(self):
        from std_msgs.msg import Bool

        expected_message = Bool(data=True)
        dictionary = {'data': expected_message.data}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/msg/Bool', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(expected_message, message)

    def test_dictionary_with_byte(self):
        from std_msgs.msg import Byte

        expected_message = Byte(data=bytes([3]))
        dictionary = {'data': expected_message.data}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/msg/Byte', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(expected_message, message)

    def test_dictionary_with_char(self):
        from std_msgs.msg import Char

        expected_message = Char(data=99)
        dictionary = {'data': expected_message.data}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/msg/Char', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(expected_message, message)

    def test_dictionary_with_duration(self):
        from builtin_interfaces.msg import Duration

        duration = Duration(sec=33, nanosec=25)
        expected_message = Duration(sec=duration.sec, nanosec=duration.nanosec)
        dictionary = {'sec': duration.sec, 'nanosec': duration.nanosec}
        message = message_converter.convert_dictionary_to_ros_message('builtin_interfaces/msg/Duration', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(expected_message, message)

    def test_dictionary_with_empty(self):
        from std_msgs.msg import Empty

        expected_message = Empty()
        dictionary = {}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/msg/Empty', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(expected_message, message)

    def test_dictionary_with_empty_additional_args_strict_mode(self):
        dictionary = {"additional_args": "should raise error"}
        with self.assertRaises(AttributeError) as context:
            message_converter.convert_dictionary_to_ros_message('std_msgs/msg/Empty', dictionary)
        self.assertEqual("'Empty' object has no attribute 'additional_args'", context.exception.args[0])

    def test_dictionary_with_empty_additional_args_forgiving(self):
        from std_msgs.msg import Empty

        expected_message = Empty()
        dictionary = {"additional_args": "should be ignored"}
        message = message_converter.convert_dictionary_to_ros_message(
            'std_msgs/msg/Empty', dictionary, strict_mode=False
        )
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(expected_message, message)

    def test_dictionary_with_nested_additional_args_strict_mode(self):
        from base64 import b64encode

        expected_data = bytes(bytearray([97, 98, 99]))
        dictionary = {"arrays": [{"data": b64encode(expected_data), "additional_args": "should raise error"}]}
        with self.assertRaises(AttributeError) as context:
            message_converter.convert_dictionary_to_ros_message(
                'rclpy_message_converter_msgs/msg/NestedUint8ArrayTestMessage', dictionary
            )
        self.assertEqual(
            "'Uint8ArrayTestMessage' object has no attribute 'additional_args'",
            context.exception.args[0],
        )

    def test_dictionary_with_nested_additional_args_forgiving(self):
        from rclpy_message_converter_msgs.msg import NestedUint8ArrayTestMessage, Uint8ArrayTestMessage
        from base64 import b64encode

        expected_data = bytes(bytearray([97, 98, 99]))
        expected_message = NestedUint8ArrayTestMessage(arrays=[Uint8ArrayTestMessage(data=expected_data)])
        dictionary = {"arrays": [{"data": b64encode(expected_data), "additional_args": "should be ignored"}]}
        message = message_converter.convert_dictionary_to_ros_message(
            'rclpy_message_converter_msgs/msg/NestedUint8ArrayTestMessage', dictionary, strict_mode=False
        )
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(expected_message, message)

    def test_dictionary_with_missing_field_unchecked(self):
        from std_msgs.msg import Bool

        expected_message = Bool(data=False)
        dictionary = {}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/msg/Bool', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(expected_message, message)

    def test_dictionary_with_missing_field_checked(self):
        dictionary = {}
        with self.assertRaises(ValueError) as context:
            message_converter.convert_dictionary_to_ros_message(
                'std_msgs/msg/Bool', dictionary, check_missing_fields=True
            )
        self.assertEqual('''fields in dictionary missing from ROS message: "['data']"''', context.exception.args[0])

    def test_dictionary_with_nested_missing_field_unchecked(self):
        from rclpy_message_converter_msgs.msg import NestedUint8ArrayTestMessage, Uint8ArrayTestMessage

        expected_message = NestedUint8ArrayTestMessage(arrays=[Uint8ArrayTestMessage(data=[])])
        dictionary = {"arrays": [{}]}
        message = message_converter.convert_dictionary_to_ros_message(
            'rclpy_message_converter_msgs/msg/NestedUint8ArrayTestMessage', dictionary
        )
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(expected_message, message)

    def test_dictionary_with_nested_missing_field_checked(self):
        dictionary = {"arrays": [{}]}
        with self.assertRaises(ValueError) as context:
            message_converter.convert_dictionary_to_ros_message(
                'rclpy_message_converter_msgs/msg/NestedUint8ArrayTestMessage', dictionary, check_missing_fields=True
            )
        self.assertEqual('''fields in dictionary missing from ROS message: "['data']"''', context.exception.args[0])

    def test_dictionary_with_implicit_conversion(self):
        from std_msgs.msg import Bool

        dictionary = {"data": "should_be_a_bool"}
        # bool('should_be_a_bool') == True (doesn't throw an error)
        expected_message = Bool(data=True)
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/msg/Bool', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(expected_message, message)

    def test_dictionary_with_wrong_type(self):
        dictionary = {"data": "should_be_a_float"}
        with self.assertRaises(ValueError) as context:
            message_converter.convert_dictionary_to_ros_message('std_msgs/msg/Float32', dictionary)
        self.assertTrue("could not convert string to float: 'should_be_a_float'" in context.exception.args[0])

    def test_dictionary_with_float32(self):
        from std_msgs.msg import Float32

        expected_message = Float32(data=struct.unpack('<f', b'\x7F\x7F\xFF\xFD')[0])
        dictionary = {'data': expected_message.data}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/msg/Float32', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(expected_message, message)

    def test_dictionary_with_float64(self):
        from std_msgs.msg import Float64

        expected_message = Float64(data=struct.unpack('<d', b'\x7F\xEF\xFF\xFF\xFF\xFF\xFF\xFD')[0])
        dictionary = {'data': expected_message.data}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/msg/Float64', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(expected_message, message)

    def test_dictionary_with_header(self):
        from std_msgs.msg import Header

        now_time = _get_now_time()
        expected_message = Header(
            stamp=now_time,
            frame_id='my_frame',
        )
        dictionary = {
            'stamp': {'sec': now_time.sec, 'nanosec': now_time.nanosec},
            'frame_id': expected_message.frame_id,
        }
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/msg/Header', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(expected_message, message)

    def test_dictionary_with_header_with_no_prefix(self):
        from std_msgs.msg import Header

        now_time = _get_now_time()
        expected_message = Header(
            stamp=now_time,
            frame_id='my_frame',
        )
        dictionary = {
            'stamp': {'sec': now_time.sec, 'nanosec': now_time.nanosec},
            'frame_id': expected_message.frame_id,
        }
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/msg/Header', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(expected_message, message)

    def test_dictionary_with_int8(self):
        from std_msgs.msg import Int8

        expected_message = Int8(data=-0x7F)
        dictionary = {'data': expected_message.data}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/msg/Int8', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(expected_message, message)

    def test_dictionary_with_uint8(self):
        from std_msgs.msg import UInt8

        expected_message = UInt8(data=0xFF)
        dictionary = {'data': expected_message.data}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/msg/UInt8', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(expected_message, message)

    def test_dictionary_with_int16(self):
        from std_msgs.msg import Int16

        expected_message = Int16(data=-0x7FFF)
        dictionary = {'data': expected_message.data}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/msg/Int16', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(expected_message, message)

    def test_dictionary_with_uint16(self):
        from std_msgs.msg import UInt16

        expected_message = UInt16(data=0xFFFF)
        dictionary = {'data': expected_message.data}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/msg/UInt16', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(expected_message, message)

    def test_dictionary_with_int32(self):
        from std_msgs.msg import Int32

        expected_message = Int32(data=-0x7FFFFFFF)
        dictionary = {'data': expected_message.data}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/msg/Int32', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(expected_message, message)

    def test_dictionary_with_uint32(self):
        from std_msgs.msg import UInt32

        expected_message = UInt32(data=0xFFFFFFFF)
        dictionary = {'data': expected_message.data}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/msg/UInt32', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(expected_message, message)

    def test_dictionary_with_int64(self):
        from std_msgs.msg import Int64

        expected_message = Int64(data=-0x7FFFFFFFFFFFFFFF)
        dictionary = {'data': expected_message.data}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/msg/Int64', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(expected_message, message)

    def test_dictionary_with_uint64(self):
        from std_msgs.msg import UInt64

        expected_message = UInt64(data=0xFFFFFFFFFFFFFFFF)
        dictionary = {'data': expected_message.data}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/msg/UInt64', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(expected_message, message)

    def test_dictionary_with_string(self):
        from std_msgs.msg import String

        expected_message = String(data='Hello')
        dictionary = {'data': expected_message.data}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/msg/String', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(expected_message, message)

    def test_dictionary_with_unicode(self):
        from std_msgs.msg import String

        expected_message = String(data=u'Hello \u00dcnicode')
        dictionary = {'data': expected_message.data}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/msg/String', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(expected_message.data, message.data)
        self.assertEqual(type(expected_message.data), type(message.data))

    def test_dictionary_with_time(self):
        from builtin_interfaces.msg import Time

        now_time = _get_now_time()
        expected_message = Time(sec=now_time.sec, nanosec=now_time.nanosec)
        dictionary = {'sec': now_time.sec, 'nanosec': now_time.nanosec}
        message = message_converter.convert_dictionary_to_ros_message('builtin_interfaces/msg/Time', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(expected_message, message)

    def test_dictionary_with_tfmessage(self):
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
        dictionary = {
            'transforms': [
                {
                    'header': {
                        'stamp': {'sec': t.header.stamp.sec, 'nanosec': t.header.stamp.nanosec},
                        'frame_id': t.header.frame_id,
                    },
                    'child_frame_id': t.child_frame_id,
                    'transform': {
                        'translation': {
                            'x': t.transform.translation.x,
                            'y': t.transform.translation.y,
                            'z': t.transform.translation.z,
                        },
                        'rotation': {
                            'x': t.transform.rotation.x,
                            'y': t.transform.rotation.y,
                            'z': t.transform.rotation.z,
                            'w': t.transform.rotation.w,
                        },
                    },
                }
            ]
        }
        message = message_converter.convert_dictionary_to_ros_message('tf2_msgs/msg/TFMessage', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(expected_message, message)

    def test_dictionary_with_child_message(self):
        from std_msgs.msg import Float64MultiArray, MultiArrayLayout, MultiArrayDimension

        expected_message = Float64MultiArray(
            layout=MultiArrayLayout(
                dim=[
                    MultiArrayDimension(label='Dimension1', size=12, stride=7),
                    MultiArrayDimension(label='Dimension2', size=90, stride=8),
                ],
                data_offset=1,
            ),
            data=[1.1, 2.2, 3.3],
        )
        dictionary = {
            'layout': {
                'dim': [
                    {
                        'label': expected_message.layout.dim[0].label,
                        'size': expected_message.layout.dim[0].size,
                        'stride': expected_message.layout.dim[0].stride,
                    },
                    {
                        'label': expected_message.layout.dim[1].label,
                        'size': expected_message.layout.dim[1].size,
                        'stride': expected_message.layout.dim[1].stride,
                    },
                ],
                'data_offset': expected_message.layout.data_offset,
            },
            'data': expected_message.data,
        }
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/msg/Float64MultiArray', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(expected_message, message)

    def test_dictionary_with_empty_service(self):
        from std_srvs.srv import Empty

        expected_req = Empty.Request()
        expected_res = Empty.Response()
        dictionary_req = {}
        dictionary_res = {}
        message = message_converter.convert_dictionary_to_ros_message('std_srvs/srv/Empty', dictionary_req, 'request')
        expected_req = serialize_deserialize(expected_req)
        self.assertEqual(expected_req, message)
        message = message_converter.convert_dictionary_to_ros_message('std_srvs/srv/Empty', dictionary_res, 'response')
        expected_res = serialize_deserialize(expected_res)
        self.assertEqual(expected_res, message)

    def test_dictionary_with_nested_service(self):
        from rclpy_message_converter_msgs.srv import NestedUint8ArrayTestService
        from rclpy_message_converter_msgs.msg import NestedUint8ArrayTestMessage, Uint8ArrayTestMessage
        from base64 import b64encode

        expected_data = bytes(bytearray([97, 98, 99]))
        expected_req = NestedUint8ArrayTestService.Request(
            input=NestedUint8ArrayTestMessage(arrays=[Uint8ArrayTestMessage(data=expected_data)])
        )
        expected_req = serialize_deserialize(expected_req)
        expected_res = NestedUint8ArrayTestService.Response(
            output=NestedUint8ArrayTestMessage(arrays=[Uint8ArrayTestMessage(data=expected_data)])
        )
        expected_res = serialize_deserialize(expected_res)

        dictionary_req = {"input": {"arrays": [{"data": b64encode(expected_data)}]}}
        dictionary_res = {"output": {"arrays": [{"data": b64encode(expected_data)}]}}
        message = message_converter.convert_dictionary_to_ros_message(
            'rclpy_message_converter_msgs/srv/NestedUint8ArrayTestService', dictionary_req, 'request'
        )
        self.assertEqual(expected_req, message)
        message = message_converter.convert_dictionary_to_ros_message(
            'rclpy_message_converter_msgs/srv/NestedUint8ArrayTestService', dictionary_res, 'response'
        )
        self.assertEqual(expected_res, message)

    def test_dictionary_with_setbool_service(self):
        from std_srvs.srv import SetBool

        expected_req = SetBool.Request(data=True)
        expected_res = SetBool.Response(success=True, message='Success!')
        dictionary_req = {'data': True}
        dictionary_res = {'success': True, 'message': 'Success!'}
        message = message_converter.convert_dictionary_to_ros_message('std_srvs/srv/SetBool', dictionary_req, 'request')
        expected_req = serialize_deserialize(expected_req)
        self.assertEqual(expected_req, message)
        message = message_converter.convert_dictionary_to_ros_message(
            'std_srvs/srv/SetBool', dictionary_res, 'response'
        )
        expected_res = serialize_deserialize(expected_res)
        self.assertEqual(expected_res, message)

    def test_dictionary_with_trigger_service(self):
        from std_srvs.srv import Trigger

        expected_req = Trigger.Request()
        expected_res = Trigger.Response(success=True, message='Success!')
        dictionary_req = {}
        dictionary_res = {'success': True, 'message': 'Success!'}
        message = message_converter.convert_dictionary_to_ros_message('std_srvs/srv/Trigger', dictionary_req, 'request')
        expected_req = serialize_deserialize(expected_req)
        self.assertEqual(expected_req, message)
        message = message_converter.convert_dictionary_to_ros_message(
            'std_srvs/srv/Trigger', dictionary_res, 'response'
        )
        expected_res = serialize_deserialize(expected_res)
        self.assertEqual(expected_res, message)

    def test_dictionary_with_invalid_kind(self):
        with self.assertRaises(ValueError) as context:
            message_converter.convert_dictionary_to_ros_message('std_msgs/msg/Empty', {}, kind='invalid')
        self.assertEqual('Unknown kind "invalid".', context.exception.args[0])

    # -------- Tests for None: --------
    def test_dictionary_with_array_none(self):
        from rclpy_message_converter_msgs.msg import TestArray

        expected_message = TestArray(data=[])
        dictionary = {'data': None}
        message = message_converter.convert_dictionary_to_ros_message(
            'rclpy_message_converter_msgs/msg/TestArray', dictionary
        )
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(expected_message, message)

    def test_dictionary_with_uint8_array_none(self):
        from rclpy_message_converter_msgs.msg import Uint8ArrayTestMessage

        expected_message = Uint8ArrayTestMessage(data=bytes())
        dictionary = {'data': None}
        message = message_converter.convert_dictionary_to_ros_message(
            'rclpy_message_converter_msgs/msg/Uint8ArrayTestMessage', dictionary
        )
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(expected_message, message)

    def test_dictionary_with_3uint8_array_none(self):
        from rclpy_message_converter_msgs.msg import Uint8Array3TestMessage

        expected_message = Uint8Array3TestMessage(data=[0, 0, 0])
        dictionary = {'data': None}
        message = message_converter.convert_dictionary_to_ros_message(
            'rclpy_message_converter_msgs/msg/Uint8Array3TestMessage', dictionary
        )
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(expected_message, message)

    def test_dictionary_with_bool_none(self):
        from std_msgs.msg import Bool

        expected_message = Bool(data=False)
        dictionary = {'data': None}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/msg/Bool', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(expected_message, message)

    def test_dictionary_with_byte_none(self):
        from std_msgs.msg import Byte

        expected_message = Byte(data=bytes([0]))
        dictionary = {'data': None}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/msg/Byte', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(expected_message, message)

    def test_dictionary_with_char_none(self):
        from std_msgs.msg import Char

        expected_message = Char(data=0)
        dictionary = {'data': None}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/msg/Char', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(expected_message, message)

    def test_dictionary_with_duration_none(self):
        from builtin_interfaces.msg import Duration

        expected_message = Duration()
        dictionary = {'sec': None, 'nanosec': None}
        message = message_converter.convert_dictionary_to_ros_message('builtin_interfaces/msg/Duration', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(expected_message, message)

    def test_dictionary_with_float32_none(self):
        from std_msgs.msg import Float32

        expected_message = Float32(data=0.0)
        dictionary = {'data': None}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/msg/Float32', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(expected_message, message)

    def test_dictionary_with_float64_none(self):
        from std_msgs.msg import Float64

        expected_message = Float64(data=0.0)
        dictionary = {'data': None}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/msg/Float64', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(expected_message, message)

    def test_dictionary_with_header_none(self):
        from std_msgs.msg import Header

        expected_message = Header()
        dictionary = {'stamp': {'sec': None, 'nanosec': 0.0}, 'frame_id': None}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/msg/Header', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(expected_message, message)

    def test_dictionary_with_int8_none(self):
        from std_msgs.msg import Int8

        expected_message = Int8(data=0)
        dictionary = {'data': None}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/msg/Int8', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(expected_message, message)

    def test_dictionary_with_uint8_none(self):
        from std_msgs.msg import UInt8

        expected_message = UInt8(data=0)
        dictionary = {'data': None}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/msg/UInt8', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(expected_message, message)

    def test_dictionary_with_int16_none(self):
        from std_msgs.msg import Int16

        expected_message = Int16(data=0)
        dictionary = {'data': None}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/msg/Int16', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(expected_message, message)

    def test_dictionary_with_uint16_none(self):
        from std_msgs.msg import UInt16

        expected_message = UInt16(data=0)
        dictionary = {'data': None}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/msg/UInt16', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(expected_message, message)

    def test_dictionary_with_int32_none(self):
        from std_msgs.msg import Int32

        expected_message = Int32(data=0)
        dictionary = {'data': None}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/msg/Int32', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(expected_message, message)

    def test_dictionary_with_uint32_none(self):
        from std_msgs.msg import UInt32

        expected_message = UInt32(data=0)
        dictionary = {'data': None}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/msg/UInt32', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(expected_message, message)

    def test_dictionary_with_int64_none(self):
        from std_msgs.msg import Int64

        expected_message = Int64(data=0)
        dictionary = {'data': None}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/msg/Int64', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(expected_message, message)

    def test_dictionary_with_uint64_none(self):
        from std_msgs.msg import UInt64

        expected_message = UInt64(data=0)
        dictionary = {'data': None}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/msg/UInt64', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(expected_message, message)

    def test_dictionary_with_string_none(self):
        from std_msgs.msg import String

        expected_message = String(data='')
        dictionary = {'data': None}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/msg/String', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(expected_message, message)

    def test_dictionary_with_time_none(self):
        from builtin_interfaces.msg import Time

        expected_message = Time(sec=0, nanosec=123456789)
        dictionary = {'sec': None, 'nanosec': expected_message.nanosec}
        message = message_converter.convert_dictionary_to_ros_message('builtin_interfaces/msg/Time', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(expected_message, message)

    def test_dictionary_with_child_message_none(self):
        from std_msgs.msg import Float64MultiArray, MultiArrayLayout, MultiArrayDimension

        expected_message = Float64MultiArray(
            layout=MultiArrayLayout(
                dim=[
                    MultiArrayDimension(label='', size=0, stride=0),
                    MultiArrayDimension(label='Dimension2', size=90, stride=8),
                ],
                data_offset=1,
            ),
            data=[1.1, 2.2, 3.3],
        )
        dictionary = {
            'layout': {
                'dim': [
                    None,
                    {
                        'label': expected_message.layout.dim[1].label,
                        'size': expected_message.layout.dim[1].size,
                        'stride': expected_message.layout.dim[1].stride,
                    },
                ],
                'data_offset': expected_message.layout.data_offset,
            },
            'data': expected_message.data,
        }
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/msg/Float64MultiArray', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(expected_message, message)


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

    This simulates sending a message between ROS nodes and makes sure that the ROS messages being
    tested are actually serializable, and are in the same format as they would be received
    over the network. In rospy, it is possible to assign an illegal data type
    to a message field (for example, `message = String(data=42)`), but trying
    to publish this message will throw `SerializationError: field data must be
    of type str`. This method will expose such bugs.
    """
    msg_serialized = serialize_message(msg)
    msg_deserialized = deserialize_message(msg_serialized, type(msg))
    assert msg == msg_deserialized
    return msg_deserialized
