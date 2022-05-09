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

import numpy as np
import struct
import sys
import unittest

import rospy
from rospy.exceptions import ROSInitException
from rospy_message_converter import message_converter

python3 = sys.hexversion > 0x03000000


class TestMessageConverter(unittest.TestCase):
    def test_ros_message_with_array(self):
        from rospy_message_converter.msg import TestArray

        expected_dictionary = {'data': [1.1, 2.2, 3.3]}
        message = TestArray(data=expected_dictionary['data'])
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(dictionary, expected_dictionary)

    def test_ros_message_with_bool(self):
        from std_msgs.msg import Bool

        expected_dictionary = {'data': True}
        message = Bool(data=expected_dictionary['data'])
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(dictionary, expected_dictionary)

    def test_ros_message_with_byte(self):
        from std_msgs.msg import Byte

        expected_dictionary = {'data': 5}
        message = Byte(data=expected_dictionary['data'])
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(dictionary, expected_dictionary)

    def test_ros_message_with_char(self):
        from std_msgs.msg import Char

        expected_dictionary = {'data': 99}
        message = Char(data=expected_dictionary['data'])
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(dictionary, expected_dictionary)

    def test_ros_message_with_duration(self):
        from std_msgs.msg import Duration

        duration = rospy.rostime.Duration(33, 25)
        expected_dictionary = {'data': {'secs': duration.secs, 'nsecs': duration.nsecs}}
        message = Duration(data=duration)
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

        expected_dictionary = {'data': struct.unpack('<f', b'\x7F\x7F\xFF\xFD')[0]}
        message = Float32(data=expected_dictionary['data'])
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(dictionary, expected_dictionary)

    def test_ros_message_with_float64(self):
        from std_msgs.msg import Float64

        expected_dictionary = {'data': struct.unpack('<d', b'\x7F\xEF\xFF\xFF\xFF\xFF\xFF\xFD')[0]}
        message = Float64(data=expected_dictionary['data'])
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(dictionary, expected_dictionary)

    def test_ros_message_with_header(self):
        from std_msgs.msg import Header
        from time import time

        now_time = rospy.Time(time())
        expected_dictionary = {
            'stamp': {'secs': now_time.secs, 'nsecs': now_time.nsecs},
            'frame_id': 'my_frame',
            'seq': 3,
        }
        message = Header(stamp=now_time, frame_id=expected_dictionary['frame_id'], seq=expected_dictionary['seq'])
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(dictionary, expected_dictionary)

    def test_ros_message_with_int8(self):
        from std_msgs.msg import Int8

        expected_dictionary = {'data': -0x7F}
        message = Int8(data=expected_dictionary['data'])
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(dictionary, expected_dictionary)

    def test_ros_message_with_uint8(self):
        from std_msgs.msg import UInt8

        expected_dictionary = {'data': 0xFF}
        message = UInt8(data=expected_dictionary['data'])
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(dictionary, expected_dictionary)

    def test_ros_message_with_uint8_array(self):
        from rospy_message_converter.msg import Uint8ArrayTestMessage
        from base64 import b64encode

        expected_data = [97, 98, 99, 100]
        message = Uint8ArrayTestMessage(data=expected_data)
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        expected_data = b64encode(bytearray(expected_data)).decode('utf-8')
        self.assertEqual(dictionary["data"], expected_data)

    def test_ros_message_with_3uint8_array(self):
        from rospy_message_converter.msg import Uint8Array3TestMessage
        from base64 import b64encode

        expected_data = [97, 98, 99]
        message = Uint8Array3TestMessage(data=expected_data)
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        expected_data = b64encode(bytearray(expected_data)).decode('utf-8')
        self.assertEqual(dictionary["data"], expected_data)

    def test_ros_message_with_3uint8_array_binary_array_as_array(self):
        from rospy_message_converter.msg import Uint8Array3TestMessage

        expected_data = [97, 98, 99]
        message = Uint8Array3TestMessage(data=expected_data)
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message, binary_array_as_bytes=False)
        self.assertEqual(dictionary["data"], expected_data)

    def test_ros_message_with_nested_uint8_array_binary_array_as_array(self):
        from rospy_message_converter.msg import NestedUint8ArrayTestMessage, Uint8ArrayTestMessage

        expected_data = [97, 98, 99]
        message = NestedUint8ArrayTestMessage(arrays=[Uint8ArrayTestMessage(data=expected_data)])
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message, binary_array_as_bytes=False)
        self.assertEqual(dictionary["arrays"][0]["data"], expected_data)

    def test_ros_message_with_nested_uint8_array(self):
        from rospy_message_converter.msg import NestedUint8ArrayTestMessage, Uint8ArrayTestMessage
        from base64 import b64encode

        expected_data = bytes(bytearray([97, 98, 99]))
        message = NestedUint8ArrayTestMessage(arrays=[Uint8ArrayTestMessage(data=expected_data)])
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(dictionary["arrays"][0]["data"], b64encode(expected_data).decode('utf-8'))

    def test_ros_message_with_int16(self):
        from std_msgs.msg import Int16

        expected_dictionary = {'data': -0x7FFF}
        message = Int16(data=expected_dictionary['data'])
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(dictionary, expected_dictionary)

    def test_ros_message_with_uint16(self):
        from std_msgs.msg import UInt16

        expected_dictionary = {'data': 0xFFFF}
        message = UInt16(data=expected_dictionary['data'])
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(dictionary, expected_dictionary)

    def test_ros_message_with_int32(self):
        from std_msgs.msg import Int32

        expected_dictionary = {'data': -0x7FFFFFFF}
        message = Int32(data=expected_dictionary['data'])
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(dictionary, expected_dictionary)

    def test_ros_message_with_uint32(self):
        from std_msgs.msg import UInt32

        expected_dictionary = {'data': 0xFFFFFFFF}
        message = UInt32(data=expected_dictionary['data'])
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(dictionary, expected_dictionary)

    def test_ros_message_with_int64(self):
        from std_msgs.msg import Int64

        expected_dictionary = {'data': -0x7FFFFFFFFFFFFFFF}
        message = Int64(data=expected_dictionary['data'])
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(dictionary, expected_dictionary)

    def test_ros_message_with_uint64(self):
        from std_msgs.msg import UInt64

        expected_dictionary = {'data': 0xFFFFFFFFFFFFFFFF}
        message = UInt64(data=expected_dictionary['data'])
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(dictionary, expected_dictionary)

    def test_ros_message_with_string(self):
        from std_msgs.msg import String

        expected_dictionary = {'data': 'Hello'}
        message = String(data=expected_dictionary['data'])
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(dictionary, expected_dictionary)

    def test_ros_message_with_unicode(self):
        """
        Test that strings are encoded as utf8
        """
        from std_msgs.msg import String

        expected_dictionary = {'data': u'Hello \u00dcnicode'}
        message = String(data=expected_dictionary['data'])
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(dictionary, expected_dictionary)

    def test_ros_message_with_time(self):
        from std_msgs.msg import Time
        from time import time

        now_time = rospy.Time(time())
        expected_dictionary = {'data': {'secs': now_time.secs, 'nsecs': now_time.nsecs}}
        message = Time(data=now_time)
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(dictionary, expected_dictionary)

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
        self.assertEqual(dictionary, expected_dictionary)

    def test_ros_message_with_empty_service(self):
        from std_srvs.srv import EmptyRequest, EmptyResponse

        expected_dictionary_req = {}
        expected_dictionary_res = {}
        request = EmptyRequest()
        request = serialize_deserialize(request)
        response = EmptyResponse()
        response = serialize_deserialize(response)
        dictionary_req = message_converter.convert_ros_message_to_dictionary(request)
        self.assertEqual(dictionary_req, expected_dictionary_req)
        dictionary_res = message_converter.convert_ros_message_to_dictionary(response)
        self.assertEqual(dictionary_res, expected_dictionary_res)

    def test_ros_message_with_nested_service(self):
        from rospy_message_converter.srv import NestedUint8ArrayTestServiceRequest, NestedUint8ArrayTestServiceResponse
        from rospy_message_converter.msg import NestedUint8ArrayTestMessage, Uint8ArrayTestMessage
        from base64 import b64encode

        expected_data = bytes(bytearray([97, 98, 99]))

        expected_dictionary_req = {"input": {"arrays": [{"data": b64encode(expected_data).decode('utf-8')}]}}
        expected_dictionary_res = {"output": {"arrays": [{"data": b64encode(expected_data).decode('utf-8')}]}}
        request = NestedUint8ArrayTestServiceRequest(
            input=NestedUint8ArrayTestMessage(arrays=[Uint8ArrayTestMessage(data=expected_data)])
        )
        request = serialize_deserialize(request)
        response = NestedUint8ArrayTestServiceResponse(
            output=NestedUint8ArrayTestMessage(arrays=[Uint8ArrayTestMessage(data=expected_data)])
        )
        response = serialize_deserialize(response)

        dictionary_req = message_converter.convert_ros_message_to_dictionary(request)
        self.assertEqual(dictionary_req, expected_dictionary_req)
        dictionary_res = message_converter.convert_ros_message_to_dictionary(response)
        self.assertEqual(dictionary_res, expected_dictionary_res)

    def test_dictionary_with_array(self):
        from rospy_message_converter.msg import TestArray

        expected_message = TestArray(data=[1.1, 2.2, 3.3, 4.4])
        dictionary = {'data': expected_message.data}
        message = message_converter.convert_dictionary_to_ros_message('rospy_message_converter/TestArray', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_uint8_array_bytes(self):
        """
        rospy treats uint8[] data as `bytes`, which is the Python representation for byte data. In Python 2, this is
        the same as `str`. The `bytes` value must be base64-encoded.
        """
        from rospy_message_converter.msg import Uint8ArrayTestMessage
        from base64 import b64encode

        expected_message = Uint8ArrayTestMessage(data=bytes(bytearray([97, 98, 99])))
        dictionary = {'data': b64encode(expected_message.data)}  # base64 encoding
        message = message_converter.convert_dictionary_to_ros_message(
            'rospy_message_converter/Uint8ArrayTestMessage', dictionary
        )
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_uint8_array_list(self):
        """
        Even though rospy treats uint8[] data as `bytes`, rospy_message_converter also handles lists of int. In that
        case, the input data must *not* be base64-encoded.
        """
        from rospy_message_converter.msg import Uint8ArrayTestMessage

        expected_message = Uint8ArrayTestMessage(data=[1, 2, 3, 4])
        dictionary = {'data': expected_message.data}  # no base64 encoding
        message = message_converter.convert_dictionary_to_ros_message(
            'rospy_message_converter/Uint8ArrayTestMessage', dictionary
        )
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_uint8_array_list_invalid(self):
        dictionary = {'data': [1, 2, 3, 4000]}
        with self.assertRaises(ValueError) as context:
            message_converter.convert_dictionary_to_ros_message(
                'rospy_message_converter/Uint8ArrayTestMessage', dictionary
            )
        self.assertEqual('byte must be in range(0, 256)', context.exception.args[0])

    def test_dictionary_with_uint8_array_bytes_unencoded(self):
        """
        If the value of a uint8[] field has type `bytes`, rospy_message_converter expects that data to be
        base64-encoded and runs b64decode on it. This test documents what happens if the value is
        not base64-encoded.
        """
        from rospy_message_converter.msg import Uint8ArrayTestMessage
        import binascii

        # this raises a TypeError, because:
        # * b64decode removes all characters that are not in the standard alphabet ([A-Za-Z0-9+/])
        # * this only leaves 97 (= 'a')
        # * the length of a base64 string must be a multiple of 4 characters (if necessary, padded at the end with '=')
        # * since the length of 'a' is not a multiple of 4, a TypeError is thrown
        dictionary = {'data': bytes(bytearray([1, 2, 97, 4]))}
        with self.assertRaises((TypeError, binascii.Error)) as context:
            message_converter.convert_dictionary_to_ros_message(
                'rospy_message_converter/Uint8ArrayTestMessage', dictionary
            )
        if type(context.exception) == TypeError:  # python2
            error_msg = context.exception.args[0].args[0]
        else:  # python3
            error_msg = context.exception.args[0]
        self.assertIn(error_msg, ['Incorrect padding', 'Non-base64 digit found'])

        dictionary = {'data': bytes(bytearray([1, 97, 97, 2, 3, 97, 4, 97]))}
        if python3:
            # On python3, we validate the input, so an error is raised.
            with self.assertRaises(binascii.Error) as context:
                message_converter.convert_dictionary_to_ros_message(
                    'rospy_message_converter/Uint8ArrayTestMessage', dictionary
                )
            self.assertEqual('Non-base64 digit found', context.exception.args[0])
        else:
            # if the dictionary contains a multiple of 4 characters from the standard alphabet, no error is raised
            # (but the result is garbage).
            message = message_converter.convert_dictionary_to_ros_message(
                'rospy_message_converter/Uint8ArrayTestMessage', dictionary
            )
            expected_message = serialize_deserialize(Uint8ArrayTestMessage(data=bytes(bytearray([105, 166, 154]))))
            self.assertEqual(message, expected_message)

    def test_dictionary_with_3uint8_array_bytes(self):
        from rospy_message_converter.msg import Uint8Array3TestMessage
        from base64 import b64encode

        expected_message = Uint8Array3TestMessage(data=bytes(bytearray([97, 98, 99])))
        dictionary = {'data': b64encode(expected_message.data)}
        message = message_converter.convert_dictionary_to_ros_message(
            'rospy_message_converter/Uint8Array3TestMessage', dictionary
        )
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_3uint8_array_list(self):
        from rospy_message_converter.msg import Uint8Array3TestMessage

        expected_message = Uint8Array3TestMessage(data=[97, 98, 99])
        dictionary = {'data': expected_message.data}
        message = message_converter.convert_dictionary_to_ros_message(
            'rospy_message_converter/Uint8Array3TestMessage', dictionary
        )
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_bool(self):
        from std_msgs.msg import Bool

        expected_message = Bool(data=True)
        dictionary = {'data': expected_message.data}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/Bool', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_byte(self):
        from std_msgs.msg import Byte

        expected_message = Byte(data=3)
        dictionary = {'data': expected_message.data}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/Byte', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_char(self):
        from std_msgs.msg import Char

        expected_message = Char(data=99)
        dictionary = {'data': expected_message.data}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/Char', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_duration(self):
        from std_msgs.msg import Duration

        duration = rospy.rostime.Duration(33, 25)
        expected_message = Duration(data=duration)
        dictionary = {'data': {'secs': duration.secs, 'nsecs': duration.nsecs}}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/Duration', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_empty(self):
        from std_msgs.msg import Empty

        expected_message = Empty()
        dictionary = {}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/Empty', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_empty_additional_args_strict_mode(self):
        dictionary = {"additional_args": "should raise value error"}
        with self.assertRaises(ValueError) as context:
            message_converter.convert_dictionary_to_ros_message('std_msgs/Empty', dictionary)
        self.assertEqual(
            '''ROS message type "std_msgs/Empty" has no field named "additional_args"''', context.exception.args[0]
        )

    def test_dictionary_with_empty_additional_args_forgiving(self):
        from std_msgs.msg import Empty

        expected_message = Empty()
        dictionary = {"additional_args": "should be ignored"}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/Empty', dictionary, strict_mode=False)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_nested_additional_args_strict_mode(self):
        from base64 import b64encode

        expected_data = bytes(bytearray([97, 98, 99]))
        dictionary = {"arrays": [{"data": b64encode(expected_data), "additional_args": "should raise value error"}]}
        with self.assertRaises(ValueError) as context:
            message_converter.convert_dictionary_to_ros_message(
                'rospy_message_converter/NestedUint8ArrayTestMessage', dictionary
            )
        self.assertEqual(
            'ROS message type "rospy_message_converter/Uint8ArrayTestMessage" has no field named "additional_args"',
            context.exception.args[0],
        )

    def test_dictionary_with_nested_additional_args_forgiving(self):
        from rospy_message_converter.msg import NestedUint8ArrayTestMessage, Uint8ArrayTestMessage
        from base64 import b64encode

        expected_data = bytes(bytearray([97, 98, 99]))
        expected_message = NestedUint8ArrayTestMessage(arrays=[Uint8ArrayTestMessage(data=expected_data)])
        dictionary = {"arrays": [{"data": b64encode(expected_data), "additional_args": "should be ignored"}]}
        message = message_converter.convert_dictionary_to_ros_message(
            'rospy_message_converter/NestedUint8ArrayTestMessage', dictionary, strict_mode=False
        )
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_missing_field_unchecked(self):
        from std_msgs.msg import Bool

        expected_message = Bool(data=False)
        dictionary = {}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/Bool', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_missing_field_checked(self):
        dictionary = {}
        with self.assertRaises(ValueError) as context:
            message_converter.convert_dictionary_to_ros_message('std_msgs/Bool', dictionary, check_missing_fields=True)
        self.assertEqual('''Missing fields "{'data': 'bool'}"''', context.exception.args[0])

    def test_dictionary_with_nested_missing_field_unchecked(self):
        from rospy_message_converter.msg import NestedUint8ArrayTestMessage, Uint8ArrayTestMessage

        expected_message = NestedUint8ArrayTestMessage(arrays=[Uint8ArrayTestMessage(data=[])])
        dictionary = {"arrays": [{}]}
        message = message_converter.convert_dictionary_to_ros_message(
            'rospy_message_converter/NestedUint8ArrayTestMessage', dictionary
        )
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_nested_missing_field_checked(self):
        dictionary = {"arrays": [{}]}
        with self.assertRaises(ValueError) as context:
            message_converter.convert_dictionary_to_ros_message(
                'rospy_message_converter/NestedUint8ArrayTestMessage', dictionary, check_missing_fields=True
            )
        self.assertEqual('''Missing fields "{'data': 'uint8[]'}"''', context.exception.args[0])

    def test_dictionary_with_wrong_type(self):
        dictionary = {"data": "should_be_a_bool"}
        with self.assertRaises(TypeError) as context:
            message_converter.convert_dictionary_to_ros_message('std_msgs/Bool', dictionary)
        self.assertTrue("Field 'data' has wrong type" in context.exception.args[0])

    def test_dictionary_with_float32(self):
        from std_msgs.msg import Float32

        expected_message = Float32(data=struct.unpack('<f', b'\x7F\x7F\xFF\xFD')[0])
        dictionary = {'data': expected_message.data}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/Float32', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_float64(self):
        from std_msgs.msg import Float64

        expected_message = Float64(data=struct.unpack('<d', b'\x7F\xEF\xFF\xFF\xFF\xFF\xFF\xFD')[0])
        dictionary = {'data': expected_message.data}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/Float64', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_header(self):
        from std_msgs.msg import Header
        from time import time

        now_time = rospy.Time(time())
        expected_message = Header(stamp=now_time, frame_id='my_frame', seq=12)
        dictionary = {
            'stamp': {'secs': now_time.secs, 'nsecs': now_time.nsecs},
            'frame_id': expected_message.frame_id,
            'seq': expected_message.seq,
        }
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/Header', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_header_with_no_prefix(self):
        from std_msgs.msg import Header
        from time import time

        now_time = rospy.Time(time())
        expected_message = Header(stamp=now_time, frame_id='my_frame', seq=12)
        dictionary = {
            'stamp': {'secs': now_time.secs, 'nsecs': now_time.nsecs},
            'frame_id': expected_message.frame_id,
            'seq': expected_message.seq,
        }
        message = message_converter.convert_dictionary_to_ros_message('Header', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_int8(self):
        from std_msgs.msg import Int8

        expected_message = Int8(data=-0x7F)
        dictionary = {'data': expected_message.data}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/Int8', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_uint8(self):
        from std_msgs.msg import UInt8

        expected_message = UInt8(data=0xFF)
        dictionary = {'data': expected_message.data}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/UInt8', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_int16(self):
        from std_msgs.msg import Int16

        expected_message = Int16(data=-0x7FFF)
        dictionary = {'data': expected_message.data}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/Int16', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_uint16(self):
        from std_msgs.msg import UInt16

        expected_message = UInt16(data=0xFFFF)
        dictionary = {'data': expected_message.data}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/UInt16', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_int32(self):
        from std_msgs.msg import Int32

        expected_message = Int32(data=-0x7FFFFFFF)
        dictionary = {'data': expected_message.data}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/Int32', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_uint32(self):
        from std_msgs.msg import UInt32

        expected_message = UInt32(data=0xFFFFFFFF)
        dictionary = {'data': expected_message.data}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/UInt32', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_int64(self):
        from std_msgs.msg import Int64

        expected_message = Int64(data=-0x7FFFFFFFFFFFFFFF)
        dictionary = {'data': expected_message.data}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/Int64', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_uint64(self):
        from std_msgs.msg import UInt64

        expected_message = UInt64(data=0xFFFFFFFFFFFFFFFF)
        dictionary = {'data': expected_message.data}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/UInt64', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_string(self):
        from std_msgs.msg import String

        expected_message = String(data='Hello')
        dictionary = {'data': expected_message.data}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/String', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_unicode(self):
        from std_msgs.msg import String

        expected_message = String(data=u'Hello \u00dcnicode')
        dictionary = {'data': expected_message.data}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/String', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message.data, expected_message.data)
        self.assertEqual(type(message.data), type(expected_message.data))

    def test_dictionary_with_time(self):
        from std_msgs.msg import Time
        from time import time

        now_time = rospy.Time(time())
        expected_message = Time(data=now_time)
        dictionary = {'data': {'secs': now_time.secs, 'nsecs': now_time.nsecs}}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/Time', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_time_now(self):
        dictionary = {'data': 'now'}
        with self.assertRaises(ROSInitException) as context:
            message_converter.convert_dictionary_to_ros_message('std_msgs/Time', dictionary)
        self.assertEqual('time is not initialized. Have you called init_node()?', context.exception.args[0])

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
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/Float64MultiArray', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_invalid_message_fields(self):
        self.assertRaises(
            ValueError, message_converter.convert_dictionary_to_ros_message, 'std_msgs/Empty', {'invalid_field': 1}
        )

    def test_dictionary_with_empty_service(self):
        from std_srvs.srv import EmptyRequest, EmptyResponse

        expected_req = EmptyRequest()
        expected_res = EmptyResponse()
        dictionary_req = {}
        dictionary_res = {}
        message = message_converter.convert_dictionary_to_ros_message('std_srvs/Empty', dictionary_req, 'request')
        expected_req = serialize_deserialize(expected_req)
        self.assertEqual(message, expected_req)
        message = message_converter.convert_dictionary_to_ros_message('std_srvs/Empty', dictionary_res, 'response')
        expected_res = serialize_deserialize(expected_res)
        self.assertEqual(message, expected_res)

    def test_dictionary_with_nested_service(self):
        from rospy_message_converter.srv import NestedUint8ArrayTestServiceRequest, NestedUint8ArrayTestServiceResponse
        from rospy_message_converter.msg import NestedUint8ArrayTestMessage, Uint8ArrayTestMessage
        from base64 import b64encode

        expected_data = bytes(bytearray([97, 98, 99]))
        expected_req = NestedUint8ArrayTestServiceRequest(
            input=NestedUint8ArrayTestMessage(arrays=[Uint8ArrayTestMessage(data=expected_data)])
        )
        expected_req = serialize_deserialize(expected_req)
        expected_res = NestedUint8ArrayTestServiceResponse(
            output=NestedUint8ArrayTestMessage(arrays=[Uint8ArrayTestMessage(data=expected_data)])
        )
        expected_res = serialize_deserialize(expected_res)

        dictionary_req = {"input": {"arrays": [{"data": b64encode(expected_data)}]}}
        dictionary_res = {"output": {"arrays": [{"data": b64encode(expected_data)}]}}
        message = message_converter.convert_dictionary_to_ros_message(
            'rospy_message_converter/NestedUint8ArrayTestService', dictionary_req, 'request'
        )
        self.assertEqual(message, expected_req)
        message = message_converter.convert_dictionary_to_ros_message(
            'rospy_message_converter/NestedUint8ArrayTestService', dictionary_res, 'response'
        )
        self.assertEqual(message, expected_res)

    def test_dictionary_with_setbool_service(self):
        from std_srvs.srv import SetBoolRequest, SetBoolResponse

        expected_req = SetBoolRequest(data=True)
        expected_res = SetBoolResponse(success=True, message='Success!')
        dictionary_req = {'data': True}
        dictionary_res = {'success': True, 'message': 'Success!'}
        message = message_converter.convert_dictionary_to_ros_message('std_srvs/SetBool', dictionary_req, 'request')
        expected_req = serialize_deserialize(expected_req)
        self.assertEqual(message, expected_req)
        message = message_converter.convert_dictionary_to_ros_message('std_srvs/SetBool', dictionary_res, 'response')
        expected_res = serialize_deserialize(expected_res)
        self.assertEqual(message, expected_res)

    def test_dictionary_with_trigger_service(self):
        from std_srvs.srv import TriggerRequest, TriggerResponse

        expected_req = TriggerRequest()
        expected_res = TriggerResponse(success=True, message='Success!')
        dictionary_req = {}
        dictionary_res = {'success': True, 'message': 'Success!'}
        message = message_converter.convert_dictionary_to_ros_message('std_srvs/Trigger', dictionary_req, 'request')
        expected_req = serialize_deserialize(expected_req)
        self.assertEqual(message, expected_req)
        message = message_converter.convert_dictionary_to_ros_message('std_srvs/Trigger', dictionary_res, 'response')
        expected_res = serialize_deserialize(expected_res)
        self.assertEqual(message, expected_res)

    def test_dictionary_with_invalid_kind(self):
        with self.assertRaises(ValueError) as context:
            message_converter.convert_dictionary_to_ros_message('std_msgs/Empty', {}, kind='invalid')
        self.assertEqual('Unknown kind "invalid".', context.exception.args[0])

    def test_dictionary_with_numpy_conversions(self):
        from std_msgs.msg import Byte, Char, Float32, Float64, Int8, Int16, Int32, Int64, UInt8, UInt16, UInt32, UInt64

        numpy_numeric_types = [
            np.int8,
            np.int16,
            np.int32,
            np.int64,
            np.uint8,
            np.uint16,
            np.uint32,
            np.uint64,
            np.float32,
            np.float64,
        ]
        min_values = [
            np.iinfo(t).min for t in [np.int8, np.int16, np.int32, np.int64, np.uint8, np.uint16, np.uint32, np.uint64]
        ] + [np.finfo(t).min for t in [np.float32, np.float64]]
        max_values = [
            np.iinfo(t).max for t in [np.int8, np.int16, np.int32, np.int64, np.uint8, np.uint16, np.uint32, np.uint64]
        ] + [np.finfo(t).max for t in [np.float32, np.float64]]
        numeric_limits = {
            num_type: (min_val, max_val)
            for (num_type, min_val, max_val) in zip(numpy_numeric_types, min_values, max_values)
        }
        ros_to_numpy_type_map = {
            Float32: [np.float32, np.int8, np.int16, np.uint8, np.uint16],
            Float64: [np.float32, np.float64, np.int8, np.int16, np.int32, np.uint8, np.uint16, np.uint32],
            Int8: [np.int8],
            Int16: [np.int8, np.int16, np.uint8],
            Int32: [np.int8, np.int16, np.int32, np.uint8, np.uint16],
            Int64: [np.int8, np.int16, np.int32, np.int64, np.uint8, np.uint16, np.uint32],
            UInt8: [np.uint8],
            UInt16: [np.uint8, np.uint16],
            UInt32: [np.uint8, np.uint16, np.uint32],
            UInt64: [np.uint8, np.uint16, np.uint32, np.uint64],
            Byte: [np.int8],
            Char: [np.uint8],
        }
        for ros_type, valid_numpy_types in ros_to_numpy_type_map.items():
            for numpy_type in valid_numpy_types:
                for value in numeric_limits[numpy_type]:
                    expected_message = ros_type(data=numpy_type(value))
                    dictionary = {'data': numpy_type(value)}
                    message = message_converter.convert_dictionary_to_ros_message(expected_message._type, dictionary)
                    expected_message = serialize_deserialize(expected_message)
                    self.assertEqual(message, expected_message)

            for wrong_numpy_type in [t for t in numpy_numeric_types if t not in valid_numpy_types]:
                for value in numeric_limits[wrong_numpy_type]:
                    with self.assertRaises(TypeError) as context:
                        expected_message = ros_type(data=wrong_numpy_type(value))
                        dictionary = {'data': wrong_numpy_type(value)}
                        message_converter.convert_dictionary_to_ros_message(expected_message._type, dictionary)
                    self.assertTrue("Field 'data' has wrong type" in context.exception.args[0])

    # -------- Tests for None: --------
    def test_dictionary_with_array_none(self):
        from rospy_message_converter.msg import TestArray

        expected_message = TestArray(data=[])
        dictionary = {'data': None}
        message = message_converter.convert_dictionary_to_ros_message('rospy_message_converter/TestArray', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_uint8_array_none(self):
        from rospy_message_converter.msg import Uint8ArrayTestMessage

        expected_message = Uint8ArrayTestMessage(data=bytes())
        dictionary = {'data': None}
        message = message_converter.convert_dictionary_to_ros_message(
            'rospy_message_converter/Uint8ArrayTestMessage', dictionary
        )
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_3uint8_array_none(self):
        from rospy_message_converter.msg import Uint8Array3TestMessage

        expected_message = Uint8Array3TestMessage(data=[0, 0, 0])
        dictionary = {'data': None}
        message = message_converter.convert_dictionary_to_ros_message(
            'rospy_message_converter/Uint8Array3TestMessage', dictionary
        )
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_bool_none(self):
        from std_msgs.msg import Bool

        expected_message = Bool(data=False)
        dictionary = {'data': None}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/Bool', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_byte_none(self):
        from std_msgs.msg import Byte

        expected_message = Byte(data=0)
        dictionary = {'data': None}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/Byte', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_char_none(self):
        from std_msgs.msg import Char

        expected_message = Char(data=0)
        dictionary = {'data': None}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/Char', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_duration_none(self):
        from std_msgs.msg import Duration

        expected_message = Duration()
        dictionary = {'data': None}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/Duration', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_duration_nested_none(self):
        from std_msgs.msg import Duration

        expected_message = Duration(data=rospy.rostime.Duration())
        dictionary = {'data': {'secs': None, 'nsecs': None}}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/Duration', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_float32_none(self):
        from std_msgs.msg import Float32

        expected_message = Float32(data=0.0)
        dictionary = {'data': None}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/Float32', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_float64_none(self):
        from std_msgs.msg import Float64

        expected_message = Float64(data=0.0)
        dictionary = {'data': None}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/Float64', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_header_none(self):
        from std_msgs.msg import Header

        expected_message = Header(stamp=rospy.Time(), frame_id='', seq=12)
        dictionary = {'stamp': {'secs': None, 'nsecs': 0.0}, 'frame_id': None, 'seq': expected_message.seq}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/Header', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_int8_none(self):
        from std_msgs.msg import Int8

        expected_message = Int8(data=0)
        dictionary = {'data': None}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/Int8', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_uint8_none(self):
        from std_msgs.msg import UInt8

        expected_message = UInt8(data=0)
        dictionary = {'data': None}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/UInt8', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_int16_none(self):
        from std_msgs.msg import Int16

        expected_message = Int16(data=0)
        dictionary = {'data': None}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/Int16', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_uint16_none(self):
        from std_msgs.msg import UInt16

        expected_message = UInt16(data=0)
        dictionary = {'data': None}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/UInt16', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_int32_none(self):
        from std_msgs.msg import Int32

        expected_message = Int32(data=0)
        dictionary = {'data': None}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/Int32', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_uint32_none(self):
        from std_msgs.msg import UInt32

        expected_message = UInt32(data=0)
        dictionary = {'data': None}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/UInt32', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_int64_none(self):
        from std_msgs.msg import Int64

        expected_message = Int64(data=0)
        dictionary = {'data': None}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/Int64', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_uint64_none(self):
        from std_msgs.msg import UInt64

        expected_message = UInt64(data=0)
        dictionary = {'data': None}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/UInt64', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_string_none(self):
        from std_msgs.msg import String

        expected_message = String(data='')
        dictionary = {'data': None}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/String', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_time_none(self):
        from std_msgs.msg import Time

        expected_message = Time()
        dictionary = {'data': None}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/Time', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_time_nested_none(self):
        from std_msgs.msg import Time

        test_time = rospy.Time.from_sec(0.123456)
        expected_message = Time(data=test_time)
        dictionary = {'data': {'secs': None, 'nsecs': test_time.nsecs}}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/Time', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

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
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/Float64MultiArray', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)


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
NAME = 'test_message_converter'
if __name__ == '__main__':
    import rosunit

    rosunit.unitrun(PKG, NAME, TestMessageConverter)
