#!/usr/bin/env python
import unittest
import rospy
import rostest
from pprint import pprint
from rospy_message_converter import message_converter

class TestMessageConverter(unittest.TestCase):

    def test_ros_message_with_array(self):
        from rospy_message_converter.msg import TestArray
        expected_dictionary = {
            'data': [1.1, 2.2, 3.3]
        }
        message = TestArray(data = expected_dictionary['data'])
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(dictionary, expected_dictionary)

    def test_ros_message_with_bool(self):
        from std_msgs.msg import Bool
        expected_dictionary = { 'data': True }
        message = Bool(data=expected_dictionary['data'])
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(dictionary, expected_dictionary)

    def test_ros_message_with_byte(self):
        from std_msgs.msg import Byte
        expected_dictionary = { 'data': 5 }
        message = Byte(data=expected_dictionary['data'])
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(dictionary, expected_dictionary)

    def test_ros_message_with_char(self):
        from std_msgs.msg import Char
        expected_dictionary = { 'data': 'c' }
        message = Char(data=expected_dictionary['data'])
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(dictionary, expected_dictionary)

    def test_ros_message_with_duration(self):
        from std_msgs.msg import Duration
        duration = rospy.rostime.Duration(33, 25)
        expected_dictionary = {
            'data': {
                'secs'  : duration.secs,
                'nsecs' : duration.nsecs
            }
        }
        message = Duration(data=duration)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(dictionary, expected_dictionary)

    def test_ros_message_with_empty(self):
        from std_msgs.msg import Empty
        expected_dictionary = {}
        message = Empty()
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(dictionary, expected_dictionary)

    def test_ros_message_with_float32(self):
        from std_msgs.msg import Float32
        expected_dictionary = { 'data': 0x7F7FFFFD }
        message = Float32(data=expected_dictionary['data'])
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(dictionary, expected_dictionary)

    def test_ros_message_with_float64(self):
        from std_msgs.msg import Float64
        expected_dictionary = { 'data': 0x7FEFFFFFFFFFFFF }
        message = Float64(data=expected_dictionary['data'])
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(dictionary, expected_dictionary)

    def test_ros_message_with_header(self):
        from std_msgs.msg import Header
        rospy.init_node('time_node')
        now_time = rospy.Time.now()
        expected_dictionary = {
            'stamp': { 'secs': now_time.secs, 'nsecs': now_time.nsecs },
            'frame_id' : 'my_frame',
            'seq': 3
        }
        message = Header(
            stamp = now_time,
            frame_id = expected_dictionary['frame_id'],
            seq = expected_dictionary['seq']
        )
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(dictionary, expected_dictionary)

    def test_ros_message_with_int8(self):
        from std_msgs.msg import Int8
        expected_dictionary = { 'data': -0x7F }
        message = Int8(data=expected_dictionary['data'])
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(dictionary, expected_dictionary)

    def test_ros_message_with_uint8(self):
        from std_msgs.msg import UInt8
        expected_dictionary = { 'data': 0xFF }
        message = UInt8(data=expected_dictionary['data'])
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(dictionary, expected_dictionary)

    def test_ros_message_with_uint8_array(self):
        from rospy_message_converter.msg import Uint8ArrayTestMessage
        from base64 import standard_b64encode
        expected_data = "".join([chr(i) for i in [97, 98, 99, 100]])
        message = Uint8ArrayTestMessage(data=expected_data)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(dictionary["data"], standard_b64encode(expected_data))

    def test_ros_message_with_3uint8_array(self):
        from rospy_message_converter.msg import Uint8Array3TestMessage
        from base64 import standard_b64encode
        expected_data = "".join([chr(i) for i in [97, 98, 99, 100]])
        message = Uint8Array3TestMessage(data=expected_data)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(dictionary["data"], standard_b64encode(expected_data))

    def test_ros_message_with_int16(self):
        from std_msgs.msg import Int16
        expected_dictionary = { 'data': -0x7FFF }
        message = Int16(data=expected_dictionary['data'])
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(dictionary, expected_dictionary)

    def test_ros_message_with_uint16(self):
        from std_msgs.msg import UInt16
        expected_dictionary = { 'data': 0xFFFF }
        message = UInt16(data=expected_dictionary['data'])
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(dictionary, expected_dictionary)

    def test_ros_message_with_int32(self):
        from std_msgs.msg import Int32
        expected_dictionary = { 'data': -0x7FFFFFFF }
        message = Int32(data=expected_dictionary['data'])
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(dictionary, expected_dictionary)

    def test_ros_message_with_uint32(self):
        from std_msgs.msg import UInt32
        expected_dictionary = { 'data': 0xFFFFFFFF }
        message = UInt32(data=expected_dictionary['data'])
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(dictionary, expected_dictionary)

    def test_ros_message_with_int64(self):
        from std_msgs.msg import Int64
        expected_dictionary = { 'data': -0x7FFFFFFFFFFFFFFF }
        message = Int64(data=expected_dictionary['data'])
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(dictionary, expected_dictionary)

    def test_ros_message_with_uint64(self):
        from std_msgs.msg import UInt64
        expected_dictionary = { 'data': 0xFFFFFFFFFFFFFFFF }
        message = UInt64(data=expected_dictionary['data'])
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(dictionary, expected_dictionary)

    def test_ros_message_with_string(self):
        from std_msgs.msg import String
        expected_dictionary = { 'data': 'Hello' }
        message = String(data=expected_dictionary['data'])
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(dictionary, expected_dictionary)

    def test_ros_message_with_time(self):
        from std_msgs.msg import Time
        rospy.init_node('time_node')
        now_time = rospy.Time.now()
        expected_dictionary = {
            'data': { 'secs': now_time.secs, 'nsecs': now_time.nsecs }
        }
        message = Time(data=now_time)
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
        multiArray = Float64MultiArray(
            layout = layout,
            data   = expected_dictionary['data']
        )
        dictionary = message_converter.convert_ros_message_to_dictionary(multiArray)
        self.assertEqual(dictionary, expected_dictionary)

    def test_dictionary_with_array(self):
        from rospy_message_converter.msg import TestArray
        expected_message = TestArray(data = [1.1, 2.2, 3.3, 4.4])
        dictionary = { 'data': expected_message.data }
        message = message_converter.convert_dictionary_to_ros_message('rospy_message_converter/TestArray', dictionary)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_bool(self):
        from std_msgs.msg import Bool
        expected_message = Bool(data = True)
        dictionary = { 'data': expected_message.data }
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/Bool', dictionary)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_byte(self):
        from std_msgs.msg import Byte
        expected_message = Byte(data = 3)
        dictionary = { 'data': expected_message.data }
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/Byte', dictionary)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_char(self):
        from std_msgs.msg import Char
        expected_message = Char(data = 'c')
        dictionary = { 'data': expected_message.data }
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/Char', dictionary)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_duration(self):
        from std_msgs.msg import Duration
        duration = rospy.rostime.Duration(33, 25)
        expected_message = Duration(data = duration)
        dictionary = {
            'data': {
                'secs'  : duration.secs,
                'nsecs' : duration.nsecs
            }
        }
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/Duration', dictionary)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_empty(self):
        from std_msgs.msg import Empty
        expected_message = Empty()
        dictionary = {}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/Empty', dictionary)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_float32(self):
        from std_msgs.msg import Float32
        expected_message = Float32(data = 0x7F7FFFFD)
        dictionary = { 'data': expected_message.data }
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/Float32', dictionary)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_float64(self):
        from std_msgs.msg import Float64
        expected_message = Float64(data = 0x7FEFFFFFFFFFFFF)
        dictionary = { 'data': expected_message.data }
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/Float64', dictionary)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_header(self):
        from std_msgs.msg import Header
        rospy.init_node('time_node')
        now_time = rospy.Time.now()
        expected_message = Header(
            stamp = now_time,
            frame_id = 'my_frame',
            seq = 12
        )
        dictionary = {
            'stamp': { 'secs': now_time.secs, 'nsecs': now_time.nsecs },
            'frame_id' : expected_message.frame_id,
            'seq': expected_message.seq
        }
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/Header', dictionary)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_header_with_no_prefix(self):
        from std_msgs.msg import Header
        rospy.init_node('time_node')
        now_time = rospy.Time.now()
        expected_message = Header(
            stamp = now_time,
            frame_id = 'my_frame',
            seq = 12
        )
        dictionary = {
            'stamp': { 'secs': now_time.secs, 'nsecs': now_time.nsecs },
            'frame_id' : expected_message.frame_id,
            'seq': expected_message.seq
        }
        message = message_converter.convert_dictionary_to_ros_message('Header', dictionary)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_int8(self):
        from std_msgs.msg import Int8
        expected_message = Int8(data = -0x7F)
        dictionary = { 'data': expected_message.data }
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/Int8', dictionary)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_uint8(self):
        from std_msgs.msg import UInt8
        expected_message = UInt8(data = 0xFF)
        dictionary = { 'data': expected_message.data }
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/UInt8', dictionary)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_int16(self):
        from std_msgs.msg import Int16
        expected_message = Int16(data = -0x7FFF)
        dictionary = { 'data': expected_message.data }
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/Int16', dictionary)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_uint16(self):
        from std_msgs.msg import UInt16
        expected_message = UInt16(data = 0xFFFF)
        dictionary = { 'data': expected_message.data }
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/UInt16', dictionary)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_int32(self):
        from std_msgs.msg import Int32
        expected_message = Int32(data = -0x7FFFFFFF)
        dictionary = { 'data': expected_message.data }
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/Int32', dictionary)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_uint32(self):
        from std_msgs.msg import UInt32
        expected_message = UInt32(data = 0xFFFFFFFF)
        dictionary = { 'data': expected_message.data }
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/UInt32', dictionary)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_int64(self):
        from std_msgs.msg import Int64
        expected_message = Int64(data = -0x7FFFFFFFFFFFFFFF)
        dictionary = { 'data': expected_message.data }
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/Int64', dictionary)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_uint64(self):
        from std_msgs.msg import UInt64
        expected_message = UInt64(data = 0xFFFFFFFFFFFFFFFF)
        dictionary = { 'data': expected_message.data }
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/UInt64', dictionary)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_string(self):
        from std_msgs.msg import String
        expected_message = String(data = 'Hello')
        dictionary = { 'data': expected_message.data }
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/String', dictionary)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_time(self):
        from std_msgs.msg import Time
        now_time = rospy.Time.now()
        expected_message = Time(data=now_time)
        dictionary = {
            'data': {
                'secs'  : now_time.secs,
                'nsecs' : now_time.nsecs
            }
        }
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/Time', dictionary)
        self.assertEqual(message, expected_message)

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
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/Float64MultiArray', dictionary)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_invalid_message_fields(self):
        self.assertRaises(ValueError,
                          message_converter.convert_dictionary_to_ros_message,
                          'std_msgs/Empty',
                          {'invalid_field': 1})



PKG = 'rospy_message_converter'
NAME = 'test_message_converter'
if __name__ == '__main__':
    rostest.unitrun(PKG, NAME, TestMessageConverter)
