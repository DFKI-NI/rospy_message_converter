#!/usr/bin/env python
import struct
import unittest
import rospy
from rospy_message_converter import message_converter

class TestMessageConverter(unittest.TestCase):

    def test_ros_message_with_array(self):
        from rospy_message_converter.msg import TestArray
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
        expected_dictionary = { 'data': 5 }
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
        from std_msgs.msg import Duration
        duration = rospy.rostime.Duration(33, 25)
        expected_dictionary = {
            'data': {
                'secs'  : duration.secs,
                'nsecs' : duration.nsecs
            }
        }
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
        expected_dictionary = { 'data': struct.unpack('<f', '\x7F\x7F\xFF\xFD')[0] }
        message = Float32(data=expected_dictionary['data'])
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(dictionary, expected_dictionary)

    def test_ros_message_with_float64(self):
        from std_msgs.msg import Float64
        expected_dictionary = { 'data': struct.unpack('<d', '\x7F\xEF\xFF\xFF\xFF\xFF\xFF\xFD')[0] }
        message = Float64(data=expected_dictionary['data'])
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(dictionary, expected_dictionary)

    def test_ros_message_with_header(self):
        from std_msgs.msg import Header
        from time import time
        now_time = rospy.Time(time())
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
        from rospy_message_converter.msg import Uint8ArrayTestMessage
        from base64 import standard_b64encode
        expected_data = "".join([chr(i) for i in [97, 98, 99, 100]])
        message = Uint8ArrayTestMessage(data=expected_data)
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(dictionary["data"], standard_b64encode(expected_data))

    def test_ros_message_with_3uint8_array(self):
        from rospy_message_converter.msg import Uint8Array3TestMessage
        from base64 import standard_b64encode
        expected_data = "".join([chr(i) for i in [97, 98, 99]])
        message = Uint8Array3TestMessage(data=expected_data)
        message = serialize_deserialize(message)
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(dictionary["data"], standard_b64encode(expected_data))

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

    def test_ros_message_with_time(self):
        from std_msgs.msg import Time
        from time import time
        now_time = rospy.Time(time())
        expected_dictionary = {
            'data': { 'secs': now_time.secs, 'nsecs': now_time.nsecs }
        }
        message = Time(data=now_time)
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

    def test_dictionary_with_array(self):
        from rospy_message_converter.msg import TestArray
        expected_message = TestArray(data = [1.1, 2.2, 3.3, 4.4])
        dictionary = { 'data': expected_message.data }
        message = message_converter.convert_dictionary_to_ros_message('rospy_message_converter/TestArray', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_bool(self):
        from std_msgs.msg import Bool
        expected_message = Bool(data = True)
        dictionary = { 'data': expected_message.data }
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/Bool', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_byte(self):
        from std_msgs.msg import Byte
        expected_message = Byte(data = 3)
        dictionary = { 'data': expected_message.data }
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/Byte', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_char(self):
        from std_msgs.msg import Char
        expected_message = Char(data = 99)
        dictionary = { 'data': expected_message.data }
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/Char', dictionary)
        expected_message = serialize_deserialize(expected_message)
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
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_empty(self):
        from std_msgs.msg import Empty
        expected_message = Empty()
        dictionary = {}
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/Empty', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_float32(self):
        from std_msgs.msg import Float32
        expected_message = Float32(data = struct.unpack('<f', '\x7F\x7F\xFF\xFD')[0])
        dictionary = { 'data': expected_message.data }
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/Float32', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_float64(self):
        from std_msgs.msg import Float64
        expected_message = Float64(data = struct.unpack('<d', '\x7F\xEF\xFF\xFF\xFF\xFF\xFF\xFD')[0])
        dictionary = { 'data': expected_message.data }
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/Float64', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_header(self):
        from std_msgs.msg import Header
        from time import time
        now_time = rospy.Time(time())
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
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_header_with_no_prefix(self):
        from std_msgs.msg import Header
        from time import time
        now_time = rospy.Time(time())
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
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_int8(self):
        from std_msgs.msg import Int8
        expected_message = Int8(data = -0x7F)
        dictionary = { 'data': expected_message.data }
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/Int8', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_uint8(self):
        from std_msgs.msg import UInt8
        expected_message = UInt8(data = 0xFF)
        dictionary = { 'data': expected_message.data }
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/UInt8', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_int16(self):
        from std_msgs.msg import Int16
        expected_message = Int16(data = -0x7FFF)
        dictionary = { 'data': expected_message.data }
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/Int16', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_uint16(self):
        from std_msgs.msg import UInt16
        expected_message = UInt16(data = 0xFFFF)
        dictionary = { 'data': expected_message.data }
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/UInt16', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_int32(self):
        from std_msgs.msg import Int32
        expected_message = Int32(data = -0x7FFFFFFF)
        dictionary = { 'data': expected_message.data }
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/Int32', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_uint32(self):
        from std_msgs.msg import UInt32
        expected_message = UInt32(data = 0xFFFFFFFF)
        dictionary = { 'data': expected_message.data }
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/UInt32', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_int64(self):
        from std_msgs.msg import Int64
        expected_message = Int64(data = -0x7FFFFFFFFFFFFFFF)
        dictionary = { 'data': expected_message.data }
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/Int64', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_uint64(self):
        from std_msgs.msg import UInt64
        expected_message = UInt64(data = 0xFFFFFFFFFFFFFFFF)
        dictionary = { 'data': expected_message.data }
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/UInt64', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_string(self):
        from std_msgs.msg import String
        expected_message = String(data = 'Hello')
        dictionary = { 'data': expected_message.data }
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/String', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_unicode(self):
        from std_msgs.msg import String
        expected_message = String(data = 'Hello')
        dictionary = { 'data': unicode(expected_message.data) }
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/String', dictionary)
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message.data,expected_message.data)
        self.assertEqual(type(message.data),type(expected_message.data))

    def test_dictionary_with_time(self):
        from std_msgs.msg import Time
        from time import time
        now_time = rospy.Time(time())
        expected_message = Time(data=now_time)
        dictionary = {
            'data': {
                'secs'  : now_time.secs,
                'nsecs' : now_time.nsecs
            }
        }
        message = message_converter.convert_dictionary_to_ros_message('std_msgs/Time', dictionary)
        expected_message = serialize_deserialize(expected_message)
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
        expected_message = serialize_deserialize(expected_message)
        self.assertEqual(message, expected_message)

    def test_dictionary_with_invalid_message_fields(self):
        self.assertRaises(ValueError,
                          message_converter.convert_dictionary_to_ros_message,
                          'std_msgs/Empty',
                          {'invalid_field': 1})

    def test_dictionary_with_empty_service(self):
        from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
        expected_req = EmptyRequest()
        expected_res = EmptyResponse()
        dictionary_req = {}
        dictionary_res = {}
        message = message_converter.convert_dictionary_to_ros_message('std_srvs/Empty', dictionary_req,
                                                                      'request')
        expected_req = serialize_deserialize(expected_req)
        self.assertEqual(message, expected_req)
        message = message_converter.convert_dictionary_to_ros_message('std_srvs/Empty', dictionary_res,
                                                                      'response')
        expected_res = serialize_deserialize(expected_res)
        self.assertEqual(message, expected_res)

    def test_dictionary_with_setbool_service(self):
        from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
        expected_req = SetBoolRequest(data=True)
        expected_res = SetBoolResponse(success=True, message='Success!')
        dictionary_req = { 'data': True }
        dictionary_res = { 'success': True, 'message': 'Success!' }
        message = message_converter.convert_dictionary_to_ros_message('std_srvs/SetBool', dictionary_req,
                                                                      'request')
        expected_req = serialize_deserialize(expected_req)
        self.assertEqual(message, expected_req)
        message = message_converter.convert_dictionary_to_ros_message('std_srvs/SetBool', dictionary_res,
                                                                      'response')
        expected_res = serialize_deserialize(expected_res)
        self.assertEqual(message, expected_res)

    def test_dictionary_with_trigger_service(self):
        from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
        expected_req = TriggerRequest()
        expected_res = TriggerResponse(success=True, message='Success!')
        dictionary_req = {}
        dictionary_res = { 'success': True, 'message': 'Success!' }
        message = message_converter.convert_dictionary_to_ros_message('std_srvs/Trigger', dictionary_req,
                                                                      'request')
        expected_req = serialize_deserialize(expected_req)
        self.assertEqual(message, expected_req)
        message = message_converter.convert_dictionary_to_ros_message('std_srvs/Trigger', dictionary_res,
                                                                      'response')
        expected_res = serialize_deserialize(expected_res)
        self.assertEqual(message, expected_res)


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
NAME = 'test_message_converter'
if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, NAME, TestMessageConverter)
