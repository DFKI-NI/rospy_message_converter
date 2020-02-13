import json
from rospy_message_converter import message_converter

def convert_json_to_ros_message(message_type, json_message, strict_mode=True):
    """
    Takes in the message type and a JSON-formatted string and returns a ROS
    message.

    If strict_mode is set, an exception will be thrown if the json message contains extra fields
    
    Example:
        message_type = "std_msgs/String"
        json_message = '{"data": "Hello, Robot"}'
        ros_message = convert_json_to_ros_message(message_type, json_message)
    """
    dictionary = json.loads(json_message)
    return message_converter.convert_dictionary_to_ros_message(message_type, dictionary, strict_mode=strict_mode)

def convert_ros_message_to_json(message):
    """
    Takes in a ROS message and returns a JSON-formatted string.

    Example:
        ros_message = std_msgs.msg.String(data="Hello, Robot")
        json_message = convert_ros_message_to_json(ros_message)
    """
    dictionary = message_converter.convert_ros_message_to_dictionary(message)
    json_message = json.dumps(dictionary)
    return json_message
