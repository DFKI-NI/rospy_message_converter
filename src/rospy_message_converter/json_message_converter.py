import json
from rospy_message_converter import message_converter

def convert_json_to_ros_message(message_type, json_message):
    dictionary = json.loads(json_message)
    return message_converter.convert_dictionary_to_ros_message(message_type, dictionary)

def convert_ros_message_to_json(message):
    dictionary = message_converter.convert_ros_message_to_dictionary(message)
    json_message = json.dumps(dictionary)
    return json_message
