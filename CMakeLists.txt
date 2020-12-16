cmake_minimum_required(VERSION 3.5.1)
cmake_policy(SET CMP0048 NEW)
project(rospy_message_converter)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

# Generate messages for testing (NOINSTALL)
add_message_files(
  FILES
  NestedUint8ArrayTestMessage.msg
  TestArray.msg
  Uint8Array3TestMessage.msg
  Uint8ArrayTestMessage.msg
  NOINSTALL
)

add_service_files(
  FILES
  NestedUint8ArrayTestService.srv
  NOINSTALL
)

catkin_python_setup()

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

# Testing
if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_json_message_converter.py)
  catkin_add_nosetests(test/test_message_converter.py)
endif()
