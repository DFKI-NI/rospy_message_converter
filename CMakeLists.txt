cmake_minimum_required(VERSION 2.8.3)
project(rospy_message_converter)

find_package(catkin REQUIRED COMPONENTS message_generation roslib rospy rostest std_msgs)

# Generate messages for testing (NOINSTALL)
add_message_files(
  FILES
  TestArray.msg
  Uint8Array3TestMessage.msg
  Uint8ArrayTestMessage.msg
  NOINSTALL
)

catkin_python_setup()

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

# Test launch files
add_rostest(test/test_all.test)
