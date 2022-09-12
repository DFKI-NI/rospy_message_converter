rclpy_message_converter
=======================

rclpy_message_converter is a lightweight ROS2 package and Python library to
convert from Python dictionaries and JSON messages to rclpy messages, and vice
versa.

ROS 1 and ROS 2 branches
------------------------

ROS 1 users should use the `master` branch. ROS 2 users should use the appropriate
branch for their distro (`foxy`/`galactic`/`humble`/`rolling`/...).

Note to ROS 2 users
-------------------

Most ROS 2 users probably *do not need* this package. This package was originally
developed for ROS 1. In ROS 2, most of the functionality is already built in.
Specifically, check out

* [rosidl_runtime_py.set_message.set_message_fields()](https://github.com/ros2/rosidl_runtime_py/blob/1979f566c3b446ddbc5c3fb6896e1f03ccbc6a27/rosidl_runtime_py/set_message.py#L28-L37)
  for converting a Python dictionary to a ROS message, and
* [rosidl_runtime_py.convert.message_to_orderreddict](https://github.com/ros2/rosidl_runtime_py/blob/1979f566c3b446ddbc5c3fb6896e1f03ccbc6a27/rosidl_runtime_py/convert.py#L159-L176)
  for converting a ROS message to a Python dictionary.

The `rclpy_message_converter` mainly adds the functionality to encode all
variable-size `uint8[]` or fixed-size `uint8[n]` fields using Base64 encoding
when the parameter `base64_encoding` is set.
This saves a lot of space when converting large messages
(such as `sensor_msgs/Image`) to JSON format. Also, encoding/decoding of Base64
strings is required by some packages (such as [mir_robot](https://github.com/dfki-ric/mir_robot))
that were originally developed in ROS1.

Usage
-----

Convert a dictionary to a ROS message

```python
from rclpy_message_converter import message_converter
from std_msgs.msg import String
dictionary = { 'data': 'Howdy' }
message = message_converter.convert_dictionary_to_ros_message('std_msgs/msg/String', dictionary)
```

Convert a ROS message to a dictionary

```python
from rclpy_message_converter import message_converter
from std_msgs.msg import String
message = String(data = 'Howdy')
dictionary = message_converter.convert_ros_message_to_dictionary(message)
```

Convert JSON to a ROS message

```python
from rclpy_message_converter import json_message_converter
from std_msgs.msg import String
json_str = '{"data": "Hello"}'
message = json_message_converter.convert_json_to_ros_message('std_msgs/msg/String', json_str)
```

Convert a ROS message to JSON

```python
from rclpy_message_converter import json_message_converter
from std_msgs.msg import String
message = String(data = 'Hello')
json_str = json_message_converter.convert_ros_message_to_json(message)
```

Test
----

To run the tests:

```bash
colcon test
```

pre-commit Formatting Checks
----------------------------

This repo has a [pre-commit](https://pre-commit.com/) check that runs in CI.
You can use this locally and set it up to run automatically before you commit
something. To install, use pip:

```bash
pip3 install --user pre-commit
```

To run over all the files in the repo manually:

```bash
pre-commit run -a
```

To run pre-commit automatically before committing in the local repo, install the git hooks:

```bash
pre-commit install
```


License
-------

This project is released under the Apache 2.0 license.
