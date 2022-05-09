rclpy_message_converter
=======================

rclpy_message_converter is a lightweight ROS2 package and Python library to
convert from Python dictionaries and JSON messages to rclpy messages, and vice
versa.

ROS 1 and ROS 2 branches
------------------------

ROS 1 users should use the `master` branch. ROS 2 users should use the appropriate
branch for their distro (`foxy`/`galactic`/`humble`/`rolling`/...).

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

Project is released under the BSD license.
