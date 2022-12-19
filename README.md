rospy_message_converter
=======================

Rospy_message_converter is a lightweight ROS package and Python library to
convert from Python dictionaries and JSON messages to rospy messages, and vice
versa.

ROS 1 and ROS 2 branches
------------------------

ROS 1 users should use the `master` branch. ROS 2 users should use the appropriate
branch for their distro (`foxy`/`galactic`/`humble`/`rolling`/...).

Usage
-----

Convert a dictionary to a ROS message

```python
from rospy_message_converter import message_converter
from std_msgs.msg import String
dictionary = { 'data': 'Howdy' }
message = message_converter.convert_dictionary_to_ros_message('std_msgs/String', dictionary)
```

Convert a ROS message to a dictionary

```python
from rospy_message_converter import message_converter
from std_msgs.msg import String
message = String(data = 'Howdy')
dictionary = message_converter.convert_ros_message_to_dictionary(message)
```

Convert JSON to a ROS message

```python
from rospy_message_converter import json_message_converter
from std_msgs.msg import String
json_str = '{"data": "Hello"}'
message = json_message_converter.convert_json_to_ros_message('std_msgs/String', json_str)
```

Convert a ROS message to JSON

```python
from rospy_message_converter import json_message_converter
from std_msgs.msg import String
message = String(data = 'Hello')
json_str = json_message_converter.convert_ros_message_to_json(message)
```

Test
----

To run the tests:

```bash
catkin_make test
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

GitHub actions - Continuous Integration
---------------------------------------

[![Build Status](https://github.com/DFKI-NI/rospy_message_converter/actions/workflows/github-actions.yml/badge.svg)](https://github.com/DFKI-NI/rospy_message_converter/actions/workflows/github-actions.yml/)


ROS Buildfarm
-------------

|           | binary deb | source deb | devel | doc |
|-----------|------------|------------|-------|-----|
| kinetic | [![Build Status](http://build.ros.org/buildStatus/icon?job=Kbin_uX64__rospy_message_converter__ubuntu_xenial_amd64__binary)](http://build.ros.org/job/Kbin_uX64__rospy_message_converter__ubuntu_xenial_amd64__binary/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Ksrc_uX__rospy_message_converter__ubuntu_xenial__source)](http://build.ros.org/job/Ksrc_uX__rospy_message_converter__ubuntu_xenial__source/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Kdev__rospy_message_converter__ubuntu_xenial_amd64)](http://build.ros.org/job/Kdev__rospy_message_converter__ubuntu_xenial_amd64) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Kdoc__rospy_message_converter__ubuntu_xenial_amd64)](http://build.ros.org/job/Kdoc__rospy_message_converter__ubuntu_xenial_amd64) |
| melodic | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_uB64__rospy_message_converter__ubuntu_bionic_amd64__binary)](http://build.ros.org/job/Mbin_uB64__rospy_message_converter__ubuntu_bionic_amd64__binary) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Msrc_uB__rospy_message_converter__ubuntu_bionic__source)](http://build.ros.org/job/Msrc_uB__rospy_message_converter__ubuntu_bionic__source/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mdev__rospy_message_converter__ubuntu_bionic_amd64)](http://build.ros.org/job/Mdev__rospy_message_converter__ubuntu_bionic_amd64) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mdoc__rospy_message_converter__ubuntu_bionic_amd64)](http://build.ros.org/job/Mdoc__rospy_message_converter__ubuntu_bionic_amd64) |
| noetic  | [![Build Status](http://build.ros.org/buildStatus/icon?job=Nbin_uF64__rospy_message_converter__ubuntu_focal_amd64__binary)](http://build.ros.org/job/Nbin_uF64__rospy_message_converter__ubuntu_focal_amd64__binary/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Nsrc_uF__rospy_message_converter__ubuntu_focal__source)](http://build.ros.org/job/Nsrc_uF__rospy_message_converter__ubuntu_focal__source/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Ndev__rospy_message_converter__ubuntu_focal_amd64)](http://build.ros.org/job/Ndev__rospy_message_converter__ubuntu_focal_amd64/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Ndoc__rospy_message_converter__ubuntu_focal_amd64)](http://build.ros.org/job/Ndoc__rospy_message_converter__ubuntu_focal_amd64/) |
