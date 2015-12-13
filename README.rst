Rospy_message_converter is a lightweight ROS package and Python library to
convert from Python dictionaries and JSON messages to rospy messages, and vice
versa.

Build status: |Build Status|

Usage
-----

Convert a dictionary to a ROS message

.. code-block:: python

    from rospy_message_converter import message_converter
    from std_msgs.msg import String
    dictionary = { 'data': 'Howdy' }
    message = message_converter.convert_dictionary_to_ros_message('std_msgs/String', dictionary)

Convert a ROS message to a dictionary

.. code-block:: python

    from rospy_message_converter import message_converter
    from std_msgs.msg import String
    message = String(data = 'Howdy')
    dictionary = message_converter.convert_ros_message_to_dictionary(message)

Convert JSON to a ROS message

.. code-block:: python

    from rospy_message_converter import json_message_converter
    from std_msgs.msg import String
    json_str = '{"data": "Hello"}'
    message = json_message_converter.convert_json_to_ros_message('std_msgs/String', json_str)

Convert a ROS message to JSON

.. code-block:: python

    from rospy_message_converter import json_message_converter
    from std_msgs.msg import String
    message = String(data = 'Hello')
    json_str = json_message_converter.convert_ros_message_to_json(message)

Test
----

To run the tests:

.. code-block:: bash

    rostest rospy_message_converter test_all.test


License
-------

Project is released under the BSD license.

.. _rospy: http://www.ros.org/wiki/rospy
.. |Build Status| image:: https://secure.travis-ci.org/baalexander/rospy_message_converter.png
   :target: http://travis-ci.org/baalexander/rospy_message_converter
