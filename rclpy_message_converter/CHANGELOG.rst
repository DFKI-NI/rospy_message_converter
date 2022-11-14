Change Log
==========

2.0.1 (2022-11-14)
------------------
* Allow message_type as class in convert_dictionary_to_ros_message
* Tests: Switch assertEqual order of arguments in test_json
* Tests: Add tests for tf2_msgs.msg.TFMessage
* Contributors: Martin Günther

2.0.0 (2022-09-12)
------------------
* Initial release into ROS2
* Convert repo to ROS2
* Rename binary_array_as_bytes to base64_encoding
* Switch to Apache 2.0 license
  This is necessary because the new ROS2 code contains code that was
  copied and modified from rosidl_runtime_py, which is licensed under
  Apache 2.0. Switching from BSD to Apache 2.0 should be okay, because
  Apache 2.0 is the more restrictive license.
* Remove support for checking types
  In ROS2, it is no longer possible to set the data field of ROS numeric
  types (Float32, Float64, int, ...) to numpy types, because the setter
  checks that it's a python primitive type.
* Remove support for time 'now'
  Implementing this properly requires a node handle and is out of scope
  for this library.
* Remove python2 support
* Tests: Change Uint8Array3TestMessage from bytes to list
  Creating from a bytes type is not supported in ROS2.
* Tests: Switch assertEqual order of arguments
  Expected is first, actual second (this way, the log output on failing
  tests is correct).
* Tests: Remove test_dictionary_with_invalid_message_fields
  This is the same as test_dictionary_with_empty_additional_args_strict_mode, so it can be safely removed.
* Tests: Add workaround for typename
  The field _type does not exist in ROS2 any more.
* Tests: Add test_dictionary_with_implicit_conversion
  The new version (based on set_message_fields) implicitly converts
  strings to bool.
* Tests: Change expected exceptions
  This makes the exceptions thrown by rclpy_message_converter the same as
  rosidl_runtime_py.set_message.set_message_fields().
  Also change test_dictionary_with_wrong_type to use floats, because
  bool('should_be_a_bool') == True (doesn't throw an error).
* Contributors: Martin Günther, relffok

0.5.6 (2021-03-01)
------------------
* Propagate strict_mode, check_missing_fields in _convert_to_ros_type
  Previously, _convert_to_ros_type dropped strict_mode and
  check_missing_fields in nested messages.
* Add NestedUint8ArrayTestService tests
* propagate check_types in _convert_to_ros_type (`#51 <https://github.com/uos/rospy_message_converter/issues/51>`_)
  Co-authored-by: Martin Günther <martin.guenther@dfki.de>
* Fix base64_encoding=False with nested msgs
* Add param base64_encoding
  Closes `#45 <https://github.com/uos/rospy_message_converter/issues/45>`_.
* Contributors: Marc Bosch-Jorge, Martin Günther, Otacon5555

0.5.5 (2020-11-09)
------------------
* Decode strings from ROS messages as UTF8
  This makes the python2 behavior equal to python3.
* python3 only: Validate base64 strings
* Add bytes to python3 string types
  This means that `bytes` will now also be base64-decoded, which fixes the following tests on python3:
  * test_dictionary_with_uint8_array_bytes
  * test_dictionary_with_uint8_array_bytes_unencoded
  * test_dictionary_with_3uint8_array_bytes
  On python2, `bytes` is just an alias for `str`, which is why it worked
  without this.
* Fix and add tests
* Contributors: Martin Günther

0.5.4 (2020-10-13)
------------------
* Avoid numpy dependency
* Contributors: Martin Günther, betaboon

0.5.3 (2020-08-20)
------------------
* Add check_types parameter to convert_dictionary_to_ros_message (`#42 <https://github.com/uos/rospy_message_converter/issues/42>`_)
* Allow numpy numeric types in numeric fields  (`#41 <https://github.com/uos/rospy_message_converter/issues/41>`_)
  Fixes `#39 <https://github.com/uos/rospy_message_converter/issues/39>`_.
* perf: Remove remaining regexes
  This is only a small speedup of about 1.03x.
* perf: Avoid regex in _is_field_type_a_primitive_array
  This makes the function almost 3x faster.
* perf: Reorder type checks
  Perform the cheaper checks first. This results in a speedup of about
  1.2x.
* perf: Avoid regex in is_ros_binary_type
  This makes is_ros_binary_type almost 5x faster and as a result the whole
  convert_ros_message_to_dictionary function almost 2x faster.
* Compare types, not type names; improve error message
  Old error message:
  TypeError: Wrong type: '1.0' must be float64
  New error message:
  TypeError: Field 'x' has wrong type <type 'numpy.float64'> (valid types: [<type 'int'>, <type 'float'>])
* Remove unused python_to_ros_type_map
* added test for convert_dictionary_to_ros_message with int8 array
* python 3 fix for _convert_to_ros_binary
* Contributors: Martin Günther, Steffen Rühl

0.5.2 (2020-07-09)
------------------
* Check for wrong field types when converting from dict to ros msg
* Check for missing fields when converting from dict to ros msg
* Contributors: Martin Günther, alecarnevale

0.5.1 (2020-05-25)
------------------
* Initial release into Noetic
* Decode base64-encoded byte arrays as unicode
* Make tests compatible with python3
* add check for python3 str serializing `#33 <https://github.com/uos/rospy_message_converter/issues/33>`_ (`#34 <https://github.com/uos/rospy_message_converter/issues/34>`_)
* efficient conversion of primitive array to ros type (`#31 <https://github.com/uos/rospy_message_converter/issues/31>`_)
* efficient conversion of primitive array
* removed unused _convert_from_ros_primitive
* optionally ignore extra fields when deserializing (`#29 <https://github.com/uos/rospy_message_converter/issues/29>`_)
* Remove EOL distros indigo + lunar from CI
* travis CI: Use matrix to split ROS distros
* Update README (convert to md, add build status)
* Contributors: Martin Günther, George Hartt, Jannik Abbenseth, Omri Rozenzaft

0.5.0 (2019-01-17)
------------------
* Initial release into Lunar and Melodic
* Remove support for Jade (EOL)
* Change maintainer from Brandon Alexander to Martin Günther
* Move repo from baalexander to uos
* Add serialize_deserialize to unit tests, fix incorrect tests caught by this
* Remove dependency on ROS master in tests; all tests are now unit
  tests  (`#18 <https://github.com/uos/rospy_message_converter/issues/18>`_)
* Add service request/response support (`#17 <https://github.com/uos/rospy_message_converter/issues/17>`_)
* Fix fixed-size uint8 array conversion failure (`#15 <https://github.com/uos/rospy_message_converter/issues/15>`_)
* Fix unicode handling in string fields (`#13 <https://github.com/uos/rospy_message_converter/issues/13>`_)
* Enable testing only if CATKIN_ENABLE_TESTING is set (`#9 <https://github.com/uos/rospy_message_converter/issues/9>`_)
* Contributors: Martin Günther, Brandon Alexander, George Laurent, Jean-Baptiste Doyon, Viktor Schlegel, Rein Appeldoorn, Will Baker, neka-nat

0.4.0 (2015-12-13)
------------------
* Adds support for ROS Jade
* Removes support for ROS Groovy and Hydro (EOL)
* Uses single branch for all ROS versions
* Docker support for local development and Travis CI

0.3.0 (2014-06-03)
------------------
* Adds support for ROS Indigo

0.2.0 (2013-07-15)
------------------
* Updates to ROS Hydro
* Builds and runs tests with Travis CI
* Adds CHANGELOG

0.1.4 (2013-04-16)
------------------
* Documents Python functions
* Throws error if invalid JSON or dictionary

0.1.3 (2013-03-04)
------------------
* Adds rostest dependency

0.1.2 (2013-03-04)
------------------
* Adds missing build_depends and run_depends

0.1.1 (2013-03-01)
------------------
* Adds message_generation dependency to fix build

0.1.0 (2013-02-27)
------------------
* Initial release of rospy_message_converter
