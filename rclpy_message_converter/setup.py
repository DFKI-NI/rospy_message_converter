from setuptools import setup

package_name = 'rclpy_message_converter'

setup(
    name=package_name,
    version='2.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Martin Günther',
    maintainer_email='martin.guenther@dfki.de',
    description='Converts between Python dictionaries and JSON to rclpy messages',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
