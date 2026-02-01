# setup.py
from setuptools import setup
import os
from glob import glob

package_name = 'uart_ros_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'docs'), glob('docs/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cristian',
    maintainer_email='cristian@todo.com',
    description='UART <-> ROS2 bridge + online control metrics for Temp_ControlMod',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'uart_temp_to_ros = uart_ros_bridge.uart_temp_to_ros:main',
            'control_metrics = uart_ros_bridge.control_metrics:main',
            'talker_min = uart_ros_bridge.talker_min:main',
        ],
    },
)
