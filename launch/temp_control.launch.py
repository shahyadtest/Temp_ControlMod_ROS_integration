from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uart_ros_bridge',
            executable='uart_temp_to_ros',
            name='temp_u_ref_node',
            output='screen'
        ),
        Node(
            package='uart_ros_bridge',
            executable='control_metrics',
            name='control_metrics',
            output='screen'
        ),
    ])
