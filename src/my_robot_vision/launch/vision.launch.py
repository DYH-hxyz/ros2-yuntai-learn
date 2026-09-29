from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='my_robot_vision', executable='target_detector', output='screen'),
        Node(package='my_robot_vision', executable='target_tracker', output='screen'),
    ])
