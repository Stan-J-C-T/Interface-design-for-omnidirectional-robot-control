from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rosbot_xl_pkg',
            executable='interface',
            output='screen'),
    ])