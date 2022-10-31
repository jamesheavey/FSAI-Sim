from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="driver",
            executable="path_follow_pursuit"
        )
    ])
