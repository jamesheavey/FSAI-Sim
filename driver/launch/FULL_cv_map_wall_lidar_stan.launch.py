from launch import LaunchDescription
from launch_ros.actions import Node
from fs_utils.launch import with_config


def generate_launch_description():
    return LaunchDescription(with_config(lambda config: [
        Node(
            package="driver",
            executable="camera1_cv",
            parameters=[config]
        ),
        Node(
            package="driver",
            executable="map_build",
            parameters=[config]
        ),
        Node(
            package="driver",
            executable="path_plan_wcl",
            parameters=[config]
        ),
        Node(
            package="driver",
            executable="path_follow_stanley",
            parameters=[config]
        ),
        Node(
            package="driver",
            executable="lidar",
            parameters=[config]
        )
    ]))
