from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="supervisor",
            executable="supervisor",
            name="supervisor"
        ),
        Node(
            package="vehicle_evaluator",
            executable="vehicle_evaluator",
            name="vehicle_evaluator"
        ),
        Node(
            package="fs_utils",
            executable="faux_lidar",
            name="faux_lidar"
        ),
        Node(
            package="fs_utils",
            executable="fuzzing",
            name="fuzzing"
        )
    ])
