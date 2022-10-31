from launch import LaunchDescription
from launch import LaunchService
from launch_ros.actions import Node


def spawn_car(path, x=0, y=0, z=0, yaw=0):
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=[
                            '-entity', 'car',
                            '-file', path,
                            '-x', str(x),
                            '-y', str(y),
                            '-z', str(z),
                            '-R', "0",
                            '-P', "0",
                            '-Y', str(yaw),
                        ],
                        output='screen')
    ld = LaunchDescription([spawn_entity])
    ls = LaunchService()
    ls.include_launch_description(ld)
    return ls.run()
