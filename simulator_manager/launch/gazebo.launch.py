from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource

from os import environ, pathsep, path

from ament_index_python.packages import get_package_share_directory

# Find the folders in the share directory
GAZEBO_LAUNCH_DIR = path.join(get_package_share_directory("gazebo_ros"), "launch")
MODEL_PATH = path.join(get_package_share_directory("fs_description"), "models")
MESH_PATH = path.join(get_package_share_directory("fs_description"), "meshes")


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("world"),
        DeclareLaunchArgument("gui", default_value="true"),
        # Add paths to models and mashes to gazebo environment variables
        # This ensures they are discovered when loading the world
        SetEnvironmentVariable(
            "GAZEBO_MODEL_PATH",
            f"{environ.get('GAZEBO_MODEL_PATH')}{MODEL_PATH}{pathsep}{MESH_PATH}"
        ),
        SetEnvironmentVariable(
            "GAZEBO_RESOURCE_PATH",
            f"{environ.get('GAZEBO_RESOURCE_PATH')}{MODEL_PATH}{pathsep}{MESH_PATH}"
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [GAZEBO_LAUNCH_DIR, "/gazebo.launch.py"]
            ),
        )
    ])
