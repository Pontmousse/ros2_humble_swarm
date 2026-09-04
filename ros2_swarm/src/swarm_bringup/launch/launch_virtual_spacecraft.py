"""Launch virtual spacecraft motion rendering for selected RoboMasters."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def robot_names_from_environment():
    """Resolve comma-separated robot namespaces, with ROBOT_IDX compatibility."""
    names = os.getenv("ROBOT_NAME")
    if names:
        return [name.strip() for name in names.split(",") if name.strip()]
    indices = os.getenv("ROBOT_IDX")
    if indices:
        return [f"RM{index.strip()}" for index in indices.split(",") if index.strip()]
    raise RuntimeError("Set ROBOT_NAME or ROBOT_IDX before launching")


def generate_launch_description():
    """Create one namespaced virtual-spacecraft node per selected robot."""
    default_parameters = os.path.join(
        get_package_share_directory("swarm_bringup"),
        "config",
        "virtual_spacecraft.yaml",
    )
    parameters_argument = DeclareLaunchArgument(
        "spacecraft_parameters", default_value=default_parameters
    )
    actions = [parameters_argument]
    for robot_name in robot_names_from_environment():
        actions.append(
            Node(
                package="swarm_spacecraft",
                executable="virtual_spacecraft",
                name="virtual_spacecraft",
                namespace=robot_name,
                output="screen",
                parameters=[LaunchConfiguration("spacecraft_parameters")],
            )
        )
    return LaunchDescription(actions)
