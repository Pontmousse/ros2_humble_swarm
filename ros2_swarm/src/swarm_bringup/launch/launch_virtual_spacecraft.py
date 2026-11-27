"""Launch the virtual spacecraft control loop (and optionally its RViz markers) for selected RoboMasters.

One `control` process per robot (nodes-to-functions Phase 4): virtual spacecraft, bounding-box search,
PID+FF tracker and velocity smoother on one 20 Hz tick, publishing cmd_vel.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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
    """Create the namespaced control nodes and, optionally, their visualizers."""
    # In direct_cmd_vel capture, guidance (capture) is the sole writer of cmd_vel and muxes this loop's output,
    # which then goes to cmd_vel_nav instead. Must match the same env read in launch_centralized_swarm.py.
    direct_cmd_vel = os.environ.get("DIRECT_CMD_VEL", "").lower() in ("1", "true", "yes", "on")
    default_parameters = os.path.join(get_package_share_directory("swarm_bringup"), "config", "virtual_spacecraft.yaml")
    actions = [
        DeclareLaunchArgument("spacecraft_parameters", default_value=default_parameters),
        DeclareLaunchArgument("enable_guidance_visualization", default_value="false", choices=["true", "false"],
                              description="Enable guidance and motion-rendering RViz markers"),
    ]
    robot_names = robot_names_from_environment()
    for robot_name in robot_names:
        configured_parameters = LaunchConfiguration("spacecraft_parameters")
        actions.append(Node(
            package="swarm_spacecraft", executable="control", name="control", namespace=robot_name,
            output="screen", parameters=[configured_parameters, {"direct_cmd_vel": direct_cmd_vel}],
        ))
        actions.append(Node(
            package="swarm_spacecraft", executable="spacecraft_visualizer", name="spacecraft_visualizer",
            namespace=robot_name, output="screen",
            parameters=[configured_parameters, {"show_capture_timing": robot_name == robot_names[0]}],
            condition=IfCondition(LaunchConfiguration("enable_guidance_visualization")),
        ))
    return LaunchDescription(actions)
