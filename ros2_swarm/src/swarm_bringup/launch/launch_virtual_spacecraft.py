"""Launch virtual spacecraft motion rendering for selected RoboMasters."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString


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
    """Create namespaced spacecraft simulator and tracker nodes."""
    default_parameters = os.path.join(
        get_package_share_directory("swarm_bringup"),
        "config",
        "virtual_spacecraft.yaml",
    )
    parameters_argument = DeclareLaunchArgument(
        "spacecraft_parameters", default_value=default_parameters
    )
    controller_mode_argument = DeclareLaunchArgument(
        "controller_mode",
        default_value="pff",
        choices=["pff", "dwb", "dwb_ff", "dwb_velocity_feedback"],
        description="Select the physical motion-rendering controller",
    )
    guidance_mode_argument = DeclareLaunchArgument(
        "guidance_mode",
        default_value="bounding_box",
        choices=["none", "bounding_box"],
        description="Select the virtual-spacecraft wrench guidance",
    )
    visualization_argument = DeclareLaunchArgument(
        "enable_guidance_visualization",
        default_value="true",
        choices=["true", "false"],
        description="Enable guidance and motion-rendering RViz markers",
    )
    controller_mode = LaunchConfiguration("controller_mode")
    guidance_mode = LaunchConfiguration("guidance_mode")
    bounding_box_condition = IfCondition(
        PythonExpression(["'", guidance_mode, "' == 'bounding_box'"])
    )
    visualization_condition = IfCondition(
        LaunchConfiguration("enable_guidance_visualization")
    )
    pff_condition = IfCondition(
        PythonExpression(["'", controller_mode, "' == 'pff'"])
    )
    dwb_condition = UnlessCondition(
        PythonExpression(["'", controller_mode, "' == 'pff'"])
    )
    actions = [
        parameters_argument,
        controller_mode_argument,
        guidance_mode_argument,
        visualization_argument,
    ]
    for robot_name in robot_names_from_environment():
        configured_parameters = ReplaceString(
            source_file=LaunchConfiguration("spacecraft_parameters"),
            replacements={"<robot_namespace>": robot_name},
        )
        actions.append(
            Node(
                package="swarm_spacecraft",
                executable="virtual_spacecraft",
                name="virtual_spacecraft",
                namespace=robot_name,
                output="screen",
                parameters=[configured_parameters],
            )
        )
        actions.append(
            Node(
                package="swarm_controller",
                executable="bounding_box_search",
                name="bounding_box_search",
                namespace=robot_name,
                output="screen",
                parameters=[configured_parameters],
                condition=bounding_box_condition,
            )
        )
        actions.append(
            Node(
                package="swarm_controller",
                executable="bounding_box_visualizer",
                name="bounding_box_visualizer",
                namespace=robot_name,
                output="screen",
                parameters=[configured_parameters],
                condition=visualization_condition,
            )
        )
        actions.append(
            Node(
                package="swarm_nav2_controller",
                executable="nav2_pidff",
                name="nav2_pidff",
                namespace=robot_name,
                output="screen",
                parameters=[configured_parameters],
                remappings=[("cmd_vel", "cmd_vel_raw")],
                condition=pff_condition,
            )
        )
        actions.append(
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                namespace=robot_name,
                output="screen",
                parameters=[configured_parameters],
                remappings=[("cmd_vel", "cmd_vel_raw")],
                condition=dwb_condition,
            )
        )
        actions.append(
            Node(
                package="swarm_nav2_controller",
                executable="nav2_dwb_path",
                name="nav2_dwb_path",
                namespace=robot_name,
                output="screen",
                parameters=[
                    configured_parameters,
                    {"controller_mode": controller_mode},
                ],
                condition=dwb_condition,
            )
        )
        actions.append(
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="controller_lifecycle_manager",
                namespace=robot_name,
                output="screen",
                parameters=[
                    {"autostart": True, "node_names": ["controller_server"]}
                ],
                condition=dwb_condition,
            )
        )
        actions.append(
            Node(
                package="nav2_velocity_smoother",
                executable="velocity_smoother",
                name="velocity_smoother",
                namespace=robot_name,
                output="screen",
                parameters=[configured_parameters],
                remappings=[
                    ("cmd_vel", "cmd_vel_raw"),
                    ("cmd_vel_smoothed", "cmd_vel"),
                ],
            )
        )
        actions.append(
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="velocity_smoother_lifecycle_manager",
                namespace=robot_name,
                output="screen",
                parameters=[
                    {"autostart": False, "node_names": ["velocity_smoother"]}
                ],
            )
        )
    return LaunchDescription(actions)
