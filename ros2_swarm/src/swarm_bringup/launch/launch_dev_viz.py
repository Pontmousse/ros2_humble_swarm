"""Dev-PC visualization: every RViz marker producer + RViz, so the Pis run none.

Pis deploy with GUIDANCE_VIZ=false (Ansible guidance_viz), which turns off
spacecraft_visualizer there. This launch rebuilds it on the dev PC from the topics
the Pis already publish (pure subscriber, same params as on the Pi). The landmark/
target memory markers need nothing here: each Pi's perception node publishes them
at 2 Hz (landmark_memory/markers, target_memory/markers) and RViz shows them.

Robots: ROBOT_NAME / ROBOT_IDX (comma list), else every robot in swarm_config.yaml.
    ROBOT_NAME=RM2,RM6 ros2 launch swarm_bringup launch_dev_viz.py [rviz:=false]
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString


def robot_names(config_dir):
    names = os.getenv("ROBOT_NAME")
    if names:
        return [n.strip() for n in names.split(",") if n.strip()]
    indices = os.getenv("ROBOT_IDX")
    if indices:
        return [f"RM{i.strip()}" for i in indices.split(",") if i.strip()]
    with open(os.path.join(config_dir, "swarm_config.yaml")) as f:
        return [r["name"] for r in yaml.safe_load(f)["robots"]]


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory("swarm_bringup"), "config")
    names = robot_names(config_dir)
    ld = LaunchDescription([DeclareLaunchArgument("rviz", default_value="true")])

    for name in names:
        ld.add_action(Node(
            package="swarm_spacecraft", executable="spacecraft_visualizer",
            name="spacecraft_visualizer", namespace=name, output="screen",
            parameters=[ReplaceString(source_file=os.path.join(config_dir, "virtual_spacecraft.yaml"),
                                      replacements={"<robot_namespace>": name}),
                        {"show_capture_timing": name == names[0]}],
        ))

    ld.add_action(Node(
        package="rviz2", executable="rviz2", name="rviz2", output="screen",
        arguments=["-d", os.path.join(config_dir, "swarm_viz.rviz")],
        condition=IfCondition(LaunchConfiguration("rviz")),
    ))
    return ld
