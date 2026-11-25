"""Dev-PC visualization: every RViz marker producer + RViz, so the Pis run none.

Pis deploy with GUIDANCE_VIZ=false (Ansible guidance_viz), which turns off both
spacecraft_visualizer and landmark_estimator's memory markers there. This launch
rebuilds both on the dev PC from the topics the Pis already publish:
  * spacecraft_visualizer per robot (pure subscriber, same params as on the Pi).
  * landmark/target memory: a viz-only copy of landmark_estimator fed the same
    *_unf + pose streams, so it draws the same spheres/normals/"id(n=count)"
    labels on the same topics. n counts the frames that reached the PC.

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

    # Same estimator settings as the Pi's landmark_estimator/target_estimator, but
    # BEST_EFFORT: a viz reader over WiFi must not make the Pi retransmit.
    estimator_params = {
        "qos_depth": 10, "qos_reliability": "BEST_EFFORT", "qos_history": "KEEP_LAST",
        "estimator_type": "averaging", "publish_rate": 15.0, "min_observations": 2,
        "forget_timeout": 0.0, "output_fov_gate": False, "camera_yaw_offset_deg": 0.0,
        "global_frame_id": "swarm_map", "publish_memory_viz": True,
    }

    for name in names:
        ld.add_action(Node(
            package="swarm_spacecraft", executable="spacecraft_visualizer",
            name="spacecraft_visualizer", namespace=name, output="screen",
            parameters=[ReplaceString(source_file=os.path.join(config_dir, "virtual_spacecraft.yaml"),
                                      replacements={"<robot_namespace>": name}),
                        {"show_capture_timing": name == names[0]}],
        ))
        # Distinct node names (the Pi's estimators share the namespace); the
        # estimate output goes to an unused topic so it never reaches controllers.
        for kind in ("landmark", "target"):
            ld.add_action(Node(
                package="swarm_target", executable="landmark_estimator",
                name=f"{kind}_memory_viz", namespace=name, output="screen",
                remappings=[("markers_unf", f"{kind}s_unf"),
                            ("markers", f"{kind}_memory/cloud"),
                            ("landmark_memory/markers", f"{kind}_memory/markers")],
                parameters=[estimator_params],
            ))

    ld.add_action(Node(
        package="rviz2", executable="rviz2", name="rviz2", output="screen",
        arguments=["-d", os.path.join(config_dir, "swarm_viz.rviz")],
        condition=IfCondition(LaunchConfiguration("rviz")),
    ))
    return ld
