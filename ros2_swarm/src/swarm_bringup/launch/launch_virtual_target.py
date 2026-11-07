"""Standalone virtual perception source (the global virtual_scene node).

One global node replaces the cameras: it publishes /<ns>/landmarks and
/<ns>/targets for every selected robot plus the /virtual_scene/markers RViz
ground-truth (frame swarm_map).

Two ways to bring it up:
  * launch_centralized_swarm.py includes this file when VIRTUAL_TARGET=1 (it
    exports ROBOT_NAME first, so the scene matches the robots it launched).
  * On the dev machine alongside a decentralized swarm, run it on its own:
        ROBOT_IDX=5,6 ros2 launch swarm_bringup launch_virtual_target.py
    (same network / same DDS domain -> the Pis receive landmarks/targets).

Robot namespaces come from ROBOT_NAME (comma-separated) or, failing that,
ROBOT_IDX (comma-separated 1-based indices -> RM<idx>). SCENE_FILE overrides the
packaged default_scene.yaml.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
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


def virtual_scene_node(robot_names):
    """Build the global virtual_scene Node for the given namespaces."""
    scene_file = os.getenv("SCENE_FILE") or os.path.join(
        get_package_share_directory("swarm_target"),
        "config", "default_scene.yaml")
    estimator_on = os.environ.get("TARGET_ESTIMATOR", "").lower() in ("1", "true", "yes", "on")
    return Node(
        package="swarm_target",
        executable="virtual_scene",
        name="virtual_scene",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "robot_names": robot_names,
                "scene_file": scene_file,
                "publish_rate": 10.0,
                "max_range": 1.0,
                "half_fov_deg": 40.0,
                "max_incidence_deg": 70.0,
                "camera_yaw_offset_deg": 0.0,
                # Raw-observation sim mode: when the landmark estimator is on, feed
                # it hardware-like noisy/dropped/starved observations on the *_unf
                # topics (with stable ground-truth IDs) instead of the clean fused
                # landmarks/targets. Defaults below preserve legacy behaviour when
                # TARGET_ESTIMATOR is off.
                "emit_raw_observations": estimator_on,
                "publish_fused": not estimator_on,
                "noise_std_cm": 10.5 if estimator_on else 0.0,
                "normal_noise_std": 0.3 if estimator_on else 0.0,
                "observation_dropout_prob": 0.7 if estimator_on else 0.0,
                "obs_update_rate": 0.5,
                "global_frame_id": "swarm_map",
            }
        ]
    )


def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(virtual_scene_node(robot_names_from_environment()))
    return ld
