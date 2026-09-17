from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from datetime import datetime

import os

# All robots are visible to the central station over DDS (same domain,
# stable LAN), so one recorder here captures every namespaced topic
# instead of running a recorder in each Pi container.
ALL_ROBOT_NAMES = ["RM1", "RM2", "RM3", "RM4", "RM5", "RM6"]


def load_robot_names():
    # Selection indices (1-based), e.g. ROBOT_IDX=1,2 for RM1 and RM2.
    # Defaults to every robot in ALL_ROBOT_NAMES.
    env_idx = os.getenv('ROBOT_IDX')

    if env_idx is None:
        return list(ALL_ROBOT_NAMES)

    idx = [int(i.strip()) for i in env_idx.split(',') if i.strip().isdigit()]

    if not idx:
        raise ValueError("ROBOT_IDX is set but contains no valid indices.")
    if len(set(idx)) != len(idx):
        raise ValueError("Duplicate indices found in selection.")
    if any(i < 1 or i > len(ALL_ROBOT_NAMES) for i in idx):
        raise IndexError("Selected index out of range.")

    return [ALL_ROBOT_NAMES[i - 1] for i in idx]


def generate_launch_description():
    robot_names = load_robot_names()
    N = len(robot_names)

    ld = LaunchDescription()

    ##############################################################################
    ############################ ROSBAG RECORDING ################################
    ##############################################################################

    bag_topics = [
        '/tf',
        '/tf_static',
    ]

    for robot_name in robot_names:
        bag_topics.extend([
            f'/{robot_name}/localization/odom',
            f'/{robot_name}/odom',

            # Virtual-spacecraft dynamics
            f'/{robot_name}/virtual_spacecraft/odom',
            f'/{robot_name}/virtual_spacecraft/applied_wrench',

            # Forces going into virtual-spacecraft dynamics
            f'/{robot_name}/spacecraft_wrench',
            f'/{robot_name}/avoidance_wrench',

            # ArUco perception
            f'/{robot_name}/landmarks_unf',
            f'/{robot_name}/landmarks',
            f'/{robot_name}/targets_unf',
            f'/{robot_name}/targets',

            # Velocity commands: before and after Nav2 velocity smoother
            f'/{robot_name}/cmd_vel_raw',
            f'/{robot_name}/cmd_vel',

            # Raw positioning / orientation comparison
            f'/{robot_name}/mm_pos_unf',
            f'/{robot_name}/localization/imu_odom',
            f'/{robot_name}/localization/mm_imu_odom',
        ])

    # ~/rosbags/swarm_20260909_185500/
    bag_root = os.path.expanduser('~/rosbags')
    os.makedirs(bag_root, exist_ok=True)

    bag_name = datetime.now().strftime(f'swarm_boundingbox_{N}{"agent" if N == 1 else "agents"}_%Y%m%d_%H%M%S')
    bag_path = os.path.join(bag_root, bag_name)

    bag_record = ExecuteProcess(
        cmd=[
            'ros2',
            'bag',
            'record',
            '-s', 'sqlite3',
            '-o', bag_path,
            *bag_topics,
        ],
        output='screen',
    )

    # Give the fleet's drivers a little time to appear before starting
    # rosbag discovery.
    ld.add_action(
        TimerAction(
            period=3.0,
            actions=[bag_record],
        )
    )

    return ld
