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

        # Absolute names: every robot's marvelmind_obs publishes into these
        # same two topics, so one entry captures the whole modem TDMA cycle.
        # Without this the bag cannot answer "was beacon N alive?" -- the
        # per-robot mm_pos_unf below is already address-filtered, so a dead
        # beacon and a beacon the modem never scheduled look identical.
        '/mm_all_pos',
        '/mm_all_imu',

        # Latched JSON snapshot of the configs this run was launched with
        # (config YAMLs + env flags + git HEAD), so the bag is self-describing.
        # Published by the centralized swarm launch; the per-robot variant below
        # covers decentralized runs. See swarm_spacecraft/run_config_snapshot.py.
        '/run_config',
    ]

    for robot_name in robot_names:
        bag_topics.extend([
            # Per-agent config snapshot for decentralized runs, where each Pi
            # publishes its own env/config/git under its namespace.
            f'/{robot_name}/run_config',

            f'/{robot_name}/localization/odom',
            f'/{robot_name}/odom',

            # Virtual-spacecraft dynamics
            f'/{robot_name}/virtual_spacecraft/odom',
            f'/{robot_name}/virtual_spacecraft/applied_wrench',

            # Forces going into virtual-spacecraft dynamics. spacecraft_wrench
            # (bounding box) and avoidance_wrench are already captured; the
            # remaining additive contributors below carry the torque, so without
            # them a yaw issue is unattributable from the bag.
            f'/{robot_name}/spacecraft_wrench',
            f'/{robot_name}/avoidance_wrench',
            f'/{robot_name}/flocking_wrench',
            f'/{robot_name}/capture_wrench',
            f'/{robot_name}/pointing_wrench',

            # ArUco perception
            f'/{robot_name}/landmarks_unf',
            f'/{robot_name}/landmarks',
            f'/{robot_name}/targets_unf',
            f'/{robot_name}/targets',

            # Landmark/target estimator: map-frame memory clouds (not
            # reconstructable offline without the estimator's internal state; the
            # target instance remaps its viz to target_memory/markers) and the
            # visualizer overlay (estimated + raw) for offline RViz replay.
            f'/{robot_name}/landmark_memory/markers',
            f'/{robot_name}/target_memory/markers',
            f'/{robot_name}/spacecraft_visualizer/markers',

            # Velocity commands: before and after Nav2 velocity smoother
            f'/{robot_name}/cmd_vel_raw',
            f'/{robot_name}/cmd_vel',

            # Raw positioning / orientation comparison
            f'/{robot_name}/mm_pos_unf',
            f'/{robot_name}/localization/imu_odom',
            f'/{robot_name}/localization/mm_imu_odom',

            # Raw gyro feeds the pose_kf heading filter consumes directly
            # (namespace-relative imu/mm_imu). /mm_all_imu above is the
            # pre-split absolute feed and imu_odom/mm_imu_odom are derived;
            # recording these lets the KF be replayed input-for-input offline.
            f'/{robot_name}/imu',
            f'/{robot_name}/mm_imu',

            # Fused pose actually consumed by the guidance laws. Without it a
            # bag records the avoidance_wrench but not the state it was
            # computed from, so the law cannot be replayed offline.
            f'/{robot_name}/pose',

            # Both neighbour sources, so the two can be compared on the same
            # run: nbh_pos comes from the Marvelmind modem (anonymous, no
            # validity flag), comms/* from the UDP broadcast (carries
            # robot_id, age and valid).
            f'/{robot_name}/nbh_pos',
            f'/{robot_name}/comms/neighbours',
            f'/{robot_name}/comms/diagnostics',

            # Guidance setpoint, i.e. the attractive term's output.
            f'/{robot_name}/target_position',
        ])

    # ~/rosbags/swarm_20260909_185500/
    bag_root = os.path.expanduser('~/rosbags')
    os.makedirs(bag_root, exist_ok=True)

    # The experiment tag used to be hardcoded to "boundingbox", so every bag
    # in ~/rosbags claims to be a bounding-box run regardless of what was
    # actually tested. Override with BAG_TAG=avoidance (etc.) so runs can be
    # told apart later without opening each one.
    tag = os.getenv('BAG_TAG', 'boundingbox')
    bag_name = datetime.now().strftime(f'swarm_{tag}_{N}{"agent" if N == 1 else "agents"}_%Y%m%d_%H%M%S')
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
