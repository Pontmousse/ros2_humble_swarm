from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

import os
from launch.substitutions import LaunchConfiguration



gpio_line = int(os.environ.get("GPIO_LINE", "4"))

# STRICT_CALIBRATION=0 downgrades the aruco intrinsics sanity check from a hard
# failure to a warning, so the swarm can be brought up on a known-bad
# calibration while it is being re-shot. Default 1 (reject).
# Uses the ROBOT_IDX env-var convention rather than a launch argument, because
# the aruco detector parameters are built as a plain dict (a LaunchConfiguration
# would arrive as a string, not an int).
strict_calibration = 0 if os.environ.get("STRICT_CALIBRATION") == "0" else 1
if strict_calibration == 0:
    print("WARNING: STRICT_CALIBRATION=0 -- implausible camera intrinsics will "
          "only warn. Poses may be mis-scaled; do not record runs.")

# CALIBRATE_CAMERA=1 puts the *landmark* detector into chessboard-capture mode
# (20 photos, one every 6 s) instead of detecting markers.
#
# Env-driven rather than an edited-in literal so it cannot be left switched on
# by accident: an unset shell variable is the safe default. Applied ONLY to
# landmark_detector -- target_detector shares the same camera and the same
# output folder, so enabling both makes them overwrite each other's photos.
#
# Calibrate ONE robot at a time: the detector nodes are built in a loop over
# ROBOT_IDX, so this activates for every selected robot simultaneously.
calibrate_camera = 1 if os.environ.get("CALIBRATE_CAMERA") == "1" else 0
if calibrate_camera == 1:
    print("CALIBRATE_CAMERA=1 -- landmark_detector will capture chessboard "
          "photos instead of detecting markers.")

# VIRTUAL_TARGET=1 replaces the four camera nodes (landmark/target detectors and
# filters) with one global swarm_target/virtual_scene node that publishes the
# final landmarks/targets topics from a geometric world + sensor model, so the
# centralized stack runs with no camera. SCENE_FILE overrides the default scene.
virtual_target_on = os.environ.get("VIRTUAL_TARGET", "").lower() in ("1", "true", "yes", "on")
if virtual_target_on:
    print("VIRTUAL_TARGET=1 -- camera detectors replaced by virtual_scene.")
    if calibrate_camera == 1:
        print("WARNING: CALIBRATE_CAMERA needs a real camera; ignored under "
              "VIRTUAL_TARGET.")

# TARGET_ESTIMATOR=1 inserts the landmark_estimator between perception and the
# controllers in place of aruco_filter: it keeps a map-frame memory keyed by
# ArUco ID and republishes the whole remembered cloud at a steady rate, so the
# controller reference persists even when markers leave the FOV (killing the
# perception-starvation yaw limit cycle). Under VIRTUAL_TARGET it also switches
# virtual_scene into raw-observation mode (noisy/dropped/starved *_unf streams
# with ground-truth IDs) so the estimator is tested before hardware.
estimator_on = os.environ.get("TARGET_ESTIMATOR", "").lower() in ("1", "true", "yes", "on")
if estimator_on:
    print("TARGET_ESTIMATOR=1 -- landmark_estimator replaces aruco_filter.")

# VIRTUAL_PLANT=1 replaces the robot + localization chain (robomaster driver,
# marvelmind, pose_publisher) with one kinematic fake_plant per robot that
# integrates cmd_vel and republishes pose + localization/odom, so the full
# mission loop runs on the dev machine with no hardware. Intended with
# VIRTUAL_TARGET=1 (no camera) and VIRTUAL_SPACECRAFT=1 (the loop that makes it move).
virtual_plant_on = os.environ.get("VIRTUAL_PLANT", "").lower() in ("1", "true", "yes", "on")
if virtual_plant_on:
    print("VIRTUAL_PLANT=1 -- robot + localization replaced by fake_plant.")
    if not virtual_target_on:
        print("WARNING: VIRTUAL_PLANT without VIRTUAL_TARGET -- the real camera "
              "path needs the driver, which is now gated off; set VIRTUAL_TARGET=1.")
    if os.environ.get("VIRTUAL_SPACECRAFT", "").lower() not in ("1", "true", "yes", "on"):
        print("WARNING: VIRTUAL_PLANT without VIRTUAL_SPACECRAFT -- nothing will "
              "produce cmd_vel, so the plant will sit still; set VIRTUAL_SPACECRAFT=1.")

# KF_LOCALIZATION=1 swaps the localization fuser for pose_kf (two decoupled
# linear Kalman filters: heading [theta, omega] + planar [x, y, vx, vy]). It
# runs in place of pose_publisher on hardware, and -- under VIRTUAL_PLANT -- is
# driven by fake_plant's simulated noisy raw sensors (emit_raw_sensors) instead
# of its zero-noise fused output, so the estimator is tested before hardware.
# Default off: nothing changes. See pose_kf.py.
kf_on = os.environ.get("KF_LOCALIZATION", "").lower() in ("1", "true", "yes", "on")
if kf_on:
    print("KF_LOCALIZATION=1 -- pose_kf replaces pose_publisher"
          + (" (fed by fake_plant raw sensors)." if virtual_plant_on
             else " (fed by Marvelmind + driver odom/imu)."))

# Single source of truth for the CAPTURE approach-law equilibrium --
# see cp_proportional_controller.py / fake_plant.py docstrings.
CONTACT_DISTANCE = 0.20
# The approach law parks the robot exactly at CONTACT_DISTANCE, so the contact
# gate must sit slightly beyond it or a settled robot reads a hair outside and
# never latches the debounce. Detection threshold = equilibrium + this margin.
CONTACT_MARGIN = 0.05

# DIRECT_CMD_VEL=1 lets virtual_spacecraft and direct capture run together:
# cp_controller becomes the sole cmd_vel writer and muxes the nav chain
# (remapped to cmd_vel_nav, see launch_virtual_spacecraft.py) against its own
# capture command. Must match on both this node and the smoother, so it is a
# single env flag read in both launch files.
DIRECT_CMD_VEL = os.environ.get("DIRECT_CMD_VEL", "").lower() in ("1", "true", "yes", "on")

def load_swarm_config():

    # Original configuration (order matters!)
    beacon_addresses = [1, 2, 3, 4, 5, 6]

    robot_names = ["RM1", "RM2", "RM3", "RM4", "RM5", "RM6"]

    robot_serial_numbers = [
        "159CKC50070ECX",
        "159CKC50070E5N",
        "159CG9J0050797",
        "159CG9V0050HED",
        "159CKCH0070F8S",
        "159CKC50070EVP"
    ]

    # Pinned robot IPs, same order as robot_names. Leave "" to fall back to
    # SN broadcast discovery (UDP 40927), which only works on a network that
    # forwards broadcast to the host.
    # Reserved scheme on the RT-BE58U (192.168.50.0/24): RMn -> 192.168.50.(150+n).
    # Fill an entry in only once that robot has a DHCP reservation on the router.
    robot_ips = ["", "", "", "", "", ""]

    env_ips = os.getenv('ROBOT_IPS')
    if env_ips:
        for entry in env_ips.split(','):
            name, _, ip = entry.partition('=')
            robot_ips[robot_names.index(name.strip())] = ip.strip()

    # initial angles for orientation propagation. should be in degree,
    # although it will be converted to radians
    init_orientations = [180.0, 180.0, 0.0, 0.0, 0.0, 0.0]



    # Selection indices (1-based indexing)
    # e.g., selecting RM1 and RM2
    # Here order doesn't matter, selecting [1, 2] or [2, 1] is exactly the same
    # idx = [1, 2]  
    # idx = [2, 3]
    # idx = [3]
    # idx = [1, 2, 3, 4]  # Select all robots


    ###############################################################################
    # Set from environment variable (e.g., ROBOT_IDX=1,2 for RM1 and RM2)
    env_idx = os.getenv('ROBOT_IDX')
    env_idx = '1,2,3'

    if env_idx is None:
        raise RuntimeError("ROBOT_IDX environment variable must be set.")

    idx = [int(i.strip()) for i in env_idx.split(',') if i.strip().isdigit()]

    if not idx:
        raise ValueError("ROBOT_IDX is set but contains no valid indices.")


    ################################################################################################
    ################################################################################################
    ################################################################################################
    ################################################################################################
    # --- Validations ---

    # Check that all lists are the same length
    n = len(robot_names)
    if not (len(beacon_addresses) == len(robot_names) == len(robot_serial_numbers) == len(init_orientations)):
        raise ValueError("All configuration lists must be of equal length.")

    # Check for uniqueness in configuration
    if len(set(beacon_addresses)) != n:
        raise ValueError("Duplicate values found in beacon_addresses.")
    if len(set(robot_names)) != n:
        raise ValueError("Duplicate values found in robot_names.")
    if len(set(robot_serial_numbers)) != n:
        raise ValueError("Duplicate values found in robot_serial_numbers.")

    # Check selection validity
    if len(set(idx)) != len(idx):
        raise ValueError("Duplicate indices found in selection.")
    if any(i < 1 or i > n for i in idx):
        raise IndexError("Selected index out of range.")
    if len(idx) > n:
        raise ValueError("More robots selected than available.")

    # Convert 1-based to 0-based indexing
    idx_zero_based = [i - 1 for i in idx]

    # Apply selection
    beacon_addresses = [beacon_addresses[i] for i in idx_zero_based]
    robot_names = [robot_names[i] for i in idx_zero_based]
    robot_serial_numbers = [robot_serial_numbers[i] for i in idx_zero_based]
    robot_ips = [robot_ips[i] for i in idx_zero_based]
    init_orientations = [init_orientations[i] for i in idx_zero_based]

    return beacon_addresses, robot_names, robot_serial_numbers, robot_ips, init_orientations


##########################################################################################################################################################
##########################################################################################################################################################
##########################################################################################################################################################
##########################################################################################################################################################
##########################################################################################################################################################
##########################################################################################################################################################
##########################################################################################################################################################
##########################################################################################################################################################
##########################################################################################################################################################
##########################################################################################################################################################
##########################################################################################################################################################
##########################################################################################################################################################
##########################################################################################################################################################
##########################################################################################################################################################

def qos_parameters(depth=5, reliability='BEST_EFFORT', history='KEEP_LAST'):
    # qos_params = {
    #     'qos_depth': depth,
    #     'qos_reliability': reliability,
    #     'qos_history': history
    # }

    qos_params = {
        'qos_depth': 10,
        'qos_reliability': 'RELIABLE',
        'qos_history': 'KEEP_LAST'
    }
    
    return qos_params

def rm_preflight(robot_ips, verbose=False):
    """Network report printed before the drivers start. See rm_preflight.py."""
    script = os.path.join(
        get_package_share_directory('swarm_bringup'), 'launch', 'rm_preflight.py')
    cmd = ['python3', script]
    if verbose:
        cmd += ['--listen', '5']
    cmd += [ip for ip in robot_ips if ip]
    return ExecuteProcess(cmd=cmd, output='screen')


def _find_repo_root(start):
    """Walk up from `start` to the first dir containing .git, else '' ."""
    path = os.path.abspath(start)
    while True:
        if os.path.isdir(os.path.join(path, '.git')):
            return path
        parent = os.path.dirname(path)
        if parent == path:
            return ''
        path = parent


def run_config_snapshot_node(robot_names, source_launch, namespace=None,
                             extra_env_vars=()):
    """Node that latches a JSON snapshot of this run's config files + env vars.

    Lets a bag answer "what was this run configured with?" offline. The config
    files are resolved the same way the stack itself resolves them (share dirs,
    SCENE_FILE override), so the snapshot is the bytes that actually served the
    run. Published latched so a recorder starting later still captures it; the
    rosbag launch records /run_config (and /<ns>/run_config). See
    swarm_spacecraft/run_config_snapshot.py.
    """
    scene_file = os.getenv('SCENE_FILE') or os.path.join(
        get_package_share_directory('swarm_target'), 'config', 'default_scene.yaml')
    config_files = [
        os.path.join(get_package_share_directory('swarm_bringup'),
                     'config', 'virtual_spacecraft.yaml'),
        scene_file,
        os.path.join(get_package_share_directory('swarm_bringup'),
                     'config', 'swarm_config.yaml'),
    ]
    env_vars = [
        'VIRTUAL_TARGET', 'VIRTUAL_PLANT', 'VIRTUAL_SPACECRAFT', 'KF_LOCALIZATION',
        'TARGET_ESTIMATOR', 'DIRECT_CMD_VEL', 'SCENE_FILE', 'ROBOT_IDX', 'ROBOT_NAME',
        'BAG_TAG', 'STRICT_CALIBRATION', 'CALIBRATE_CAMERA', 'CONN_TYPE',
        'CAMERA_SOURCE', 'RM_DIAGNOSTICS',
    ] + list(extra_env_vars)
    return Node(
        package='swarm_spacecraft',
        executable='run_config_snapshot',
        name='run_config_snapshot',
        namespace=namespace,
        output='screen',
        emulate_tty=True,
        parameters=[
            {
                'config_files': config_files,
                'env_vars': env_vars,
                'topic': 'run_config',
                'publish_period': 5.0,
                'repo_path': _find_repo_root(__file__),
                'source_launch': source_launch,
                'robot_names': list(robot_names),
            }
        ],
    )

def generate_launch_description():
    init_period = 2.0 # in seconds
    alpha = 0.9 # Filter coefficient (1.0 = no filter, 0.0 = freeze first value)
    # Separate, heavier smoothing for the capture target only. alpha=0.9 is
    # near-passthrough, so raw solvePnP depth jitter (amplified 1.5x by
    # aruco_scale) reached the capture law and made the real robot lurch toward
    # the target in steps. 0.25 trades ~80-120 ms of lag for a smooth setpoint.
    # Kept distinct from `alpha` so landmark_filter / mm_filtered are unchanged.
    target_alpha = 0.25

    # Choose timer period (or timer frequency) for performance testing (higher period = lower CPU usage)
    timer_frequency = 0.01 # in seconds
    # timer_frequency = 0.1 # in seconds
    # timer_frequency = 1.0 # in seconds

    # Shared spacecraft params yaml. The guidance nodes (pointing/motion/capture)
    # read the global /** translation_response_scale / rotation_response_scale
    # from here so their 1/sqrt(scale) damping boost stays consistent with the
    # wrench scaling the virtual_spacecraft node applies. Listed FIRST in each
    # node's parameters so the inline dict still wins for timer_frequency etc.;
    # the yaml only contributes the /** scales (it uses /** wildcards, so it
    # matches the launch-renamed node names in any namespace).
    spacecraft_params = os.path.join(
        get_package_share_directory('swarm_bringup'),
        'config', 'virtual_spacecraft.yaml')

    beacon_addresses, robot_names, robot_serial_numbers, robot_ips, init_orientations = load_swarm_config()
    N = len(robot_names)

    ld = LaunchDescription()

    # Latched JSON snapshot of this run's config files + env flags, so the bag is
    # self-describing. Global /run_config (this process owns the real env flags).
    ld.add_action(run_config_snapshot_node(robot_names, source_launch='centralized'))

    # 'off' (default), 'on', or 'verbose'. See rm_preflight.py and
    # robomaster_ros/diagnostics.py.
    diagnostics = os.environ.get('RM_DIAGNOSTICS', 'off').lower()
    preflight_action = None
    if diagnostics != 'off':
        preflight_action = rm_preflight(robot_ips, verbose=(diagnostics == 'verbose'))
        ld.add_action(preflight_action)

    ##############################################################################
    ##############################################################################
    # Pre-load the saved KF debug layout (installed into swarm_filter's share
    # dir via its setup.py: estimate vs ground_truth vs raw sensors). -n skips
    # the startup dialog; -l restores the tabs + topic subscriptions, like an
    # rviz config, so the debug view no longer has to be rebuilt each launch.
    plot_juggler = Node(
        package="plotjuggler",
        executable="plotjuggler",
        name="plot_juggler",
        arguments=[
            "-n",
            "-l",
            os.path.join(
                get_package_share_directory("swarm_filter"), "layout.xml"),
        ],
    )

    # ld.add_action(plot_juggler)

    ##############################################################################
    ##############################################################################

    

    for i in range(N):
        robot_name = robot_names[i]
        serial_number = robot_serial_numbers[i]

        # Centralized launches from the dev machine over WiFi.
        conn_type = os.getenv("CONN_TYPE", "sta")

        name_arg = DeclareLaunchArgument(f'name_{i}', default_value=robot_name)
        serial_arg = DeclareLaunchArgument(f'serial_number_{i}', default_value=serial_number)
        robot_ip_arg = DeclareLaunchArgument(f'robot_ip_{i}', default_value=robot_ips[i])
        enable_led_arg = DeclareLaunchArgument(f'enable_led_{i}', default_value='true')
        enable_speaker_arg = DeclareLaunchArgument(f'enable_speaker_{i}', default_value='true')
        enable_chassis_arg = DeclareLaunchArgument(f'enable_chassis_{i}', default_value='true')
        twist_to_wheel_speeds_arg = DeclareLaunchArgument(f'twist2wheel_{i}', default_value='true')

        ld.add_action(name_arg)
        ld.add_action(serial_arg)
        ld.add_action(robot_ip_arg)
        ld.add_action(enable_led_arg)
        ld.add_action(enable_speaker_arg)
        ld.add_action(enable_chassis_arg)
        ld.add_action(twist_to_wheel_speeds_arg)

        # Hardware driver + its LED/speaker greetings. Skipped under VIRTUAL_PLANT
        # (no robot to drive, and the action send_goal would hang on an absent
        # server); fake_plant replaces the robot further down. The robomaster_ros
        # package is only present on the robots, so its share lookup must stay
        # inside this guard or the dev-machine fake run can't even load the file.
        if not virtual_plant_on:
            path = os.path.join(
                get_package_share_directory('robomaster_ros'),
                'launch',
                's1.launch'
            )

            # Camera wiring, set explicitly instead of inheriting driver defaults.
            #  VIRTUAL_TARGET=1 -> unload the module (no camera, saves bandwidth);
            #  real run (=0)    -> h264 ON_DEMAND decoded client-side by the detectors,
            #                      raw + ffmpeg streams killed as unused footguns.
            if virtual_target_on:
                camera_args = {'camera': 'false', 'vision_targets': '[]'}
            else:
                camera_args = {'camera': 'true', 'video_h264': '2',
                               'video_raw': '-1', 'video_ffmpeg': '-1'}

            s1 = IncludeLaunchDescription(
                XMLLaunchDescriptionSource(path),
                launch_arguments={
                    'name': LaunchConfiguration(f'name_{i}'),
                    'serial_number': LaunchConfiguration(f'serial_number_{i}'),
                    'conn_type': conn_type,
                    'robot_ip': LaunchConfiguration(f'robot_ip_{i}'),
                    'diagnostics': diagnostics,
                    'leds.enabled': LaunchConfiguration(f'enable_led_{i}'),
                    'speaker.enabled': LaunchConfiguration(f'enable_speaker_{i}'),
                    'chassis.enabled': LaunchConfiguration(f'enable_chassis_{i}'),
                    'chassis.twist_to_wheel_speeds': LaunchConfiguration(f'twist2wheel_{i}'),
                    **camera_args,
                }.items()
            )

            if diagnostics == 'verbose' and preflight_action is not None:
                # verbose preflight binds UDP 40927 itself (passive SN-broadcast
                # listen), the same port the driver's own discovery needs. Starting
                # both at once races for the port and the driver sees a spurious
                # "Address already in use". Delay the driver until preflight exits.
                ld.add_action(RegisterEventHandler(
                    OnProcessExit(target_action=preflight_action, on_exit=[s1])))
            else:
                ld.add_action(s1)

            ##############################################################################

            topic = '/'+robot_name+'/leds/color'
            led = ExecuteProcess(
                cmd=[
                    'ros2', 'topic', 'pub', '--once',
                    topic,
                    'std_msgs/msg/ColorRGBA',
                    '{r: 0.4, g: 0.4, b: 0.0, a: 1.0}'
                ]
            )

            ld.add_action(led)

            ##############################################################################

            action = '/'+robot_name+'/play'
            speaker = ExecuteProcess(
                cmd=[
                    'ros2', 'action', 'send_goal',
                    action,
                    'robomaster_msgs/action/PlaySound',
                    '{sound_id: 5, times: 1}'
                ]
            )

            ld.add_action(speaker)


    ##############################################################################
    ##############################################################################
 
    mm = Node(
    package="swarm_marvelmind",
    executable="marvelmind_obs",
    name="marvelmind_obs",
    output="screen",
    emulate_tty=True,
    parameters=[
            {
                "robot_names": robot_names,
                "beacon_addresses": beacon_addresses,
                "port_address": "/dev/ttyACM0",
                **qos_parameters(depth=1, reliability='BEST_EFFORT')
                        }
        ]
    )

    # Marvelmind is irrelevant to camera calibration, and with no beacon on
    # /dev/ttyACM0 it retries in a tight loop whose log spam buries the photo
    # countdown. Skip it while calibrating; restored automatically afterwards
    # because this keys off the env var, not a commented-out line.
    if calibrate_camera == 1:
        print("CALIBRATE_CAMERA=1 -- skipping marvelmind_obs (log spam would "
              "bury the photo countdown).")
    elif virtual_plant_on:
        print("VIRTUAL_PLANT=1 -- skipping marvelmind_obs (fake_plant provides pose).")
    else:
        ld.add_action(mm)

    ##############################################################################
    ##############################################################################
    for i in range(N):
        robot_name = robot_names[i]
        beacon_address = beacon_addresses[i]

        hedge = Node(
        package="swarm_marvelmind",
        executable="hedgehog_obs",
        name="hedgehog_obs",
        namespace=robot_name,
        output="screen",
        emulate_tty=True,
        parameters=[
                {
                    "robot_name": robot_name,
                    "beacon_address": beacon_address,
                    "timer_frequency": timer_frequency,
                    "init_period": init_period,
                    **qos_parameters(depth=1, reliability='BEST_EFFORT')
                }
            ]
        )

        # hedgehog_obs only consumes marvelmind_obs output, which is not
        # running during calibration. Silent either way; skipped for tidiness.
        # Also skipped under VIRTUAL_PLANT (no beacon, fake_plant provides pose).
        if calibrate_camera != 1 and not virtual_plant_on:
            ld.add_action(hedge)

    ##############################################################################
    ########################## - UDP state broadcast - ###########################
    # Sole publisher of comms/neighbours, which motion_planner's avoidance law
    # consumes. On one dev host the instances co-bind via SO_REUSEADDR and each
    # receives the subnet broadcast.
    comms_broadcast_address = os.getenv("SWARM_BROADCAST_ADDR", "192.168.50.255")
    comms_bind_device = os.getenv("SWARM_BROADCAST_IFACE", "")

    for i in range(N):
        robot_name = robot_names[i]

        comms = Node(
            package="swarm_comms",
            executable="state_broadcaster",
            name="state_broadcaster",
            namespace=robot_name,
            output="screen",
            emulate_tty=True,
            parameters=[
                    {
                        "robot_id": beacon_addresses[i],
                        "broadcast_address": comms_broadcast_address,
                        "bind_device": comms_bind_device,
                        "broadcast_rate": 10.0,
                        "receive_rate": 50.0,
                        "stale_timeout": 0.5,
                        "drop_timeout": 5.0,
                    }
                ]
            )

        ld.add_action(comms)

    ##############################################################################
    ##############################################################################
    for i in range(N):
        robot_name = robot_names[i]

        mm_filtered = Node(
        package="swarm_filter",
        executable="mm_filtered",
        name="mm_filtered",
        namespace=robot_name,
        output="screen",
        emulate_tty=True,
        parameters=[
                {
                    **qos_parameters(depth=1, reliability='BEST_EFFORT'),
                    "alpha": alpha,  # Filter coefficient
                }
            ]
        )

        # mm_filtered fuses Marvelmind; nothing feeds it under VIRTUAL_PLANT.
        if not virtual_plant_on:
            ld.add_action(mm_filtered)

    ##############################################################################
    ##############################################################################
    for i in range(N):
        robot_name = robot_names[i]
        init_orientation = init_orientations[i]

        # Virtual-plant start geometry. Computed here (not just inside the
        # if virtual_plant_on block) because the KF's init_orientation must
        # match fake_plant's true sim heading -- otherwise the estimator seeds
        # a different heading than the simulated ground truth and the whole
        # map-frame cloud is rotated. heading_deg is the sim heading; on
        # hardware the KF falls back to init_orientation (per-robot boot yaw).
        cluster_x, cluster_y, heading_deg = -4.7, -2.5, 0.0
        spacing = 0.4
        init_y = cluster_y + (i - (N - 1) / 2.0) * spacing

        pose = Node(
        package="swarm_filter",

        # executable="kalman_filter",
        # name="kalman_filter",

        executable="pose_publisher",
        name="pose_publisher",


        namespace=robot_name,
        output="screen",
        emulate_tty=True,
        parameters=[
                {
                    "init_period": init_period,
                    "timer_frequency": timer_frequency,
                    "init_orientation": init_orientation,
                    "global_frame_id": "swarm_map",
                    "gps_timeout": 0.25,
                    "odometry_timeout": 0.25,
                    "gps_position_variance": 0.01,
                    **qos_parameters(depth=1, reliability='BEST_EFFORT')
                }
            ]
        )

        # Kalman-filter localization (KF_LOCALIZATION=1). Same output contract
        # as pose_publisher (pose + localization/odom + swarm_map->odom TF), so
        # every downstream consumer is unchanged. Runs on hardware and, under
        # VIRTUAL_PLANT, on fake_plant's simulated raw sensors.
        pose_kf = Node(
            package="swarm_filter",
            executable="pose_kf",
            name="pose_kf",
            namespace=robot_name,
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    "init_period": init_period,
                    "timer_frequency": timer_frequency,
                    "init_orientation": heading_deg if virtual_plant_on else init_orientation,
                    "global_frame_id": "swarm_map",
                    "gps_timeout": 0.25,
                    "odometry_timeout": 0.25,
                    "gyro_timeout": 0.25,
                    "gps_position_variance": 0.01,
                    "command_tau": 0.25,
                    "q_pos": 1.0e-4,
                    "q_vel": 0.005,
                    "q_theta": 1.0e-4,
                    "q_omega": 0.005,
                    "r_odom_vel": 0.001,
                    "r_yaw": 0.01,
                    "r_gyro": 0.01,
                    "innovation_gate_chi2": 9.0,
                    "require_mm_for_init": True,
                    **qos_parameters(depth=1, reliability='BEST_EFFORT'),
                }
            ]
        )

        if kf_on:
            ld.add_action(pose_kf)

        if virtual_plant_on:
            # Kinematic stand-in for the robot + localization: integrates cmd_vel
            # and republishes pose + localization/odom. Robots start clustered on
            # ONE side of the box, same heading, so the encapsulation controller
            # (flocking + avoidance) has to spread them around it -- that is the
            # behaviour under test, so they are NOT pre-distributed. Accel limits
            # cut to 1/3 of default to mimic a laggy chassis response.
            # Robots cluster on the LEFT of the target (com x -2.6, left edge
            # -2.975) facing +x toward it; -3.7 sits inside the search region
            # (x_min -4.5, boundary_margin 0.4 -> margin edge -4.1), clear of the
            # rebound band. cluster_x / cluster_y / heading_deg / init_y are
            # computed above the pose_kf Node so the KF seeds the same heading.
            fake_plant = Node(
                package="swarm_spacecraft",
                executable="fake_plant",
                name="fake_plant",
                namespace=robot_name,
                output="screen",
                emulate_tty=True,
                parameters=[
                        {
                            "init_pose": [cluster_x, init_y, heading_deg],
                            "timer_frequency": 0.02,
                            "global_frame_id": "swarm_map",
                            "max_linear_accel": 50.0,
                            "max_yaw_accel": 50.0,
                            "max_speed": 50.0,
                            "max_yaw_rate": 50.0,
                            "contact_distance": CONTACT_DISTANCE + CONTACT_MARGIN,
                            "contact_heading_tolerance": 0.26,
                            "contact_debounce_frames": 3,
                            # Under KF_LOCALIZATION, emit hardware-like noisy raw
                            # sensors for pose_kf and stop publishing the fused
                            # pose/localization (the KF owns them).
                            "emit_raw_sensors": kf_on,
                            "publish_fused": not kf_on,
                            **qos_parameters(depth=10, reliability='RELIABLE')
                        }
                    ]
                )
            ld.add_action(fake_plant)
        elif not kf_on:
            ld.add_action(pose)

    ##############################################################################
    ##############################################################################
    # Camera detectors/filters are skipped when VIRTUAL_TARGET is on; the virtual
    # scene below publishes landmarks/targets directly instead.
    n_cam = 0 if virtual_target_on else N
    for i in range(n_cam):
        robot_name = robot_names[i]
        serial_number = robot_serial_numbers[i]

        landmarks = Node(
        package="swarm_aruco",
        executable="aruco_detector",
        name="landmark_detector",
        namespace=robot_name,
        output="screen",
        remappings=[
                ('markers_unf', 'landmarks_unf'),  # Remap markers to landmarks
            ],
        emulate_tty=True,
        parameters=[
                {
                    "timer_frequency": timer_frequency,
                    "serial_number": serial_number,
                    "aruco_type": "DICT_4X4_100",
                    "aruco_size": 3.56,
                    "aruco_scale": (1.15, 0.8),
                    "calibrate_camera": calibrate_camera,
                    "strict_calibration": strict_calibration,
                    "use_h264": True,  # decode the driver's h264 stream client-side
                    **qos_parameters(depth=1, reliability='BEST_EFFORT')
                }
            ]
        )

        ld.add_action(landmarks)
    
    ##############################################################################
    ##############################################################################
    for i in range(n_cam):
        robot_name = robot_names[i]
        serial_number = robot_serial_numbers[i]

        aruco_landmark_filter = Node(
        package="swarm_filter",
        executable="aruco_filter",
        name="landmark_filter",
        namespace=robot_name,
        output="screen",
        remappings=[
                ('markers_unf', 'landmarks_unf'),  # Remap markers to landmarks
                ('markers', 'landmarks')
            ],
        emulate_tty=True,
        parameters=[
                {
                    **qos_parameters(depth=1, reliability='BEST_EFFORT'),
                    "alpha": alpha,  # Filter coefficient
                    "hold_timeout": 0.0,  # no stale-hold: a dropout emits empty, not a ghost
                }
            ]
        )

        # TARGET_ESTIMATOR=1 replaces aruco_filter with the landmark_estimator
        # (added in its own loop below), so the raw filter is gated off here.
        if not estimator_on:
            ld.add_action(aruco_landmark_filter)



    ##############################################################################
    ##############################################################################
    for i in range(n_cam):
        robot_name = robot_names[i]
        serial_number = robot_serial_numbers[i]

        targets = Node(
        package="swarm_aruco",
        executable="aruco_detector",
        name="target_detector",
        namespace=robot_name,
        output="screen",
        remappings=[
                ('markers_unf', 'targets_unf'),  # Remap markers to landmarks
            ],
        emulate_tty=True,
        parameters=[
                {
                    "timer_frequency": timer_frequency,
                    "serial_number": serial_number,
                    "aruco_type": "DICT_5X5_50",
                    "aruco_size": 5.76,
                    # Deliberately NOT wired to CALIBRATE_CAMERA: this node
                    # shares the camera and the photo output folder with
                    # landmark_detector, and two capturers overwrite each
                    # other's photos.
                    "aruco_scale": (1.15, 0.8),
                    "calibrate_camera": 0,
                    "strict_calibration": strict_calibration,
                    "use_h264": True,  # decode the driver's h264 stream client-side
                    **qos_parameters(depth=1, reliability='BEST_EFFORT')
                }
            ]
        )

        ld.add_action(targets)

    ##############################################################################
    ##############################################################################
    for i in range(n_cam):
        robot_name = robot_names[i]

        targets_filter = Node(
        package="swarm_filter",
        executable="aruco_filter",
        name="targets_filter",
        namespace=robot_name,
        output="screen",
        remappings=[
                ('markers_unf', 'targets_unf'),  # Remap markers to landmarks
                ('markers', 'targets')
            ],
        emulate_tty=True,
        parameters=[
                {
                    **qos_parameters(depth=1, reliability='BEST_EFFORT'),
                    "alpha": target_alpha,  # heavier smoothing, see target_alpha
                    "hold_timeout": 0.0,  # no stale-hold: a dropout emits empty, not a ghost
                }
            ]
        )

        if not estimator_on:
            ld.add_action(targets_filter)

    ##############################################################################
    ##############################################################################
    # TARGET_ESTIMATOR=1: the landmark_estimator in place of both aruco_filter
    # instances. Generic single-stream node instantiated twice via remappings
    # (landmark + target); it keeps a map-frame memory keyed by ArUco ID and
    # republishes the whole remembered cloud at publish_rate projected through the
    # live pose, so the controller reference persists through FOV loss. QoS mirrors
    # the aruco_filter it replaces, so controllers are untouched. Runs for all
    # robots: the *_unf input comes from real detectors (VIRTUAL_TARGET=0) or from
    # virtual_scene's raw-observation mode (VIRTUAL_TARGET=1).
    if estimator_on:
        for i in range(N):
            robot_name = robot_names[i]

            landmark_estimator = Node(
                package="swarm_target",
                executable="landmark_estimator",
                name="landmark_estimator",
                namespace=robot_name,
                output="screen",
                remappings=[
                    ('markers_unf', 'landmarks_unf'),
                    ('markers', 'landmarks'),
                ],
                emulate_tty=True,
                parameters=[
                    {
                        **qos_parameters(depth=1, reliability='BEST_EFFORT'),
                        "estimator_type": "averaging",
                        "publish_rate": 15.0,
                        "min_observations": 2,
                        "forget_timeout": 0.0,
                        "output_fov_gate": False,
                        "camera_yaw_offset_deg": 0.0,
                        "global_frame_id": "swarm_map",
                    }
                ]
            )
            ld.add_action(landmark_estimator)

            target_estimator = Node(
                package="swarm_target",
                executable="landmark_estimator",
                name="target_estimator",
                namespace=robot_name,
                output="screen",
                remappings=[
                    ('markers_unf', 'targets_unf'),
                    ('markers', 'targets'),
                    # Distinct memory-viz topic so the two instances in this
                    # namespace do not collide (rosbag records both).
                    ('landmark_memory/markers', 'target_memory/markers'),
                ],
                emulate_tty=True,
                parameters=[
                    {
                        **qos_parameters(depth=1, reliability='BEST_EFFORT'),
                        "estimator_type": "averaging",
                        "publish_rate": 15.0,
                        "min_observations": 2,
                        "forget_timeout": 0.0,
                        "output_fov_gate": False,
                        "camera_yaw_offset_deg": 0.0,
                        "global_frame_id": "swarm_map",
                    }
                ]
            )
            ld.add_action(target_estimator)

    ##############################################################################
    ##############################################################################
    # One global virtual perception source replacing the cameras. Publishes
    # /<ns>/landmarks and /<ns>/targets for every selected robot, plus the
    # /virtual_scene/markers RViz ground-truth (frame swarm_map).
    if virtual_target_on:
        # Export ROBOT_NAME so the included launch targets the same robots we
        # launched (it falls back to ROBOT_IDX when run standalone on the dev PC).
        os.environ["ROBOT_NAME"] = ",".join(robot_names)
        virtual_target_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("swarm_bringup"),
                    "launch",
                    "launch_virtual_target.py"
                )
            )
        )
        ld.add_action(virtual_target_launch)

    ##############################################################################
    ############################## - Encapsulation Phase - #######################
    for i in range(N):
        robot_name = robot_names[i]

        ep_control = Node(
            package="swarm_controller",
            executable="ep_proportional_controller",
            name="ep_controller",
            namespace=robot_name,
            output="screen",
            emulate_tty=True,
            parameters=[
                    {
                        "timer_frequency": timer_frequency,
                        "init_period": init_period,
                        **qos_parameters(depth=1, reliability='RELIABLE')
                    }
                ]
            )

        # ld.add_action(ep_control)

    ##############################################################################
    ############################## - Encapsulation Phase - #######################
    for i in range(N):
        robot_name = robot_names[i]

        ep_pointing_planner = Node(
            package="swarm_controller",
            executable="pointing_guidance2",
            name="pointing_planner",
            namespace=robot_name,
            output="screen",
            emulate_tty=True,
            parameters=[
                    spacecraft_params,
                    {
                        "timer_frequency": timer_frequency,
                        "init_period": init_period,
                        **qos_parameters(depth=1, reliability='RELIABLE')
                    }
                ]
            )
        # Publishes pointing_wrench (heading-hold torque) to the virtual spacecraft.
        ld.add_action(ep_pointing_planner)

        ###################################
        ##### - Encapsulation Phase - #####

        ep_motion_planner = Node(
            package="swarm_controller",
            executable="position_guidance3",
            name="motion_planner",
            namespace=robot_name,
            output="screen",
            emulate_tty=True,
            parameters=[
                    spacecraft_params,
                    {
                        "timer_frequency": timer_frequency,
                        "init_period": init_period,
                        **qos_parameters(depth=1, reliability='RELIABLE')
                    }
                ]
            )
        # Sole publisher of avoidance_wrench.
        ld.add_action(ep_motion_planner)


    ##############################################################################
    ########################### - Capture Phase - ################################
    for i in range(N):
        robot_name = robot_names[i]

        cp_control = Node(
            package="swarm_controller",
            executable="cp_proportional_controller",
            name="cp_controller",
            namespace=robot_name,
            output="screen",
            emulate_tty=True,
            parameters=[
                    spacecraft_params,
                    {
                        "timer_frequency": timer_frequency,
                        "init_period": init_period,
                        "approach_gain": 0.9,
                        "align_gain": 0.05,
                        "heading_gain": 1.0,
                        "target_velocity_span": 5.25,
                        "feedforward_gain": 0.0,
                        "contact_distance": CONTACT_DISTANCE,
                        "direct_cmd_vel": DIRECT_CMD_VEL,
                        **qos_parameters(depth=1, reliability='RELIABLE')
                    }
                ]
            )

        ld.add_action(cp_control)

    ##############################################################################
    ################## - AP auction: distinct attachment points - ################
    for i in range(N):
        robot_name = robot_names[i]
        ld.add_action(Node(
            package="swarm_controller",
            executable="ap_auction",
            name="ap_auction",
            namespace=robot_name,
            output="screen",
            emulate_tty=True,
            parameters=[
                spacecraft_params,
                {
                    "robot_id": beacon_addresses[i],
                    "bid_rate": 5.0,
                    "approach_distance": 0.8,
                    "w_proximity": 1.0,
                    "w_velocity_align": 0.3,
                    "w_normal_align": 0.3,
                    "neighbor_bid_timeout": 0.5,
                    "target_hold_timeout": 0.4,
                    "lock_hysteresis": 0.1,
                    "lock_rounds": 3,
                    "allow_reassignment_on_dropout": True,
                    **qos_parameters(depth=1, reliability='RELIABLE')
                }
            ]
        ))

    ##############################################################################
    ####### - Mode supervisor: SEARCH -> ENCAPSULATION -> CAPTURE - ##############
    for i in range(N):
        robot_name = robot_names[i]

        mode_supervisor = Node(
            package="swarm_controller",
            executable="mode_supervisor",
            name="mode_supervisor",
            namespace=robot_name,
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    "detect_frames": 3,
                    "lost_timeout": 4.0,
                    "miss_tolerance": 3,
                    "latch_capture": False,
                    # Distributed synchronized capture: detection drives SEARCH<->ENCAP
                    # and stages a stable target, but CAPTURE promotion is held until
                    # readiness_consensus clears capture_sync, so the whole swarm flips
                    # together. latch_capture=False keeps per-robot fallback on AP loss.
                    "auto_capture": True,
                    "require_capture_sync": True,
                    "initial_mode": "search",
                    "qos_reliability": "BEST_EFFORT",
                }
            ],
        )

        ld.add_action(mode_supervisor)


    ##############################################################################
    ############################ - Magnet node - #################################
    for i in range(N):
        robot_name = robot_names[i]

        magnet_node = Node(
            package="swarm_magnet",
            executable="gpio_control_node",
            name="magnet_node",
            namespace=robot_name,
            output="screen",
            emulate_tty=True,
            parameters=[
                {"gpio_line": gpio_line}
            ],
        )

        # ld.add_action(magnet_node)


    ##############################################################################
    ############ - Readiness consensus: synchronized capture trigger - ###########
    for i in range(N):
        robot_name = robot_names[i]
        ld.add_action(Node(
            package="swarm_controller",
            executable="readiness_consensus",
            name="readiness_consensus",
            namespace=robot_name,
            output="screen",
            emulate_tty=True,
            parameters=[
                spacecraft_params,
                {
                    "robot_id": beacon_addresses[i],
                    "consensus_rate": 5.0,
                    "approach_distance": 0.8,
                    "perception_stale_timeout": 0.5,
                    "action_lead": 2.0,
                    "min_lead": 0.5,
                    "time_tol": 0.05,
                    "neighbor_timeout": 0.5,
                    "min_consensus_hold": 0.3,
                    **qos_parameters(depth=1, reliability='BEST_EFFORT')
                }
            ]
        ))


    ##############################################################################
    ###################### - Virtual Spacecraft Launch - #########################
    ##############################################################################

    enable_virtual_spacecraft = os.environ.get(
        "VIRTUAL_SPACECRAFT", "false"
    ).lower() in ("true", "1", "yes", "on")

    if enable_virtual_spacecraft:
        # Hand the include the robot set we already resolved (ROBOT_NAME wins over
        # ROBOT_IDX in its env lookup) so both halves agree instead of re-reading.
        os.environ["ROBOT_NAME"] = ",".join(robot_names)
        virtual_spacecraft_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("swarm_bringup"),
                    "launch",
                    "launch_virtual_spacecraft.py"
                )
            ),
            launch_arguments={"enable_guidance_visualization": "true"}.items(),
        )

        ld.add_action(virtual_spacecraft_launch)





    return ld
