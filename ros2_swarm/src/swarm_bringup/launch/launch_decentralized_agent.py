from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import datetime
from launch.actions import TimerAction
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

import os
from launch.substitutions import LaunchConfiguration

gpio_line = int(os.environ.get("GPIO_LINE", "4"))

def _split_env(name, cast=str):
    val = os.getenv(name)
    if val is None:
        raise RuntimeError(f"Missing required environment variable: {name}")
    return [cast(x.strip()) for x in val.split(",") if x.strip()]

def load_swarm_config():
    """
    Load swarm configuration from resolved environment variables.
    Supports 1 or more robots via comma-separated ENV vars.
    """

    robot_names = _split_env("ROBOT_NAME", str)
    beacon_addresses = _split_env("BEACON_ADDR", int)
    robot_serial_numbers = _split_env("ROBOT_SERIAL", str)
    init_orientations = _split_env("INITIAL_ORIENT", float)

    if not robot_names:
        raise RuntimeError("No robots resolved from environment variables")

    n = len(robot_names)

    # Optional. Pinning the IP skips SN broadcast discovery (UDP 40927), which
    # is dropped on networks that filter broadcast between clients.
    robot_ips = [x.strip() for x in os.getenv("ROBOT_IP", "").split(",")]
    robot_ips = (robot_ips + [""] * n)[:n]

    if not (
        len(beacon_addresses) ==
        len(robot_serial_numbers) ==
        len(init_orientations) ==
        n
    ):
        raise ValueError("ENV var lists must all have the same length")

    return (
        beacon_addresses,
        robot_names,
        robot_serial_numbers,
        robot_ips,
        init_orientations,
    )


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

def generate_launch_description():
    init_period = 2.0 # in seconds
    alpha = 0.9 # Filter coefficient (1.0 = no filter, 0.0 = freeze first value)

    # Choose timer period (or timer frequency) for performance testing (higher period = lower CPU usage)
    timer_frequency = 0.01 # in seconds
    # timer_frequency = 0.1 # in seconds
    # timer_frequency = 1.0 # in seconds

    beacon_addresses, robot_names, robot_serial_numbers, robot_ips, init_orientations = load_swarm_config()
    N = len(robot_names)

    ld = LaunchDescription()

    # 'off' (default), 'on', or 'verbose'. See rm_preflight.py and
    # robomaster_ros/diagnostics.py.
    diagnostics = os.environ.get('RM_DIAGNOSTICS', 'off').lower()
    preflight_action = None
    if diagnostics != 'off':
        preflight_action = rm_preflight(robot_ips, verbose=(diagnostics == 'verbose'))
        ld.add_action(preflight_action)

    ##############################################################################
    ##############################################################################
    plot_juggler = Node(
        package="plotjuggler",
        executable="plotjuggler",
        name="plot_juggler"
    )

    # ld.add_action(plot_juggler)

    ##############################################################################
    ##############################################################################

    

    for i in range(N):
        robot_name = robot_names[i]
        serial_number = robot_serial_numbers[i]

        name_arg = DeclareLaunchArgument(f'name_{i}', default_value=robot_name)
        serial_arg = DeclareLaunchArgument(f'serial_number_{i}', default_value=serial_number)
        robot_ip_arg = DeclareLaunchArgument(f'robot_ip_{i}', default_value=robot_ips[i])
        enable_led_arg = DeclareLaunchArgument(f'enable_led_{i}', default_value='true')
        enable_speaker_arg = DeclareLaunchArgument(f'enable_speaker_{i}', default_value='true')
        enable_chassis_arg = DeclareLaunchArgument(f'enable_chassis_{i}', default_value='true')
        twist_to_wheel_speeds_arg = DeclareLaunchArgument(f'twist2wheel_{i}', default_value='true')

        path = os.path.join(
            get_package_share_directory('robomaster_ros'),
            'launch',
            's1.launch'
        )

        s1 = IncludeLaunchDescription(
            XMLLaunchDescriptionSource(path),
            launch_arguments={
                'name': LaunchConfiguration(f'name_{i}'),
                'serial_number': LaunchConfiguration(f'serial_number_{i}'),
                'robot_ip': LaunchConfiguration(f'robot_ip_{i}'),
                'diagnostics': diagnostics,
                'leds.enabled': LaunchConfiguration(f'enable_led_{i}'),
                'speaker.enabled': LaunchConfiguration(f'enable_speaker_{i}'),
                'chassis.enabled': LaunchConfiguration(f'enable_chassis_{i}'),
                'chassis.twist_to_wheel_speeds': LaunchConfiguration(f'twist2wheel_{i}')
            }.items()
        )

        ld.add_action(name_arg)
        ld.add_action(serial_arg)
        ld.add_action(robot_ip_arg)
        ld.add_action(enable_led_arg)
        ld.add_action(enable_speaker_arg)
        ld.add_action(enable_chassis_arg)
        ld.add_action(twist_to_wheel_speeds_arg)

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
    # remappings=[
    #             ('/mm_all_pos', 'mm_all_pos'),  # Remap global to relative path
    #             ('/mm_all_imu', 'mm_all_imu')
    #         ],
    namespace=robot_name,
    output="screen",
    emulate_tty=True,
    parameters=[
            {
                "robot_names": robot_names,
                "beacon_addresses": beacon_addresses,
                **qos_parameters(depth=1, reliability='BEST_EFFORT')
            }
        ]
    )

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
        output="screen",
        emulate_tty=True,
        # remappings=[
        #         ('/mm_all_pos', 'mm_all_pos'),  # Remap global to relative path
        #         ('/mm_all_imu', 'mm_all_imu')
        #     ],
        namespace=robot_name,
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

        ld.add_action(hedge)

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

        ld.add_action(mm_filtered)

    ##############################################################################
    ##############################################################################
    for i in range(N):
        robot_name = robot_names[i]
        init_orientation = init_orientations[i]

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
                    **qos_parameters(depth=1, reliability='BEST_EFFORT'),
                    "global_frame_id": "swarm_map",
                    "gps_timeout": 0.25,
                    "odometry_timeout": 0.25,
                    "gps_position_variance": 0.01,
                }
            ]
        )

        ld.add_action(pose)

    ##############################################################################
    ##############################################################################
    for i in range(N):
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
                    "aruco_type": "DICT_6X6_250",
                    "aruco_size": 3.556,
                    "aruco_scale": (1.5, 0.8),
                    "calibrate_camera": 0,
                    **qos_parameters(depth=1, reliability='BEST_EFFORT')
                }
            ]
        )

        ld.add_action(landmarks)
    
    ##############################################################################
    ##############################################################################
    for i in range(N):
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
                }
            ]
        )

        ld.add_action(aruco_landmark_filter)



    ##############################################################################
    ##############################################################################
    for i in range(N):
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
                    "aruco_type": "DICT_5X5_100",
                    "aruco_size": 3.556,
                    "aruco_scale": (1.5, 0.8),
                    "calibrate_camera": 0,
                    **qos_parameters(depth=1, reliability='BEST_EFFORT')
                }
            ]
        )

        ld.add_action(targets)

    ##############################################################################
    ##############################################################################
    for i in range(N):
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
                    "alpha": alpha,  # Filter coefficient
                }
            ]
        )

        ld.add_action(targets_filter)


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
                    {
                        "timer_frequency": timer_frequency,
                        "init_period": init_period,
                        **qos_parameters(depth=1, reliability='RELIABLE')
                    }
                ]
            )
        # ld.add_action(ep_pointing_planner)

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
                    {
                        "timer_frequency": timer_frequency,
                        "init_period": init_period,
                        **qos_parameters(depth=1, reliability='RELIABLE')
                    }
                ]
            )
        # ld.add_action(ep_motion_planner)


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
                    {
                        "timer_frequency": timer_frequency,
                        "init_period": init_period,
                        **qos_parameters(depth=1, reliability='RELIABLE')
                    }
                ]
            )

        # ld.add_action(cp_control)


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



    # ##############################################################################
    # ##################### - Consensus on capture time - ##########################
    # for i in range(N):
    #     robot_name = robot_names[i]
    #     init_pos = initial_positions[i]

    #     consensus_node = Node(
    #         package="swarm_magnet",
    #         executable="linear_consensus_node",
    #         name="linear_consensus_node",
    #         namespace=robot_name,
    #         output="screen",
    #         emulate_tty=True,
    #         parameters=[
    #             {
    #                 "robot_name": robot_name,
    #                 "initial_position": init_pos,
    #                 "timer_frequency": timer_frequency,
    #                 "consensus_rate": consensus_rate,
    #                 **qos_parameters(depth=1, reliability='BEST_EFFORT')
    #             }
    #         ]
    #     )

    #     ld.add_action(consensus_node)



    ##############################################################################
    ###################### - Virtual Spacecraft Launch - #########################
    ##############################################################################

    enable_virtual_spacecraft = os.environ.get(
        "VIRTUAL_SPACECRAFT", "false"
    ).lower() in ("true", "1", "yes", "on")

    if enable_virtual_spacecraft:
        virtual_spacecraft_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("swarm_bringup"),
                    "launch",
                    "launch_virtual_spacecraft.py"
                )
            )
        )

        ld.add_action(virtual_spacecraft_launch)




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

    # Give both launch files / robot drivers a little time to appear
    # before starting rosbag discovery.
    ld.add_action(
        TimerAction(
            period=3.0,
            actions=[bag_record],
        )
    )

    return ld