from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

import os
from launch.substitutions import LaunchConfiguration

def load_swarm_config():

    # Original configuration (order matters!)
    beacon_addresses = [1, 2, 3, 4]

    robot_names = ["RM1", "RM2", "RM3", "RM4"]

    robot_serial_numbers = [
        "159CKC50070ECX",
        "159CKC50070E5N",
        "159CG9J0050797",
        "159CG9V0050HED"
    ]

    # initial angles for orientation propagation. should be in degree,
    # although it will be converted to radians
    init_orientations = [90.0, 0.0, 90.0, 0.0] 


    # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    # Select ONLY one here for this launch file
    idx = [1]

    ###############################################################################
    # Set from environment variable (e.g., ROBOT_IDX=1,2 for RM1 and RM2)
    env_idx = os.getenv('ROBOT_IDX')

    if env_idx is None:
        raise RuntimeError("ROBOT_IDX environment variable must be set.")

    idx = [int(env_idx)]

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
    init_orientations = [init_orientations[i] for i in idx_zero_based]

    return beacon_addresses, robot_names, robot_serial_numbers, init_orientations


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

def generate_launch_description():
    init_period = 2.0 # in seconds
    alpha = 1.0 # Filter coefficient (1.0 = no filter, 0.0 = freeze first value)

    # Choose timer period (or timer frequency) for performance testing (higher period = lower CPU usage)
    timer_frequency = 0.01 # in seconds
    # timer_frequency = 0.1 # in seconds
    # timer_frequency = 1.0 # in seconds

    beacon_addresses, robot_names, robot_serial_numbers, init_orientations = load_swarm_config()
    N = len(robot_names)

    ld = LaunchDescription()

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
                'leds.enabled': LaunchConfiguration(f'enable_led_{i}'),
                'speaker.enabled': LaunchConfiguration(f'enable_speaker_{i}'),
                'chassis.enabled': LaunchConfiguration(f'enable_chassis_{i}'),
                'chassis.twist_to_wheel_speeds': LaunchConfiguration(f'twist2wheel_{i}')
            }.items()
        )

        ld.add_action(name_arg)
        ld.add_action(serial_arg)
        ld.add_action(enable_led_arg)
        ld.add_action(enable_speaker_arg)
        ld.add_action(enable_chassis_arg)
        ld.add_action(twist_to_wheel_speeds_arg)

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
                    **qos_parameters(depth=1, reliability='BEST_EFFORT')
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
                    "aruco_size": 2.5,
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
                    "aruco_type": "DICT_6X6_250",
                    "aruco_size": 4.3,
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
                    {
                        "timer_frequency": timer_frequency,
                        "init_period": init_period,
                        **qos_parameters(depth=1, reliability='RELIABLE')
                    }
                ]
            )
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
                    {
                        "timer_frequency": timer_frequency,
                        "init_period": init_period,
                        **qos_parameters(depth=1, reliability='RELIABLE')
                    }
                ]
            )

        ld.add_action(cp_control)



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

    return ld