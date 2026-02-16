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
    # although it will be converted to radians for kalman filtering but later reconverted back to degrees for publishing/vizualizing
    init_orientations = [0.0, 0.0, 0.0, 0.0] 



    # Selection indices (1-based indexing)
    # e.g., selecting RM1 and RM2
    # Here order doesn't matter, selecting [1, 2] or [2, 1] is exactly the same
    # idx = [3]  


    ###############################################################################
    # Set from environment variable (e.g., ROBOT_IDX=1,2 for RM1 and RM2)
    env_idx = os.getenv('ROBOT_IDX')

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


def generate_launch_description():
    gpio_line = int(os.environ.get("GPIO_LINE", "4"))

    beacon_addresses, robot_names, robot_serial_numbers, init_orientations = load_swarm_config()
    N = len(robot_names)

    ld = LaunchDescription()

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

        ld.add_action(magnet_node)

    return ld