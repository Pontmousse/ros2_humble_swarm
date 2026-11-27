from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os

gpio_line = int(os.environ.get("GPIO_LINE", "4"))
# BCM GPIO lines of the contact limit switches (comma-separated); any pressed = contact.
contact_gpio_lines = [int(x) for x in os.environ.get("CONTACT_GPIO", "5").split(",") if x.strip()]
# Chassis, Marvelmind and USB camera + ArUco run in the drivers container (swarm_rover_v2 image); this
# stack reads its gnss/position and aruco/{landmarks,markers} in the perception node.
# ArUco output scale / max range (m) per marker set (aruco_convert in perception).
LANDMARK_SCALE, LANDMARK_RANGE = (1.0, 1.0), 2.5
TARGET_SCALE, TARGET_RANGE = (1.15, 0.8), 4.0
# GUIDANCE_VIZ=false (Pi, headless batch): no RViz markers; the dev PC draws them (launch_dev_viz.py).
guidance_viz = os.environ.get("GUIDANCE_VIZ", "true").lower() in ("1", "true", "yes", "on")

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
    init_orientations = _split_env("INITIAL_ORIENT", float)

    if not robot_names:
        raise RuntimeError("No robots resolved from environment variables")

    n = len(robot_names)

    if not len(beacon_addresses) == len(init_orientations) == n:
        raise ValueError("ENV var lists must all have the same length")

    return beacon_addresses, robot_names, init_orientations


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
        'qos_depth': depth,  # honoured (was forced to 10): latest-only stops a starved node chewing a stale backlog
        'qos_reliability': 'RELIABLE',
        'qos_history': 'KEEP_LAST'
    }
    
    return qos_params

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

    On a decentralized run this executes on each Pi, so it captures that Pi's own
    env flags, its own on-disk config bytes and (best-effort) its own git HEAD --
    exactly the per-agent install drift worth recording. Published latched under
    the robot namespace (/<ns>/run_config); the rosbag launch records it. See
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
    ] + [  # launch files carry the inline node params (auction, consensus, capture, avoidance)
        os.path.join(get_package_share_directory('swarm_bringup'), 'launch', f)
        for f in (os.path.basename(__file__), 'launch_virtual_spacecraft.py', 'launch_virtual_target.py')]
    env_vars = [
        'VIRTUAL_TARGET', 'VIRTUAL_PLANT', 'VIRTUAL_SPACECRAFT', 'DIRECT_CMD_VEL', 'SCENE_FILE', 'ROBOT_IDX',
        'ROBOT_NAME', 'BAG_TAG', 'CONTACT_GPIO', 'SWARM_BROADCAST_ADDR', 'SWARM_COMMS_PORT', 'GUIDANCE_VIZ',
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
                'publish_period': 0.0,  # publish once: latched (TRANSIENT_LOCAL), ~100 KB
                'repo_path': _find_repo_root(__file__),
                'source_launch': source_launch,
                'robot_names': list(robot_names),
            }
        ],
    )

def generate_launch_description():
    init_period = 2.0 # in seconds
    # Choose timer period (or timer frequency) for performance testing (higher period = lower CPU usage)
    timer_frequency = 0.05 # 20 Hz - drives cp_controller
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

    beacon_addresses, robot_names, init_orientations = load_swarm_config()
    N = len(robot_names)

    ld = LaunchDescription()

    # Latched JSON snapshot of this agent's config files + env flags, one per
    # robot namespace (/<ns>/run_config), so each Pi's bag is self-describing and
    # per-agent install drift is captured. See run_config_snapshot_node.
    decentralized_env = ('GPIO_LINE', 'SWARM_BROADCAST_ADDR', 'SWARM_BROADCAST_IFACE')
    for robot_name in robot_names:
        ld.add_action(run_config_snapshot_node(
            robot_names, source_launch='decentralized',
            namespace=robot_name, extra_env_vars=decentralized_env))

    # When set, cp_controller becomes the sole writer of cmd_vel and muxes the
    # nav chain through it (velocity_smoother is remapped to cmd_vel_nav inside
    # launch_virtual_spacecraft.py, which reads the same env). Must match the
    # reads in launch_centralized_swarm.py and launch_virtual_spacecraft.py.
    direct_cmd_vel = os.environ.get("DIRECT_CMD_VEL", "").lower() in (
        "1", "true", "yes", "on"
    )

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

    

    ##############################################################################
    ########################## - UDP state broadcast - ###########################
    # Directed broadcast of the WiFi subnet, not 255.255.255.255: with RNDIS up
    # the route table can pick usb0, where only the RoboMaster body listens.
    comms_broadcast_address = os.getenv("SWARM_BROADCAST_ADDR", "192.168.50.255")
    comms_bind_device = os.getenv("SWARM_BROADCAST_IFACE", "")
    comms_port = int(os.getenv("SWARM_COMMS_PORT", "9870"))  # SIL batch uses its own port so lab robots stay unheard

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
                        # Beacon address doubles as the swarm-wide robot id so the
                        # two identity schemes cannot drift apart.
                        "robot_id": beacon_addresses[i],
                        "broadcast_address": comms_broadcast_address,
                        "bind_device": comms_bind_device,
                        "port": comms_port,
                        "broadcast_rate": 10.0,
                        "receive_rate": 10.0,  # = broadcast_rate = avoidance rate; drains all queued packets per tick
                        "stale_timeout": 0.5,
                        "drop_timeout": 5.0,
                    }
                ]
            )

        ld.add_action(comms)

    # mm_filtered (mm_pos_unf -> mm_pos) removed: only the legacy kalman_filter /
    # pose_publisher read mm_pos, and pose_kf/pose_publisher use mm_pos_unf.

    ##############################################################################
    ############## - perception: localization KF + marker memories - ############
    # One process per robot (nodes-to-functions Phase 2): the KF publishes pose / localization/odom
    # at 20 Hz and, in the same tick, projects the landmark/target memories through that pose, so the
    # controllers get a steady marker cloud through camera stalls, never through a stale pose. Inputs
    # come straight from the drivers container: gnss/position, aruco/landmarks, aruco/markers.
    for i in range(N):
        ld.add_action(Node(
            package="swarm_target",
            executable="perception",
            name="perception",
            namespace=robot_names[i],
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    "init_period": init_period,
                    "timer_frequency": 0.05,
                    "init_orientation": init_orientations[i],
                    "global_frame_id": "swarm_map",
                    "odometry_timeout": 1.0,  # wheel odom silent this long = dead driver: output withheld
                    "gps_position_variance": 0.01,
                    "command_tau": 0.25,
                    "q_pos": 1.0e-4,
                    "q_vel": 0.5,
                    "q_theta": 1.0e-4,
                    "q_omega": 0.5,
                    "r_odom_vel": 0.04,
                    "r_yaw": 0.01,
                    "r_gyro": 0.01,
                    "innovation_gate_chi2": 9.0,
                    "require_mm_for_init": True,
                    **qos_parameters(depth=1, reliability='BEST_EFFORT'),
                    "position_input": "v2",
                    "marker_input": "v2",
                    "beacon": beacon_addresses[i],
                    "debug_topics": True,     # republish mm_pos_unf / landmarks_unf for bags (targets_unf always)
                    "lift_at_stamp": False,   # lift at capture stamp: off until a HIL A/B
                    "memory_viz_rate": 2.0,   # landmark/target_memory/markers for the dev PC (launch_dev_viz.py)
                    "landmarks": {"min_observations": 2, "forget_timeout": 0.0,
                                  "scale": list(LANDMARK_SCALE), "max_range": LANDMARK_RANGE},
                    "targets": {"min_observations": 2, "forget_timeout": 0.0,
                                "scale": list(TARGET_SCALE), "max_range": TARGET_RANGE},
                }
            ]
        ))

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
        # ENABLED: sole publisher of avoidance_wrench, which virtual_spacecraft
        # has been subscribing to all along with nothing on the other end.
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
                        # Approach gain, s^-1, bracketed three ways: the
                        # zero-order-hold budget on the Marvelmind period
                        # (< 3.3), the body-x cap over the 52 mm noise floor
                        # (< 0.96, else noise alone saturates the command),
                        # and tracking lag v_target / gain (> 0.58).
                        "approach_gain": 0.5,
                        # Tangential alignment, (m/s)/rad. 0.0 steers by
                        # rotation alone.
                        "align_gain": 0.12,
                        "align_damping_gain": 0.4,
                        "preparation_max_force": 0.01,
                        # Heading gain, s^-1. Capped at 2.3 by the rotation
                        # cap over the bearing noise -- 52 mm of position
                        # noise subtends 0.087 rad at buffer_radius = 0.60 --
                        # well under the zero-order-hold budget of 3.3.
                        "heading_gain": 1.0,
                        # Target-velocity feedforward. The span is a noise
                        # budget: a two-point difference on 52 mm fixes at
                        # 0.35 s carries 0.21 m/s against a 0.03 m/s signal,
                        # while a slope over 5.25 s (15 independent fixes)
                        # gives 9 mm/s -- at the cost of lagging a change in
                        # target velocity by ~2.6 s. SECONDS, not samples:
                        # target_callback runs at the 10 Hz markers rate, so
                        # a sample count would silently shrink the span.
                        # Set feedforward_gain to 0.0 to A/B against the pure
                        # proportional law.
                        "target_velocity_span": 5.25,
                        "feedforward_gain": 1.0,
                        # Contact equilibrium, metres from the robot centre --
                        # half the 0.40 m RoboMaster length, matching the
                        # contact_distance used by geometric_contact.py's
                        # gate on the centralized/virtual side.
                        "contact_distance": 0.20,
                        # Near-target speed envelope: inside capture_slow_radius
                        # the summed approach command tapers to
                        # capture_min_speed_frac at contact so the weak thruster
                        # can track the final corridor run-in.
                        "capture_slow_radius": 0.5,
                        "capture_min_speed_frac": 0.3,
                        # True bypasses virtual_spacecraft/nav2_pidff and
                        # publishes straight to cmd_vel for real-hardware
                        # bench testing; do not also run velocity_smoother on
                        # cmd_vel at the same time.
                        "direct_cmd_vel": direct_cmd_vel,
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
                    "w_proximity": 0.3,
                    "w_velocity_align": 0.3,
                    "velocity_min_speed": 0.05,
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
                    # Consecutive detected frames before advancing a mode.
                    "detect_frames": 3,
                    # Seconds a signal may be lost before a backward transition
                    # (CAPTURE->ENCAP/SEARCH, ENCAP->SEARCH). Raised 2.0->4.0 to
                    # ride through brief detection dropouts (low-fps flicker) and
                    # stop CAPTURE<->SEARCH oscillation.
                    "lost_timeout": 4.0,  # inert while both estimators keep memory (forget_timeout 0): landmarks/targets never go empty
                    # Empty frames tolerated before the ENTRY streak resets, so a
                    # steadily-held marker that flickers still advances the mode.
                    "miss_tolerance": 3,
                    # False so losing the target falls back after lost_timeout.
                    # Required to validate following a MOVING target: latched,
                    # the robot would stay in CAPTURE driving on a stale marker
                    # instead of re-acquiring.
                    "latch_capture": False,
                    # Distributed synchronized capture: detection drives SEARCH<->ENCAP
                    # and stages a stable target, but CAPTURE promotion is held until
                    # readiness_consensus clears capture_sync, so the whole swarm flips
                    # together. latch_capture=False keeps per-robot fallback on AP loss.
                    "auto_capture": True,
                    "require_capture_sync": True,
                    # Mode assumed at startup; launch brings up search-enabled /
                    # capture-disabled, so SEARCH matches reality with no forced
                    # service call on boot.
                    "initial_mode": "search",
                    # Match the BEST_EFFORT publisher on the detection topics.
                    "qos_reliability": "BEST_EFFORT",
                    # Cut the wheel motors on DOCK (real hardware; no VIRTUAL_PLANT here).
                    "disengage_wheels_on_dock": True,
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
    ###################### - Contact limit switch node - #########################
    # Publishes contact_detected from the GPIO limit switches (pressed = LOW,
    # internal pull-up). No VIRTUAL_PLANT here, so always on (see centralized).
    for i in range(N):
        ld.add_action(Node(
            package="swarm_magnet",
            executable="contact_switch_node",
            name="contact_switch",
            namespace=robot_names[i],
            output="screen",
            emulate_tty=True,
            parameters=[{"gpio_lines": contact_gpio_lines}],
        ))



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
                    "perception_stale_timeout": 2.0,
                    "action_lead": 5.0,
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
        virtual_spacecraft_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("swarm_bringup"),
                    "launch",
                    "launch_virtual_spacecraft.py"
                )
            ),
        launch_arguments={"enable_guidance_visualization": os.getenv("GUIDANCE_VIZ", "true")}.items(),  # GUIDANCE_VIZ=false: headless batch
        )

        ld.add_action(virtual_spacecraft_launch)




    return ld