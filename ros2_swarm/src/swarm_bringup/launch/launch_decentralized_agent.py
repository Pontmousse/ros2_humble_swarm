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

# KF_LOCALIZATION=1 swaps the localization fuser for pose_kf (two decoupled
# linear Kalman filters). Same output contract as pose_publisher, so downstream
# consumers are unchanged. Default off. See pose_kf.py.
kf_on = os.environ.get("KF_LOCALIZATION", "").lower() in ("1", "true", "yes", "on")

# TARGET_ESTIMATOR=1 inserts the landmark_estimator between the ArUco detectors
# and the controllers in place of aruco_filter: a map-frame memory keyed by ArUco
# ID, republished as a steady body-frame cloud through the live pose, so the
# controller reference persists when markers leave the FOV (killing the
# perception-starvation yaw limit cycle). Same landmarks/targets output contract
# and QoS as aruco_filter, so controllers are untouched. Default off.
estimator_on = os.environ.get("TARGET_ESTIMATOR", "").lower() in ("1", "true", "yes", "on")

def _split_env(name, cast=str):
    val = os.getenv(name)
    if val is None:
        raise RuntimeError(f"Missing required environment variable: {name}")
    return [cast(x.strip()) for x in val.split(",") if x.strip()]

def _detect_marvelmind_port(fallback):
    """Return the /dev/ttyACM* whose USB manufacturer is Marvelmind.

    RNDIS adds a DJI CDC-ACM device that can steal /dev/ttyACM0, shifting the
    Marvelmind modem to ttyACM1 on some boots. A fixed port number then silently
    opens the DJI serial device. We walk up the /sys device tree from each ACM
    node (the 'manufacturer' attribute lives on the USB device, a few parents
    above the tty) and match 'marvelmind' case-insensitively. /dev/serial/by-id
    is absent in this privileged container, so /sys is the only identity source.
    Falls back to the given port (MARVELMIND_PORT / ttyACM1) if nothing matches.
    """
    import glob
    for dev in sorted(glob.glob("/dev/ttyACM*")):
        name = os.path.basename(dev)
        up = os.path.realpath(f"/sys/class/tty/{name}/device")
        for _ in range(6):
            up = os.path.dirname(up)
            man_path = os.path.join(up, "manufacturer")
            if os.path.isfile(man_path):
                try:
                    if "marvelmind" in open(man_path).read().strip().lower():
                        return dev
                except OSError:
                    pass
                break
    return fallback

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

    # Resolve the Marvelmind modem's serial port by USB identity, not by a fixed
    # /dev/ttyACMn number. RNDIS matters here: when the RoboMaster is on USB
    # (RNDIS) instead of WiFi it enumerates a DJI CDC-ACM device that can grab
    # /dev/ttyACM0 and push the Marvelmind modem to ttyACM1 (or vice versa on a
    # different boot). A fixed number then silently opens the DJI port, no
    # Marvelmind fixes arrive, and virtual_spacecraft reports "No fresh
    # localization odometry". Scan the ACM ports and pick the one whose USB
    # manufacturer is Marvelmind; fall back to MARVELMIND_PORT / ttyACM1 only if
    # detection fails. (/dev/serial/by-id is not present inside this privileged
    # container, so we read the USB descriptor from /sys instead.)
    marvelmind_port = _detect_marvelmind_port(
        os.environ.get("MARVELMIND_PORT", "/dev/ttyACM1"))

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
        marvelmind_port,
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
    timer_frequency = 0.05 # 20 Hz - drives cp_controller + hedgehog_obs
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

    beacon_addresses, robot_names, robot_serial_numbers, robot_ips, init_orientations, marvelmind_port = load_swarm_config()
    N = len(robot_names)

    ld = LaunchDescription()

    # Latched JSON snapshot of this agent's config files + env flags, one per
    # robot namespace (/<ns>/run_config), so each Pi's bag is self-describing and
    # per-agent install drift is captured. See run_config_snapshot_node.
    decentralized_env = (
        'GPIO_LINE', 'MARVELMIND_PORT', 'SWARM_BROADCAST_ADDR',
        'SWARM_BROADCAST_IFACE', 'USB_CAM_DEVICE',
    )
    for robot_name in robot_names:
        ld.add_action(run_config_snapshot_node(
            robot_names, source_launch='decentralized',
            namespace=robot_name, extra_env_vars=decentralized_env))

    # 'off' (default), 'on', or 'verbose'. See rm_preflight.py and
    # robomaster_ros/diagnostics.py.
    diagnostics = os.environ.get('RM_DIAGNOSTICS', 'off').lower()
    # When set, cp_controller becomes the sole writer of cmd_vel and muxes the
    # nav chain through it (velocity_smoother is remapped to cmd_vel_nav inside
    # launch_virtual_spacecraft.py, which reads the same env). Must match the
    # reads in launch_centralized_swarm.py and launch_virtual_spacecraft.py.
    direct_cmd_vel = os.environ.get("DIRECT_CMD_VEL", "").lower() in (
        "1", "true", "yes", "on"
    )
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

        conn_type = os.getenv("CONN_TYPE", "rndis")
        # rndis reaches the robot over USB at a fixed 192.168.42.2; a pinned WiFi
        # robot_ip would override that and break the USB link, so drop it.
        robot_ip = "" if conn_type == "rndis" else robot_ips[i]

        name_arg = DeclareLaunchArgument(f'name_{i}', default_value=robot_name)
        serial_arg = DeclareLaunchArgument(f'serial_number_{i}', default_value=serial_number)
        robot_ip_arg = DeclareLaunchArgument(f'robot_ip_{i}', default_value=robot_ip)
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
                'conn_type': conn_type,
                'robot_ip': LaunchConfiguration(f'robot_ip_{i}'),
                'diagnostics': diagnostics,
                'leds.enabled': LaunchConfiguration(f'enable_led_{i}'),
                'speaker.enabled': LaunchConfiguration(f'enable_speaker_{i}'),
                'chassis.enabled': LaunchConfiguration(f'enable_chassis_{i}'),
                'chassis.twist_to_wheel_speeds': LaunchConfiguration(f'twist2wheel_{i}'),
                # CAMERA MODULE UNLOADED. Not "video off" -- the whole module,
                # via main.launch's `camera` arg (main.launch:106 -> 172
                # camera.enabled). The per-stream switches below cannot achieve
                # this on their own: video_ffmpeg defaults to $(var video) == 2,
                # so leaving it unset silently turns the ffmpeg stream ON. RM6
                # was found running exactly that way -- h264 -1, raw -1,
                # ffmpeg 2 -- which is why this is a module-level switch.
                #
                # Why it is off: with h264 ON_DEMAND, a PC-side subscriber
                # starts the stream on every robot at once (camera.py:303-305,
                # a matched-publisher event, so immediate). The publish path is
                # unthrottled -- camera.py:494-501 sends every chunk at ~55 Hz,
                # and video_rate is only honoured in the raw branch at :510, so
                # it does nothing for h264. The Pi then starves, localization
                # gaps exceed virtual_spacecraft's 0.25 s odometry_timeout, the
                # velocity_smoother bond (both ends on the same Pi) misses its
                # 4 s deadline, lifecycle manager cleans it up, cmd_vel stops.
                # Observed 2026-10-09: both robots stopped mid-run.
                #
                # To restore, uncomment the three below and drop this arg.
                # Note the raw/h264 asymmetry is deliberate: the SDK software
                # decoder is the only path yielding a usable image_color here,
                # because the Pi hardware VPU cannot decode the RoboMaster
                # multi-slice bitstream. And prefer ON_DEMAND (2) over ON (1) --
                # ON emits SPS/PPS and the first IDR at driver boot, so a
                # decoder joining later never gets a parameter set and every
                # packet fails avcodec_send_packet forever.
                'camera': 'false',
                # 'video_raw': '-1',
                # 'video_h264': '2',
                # 'video_rate': '6.0',
                # Onboard vision, which is a different path from video entirely:
                # robomaster_ros/modules/vision.py calls sub_detect_info(), and
                # neither it nor the SDK's vision.py mentions video or camera.
                # Detection runs on the robot's own vision processor and comes
                # back over the existing DDS-event channel as 9 bytes of header
                # plus 20 bytes per detection -- 29 bytes for one target. No
                # H.264 chunk reaches the Pi, so none of the decode cost that
                # forced video_raw/video_h264 to -1 above applies. main.launch
                # derives vision.enabled from this list being non-empty, so ''
                # (the default) leaves the module unloaded exactly as before.
                #
                # 'robot' needs no marker on the target at all. UNVERIFIED on
                # hardware: the source proves the client never asks for video,
                # but not that the firmware is willing to run detection while no
                # video stream is subscribed. That is what the /RM2/vision probe
                # is for; if the topic stays empty, set this back to '' rather
                # than reaching for video.
                # Double quotes inside, single quotes outside, and that is not a
                # style choice. main.launch does
                #   <let name="vision" value="$(eval 'len($(var vision_targets)) > 0')"/>
                # so this string is pasted inside a single-quoted $(eval ...).
                # "['robot']" closes the eval's quote at the quote before robot
                # and the substitution parser aborts the whole launch -- every
                # node on the robot, not just vision. '["robot"]' is the same
                # Python list literal with no quote collision.
                # Empty: the robot's onboard detector does not work here, and
                # this was measured, not assumed. With the video stream up
                # (image_h264 at 55.6 Hz) and both 'robot' and 'person'
                # subscribed, /RM2/vision produced zero messages over repeated
                # 30 s windows with a person in frame. The SDK's four failure
                # paths in sub_detect_info all log and return False, and
                # robomaster_ros ignores that return value -- but the logs are
                # clean, so the firmware ACKs ProtoVisionDetectEnable and then
                # never pushes an event. Nothing left to read in the source.
                # Leaving a target here costs a pointless handshake per boot.
                # '[]', not '': this is pasted into $(eval 'len(...) > 0'), so
                # an empty string makes that len() take no argument at all.
                'vision_targets': '[]',
                # CHASSIS_LEAD (2): gimbal yaw follows the chassis heading so the
                # camera always points out the front of the robot. This is the
                # desired behaviour. The ~58 deg pitch bob seen earlier was a
                # symptom of the latched-CAPTURE runaway (the robot circling
                # violently drove the gimbal pitch stabilizer to the floor), not
                # inherent to CHASSIS_LEAD; that runaway is fixed upstream by
                # mode_supervisor.stop_chassis() zeroing cmd_vel around the
                # velocity_smoother PAUSE. FREE (0) was tried but world-fixes the
                # yaw, so the camera stops facing forward -> rejected.
                'gimbal_mode': '2'
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
                "port_address": marvelmind_port,
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
    ########################## - UDP state broadcast - ###########################
    # Directed broadcast of the WiFi subnet, not 255.255.255.255: with RNDIS up
    # the route table can pick usb0, where only the RoboMaster body listens.
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
                        # Beacon address doubles as the swarm-wide robot id so the
                        # two identity schemes cannot drift apart.
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
                    "timer_frequency": 0.05,
                    "init_orientation": init_orientation,
                    **qos_parameters(depth=1, reliability='BEST_EFFORT'),
                    "global_frame_id": "swarm_map",
                    "gps_timeout": 0.25,
                    "odometry_timeout": 0.25,
                    "gps_position_variance": 0.01,
                }
            ]
        )

        # Kalman-filter localization (KF_LOCALIZATION=1): same output contract
        # as pose_publisher, fed by Marvelmind + driver odom/imu.
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
                    "timer_frequency": 0.05,
                    "init_orientation": init_orientation,
                    "global_frame_id": "swarm_map",
                    "gps_timeout": 0.25,
                    "odometry_timeout": 0.25,
                    "gyro_timeout": 0.25,
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
                }
            ]
        )

        if kf_on:
            ld.add_action(pose_kf)
        else:
            ld.add_action(pose)

    ##############################################################################
    ##############################################################################
    ##############################################################################
    ######################### - USB (UVC) camera source - ########################
    for i in range(N):
        robot_name = robot_names[i]

        ext_camera = Node(
            package="swarm_aruco",
            executable="ext_camera",
            name="ext_camera",
            namespace=robot_name,
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    # Capture backend. Defaults to the Pi Camera Module 3 (csi);
                    # export CAMERA_SOURCE=usb to use the See3CAM instead. Normal
                    # runs leave it unset and get csi.
                    "source": os.getenv("CAMERA_SOURCE", "csi"),
                    "device": os.getenv("USB_CAM_DEVICE", "/dev/video0"),
                    # See3CAM_CU30. 640x480 is its lowest mode -- it has no
                    # 640x360, which is what the stored calibration assumes.
                    "width": 640,
                    "height": 480,
                    # UYVY, not MJPG: luma is a strided copy out of the packed
                    # 4:2:2 buffer (~1 ms/frame on the Pi 4) against ~25 ms for
                    # a JPEG decode. Measured 2026-10-19.
                    "fourcc": "UYVY",
                    "fps": 30,
                    # The detectors throttle to detect_max_hz anyway, so
                    # publishing faster only burns CPU and DDS bandwidth.
                    "publish_max_hz": 10.0,
                    **qos_parameters(depth=1, reliability='BEST_EFFORT')
                }
            ],
        )

        # Replaces the RoboMaster's own camera, which stays off ('camera':
        # 'false' above): the SDK video path cost ~82% of a core in the driver
        # and delivered 3-5 fps of frequently-corrupt frames over the double-hop
        # WiFi. Measured on the Pi 4 with robot2 running, the usb path gives 23.8
        # fps end to end at 35% of the 4-core machine with a 100% detection rate.
        # The backend (csi default, usb override) is chosen by CAMERA_SOURCE.
        ld.add_action(ext_camera)

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
                    "timer_frequency": 0.05,
                    "serial_number": serial_number,
                    "aruco_type": "DICT_4X4_100",
                    "aruco_size": 3.56,
                    "aruco_scale": (1.0, 1.0),
                    "calibrate_camera": 0,
                    "detect_max_hz": 15.0,  # see target_detector note
                    **qos_parameters(depth=1, reliability='BEST_EFFORT')
                }
            ]
        )

        ld.add_action(landmarks)


    ##############################################################################
    ##############################################################################
    for i in range(N):
        # See the note on targets_filter below: index the list, don't inherit
        # whatever `robot_name` the previous loop happened to leave behind.
        robot_name = robot_names[i]

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

        # TARGET_ESTIMATOR=1 replaces aruco_filter with landmark_estimator (added
        # in its own loop below), so the raw filter is gated off here.
        if not estimator_on:
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
                    "timer_frequency": 0.05,
                    "serial_number": serial_number,
                    "aruco_type": "DICT_5X5_50",
                    "aruco_size": 5.76,
                    "aruco_scale": (1.15, 0.8),
                    "calibrate_camera": 0,
                    # Cap ArUco detection at 5 Hz. solvePnP-per-frame was the #1
                    # CPU consumer (~116%) on the Pi; dropping frames before
                    # conversion/detection frees ~a core. publish_markers keeps
                    # republishing the last markers, so downstream rate is kept.
                    "detect_max_hz": 15.0,
                    **qos_parameters(depth=1, reliability='BEST_EFFORT')
                }
            ]
        )


        ld.add_action(targets)

    ##############################################################################
    ##############################################################################
    for i in range(N):
        # robot_names[i], not the bare `robot_name` leaking from the previous
        # loop -- that put every filter in the LAST robot's namespace. Invisible
        # while N == 1 (one container per robot), wrong the moment it is not.
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
    # TARGET_ESTIMATOR=1: landmark_estimator in place of both aruco_filter
    # instances. Generic single-stream node instantiated twice via remappings
    # (landmark + target); keeps a map-frame memory keyed by ArUco ID and
    # republishes the whole remembered cloud at publish_rate through the live pose,
    # so the controller reference persists through FOV loss. QoS mirrors the
    # aruco_filter it replaces, so controllers are untouched.
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
        # has been subscribing to all along with nothing on the other end. Its
        # other output, target_position, still has no consumer (ep_control below
        # stays disabled), so this adds the avoidance wrench and nothing else.
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
                        "approach_gain": 0.9,
                        # Tangential alignment, (m/s)/rad. 0.0 steers by
                        # rotation alone.
                        "align_gain": 0.05,
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
                        # contact_distance used by fake_plant.py's geometric
                        # gate on the centralized/virtual side.
                        "contact_distance": 0.20,
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
                    "lost_timeout": 4.0,
                    # Empty frames tolerated before the ENTRY streak resets, so a
                    # steadily-held marker that flickers still advances the mode.
                    "miss_tolerance": 3,
                    # False so losing the target falls back after lost_timeout.
                    # Required to validate following a MOVING target: latched,
                    # the robot would stay in CAPTURE driving on a stale marker
                    # instead of re-acquiring.
                    "latch_capture": False,
                    # Manual capture: detection only cycles SEARCH<->ENCAP;
                    # publish `/mode_cmd` "capture" to trigger all robots at once.
                    "auto_capture": False,
                    # Mode assumed at startup; launch brings up search-enabled /
                    # capture-disabled, so SEARCH matches reality with no forced
                    # service call on boot.
                    "initial_mode": "search",
                    # Match the BEST_EFFORT publisher on the detection topics.
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
            ),
        launch_arguments={"enable_guidance_visualization": "true"}.items(),
        )

        ld.add_action(virtual_spacecraft_launch)




    return ld