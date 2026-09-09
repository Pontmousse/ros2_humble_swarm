"""Publish physical localization using a Marvelmind initial anchor
and filtered RoboMaster odometry propagation.
"""

from collections import deque
from copy import deepcopy
import math
from statistics import median
from typing import Optional

from geometry_msgs.msg import TransformStamped
from marvelmind_ros2_msgs.msg import HedgePositionAddressed
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from swarm_interfaces.msg import StatePos as State
from tf2_ros import TransformBroadcaster
from transforms3d.euler import quat2euler


UNOBSERVED_VARIANCE = 1.0e6


# =============================================================================
# Utilities
# =============================================================================

def degrees(angle):
    """Convert radians to degrees wrapped to [-180, 180)."""
    angle = angle / np.pi * 180.0
    return (angle + 180.0) % 360.0 - 180.0


def wrap_angle(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def source_is_fresh(now_nanoseconds: int, update_nanoseconds: Optional[int], timeout: float) -> bool:
    """Return whether a source has supplied a recent genuine update."""
    if update_nanoseconds is None:
        return False
    age = (now_nanoseconds - update_nanoseconds) * 1.0e-9
    return 0.0 <= age <= timeout


def genuine_gps_update_time(
    timestamp_ms: int,
    previous_timestamp_ms: Optional[int],
    now_nanoseconds: int,
    previous_update_nanoseconds: Optional[int],
) -> Optional[int]:
    """Refresh receipt time only when the Marvelmind timestamp changes."""
    return now_nanoseconds if timestamp_ms != previous_timestamp_ms else previous_update_nanoseconds


def namespaced_base_frame(namespace: str, odometry_child_frame: str) -> str:
    """Use driver child frame or derive a namespaced base frame."""
    child_frame = odometry_child_frame.lstrip("/")
    if child_frame:
        return child_frame
    robot_namespace = namespace.strip("/")
    return f"{robot_namespace}/base_link" if robot_namespace else "base_link"


# =============================================================================
# Median filter
# =============================================================================

class RollingMedian:
    """Causal rolling median."""

    def __init__(self, window_size: int):
        if window_size < 1:
            raise ValueError("median window size must be >= 1")
        if window_size % 2 == 0:
            raise ValueError("median window size must be odd")
        self.values = deque(maxlen=window_size)

    def reset(self):
        self.values.clear()

    def update(self, value: float) -> float:
        self.values.append(float(value))
        return float(median(self.values))


# =============================================================================
# One Euro filter
# =============================================================================

class LowPassFilter:
    def __init__(self):
        self.initialized = False
        self.value = 0.0

    def reset(self, value: float):
        self.value = float(value)
        self.initialized = True

    def update(self, value: float, alpha: float) -> float:
        value = float(value)
        if not self.initialized:
            self.reset(value)
            return value
        self.value = alpha * value + (1.0 - alpha) * self.value
        return self.value


class OneEuroFilter:
    """Adaptive low-pass filter."""

    def __init__(self, min_cutoff: float, beta: float, derivative_cutoff: float):
        if min_cutoff <= 0.0:
            raise ValueError("min_cutoff must be positive")
        if beta < 0.0:
            raise ValueError("beta must be non-negative")
        if derivative_cutoff <= 0.0:
            raise ValueError("derivative_cutoff must be positive")

        self.min_cutoff = float(min_cutoff)
        self.beta = float(beta)
        self.derivative_cutoff = float(derivative_cutoff)

        self.signal_filter = LowPassFilter()
        self.derivative_filter = LowPassFilter()

        self.previous_raw = None
        self.previous_time = None

    @staticmethod
    def smoothing_alpha(dt: float, cutoff: float) -> float:
        tau = 1.0 / (2.0 * math.pi * cutoff)
        return 1.0 / (1.0 + tau / dt)

    def reset(self, value: float, time_seconds: float):
        self.previous_raw = float(value)
        self.previous_time = float(time_seconds)
        self.signal_filter.reset(value)
        self.derivative_filter.reset(0.0)

    def update(self, value: float, time_seconds: float) -> float:
        value = float(value)
        time_seconds = float(time_seconds)

        if self.previous_raw is None or self.previous_time is None:
            self.reset(value, time_seconds)
            return value

        dt = time_seconds - self.previous_time

        if not math.isfinite(dt) or dt <= 0.0:
            return self.signal_filter.value

        raw_derivative = (value - self.previous_raw) / dt
        derivative_alpha = self.smoothing_alpha(dt, self.derivative_cutoff)
        filtered_derivative = self.derivative_filter.update(raw_derivative, derivative_alpha)

        cutoff = self.min_cutoff + self.beta * abs(filtered_derivative)
        signal_alpha = self.smoothing_alpha(dt, cutoff)
        filtered_value = self.signal_filter.update(value, signal_alpha)

        self.previous_raw = value
        self.previous_time = time_seconds

        return filtered_value


class AngleOneEuroFilter:
    """One Euro filter for wrapped yaw angles."""

    def __init__(self, min_cutoff: float, beta: float, derivative_cutoff: float):
        self.filter = OneEuroFilter(min_cutoff, beta, derivative_cutoff)
        self.previous_wrapped = None
        self.unwrapped_angle = 0.0

    def reset(self, angle: float, time_seconds: float):
        angle = wrap_angle(angle)
        self.previous_wrapped = angle
        self.unwrapped_angle = angle
        self.filter.reset(angle, time_seconds)

    def update(self, angle: float, time_seconds: float) -> float:
        angle = wrap_angle(angle)

        if self.previous_wrapped is None:
            self.reset(angle, time_seconds)
            return angle

        delta = wrap_angle(angle - self.previous_wrapped)
        self.unwrapped_angle += delta
        self.previous_wrapped = angle

        return wrap_angle(self.filter.update(self.unwrapped_angle, time_seconds))


# =============================================================================
# Odometry construction
# =============================================================================

def make_localization_odometry(
    x: float,
    y: float,
    yaw: float,
    body_vx: float,
    body_vy: float,
    yaw_rate: float,
    source_odometry: Odometry,
    stamp,
    global_frame_id: str,
    namespace: str,
    gps_position_variance: float,
) -> Odometry:
    """Construct localization odometry."""

    message = Odometry()
    message.header.stamp = stamp
    message.header.frame_id = global_frame_id
    message.child_frame_id = namespaced_base_frame(namespace, source_odometry.child_frame_id)

    message.pose.pose.position.x = x
    message.pose.pose.position.y = y
    message.pose.pose.position.z = 0.0
    message.pose.pose.orientation.z = math.sin(0.5 * yaw)
    message.pose.pose.orientation.w = math.cos(0.5 * yaw)

    message.pose.covariance[0] = gps_position_variance
    message.pose.covariance[7] = gps_position_variance
    message.pose.covariance[14] = UNOBSERVED_VARIANCE
    message.pose.covariance[21] = UNOBSERVED_VARIANCE
    message.pose.covariance[28] = UNOBSERVED_VARIANCE
    message.pose.covariance[35] = source_odometry.pose.covariance[35]

    message.twist = deepcopy(source_odometry.twist)
    message.twist.twist.linear.x = body_vx
    message.twist.twist.linear.y = body_vy
    message.twist.twist.angular.z = yaw_rate

    return message


def make_map_to_odometry_transform(
    x: float,
    y: float,
    yaw: float,
    source_odometry: Odometry,
    stamp,
    global_frame_id: str,
) -> TransformStamped:
    """Calculate global-to-driver-odometry transform."""

    odometry_frame = source_odometry.header.frame_id.lstrip("/")

    if not odometry_frame:
        raise ValueError("source odometry frame_id must not be empty")

    quaternion = source_odometry.pose.pose.orientation
    odometry_yaw = quat2euler(
        [quaternion.w, quaternion.x, quaternion.y, quaternion.z],
        axes="sxyz",
    )[2]

    map_to_odometry_yaw = yaw - odometry_yaw
    cosine = math.cos(map_to_odometry_yaw)
    sine = math.sin(map_to_odometry_yaw)
    odometry_position = source_odometry.pose.pose.position

    message = TransformStamped()
    message.header.stamp = stamp
    message.header.frame_id = global_frame_id
    message.child_frame_id = odometry_frame

    message.transform.translation.x = x - (
        cosine * odometry_position.x - sine * odometry_position.y
    )

    message.transform.translation.y = y - (
        sine * odometry_position.x + cosine * odometry_position.y
    )

    message.transform.rotation.z = math.sin(0.5 * map_to_odometry_yaw)
    message.transform.rotation.w = math.cos(0.5 * map_to_odometry_yaw)

    return message


# =============================================================================
# Pose publisher
# =============================================================================

class PosePublisherNode(Node):
    """
    Initialization:
        Marvelmind -> initial global x/y
        RoboMaster odom -> initial odometry x/y
        RoboMaster yaw -> initial yaw offset

    Running:
        RoboMaster odometry displacement
            -> rotate into swarm_map
            -> One Euro x/y filtering
            -> add initial Marvelmind position
    """

    def __init__(self):
        super().__init__("pose_publisher")
        self.get_logger().info("Odometry-propagated filtered pose publisher started.")

        # ---------------------------------------------------------------------
        # General parameters
        # ---------------------------------------------------------------------

        self.declare_parameter("init_orientation", 0.0)
        self.declare_parameter("init_period", 1.0)
        self.declare_parameter("timer_frequency", 0.01)
        self.declare_parameter("global_frame_id", "swarm_map")
        self.declare_parameter("gps_timeout", 0.25)
        self.declare_parameter("odometry_timeout", 0.25)
        self.declare_parameter("gps_position_variance", 0.01)

        # ---------------------------------------------------------------------
        # Marvelmind initial-position filter
        # ---------------------------------------------------------------------

        self.declare_parameter("position_filter_enabled", True)
        self.declare_parameter("position_median_window", 3)
        self.declare_parameter("position_min_cutoff", 2.0)
        self.declare_parameter("position_beta", 2.0)
        self.declare_parameter("position_derivative_cutoff", 1.0)

        # ---------------------------------------------------------------------
        # Running RoboMaster odometry x/y displacement filter
        # ---------------------------------------------------------------------

        self.declare_parameter("odom_position_filter_enabled", True)
        self.declare_parameter("odom_position_min_cutoff", 2.0)
        self.declare_parameter("odom_position_beta", 2.0)
        self.declare_parameter("odom_position_derivative_cutoff", 1.0)

        # ---------------------------------------------------------------------
        # Yaw filter
        # ---------------------------------------------------------------------

        self.declare_parameter("yaw_filter_enabled", True)
        self.declare_parameter("yaw_min_cutoff", 3.0)
        self.declare_parameter("yaw_beta", 1.0)
        self.declare_parameter("yaw_derivative_cutoff", 1.0)

        # ---------------------------------------------------------------------
        # Twist filter
        # ---------------------------------------------------------------------

        self.declare_parameter("twist_filter_enabled", True)
        self.declare_parameter("twist_min_cutoff", 4.0)
        self.declare_parameter("twist_beta", 0.5)
        self.declare_parameter("twist_derivative_cutoff", 1.0)

        # ---------------------------------------------------------------------
        # QoS
        # ---------------------------------------------------------------------

        self.declare_parameter("qos_depth", 10)
        self.declare_parameter("qos_reliability", "RELIABLE")
        self.declare_parameter("qos_history", "KEEP_LAST")

        # ---------------------------------------------------------------------
        # Read general parameters
        # ---------------------------------------------------------------------

        initial_orientation = float(self.get_parameter("init_orientation").value)
        self.init_period = float(self.get_parameter("init_period").value)
        self.timer_frequency = float(self.get_parameter("timer_frequency").value)
        self.global_frame_id = str(self.get_parameter("global_frame_id").value).strip("/")
        self.gps_timeout = float(self.get_parameter("gps_timeout").value)
        self.odometry_timeout = float(self.get_parameter("odometry_timeout").value)
        self.gps_position_variance = float(self.get_parameter("gps_position_variance").value)

        # ---------------------------------------------------------------------
        # Initial Marvelmind filtering
        # ---------------------------------------------------------------------

        self.position_filter_enabled = bool(self.get_parameter("position_filter_enabled").value)
        position_median_window = int(self.get_parameter("position_median_window").value)
        position_min_cutoff = float(self.get_parameter("position_min_cutoff").value)
        position_beta = float(self.get_parameter("position_beta").value)
        position_derivative_cutoff = float(self.get_parameter("position_derivative_cutoff").value)

        # ---------------------------------------------------------------------
        # Running odometry-position filtering
        # ---------------------------------------------------------------------

        self.odom_position_filter_enabled = bool(
            self.get_parameter("odom_position_filter_enabled").value
        )

        odom_position_min_cutoff = float(
            self.get_parameter("odom_position_min_cutoff").value
        )

        odom_position_beta = float(
            self.get_parameter("odom_position_beta").value
        )

        odom_position_derivative_cutoff = float(
            self.get_parameter("odom_position_derivative_cutoff").value
        )

        # ---------------------------------------------------------------------
        # Yaw
        # ---------------------------------------------------------------------

        self.yaw_filter_enabled = bool(self.get_parameter("yaw_filter_enabled").value)
        yaw_min_cutoff = float(self.get_parameter("yaw_min_cutoff").value)
        yaw_beta = float(self.get_parameter("yaw_beta").value)
        yaw_derivative_cutoff = float(self.get_parameter("yaw_derivative_cutoff").value)

        # ---------------------------------------------------------------------
        # Twist
        # ---------------------------------------------------------------------

        self.twist_filter_enabled = bool(self.get_parameter("twist_filter_enabled").value)
        twist_min_cutoff = float(self.get_parameter("twist_min_cutoff").value)
        twist_beta = float(self.get_parameter("twist_beta").value)
        twist_derivative_cutoff = float(self.get_parameter("twist_derivative_cutoff").value)

        # ---------------------------------------------------------------------
        # Validation
        # ---------------------------------------------------------------------

        positive_values = (
            self.init_period,
            self.timer_frequency,
            self.gps_timeout,
            self.odometry_timeout,
        )

        if not all(math.isfinite(value) and value > 0.0 for value in positive_values):
            raise ValueError("period and timeout parameters must be positive")

        if not math.isfinite(self.gps_position_variance) or self.gps_position_variance < 0.0:
            raise ValueError("gps_position_variance must be finite and non-negative")

        if not self.global_frame_id:
            raise ValueError("global_frame_id must not be empty")

        # ---------------------------------------------------------------------
        # QoS
        # ---------------------------------------------------------------------

        qos_depth = int(self.get_parameter("qos_depth").value)
        qos_reliability = str(self.get_parameter("qos_reliability").value)
        qos_history = str(self.get_parameter("qos_history").value)

        if qos_depth <= 0:
            raise ValueError("qos_depth must be positive")

        if qos_reliability not in ("RELIABLE", "BEST_EFFORT"):
            raise ValueError(f"Invalid qos_reliability: {qos_reliability}")

        if qos_history not in ("KEEP_LAST", "KEEP_ALL"):
            raise ValueError(f"Invalid qos_history: {qos_history}")

        self.qos_profile = QoSProfile(
            depth=qos_depth,
            reliability=(
                ReliabilityPolicy.RELIABLE
                if qos_reliability == "RELIABLE"
                else ReliabilityPolicy.BEST_EFFORT
            ),
            history=(
                HistoryPolicy.KEEP_LAST
                if qos_history == "KEEP_LAST"
                else HistoryPolicy.KEEP_ALL
            ),
        )

        # ---------------------------------------------------------------------
        # Initial Marvelmind filter
        # ---------------------------------------------------------------------

        self.x_median = RollingMedian(position_median_window)
        self.y_median = RollingMedian(position_median_window)

        self.x_filter = OneEuroFilter(
            position_min_cutoff,
            position_beta,
            position_derivative_cutoff,
        )

        self.y_filter = OneEuroFilter(
            position_min_cutoff,
            position_beta,
            position_derivative_cutoff,
        )

        # ---------------------------------------------------------------------
        # Running odometry displacement One Euro filters
        #
        # These operate on delta_map_x and delta_map_y, not absolute odom x/y.
        # ---------------------------------------------------------------------

        self.odom_delta_x_filter = OneEuroFilter(
            odom_position_min_cutoff,
            odom_position_beta,
            odom_position_derivative_cutoff,
        )

        self.odom_delta_y_filter = OneEuroFilter(
            odom_position_min_cutoff,
            odom_position_beta,
            odom_position_derivative_cutoff,
        )

        # ---------------------------------------------------------------------
        # Yaw and twist
        # ---------------------------------------------------------------------

        self.yaw_filter = AngleOneEuroFilter(
            yaw_min_cutoff,
            yaw_beta,
            yaw_derivative_cutoff,
        )

        self.vx_filter = OneEuroFilter(
            twist_min_cutoff,
            twist_beta,
            twist_derivative_cutoff,
        )

        self.vy_filter = OneEuroFilter(
            twist_min_cutoff,
            twist_beta,
            twist_derivative_cutoff,
        )

        self.wz_filter = OneEuroFilter(
            twist_min_cutoff,
            twist_beta,
            twist_derivative_cutoff,
        )

        # ---------------------------------------------------------------------
        # Localization state
        # ---------------------------------------------------------------------

        self.x = np.array([0.0, 0.0, initial_orientation / 180.0 * np.pi])

        self.body_vx = 0.0
        self.body_vy = 0.0
        self.yaw_rate = 0.0

        self.gps_initializing = True
        self.odom_initializing = True

        self.init_start_time = self.get_clock().now()

        self.pos_buffer = []
        self.odom_position_buffer = []
        self.angle_buffer = []

        # Global anchor from Marvelmind.
        self.init_pos = np.zeros(2)

        # Local odometry anchor from RoboMaster.
        self.init_odom_pos = np.zeros(2)

        # global_yaw = raw_odom_yaw - init_angle
        self.init_angle = 0.0

        self.latest_odometry = None

        self.last_gps_timestamp_ms = None
        self.last_gps_update_nanoseconds = None
        self.last_odometry_update_nanoseconds = None

        self.get_logger().info(
            "Initializing global position and odometry; keep robot stationary."
        )

        # ---------------------------------------------------------------------
        # ROS interfaces
        # ---------------------------------------------------------------------

        self.create_subscription(
            HedgePositionAddressed,
            "mm_pos_unf",
            self.indoor_gps_callback,
            self.qos_profile,
        )

        self.create_subscription(
            Odometry,
            "odom",
            self.odom_callback,
            self.qos_profile,
        )

        self.pose_pub = self.create_publisher(State, "pose", self.qos_profile)

        localization_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.localization_pub = self.create_publisher(
            Odometry,
            "localization/odom",
            localization_qos,
        )

        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_timer(self.timer_frequency, self.publish_state)

        self.get_logger().info(
            f"Initial GPS filter: {self.position_filter_enabled}, "
            f"odom x/y filter: {self.odom_position_filter_enabled}, "
            f"yaw filter: {self.yaw_filter_enabled}, "
            f"twist filter: {self.twist_filter_enabled}"
        )

    # =========================================================================
    # Marvelmind initialization
    # =========================================================================

    def indoor_gps_callback(self, message: HedgePositionAddressed) -> None:
        """Use Marvelmind only for the initial global x/y."""

        now = self.get_clock().now()
        now_seconds = now.nanoseconds * 1.0e-9

        genuine_update = message.timestamp_ms != self.last_gps_timestamp_ms

        if not genuine_update:
            return

        self.last_gps_update_nanoseconds = genuine_gps_update_time(
            message.timestamp_ms,
            self.last_gps_timestamp_ms,
            now.nanoseconds,
            self.last_gps_update_nanoseconds,
        )

        self.last_gps_timestamp_ms = message.timestamp_ms

        if not self.gps_initializing:
            return

        raw_x = float(message.x_m)
        raw_y = float(message.y_m)

        if self.position_filter_enabled:
            median_x = self.x_median.update(raw_x)
            median_y = self.y_median.update(raw_y)
            candidate_x = self.x_filter.update(median_x, now_seconds)
            candidate_y = self.y_filter.update(median_y, now_seconds)
        else:
            candidate_x = raw_x
            candidate_y = raw_y

        self.pos_buffer.append([candidate_x, candidate_y])

        elapsed = (now - self.init_start_time).nanoseconds * 1.0e-9

        if elapsed >= self.init_period:
            self.init_pos = np.mean(self.pos_buffer, axis=0)
            self.x[0] = float(self.init_pos[0])
            self.x[1] = float(self.init_pos[1])
            self.gps_initializing = False

            self.get_logger().info("Marvelmind global-position initialization complete.")
            self.get_logger().info(f"Initial global position: {self.init_pos} meters")

    # =========================================================================
    # RoboMaster odometry
    # =========================================================================

    def odom_callback(self, message: Odometry) -> None:
        """Propagate position, orientation, and velocity from RoboMaster odometry."""

        now = self.get_clock().now()
        now_seconds = now.nanoseconds * 1.0e-9

        self.latest_odometry = message
        self.last_odometry_update_nanoseconds = now.nanoseconds

        raw_odom_x = float(message.pose.pose.position.x)
        raw_odom_y = float(message.pose.pose.position.y)

        quaternion = message.pose.pose.orientation

        raw_yaw = quat2euler(
            [quaternion.w, quaternion.x, quaternion.y, quaternion.z],
            axes="sxyz",
        )[2]

        raw_vx = float(message.twist.twist.linear.x)
        raw_vy = float(message.twist.twist.linear.y)
        raw_wz = float(message.twist.twist.angular.z)

        # ---------------------------------------------------------------------
        # Twist
        # ---------------------------------------------------------------------

        if self.twist_filter_enabled:
            self.body_vx = self.vx_filter.update(raw_vx, now_seconds)
            self.body_vy = self.vy_filter.update(raw_vy, now_seconds)
            self.yaw_rate = self.wz_filter.update(raw_wz, now_seconds)
        else:
            self.body_vx = raw_vx
            self.body_vy = raw_vy
            self.yaw_rate = raw_wz

        # ---------------------------------------------------------------------
        # Initial odometry position + yaw
        # ---------------------------------------------------------------------

        if self.odom_initializing:
            self.odom_position_buffer.append([raw_odom_x, raw_odom_y])
            self.angle_buffer.append(raw_yaw)

            elapsed = (now - self.init_start_time).nanoseconds * 1.0e-9

            if elapsed >= self.init_period:
                self.init_odom_pos = np.mean(self.odom_position_buffer, axis=0)
                self.init_angle = float(np.mean(self.angle_buffer)) - self.x[2]
                self.odom_initializing = False

                # Running displacement begins at exactly zero.
                self.odom_delta_x_filter.reset(0.0, now_seconds)
                self.odom_delta_y_filter.reset(0.0, now_seconds)

                if self.yaw_filter_enabled:
                    self.yaw_filter.reset(float(self.x[2]), now_seconds)

                self.get_logger().info("RoboMaster odometry initialization complete.")
                self.get_logger().info(
                    f"Initial odom position: {self.init_odom_pos} meters"
                )
                self.get_logger().info(
                    f"Initial yaw offset: {self.init_angle} rad"
                )

            return

        # ---------------------------------------------------------------------
        # Yaw
        # ---------------------------------------------------------------------

        calibrated_yaw = raw_yaw - self.init_angle

        if self.yaw_filter_enabled:
            self.x[2] = self.yaw_filter.update(calibrated_yaw, now_seconds)
        else:
            self.x[2] = calibrated_yaw

        # Global x/y cannot be constructed until Marvelmind init_pos exists.
        if self.gps_initializing:
            return

        # ---------------------------------------------------------------------
        # Local odometry displacement
        # ---------------------------------------------------------------------

        delta_odom_x = raw_odom_x - self.init_odom_pos[0]
        delta_odom_y = raw_odom_y - self.init_odom_pos[1]

        # ---------------------------------------------------------------------
        # Rotate displacement from odom coordinates into swarm_map coordinates.
        #
        # global_yaw = raw_odom_yaw - init_angle
        #
        # Therefore:
        #
        # R_map_odom = R(-init_angle)
        # ---------------------------------------------------------------------

        map_from_odom_angle = -self.init_angle
        cosine = math.cos(map_from_odom_angle)
        sine = math.sin(map_from_odom_angle)

        delta_map_x = cosine * delta_odom_x - sine * delta_odom_y
        delta_map_y = sine * delta_odom_x + cosine * delta_odom_y

        # ---------------------------------------------------------------------
        # One Euro filtering of running x/y displacement
        # ---------------------------------------------------------------------

        if self.odom_position_filter_enabled:
            delta_map_x = self.odom_delta_x_filter.update(delta_map_x, now_seconds)
            delta_map_y = self.odom_delta_y_filter.update(delta_map_y, now_seconds)

        # ---------------------------------------------------------------------
        # Global localization position
        # ---------------------------------------------------------------------

        self.x[0] = self.init_pos[0] + delta_map_x
        self.x[1] = self.init_pos[1] + delta_map_y

    # =========================================================================
    # Readiness
    # =========================================================================

    def localization_is_ready(self, now_nanoseconds: int) -> bool:
        """Marvelmind is required only for initialization."""

        return (
            not self.gps_initializing
            and not self.odom_initializing
            and self.latest_odometry is not None
            and source_is_fresh(
                now_nanoseconds,
                self.last_odometry_update_nanoseconds,
                self.odometry_timeout,
            )
        )

    # =========================================================================
    # Publish
    # =========================================================================

    def publish_state(self) -> None:
        """Publish pose and localization odometry."""

        legacy_message = State()
        legacy_message.x = float(self.x[0])
        legacy_message.y = float(self.x[1])
        legacy_message.theta = float(degrees(self.x[2]))

        self.pose_pub.publish(legacy_message)

        now = self.get_clock().now()

        if not self.localization_is_ready(now.nanoseconds):
            return

        message = make_localization_odometry(
            x=float(self.x[0]),
            y=float(self.x[1]),
            yaw=float(self.x[2]),
            body_vx=float(self.body_vx),
            body_vy=float(self.body_vy),
            yaw_rate=float(self.yaw_rate),
            source_odometry=self.latest_odometry,
            stamp=now.to_msg(),
            global_frame_id=self.global_frame_id,
            namespace=self.get_namespace(),
            gps_position_variance=self.gps_position_variance,
        )

        self.localization_pub.publish(message)

        if self.latest_odometry.header.frame_id.lstrip("/"):
            transform = make_map_to_odometry_transform(
                x=float(self.x[0]),
                y=float(self.x[1]),
                yaw=float(self.x[2]),
                source_odometry=self.latest_odometry,
                stamp=now.to_msg(),
                global_frame_id=self.global_frame_id,
            )

            self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = PosePublisherNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()