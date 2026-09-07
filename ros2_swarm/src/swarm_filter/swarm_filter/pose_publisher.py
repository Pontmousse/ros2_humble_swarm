"""Publish legacy and standard physical pose interfaces for each robot."""

from copy import deepcopy
import math
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


def degrees(angle):
    """Convert radians to degrees wrapped to [-180, 180)."""
    angle = angle / np.pi * 180
    return (angle + 180) % 360 - 180


def source_is_fresh(
    now_nanoseconds: int,
    update_nanoseconds: Optional[int],
    timeout: float,
) -> bool:
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
    if timestamp_ms != previous_timestamp_ms:
        return now_nanoseconds
    return previous_update_nanoseconds


def namespaced_base_frame(namespace: str, odometry_child_frame: str) -> str:
    """Use the driver child frame or derive a namespaced base frame."""
    child_frame = odometry_child_frame.lstrip("/")
    if child_frame:
        return child_frame
    robot_namespace = namespace.strip("/")
    if robot_namespace:
        return f"{robot_namespace}/base_link"
    return "base_link"


def make_localization_odometry(
    x: float,
    y: float,
    yaw: float,
    source_odometry: Odometry,
    stamp,
    global_frame_id: str,
    namespace: str,
    gps_position_variance: float,
) -> Odometry:
    """Combine filtered GPS position with RoboMaster attitude and twist."""
    message = Odometry()
    message.header.stamp = stamp
    message.header.frame_id = global_frame_id
    message.child_frame_id = namespaced_base_frame(
        namespace,
        source_odometry.child_frame_id,
    )
    message.pose.pose.position.x = x
    message.pose.pose.position.y = y
    message.pose.pose.orientation.z = math.sin(0.5 * yaw)
    message.pose.pose.orientation.w = math.cos(0.5 * yaw)

    message.pose.covariance[0] = gps_position_variance
    message.pose.covariance[7] = gps_position_variance
    message.pose.covariance[14] = UNOBSERVED_VARIANCE
    message.pose.covariance[21] = UNOBSERVED_VARIANCE
    message.pose.covariance[28] = UNOBSERVED_VARIANCE
    message.pose.covariance[35] = source_odometry.pose.covariance[35]
    message.twist = deepcopy(source_odometry.twist)
    return message


def make_map_to_odometry_transform(
    x: float,
    y: float,
    yaw: float,
    source_odometry: Odometry,
    stamp,
    global_frame_id: str,
) -> TransformStamped:
    """Calculate the global-to-driver-odometry transform from paired poses."""
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


class PosePublisherNode(Node):
    """Combine filtered GPS position with calibrated RoboMaster attitude."""

    def __init__(self):
        super().__init__("pose_publisher")
        self.get_logger().info("Pose publisher has been started.")

        self.declare_parameter("init_orientation", 0.0)
        self.declare_parameter("init_period", 1.0)
        self.declare_parameter("timer_frequency", 0.01)
        self.declare_parameter("global_frame_id", "swarm_map")
        self.declare_parameter("gps_timeout", 0.25)
        self.declare_parameter("odometry_timeout", 0.25)
        self.declare_parameter("gps_position_variance", 0.01)
        self.declare_parameter("qos_depth", 10)
        self.declare_parameter("qos_reliability", "RELIABLE")
        self.declare_parameter("qos_history", "KEEP_LAST")

        initial_orientation = float(
            self.get_parameter("init_orientation").value
        )
        self.init_period = float(self.get_parameter("init_period").value)
        self.timer_frequency = float(
            self.get_parameter("timer_frequency").value
        )
        self.global_frame_id = str(
            self.get_parameter("global_frame_id").value
        ).strip("/")
        self.gps_timeout = float(self.get_parameter("gps_timeout").value)
        self.odometry_timeout = float(
            self.get_parameter("odometry_timeout").value
        )
        self.gps_position_variance = float(
            self.get_parameter("gps_position_variance").value
        )

        positive_values = (
            self.init_period,
            self.timer_frequency,
            self.gps_timeout,
            self.odometry_timeout,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in positive_values):
            raise ValueError("period and timeout parameters must be positive")
        if (
            not math.isfinite(self.gps_position_variance)
            or self.gps_position_variance < 0.0
        ):
            raise ValueError("gps_position_variance must be finite and non-negative")
        if not self.global_frame_id:
            raise ValueError("global_frame_id must not be empty")

        qos_depth = int(self.get_parameter("qos_depth").value)
        qos_reliability = str(self.get_parameter("qos_reliability").value)
        qos_history = str(self.get_parameter("qos_history").value)
        valid_reliabilities = ("RELIABLE", "BEST_EFFORT")
        valid_histories = ("KEEP_LAST", "KEEP_ALL")
        if qos_depth <= 0:
            raise ValueError("qos_depth must be positive")
        if qos_reliability not in valid_reliabilities:
            raise ValueError(f"Invalid qos_reliability: {qos_reliability}")
        if qos_history not in valid_histories:
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

        self.x = np.array([0.0, 0.0, initial_orientation / 180.0 * np.pi])
        self.gps_initializing = True
        self.odom_initializing = True
        self.init_start_time = self.get_clock().now()
        self.pos_buffer = []
        self.angle_buffer = []
        self.init_pos = np.zeros(2)
        self.init_angle = 0.0
        self.latest_odometry = None
        self.last_gps_timestamp_ms = None
        self.last_gps_update_nanoseconds = None
        self.last_odometry_update_nanoseconds = None
        self.get_logger().info(
            "Initializing initial position/orientation; keep the devices stationary."
        )

        self.create_subscription(
            HedgePositionAddressed,
            "mm_pos",
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

    def indoor_gps_callback(self, message: HedgePositionAddressed) -> None:
        """Store filtered GPS position and track genuine sensor updates."""
        now = self.get_clock().now()
        if message.timestamp_ms != self.last_gps_timestamp_ms:
            self.last_gps_update_nanoseconds = genuine_gps_update_time(
                message.timestamp_ms,
                self.last_gps_timestamp_ms,
                now.nanoseconds,
                self.last_gps_update_nanoseconds,
            )
            self.last_gps_timestamp_ms = message.timestamp_ms

        if self.gps_initializing:
            self.pos_buffer.append([message.x_m, message.y_m])
            elapsed = (now - self.init_start_time).nanoseconds * 1.0e-9
            if elapsed >= self.init_period:
                self.init_pos = np.mean(self.pos_buffer, axis=0)
                self.x[:2] = self.init_pos
                self.gps_initializing = False
                self.get_logger().info("Indoor GPS initialization complete.")
                self.get_logger().info(
                    f"Initial position: {self.init_pos} meters"
                )
            return

        self.x[:2] = np.array([message.x_m, message.y_m])

    def odom_callback(self, message: Odometry) -> None:
        """Store RoboMaster attitude and body-frame twist."""
        now = self.get_clock().now()
        self.latest_odometry = message
        self.last_odometry_update_nanoseconds = now.nanoseconds
        quaternion = message.pose.pose.orientation
        yaw = quat2euler(
            [quaternion.w, quaternion.x, quaternion.y, quaternion.z],
            axes="sxyz",
        )[2]

        if self.odom_initializing:
            self.angle_buffer.append(yaw)
            elapsed = (now - self.init_start_time).nanoseconds * 1.0e-9
            if elapsed >= self.init_period:
                self.init_angle = float(np.mean(self.angle_buffer)) - self.x[2]
                self.odom_initializing = False
                self.get_logger().info("Yaw angle initialization complete.")
                self.get_logger().info(
                    f"Initial angle wrap: {self.init_angle} rad"
                )
            return

        self.x[2] = yaw - self.init_angle

    def localization_is_ready(self, now_nanoseconds: int) -> bool:
        """Return whether initialized GPS and odometry inputs are fresh."""
        return (
            not self.gps_initializing
            and not self.odom_initializing
            and self.latest_odometry is not None
            and source_is_fresh(
                now_nanoseconds,
                self.last_gps_update_nanoseconds,
                self.gps_timeout,
            )
            and source_is_fresh(
                now_nanoseconds,
                self.last_odometry_update_nanoseconds,
                self.odometry_timeout,
            )
        )

    def publish_state(self) -> None:
        """Publish the legacy pose and, when valid, standard odometry."""
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
    """Run the physical pose publisher."""
    rclpy.init(args=args)
    node = PosePublisherNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
