"""Position-error plus velocity-feedforward tracking controller."""

from dataclasses import dataclass
import math
from typing import Optional, Tuple

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


@dataclass
class ReferenceState:
    """Virtual pose with body-frame velocity feedforward."""

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    body_vx: float = 0.0
    body_vy: float = 0.0
    yaw_rate: float = 0.0


@dataclass
class MeasuredPose:
    """Measured planar chassis pose in the reference global frame."""

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


def yaw_from_quaternion(quaternion) -> float:
    """Extract planar yaw from a geometry_msgs quaternion."""
    sin_yaw = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cos_yaw = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(sin_yaw, cos_yaw)


def wrap_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def rotate_body_to_inertial(x: float, y: float, yaw: float) -> Tuple[float, float]:
    """Rotate a planar vector from body coordinates to inertial coordinates."""
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return cosine * x - sine * y, sine * x + cosine * y


def rotate_inertial_to_body(x: float, y: float, yaw: float) -> Tuple[float, float]:
    """Rotate a planar vector from inertial coordinates to body coordinates."""
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return cosine * x + sine * y, -sine * x + cosine * y


def limit_vector(x: float, y: float, maximum: float) -> Tuple[float, float, bool]:
    """Apply direction-preserving magnitude saturation to a planar vector."""
    if maximum < 0.0:
        raise ValueError("maximum must be non-negative")
    magnitude = math.hypot(x, y)
    if magnitude <= maximum or magnitude == 0.0:
        return x, y, False
    scale = maximum / magnitude
    return scale * x, scale * y, True


def tracking_command(
    reference: ReferenceState,
    measured: MeasuredPose,
    position_gain: float,
    yaw_gain: float,
    maximum_speed: float,
    maximum_yaw_rate: float,
) -> Tuple[float, float, float, bool]:
    """Calculate a saturated chassis-body-frame velocity command."""
    inertial_vx, inertial_vy = rotate_body_to_inertial(
        reference.body_vx, reference.body_vy, reference.yaw
    )
    inertial_vx += position_gain * (reference.x - measured.x)
    inertial_vy += position_gain * (reference.y - measured.y)
    body_x, body_y = rotate_inertial_to_body(
        inertial_vx, inertial_vy, measured.yaw
    )
    body_x, body_y, linear_saturated = limit_vector(
        body_x, body_y, maximum_speed
    )
    yaw_command = reference.yaw_rate + yaw_gain * wrap_angle(
        reference.yaw - measured.yaw
    )
    limited_yaw = max(-maximum_yaw_rate, min(maximum_yaw_rate, yaw_command))
    return (
        body_x,
        body_y,
        limited_yaw,
        linear_saturated or limited_yaw != yaw_command,
    )


def inputs_are_fresh(
    now_nanoseconds: int,
    odometry_time: Optional[int],
    reference_time: Optional[int],
    odometry_timeout: float,
    reference_timeout: float,
) -> bool:
    """Return whether both controller inputs exist and are recent."""
    if odometry_time is None or reference_time is None:
        return False
    odometry_age = (now_nanoseconds - odometry_time) * 1.0e-9
    reference_age = (now_nanoseconds - reference_time) * 1.0e-9
    return (
        0.0 <= odometry_age <= odometry_timeout
        and 0.0 <= reference_age <= reference_timeout
    )


def frames_match(odometry_frame: str, reference_frame: str) -> bool:
    """Return whether both inputs name the same non-empty global frame."""
    return bool(odometry_frame) and odometry_frame == reference_frame


def qos_profile_from_parameters(node: Node) -> QoSProfile:
    """Build the repository-standard configurable QoS profile."""
    qos_depth = int(node.get_parameter("qos_depth").value)
    qos_reliability = str(node.get_parameter("qos_reliability").value)
    qos_history = str(node.get_parameter("qos_history").value)
    if qos_depth <= 0:
        raise ValueError("qos_depth must be positive")
    if qos_reliability not in ("RELIABLE", "BEST_EFFORT"):
        raise ValueError(f"Invalid qos_reliability: {qos_reliability}")
    if qos_history not in ("KEEP_LAST", "KEEP_ALL"):
        raise ValueError(f"Invalid qos_history: {qos_history}")
    return QoSProfile(
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


def validate_controller_parameters(
    timer_frequency: float,
    position_gain: float,
    yaw_gain: float,
    maximum_speed: float,
    maximum_yaw_rate: float,
    odometry_timeout: float,
    reference_timeout: float,
) -> None:
    """Reject unsafe or non-finite controller parameters."""
    positive_values = (
        timer_frequency,
        maximum_speed,
        maximum_yaw_rate,
        odometry_timeout,
        reference_timeout,
    )
    if not all(math.isfinite(value) and value > 0.0 for value in positive_values):
        raise ValueError("period, timeout, and limit parameters must be positive")
    if not math.isfinite(position_gain) or position_gain < 0.0:
        raise ValueError("position_gain must be finite and non-negative")
    if not math.isfinite(yaw_gain) or yaw_gain < 0.0:
        raise ValueError("yaw_gain must be finite and non-negative")


class Nav2PffNode(Node):
    """Track virtual spacecraft odometry with P feedback and feedforward."""

    def __init__(self):
        super().__init__("nav2_pff")
        self.declare_parameter("timer_frequency", 0.01)
        self.declare_parameter("position_gain", 1.0)
        self.declare_parameter("yaw_gain", 2.0)
        self.declare_parameter("maximum_speed", 0.5)
        self.declare_parameter("maximum_yaw_rate", 1.0)
        self.declare_parameter("odometry_timeout", 0.25)
        self.declare_parameter("reference_timeout", 0.25)
        self.declare_parameter("qos_depth", 10)
        self.declare_parameter("qos_reliability", "RELIABLE")
        self.declare_parameter("qos_history", "KEEP_LAST")

        timer_frequency = float(
            self.get_parameter("timer_frequency").value
        )
        self.position_gain = float(self.get_parameter("position_gain").value)
        self.yaw_gain = float(self.get_parameter("yaw_gain").value)
        self.maximum_speed = float(self.get_parameter("maximum_speed").value)
        self.maximum_yaw_rate = float(
            self.get_parameter("maximum_yaw_rate").value
        )
        self.odometry_timeout = float(
            self.get_parameter("odometry_timeout").value
        )
        self.reference_timeout = float(
            self.get_parameter("reference_timeout").value
        )
        validate_controller_parameters(
            timer_frequency,
            self.position_gain,
            self.yaw_gain,
            self.maximum_speed,
            self.maximum_yaw_rate,
            self.odometry_timeout,
            self.reference_timeout,
        )

        self.reference = ReferenceState()
        self.measured = MeasuredPose()
        self.odometry_frame_id = ""
        self.reference_frame_id = ""
        self.last_odometry_time = None
        self.last_reference_time = None
        self.saturation_count = 0

        qos_profile = qos_profile_from_parameters(self)
        self.create_subscription(
            Odometry,
            "localization/odom",
            self.odometry_callback,
            qos_profile,
        )
        self.create_subscription(
            Odometry,
            "virtual_spacecraft/odom",
            self.reference_callback,
            qos_profile,
        )
        self.command_publisher = self.create_publisher(
            Twist, "cmd_vel", qos_profile
        )
        self.create_timer(timer_frequency, self.update)
        self.get_logger().info("P+FF spacecraft tracking controller ready")

    def odometry_callback(self, message: Odometry) -> None:
        """Store the latest physical chassis pose."""
        self.measured = MeasuredPose(
            x=message.pose.pose.position.x,
            y=message.pose.pose.position.y,
            yaw=yaw_from_quaternion(message.pose.pose.orientation),
        )
        self.odometry_frame_id = message.header.frame_id.lstrip("/")
        self.last_odometry_time = self.get_clock().now().nanoseconds

    def reference_callback(self, message: Odometry) -> None:
        """Store the latest virtual state reference."""
        self.reference = ReferenceState(
            x=message.pose.pose.position.x,
            y=message.pose.pose.position.y,
            yaw=yaw_from_quaternion(message.pose.pose.orientation),
            body_vx=message.twist.twist.linear.x,
            body_vy=message.twist.twist.linear.y,
            yaw_rate=message.twist.twist.angular.z,
        )
        self.reference_frame_id = message.header.frame_id.lstrip("/")
        self.last_reference_time = self.get_clock().now().nanoseconds

    def update(self) -> None:
        """Publish a tracking command, or stop when an input is stale."""
        now_nanoseconds = self.get_clock().now().nanoseconds
        inputs_fresh = inputs_are_fresh(
            now_nanoseconds,
            self.last_odometry_time,
            self.last_reference_time,
            self.odometry_timeout,
            self.reference_timeout,
        )
        if not inputs_fresh or not frames_match(
            self.odometry_frame_id, self.reference_frame_id
        ):
            self.publish_stop()
            return

        body_x, body_y, yaw_rate, saturated = tracking_command(
            self.reference,
            self.measured,
            self.position_gain,
            self.yaw_gain,
            self.maximum_speed,
            self.maximum_yaw_rate,
        )
        command = Twist()
        command.linear.x = body_x
        command.linear.y = body_y
        command.angular.z = yaw_rate
        self.command_publisher.publish(command)
        if saturated:
            self.saturation_count += 1
            if self.saturation_count % 100 == 1:
                self.get_logger().warn("Physical velocity command is saturated")

    def publish_stop(self) -> None:
        """Publish a zero velocity command."""
        self.command_publisher.publish(Twist())


def main(args=None):
    """Run the P+FF tracking node."""
    rclpy.init(args=args)
    node = Nav2PffNode()
    try:
        rclpy.spin(node)
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
