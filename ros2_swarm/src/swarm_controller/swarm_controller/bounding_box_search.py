"""Generate intermittent wrench commands for a bounded free-drift search."""

import math

from geometry_msgs.msg import Wrench
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


def yaw_from_quaternion(quaternion) -> float:
    """Extract planar yaw from a geometry_msgs quaternion."""
    sin_yaw = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cos_yaw = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(sin_yaw, cos_yaw)


def rotate_body_to_global(x: float, y: float, yaw: float):
    """Rotate a planar vector from the spacecraft body into the global frame."""
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return cosine * x - sine * y, sine * x + cosine * y


def rotate_global_to_body(x: float, y: float, yaw: float):
    """Rotate a planar vector from the global frame into the spacecraft body."""
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return cosine * x + sine * y, -sine * x + cosine * y


def limit_vector(x: float, y: float, maximum: float):
    """Limit a planar vector while preserving its direction."""
    magnitude = math.hypot(x, y)
    if magnitude <= maximum or magnitude == 0.0:
        return x, y
    scale = maximum / magnitude
    return scale * x, scale * y


def boundary_firing(
    x: float,
    y: float,
    velocity_x: float,
    velocity_y: float,
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
    boundary_margin: float,
    firing_force: float,
    rebound_speed: float,
):
    """Return an inward global force and whether a boundary region is active."""
    force_x = 0.0
    force_y = 0.0
    near_boundary = False

    if x >= x_max - boundary_margin:
        near_boundary = True
        if velocity_x > -rebound_speed:
            force_x = -firing_force
    elif x <= x_min + boundary_margin:
        near_boundary = True
        if velocity_x < rebound_speed:
            force_x = firing_force

    if y >= y_max - boundary_margin:
        near_boundary = True
        if velocity_y > -rebound_speed:
            force_y = -firing_force
    elif y <= y_min + boundary_margin:
        near_boundary = True
        if velocity_y < rebound_speed:
            force_y = firing_force

    force_x, force_y = limit_vector(force_x, force_y, firing_force)
    return force_x, force_y, near_boundary


def qos_profile_from_parameters(node: Node) -> QoSProfile:
    """Build a configurable QoS profile with reliable defaults."""
    depth = int(node.get_parameter("qos_depth").value)
    reliability = str(node.get_parameter("qos_reliability").value)
    history = str(node.get_parameter("qos_history").value)
    if depth <= 0:
        raise ValueError("qos_depth must be positive")
    if reliability not in ("RELIABLE", "BEST_EFFORT"):
        raise ValueError(f"Invalid qos_reliability: {reliability}")
    if history not in ("KEEP_LAST", "KEEP_ALL"):
        raise ValueError(f"Invalid qos_history: {history}")
    return QoSProfile(
        depth=depth,
        reliability=(
            ReliabilityPolicy.RELIABLE
            if reliability == "RELIABLE"
            else ReliabilityPolicy.BEST_EFFORT
        ),
        history=(
            HistoryPolicy.KEEP_LAST
            if history == "KEEP_LAST"
            else HistoryPolicy.KEEP_ALL
        ),
    )


class BoundingBoxSearchNode(Node):
    """Fire briefly at startup and near search-box boundaries."""

    def __init__(self):
        super().__init__("bounding_box_search")
        self.declare_parameter("timer_frequency", 0.02)
        self.declare_parameter("reference_timeout", 0.25)
        self.declare_parameter("global_frame_id", "swarm_map")
        self.declare_parameter("x_min", -1.0)
        self.declare_parameter("x_max", 1.0)
        self.declare_parameter("y_min", -1.0)
        self.declare_parameter("y_max", 1.0)
        self.declare_parameter("boundary_margin", 0.25)
        self.declare_parameter("boundary_force", 1.0)
        self.declare_parameter("rebound_speed", 0.15)
        self.declare_parameter("maximum_force", 1.0)
        self.declare_parameter("maximum_torque", 0.2)
        self.declare_parameter("initial_burn_duration", 1.0)
        self.declare_parameter("initial_force_x", 0.8)
        self.declare_parameter("initial_force_y", 0.4)
        self.declare_parameter("initial_torque", 0.1)
        self.declare_parameter("qos_depth", 10)
        self.declare_parameter("qos_reliability", "RELIABLE")
        self.declare_parameter("qos_history", "KEEP_LAST")

        self.timer_frequency = float(self.get_parameter("timer_frequency").value)
        self.reference_timeout = float(
            self.get_parameter("reference_timeout").value
        )
        self.global_frame_id = str(
            self.get_parameter("global_frame_id").value
        ).strip("/")
        self.x_min = float(self.get_parameter("x_min").value)
        self.x_max = float(self.get_parameter("x_max").value)
        self.y_min = float(self.get_parameter("y_min").value)
        self.y_max = float(self.get_parameter("y_max").value)
        self.boundary_margin = float(
            self.get_parameter("boundary_margin").value
        )
        self.boundary_force = float(self.get_parameter("boundary_force").value)
        self.rebound_speed = float(self.get_parameter("rebound_speed").value)
        self.maximum_force = float(self.get_parameter("maximum_force").value)
        self.maximum_torque = float(self.get_parameter("maximum_torque").value)
        self.initial_burn_duration = float(
            self.get_parameter("initial_burn_duration").value
        )
        self.initial_force_x = float(
            self.get_parameter("initial_force_x").value
        )
        self.initial_force_y = float(
            self.get_parameter("initial_force_y").value
        )
        self.initial_torque = float(self.get_parameter("initial_torque").value)
        self.validate_parameters()

        self.latest_reference = None
        self.last_reference_time = None
        self.initial_burn_start = None

        qos_profile = qos_profile_from_parameters(self)
        self.create_subscription(
            Odometry,
            "virtual_spacecraft/odom",
            self.reference_callback,
            qos_profile,
        )
        self.wrench_publisher = self.create_publisher(
            Wrench, "spacecraft_wrench", qos_profile
        )
        self.create_timer(self.timer_frequency, self.update)
        self.get_logger().info("Bounding-box free-drift search guidance ready")

    def validate_parameters(self) -> None:
        """Reject inconsistent search geometry and firing parameters."""
        values = (
            self.timer_frequency,
            self.reference_timeout,
            self.x_min,
            self.x_max,
            self.y_min,
            self.y_max,
            self.boundary_margin,
            self.boundary_force,
            self.rebound_speed,
            self.maximum_force,
            self.maximum_torque,
            self.initial_burn_duration,
            self.initial_force_x,
            self.initial_force_y,
            self.initial_torque,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("Bounding-box parameters must be finite")
        if self.x_min >= self.x_max or self.y_min >= self.y_max:
            raise ValueError("Bounding-box minimums must be below maximums")
        if not 0.0 < self.boundary_margin < 0.5 * min(
            self.x_max - self.x_min, self.y_max - self.y_min
        ):
            raise ValueError("boundary_margin is inconsistent with the box")
        positive = (
            self.timer_frequency,
            self.reference_timeout,
            self.boundary_force,
            self.rebound_speed,
            self.maximum_force,
            self.maximum_torque,
            self.initial_burn_duration,
        )
        if not all(value > 0.0 for value in positive):
            raise ValueError("Timing, limits, and firing values must be positive")

    def reference_callback(self, message: Odometry) -> None:
        """Store the latest virtual state and its receipt time."""
        now = self.get_clock().now()
        self.latest_reference = message
        self.last_reference_time = now.nanoseconds
        if (
            self.initial_burn_start is None
            and message.header.frame_id.lstrip("/") == self.global_frame_id
        ):
            self.initial_burn_start = now

    def reference_is_valid(self, now) -> bool:
        """Return whether virtual odometry is fresh and in the expected frame."""
        if self.latest_reference is None or self.last_reference_time is None:
            return False
        age = (now.nanoseconds - self.last_reference_time) * 1.0e-9
        return (
            0.0 <= age <= self.reference_timeout
            and self.latest_reference.header.frame_id.lstrip("/")
            == self.global_frame_id
        )

    def update(self) -> None:
        """Publish the current finite firing or an explicit zero wrench."""
        now = self.get_clock().now()
        if not self.reference_is_valid(now) or self.initial_burn_start is None:
            self.wrench_publisher.publish(Wrench())
            return

        reference = self.latest_reference
        yaw = yaw_from_quaternion(reference.pose.pose.orientation)
        global_velocity_x, global_velocity_y = rotate_body_to_global(
            reference.twist.twist.linear.x,
            reference.twist.twist.linear.y,
            yaw,
        )
        global_force_x, global_force_y, near_boundary = boundary_firing(
            reference.pose.pose.position.x,
            reference.pose.pose.position.y,
            global_velocity_x,
            global_velocity_y,
            self.x_min,
            self.x_max,
            self.y_min,
            self.y_max,
            self.boundary_margin,
            min(self.boundary_force, self.maximum_force),
            self.rebound_speed,
        )

        torque = 0.0
        elapsed = (now - self.initial_burn_start).nanoseconds * 1.0e-9
        if not near_boundary and elapsed <= self.initial_burn_duration:
            global_force_x = self.initial_force_x
            global_force_y = self.initial_force_y
            torque = self.initial_torque

        global_force_x, global_force_y = limit_vector(
            global_force_x, global_force_y, self.maximum_force
        )
        torque = max(-self.maximum_torque, min(self.maximum_torque, torque))
        body_force_x, body_force_y = rotate_global_to_body(
            global_force_x, global_force_y, yaw
        )

        wrench = Wrench()
        wrench.force.x = body_force_x
        wrench.force.y = body_force_y
        wrench.torque.z = torque
        self.wrench_publisher.publish(wrench)


def main(args=None):
    """Run the bounded-search guidance node."""
    rclpy.init(args=args)
    node = BoundingBoxSearchNode()
    try:
        rclpy.spin(node)
    finally:
        node.wrench_publisher.publish(Wrench())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
