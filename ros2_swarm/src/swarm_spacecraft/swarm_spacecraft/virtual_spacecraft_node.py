"""ROS 2 node that renders virtual free-floating dynamics on a chassis."""

import math

from geometry_msgs.msg import Wrench
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger

from .dynamics import PlanarState
from .dynamics import integrate_constant_wrench
from .dynamics import rotate_body_to_inertial
from .dynamics import rotate_inertial_to_body


def yaw_from_quaternion(quaternion) -> float:
    """Extract planar yaw from a geometry_msgs quaternion."""
    sin_yaw = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cos_yaw = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(sin_yaw, cos_yaw)


def quaternion_from_yaw(yaw: float):
    """Return the z and w components of a planar quaternion."""
    return math.sin(0.5 * yaw), math.cos(0.5 * yaw)


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


def namespaced_virtual_frame(namespace: str) -> str:
    """Return a virtual-spacecraft frame unique to the robot namespace."""
    robot_namespace = namespace.strip("/")
    if robot_namespace:
        return f"{robot_namespace}/virtual_spacecraft"
    return "virtual_spacecraft"


def timestamp_is_fresh(
    now_nanoseconds: int,
    update_nanoseconds,
    timeout: float,
) -> bool:
    """Return whether a timestamp exists and is within its timeout."""
    if update_nanoseconds is None:
        return False
    age = (now_nanoseconds - update_nanoseconds) * 1.0e-9
    return 0.0 <= age <= timeout


class VirtualSpacecraftNode(Node):
    """Integrate and publish the state of an ideal planar spacecraft."""

    def __init__(self):
        super().__init__("virtual_spacecraft")
        self.declare_parameter("mass", 5.0)
        self.declare_parameter("yaw_inertia", 0.25)
        self.declare_parameter("timer_frequency", 0.01)
        self.declare_parameter("maximum_force", 2.0)
        self.declare_parameter("maximum_torque", 0.5)
        self.declare_parameter("odometry_timeout", 0.25)
        self.declare_parameter("wrench_timeout", 0.25)
        self.declare_parameter("maximum_time_step", 0.05)
        self.declare_parameter("wrench_in_body_frame", True)
        self.declare_parameter("qos_depth", 10)
        self.declare_parameter("qos_reliability", "RELIABLE")
        self.declare_parameter("qos_history", "KEEP_LAST")

        self.mass = float(self.get_parameter("mass").value)
        self.inertia = float(self.get_parameter("yaw_inertia").value)
        timer_frequency = float(self.get_parameter("timer_frequency").value)
        self.maximum_force = float(self.get_parameter("maximum_force").value)
        self.maximum_torque = float(self.get_parameter("maximum_torque").value)
        self.odometry_timeout = float(
            self.get_parameter("odometry_timeout").value
        )
        self.wrench_timeout = float(self.get_parameter("wrench_timeout").value)
        self.maximum_time_step = float(self.get_parameter("maximum_time_step").value)
        self.wrench_in_body_frame = bool(
            self.get_parameter("wrench_in_body_frame").value
        )
        positive_values = (
            self.mass,
            self.inertia,
            timer_frequency,
            self.maximum_force,
            self.maximum_torque,
            self.odometry_timeout,
            self.wrench_timeout,
            self.maximum_time_step,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in positive_values):
            raise ValueError(
                "physical, rate, timeout, and limit parameters must be positive"
            )
        self.reference = PlanarState()
        self.measured = PlanarState()
        self.measured_frame_id = ""
        self.reference_frame_id = ""
        self.virtual_frame_id = namespaced_virtual_frame(self.get_namespace())
        self.force_x = 0.0
        self.force_y = 0.0
        self.torque = 0.0
        self.initialized = False
        self.last_odometry_time = None
        self.last_wrench_time = None
        self.last_update_time = self.get_clock().now()

        qos_profile = qos_profile_from_parameters(self)
        self.create_subscription(
            Odometry,
            "localization/odom",
            self.odometry_callback,
            qos_profile,
        )
        self.create_subscription(
            Wrench,
            "spacecraft_wrench",
            self.wrench_callback,
            qos_profile,
        )
        self.reference_publisher = self.create_publisher(
            Odometry, "virtual_spacecraft/odom", qos_profile
        )
        self.create_service(Trigger, "virtual_spacecraft/reset", self.reset_callback)
        self.create_timer(timer_frequency, self.update)
        self.get_logger().info("Virtual spacecraft simulator ready")

    def odometry_callback(self, message: Odometry) -> None:
        """Store the latest physical chassis state."""
        self.measured = PlanarState(
            x=message.pose.pose.position.x,
            y=message.pose.pose.position.y,
            yaw=yaw_from_quaternion(message.pose.pose.orientation),
        )
        self.measured_frame_id = message.header.frame_id.lstrip("/")
        self.last_odometry_time = self.get_clock().now().nanoseconds
        if not self.initialized and self.measured_frame_id:
            self.reset_reference()

    def wrench_callback(self, message: Wrench) -> None:
        """Store a bounded virtual force and yaw torque command."""
        force_x = message.force.x
        force_y = message.force.y
        torque = message.torque.z
        if not all(math.isfinite(value) for value in (force_x, force_y, torque)):
            self.get_logger().error("Ignoring non-finite spacecraft wrench")
            return
        magnitude = math.hypot(force_x, force_y)
        if magnitude > self.maximum_force:
            scale = self.maximum_force / magnitude
            force_x *= scale
            force_y *= scale
        self.force_x = force_x
        self.force_y = force_y
        self.torque = max(
            -self.maximum_torque,
            min(self.maximum_torque, torque),
        )
        self.last_wrench_time = self.get_clock().now()

    def reset_callback(self, request, response):
        """Reset the virtual state to the latest physical pose at rest."""
        del request
        if not self.localization_is_fresh():
            response.success = False
            response.message = "No fresh localization odometry received"
            return response
        if not self.measured_frame_id:
            response.success = False
            response.message = "Localization odometry frame_id is empty"
            return response
        self.reset_reference()
        response.success = True
        response.message = "Virtual state reset to physical pose"
        return response

    def reset_reference(self) -> None:
        """Place a stationary virtual spacecraft at the measured pose."""
        self.reference = PlanarState(
            x=self.measured.x,
            y=self.measured.y,
            yaw=self.measured.yaw,
        )
        self.reference_frame_id = self.measured_frame_id
        self.initialized = True
        self.last_update_time = self.get_clock().now()

    def localization_is_fresh(self) -> bool:
        """Return whether physical localization is recent enough to reset."""
        if self.last_odometry_time is None:
            return False
        return timestamp_is_fresh(
            self.get_clock().now().nanoseconds,
            self.last_odometry_time,
            self.odometry_timeout,
        )

    def update(self) -> None:
        """Advance the virtual dynamics and publish the reference state."""
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds * 1.0e-9
        self.last_update_time = now
        if not self.initialized:
            return

        dt = max(0.0, min(dt, self.maximum_time_step))
        force_x = self.force_x
        force_y = self.force_y
        if (
            self.last_wrench_time is None
            or (now - self.last_wrench_time).nanoseconds * 1.0e-9 > self.wrench_timeout
        ):
            force_x = 0.0
            force_y = 0.0
            self.torque = 0.0
        if self.wrench_in_body_frame:
            force_x, force_y = rotate_body_to_inertial(
                force_x, force_y, self.reference.yaw
            )
        self.reference = integrate_constant_wrench(
            self.reference,
            force_x,
            force_y,
            self.torque,
            self.mass,
            self.inertia,
            dt,
        )
        self.publish_reference(now)

    def publish_reference(self, stamp) -> None:
        """Publish the observable virtual reference as odometry."""
        message = Odometry()
        message.header.stamp = stamp.to_msg()
        message.header.frame_id = self.reference_frame_id
        message.child_frame_id = self.virtual_frame_id
        message.pose.pose.position.x = self.reference.x
        message.pose.pose.position.y = self.reference.y
        z, w = quaternion_from_yaw(self.reference.yaw)
        message.pose.pose.orientation.z = z
        message.pose.pose.orientation.w = w
        body_vx, body_vy = rotate_inertial_to_body(
            self.reference.vx, self.reference.vy, self.reference.yaw
        )
        message.twist.twist.linear.x = body_vx
        message.twist.twist.linear.y = body_vy
        message.twist.twist.angular.z = self.reference.yaw_rate
        self.reference_publisher.publish(message)


def main(args=None):
    """Run the virtual spacecraft node."""
    rclpy.init(args=args)
    node = VirtualSpacecraftNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
