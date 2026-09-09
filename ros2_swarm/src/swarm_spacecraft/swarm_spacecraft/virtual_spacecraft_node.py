"""ROS 2 node that renders virtual free-floating dynamics on a chassis."""

from functools import partial
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


BODY_FRAME = "body"
INERTIAL_FRAME = "inertial"
WRENCH_FRAMES = (BODY_FRAME, INERTIAL_FRAME)


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
        self.declare_parameter("wrench_sources", ["bounding_box"])
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
        self.initialized = False
        self.last_odometry_time = None
        self.last_update_time = self.get_clock().now()

        qos_profile = qos_profile_from_parameters(self)
        self.create_subscription(
            Odometry,
            "localization/odom",
            self.odometry_callback,
            qos_profile,
        )
        self.subscribe_wrench_sources(qos_profile)
        self.reference_publisher = self.create_publisher(
            Odometry, "virtual_spacecraft/odom", qos_profile
        )
        # Summed, clamped, inertial-frame wrench actually integrated this step.
        self.applied_wrench_publisher = self.create_publisher(
            Wrench, "virtual_spacecraft/applied_wrench", qos_profile
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

    def subscribe_wrench_sources(self, qos_profile) -> None:
        """Subscribe to every configured wrench contributor.

        Each source declares its own frame because contributors disagree:
        box guidance commands thrust in the body frame, while neighbour
        avoidance is computed from map coordinates.
        """
        source_names = list(self.get_parameter("wrench_sources").value)
        if not source_names:
            raise ValueError("wrench_sources must name at least one contributor")
        if len(set(source_names)) != len(source_names):
            raise ValueError("wrench_sources names must be unique")

        self.wrench_frames = {}
        self.wrench_values = {}
        self.wrench_update_times = {}
        for name in source_names:
            self.declare_parameter(f"wrench_sources.{name}.topic", "")
            self.declare_parameter(f"wrench_sources.{name}.frame", BODY_FRAME)
            topic = str(
                self.get_parameter(f"wrench_sources.{name}.topic").value
            ).strip()
            frame = str(self.get_parameter(f"wrench_sources.{name}.frame").value)
            if not topic:
                raise ValueError(f"wrench source '{name}' requires a topic")
            if frame not in WRENCH_FRAMES:
                raise ValueError(
                    f"wrench source '{name}' frame must be one of {WRENCH_FRAMES}"
                )
            self.wrench_frames[name] = frame
            self.wrench_values[name] = (0.0, 0.0, 0.0)
            self.wrench_update_times[name] = None
            self.create_subscription(
                Wrench,
                topic,
                partial(self.wrench_callback, name),
                qos_profile,
            )
            self.get_logger().info(
                f"Wrench source '{name}' on '{topic}' in the {frame} frame"
            )

    def wrench_callback(self, name: str, message: Wrench) -> None:
        """Store one contributor's latest wrench, unclamped."""
        force_x = message.force.x
        force_y = message.force.y
        torque = message.torque.z
        if not all(math.isfinite(value) for value in (force_x, force_y, torque)):
            self.get_logger().error(
                f"Ignoring non-finite wrench from source '{name}'"
            )
            return
        self.wrench_values[name] = (force_x, force_y, torque)
        self.wrench_update_times[name] = self.get_clock().now().nanoseconds

    def total_wrench(self, now_nanoseconds: int):
        """Sum fresh contributors in the inertial frame, then clamp once."""
        force_x = 0.0
        force_y = 0.0
        torque = 0.0
        for name, frame in self.wrench_frames.items():
            if not timestamp_is_fresh(
                now_nanoseconds,
                self.wrench_update_times[name],
                self.wrench_timeout,
            ):
                continue
            source_x, source_y, source_torque = self.wrench_values[name]
            if frame == BODY_FRAME:
                source_x, source_y = rotate_body_to_inertial(
                    source_x, source_y, self.reference.yaw
                )
            force_x += source_x
            force_y += source_y
            torque += source_torque

        magnitude = math.hypot(force_x, force_y)
        if magnitude > self.maximum_force:
            scale = self.maximum_force / magnitude
            force_x *= scale
            force_y *= scale
        torque = max(-self.maximum_torque, min(self.maximum_torque, torque))
        return force_x, force_y, torque

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
        force_x, force_y, torque = self.total_wrench(now.nanoseconds)
        applied = Wrench()
        applied.force.x = force_x
        applied.force.y = force_y
        applied.torque.z = torque
        self.applied_wrench_publisher.publish(applied)
        self.reference = integrate_constant_wrench(
            self.reference,
            force_x,
            force_y,
            torque,
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
