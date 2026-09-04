"""ROS 2 node that renders virtual free-floating dynamics on a chassis."""

import math

from geometry_msgs.msg import Twist, Wrench
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger

from .dynamics import PlanarState
from .dynamics import integrate_constant_wrench
from .dynamics import rotate_body_to_inertial
from .dynamics import rotate_inertial_to_body
from .dynamics import tracking_command


def yaw_from_quaternion(quaternion) -> float:
    """Extract planar yaw from a geometry_msgs quaternion."""
    sin_yaw = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cos_yaw = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(sin_yaw, cos_yaw)


def quaternion_from_yaw(yaw: float):
    """Return the z and w components of a planar quaternion."""
    return math.sin(0.5 * yaw), math.cos(0.5 * yaw)


class VirtualSpacecraftNode(Node):
    """Integrate an ideal spacecraft and make the robot shadow its state."""

    def __init__(self):
        super().__init__("virtual_spacecraft")
        self.declare_parameter("mass", 5.0)
        self.declare_parameter("yaw_inertia", 0.25)
        self.declare_parameter("update_rate", 100.0)
        self.declare_parameter("position_gain", 1.0)
        self.declare_parameter("yaw_gain", 2.0)
        self.declare_parameter("maximum_speed", 0.5)
        self.declare_parameter("maximum_yaw_rate", 1.0)
        self.declare_parameter("maximum_force", 2.0)
        self.declare_parameter("maximum_torque", 0.5)
        self.declare_parameter("odometry_timeout", 0.25)
        self.declare_parameter("wrench_timeout", 0.25)
        self.declare_parameter("maximum_time_step", 0.05)
        self.declare_parameter("wrench_in_body_frame", True)

        self.mass = float(self.get_parameter("mass").value)
        self.inertia = float(self.get_parameter("yaw_inertia").value)
        update_rate = float(self.get_parameter("update_rate").value)
        self.position_gain = float(self.get_parameter("position_gain").value)
        self.yaw_gain = float(self.get_parameter("yaw_gain").value)
        self.maximum_speed = float(self.get_parameter("maximum_speed").value)
        self.maximum_yaw_rate = float(self.get_parameter("maximum_yaw_rate").value)
        self.maximum_force = float(self.get_parameter("maximum_force").value)
        self.maximum_torque = float(self.get_parameter("maximum_torque").value)
        self.odometry_timeout = float(self.get_parameter("odometry_timeout").value)
        self.wrench_timeout = float(self.get_parameter("wrench_timeout").value)
        self.maximum_time_step = float(self.get_parameter("maximum_time_step").value)
        self.wrench_in_body_frame = bool(
            self.get_parameter("wrench_in_body_frame").value
        )
        positive_values = (
            self.mass,
            self.inertia,
            update_rate,
            self.maximum_speed,
            self.maximum_yaw_rate,
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
        if not math.isfinite(self.position_gain) or self.position_gain < 0.0:
            raise ValueError("position_gain must be finite and non-negative")
        if not math.isfinite(self.yaw_gain) or self.yaw_gain < 0.0:
            raise ValueError("yaw_gain must be finite and non-negative")

        self.reference = PlanarState()
        self.measured = PlanarState()
        self.force_x = 0.0
        self.force_y = 0.0
        self.torque = 0.0
        self.initialized = False
        self.enabled = False
        self.last_odometry_time = None
        self.last_wrench_time = None
        self.last_update_time = self.get_clock().now()
        self.saturation_count = 0

        self.create_subscription(Odometry, "odom", self.odometry_callback, 10)
        self.create_subscription(Wrench, "spacecraft_wrench", self.wrench_callback, 10)
        self.command_publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.reference_publisher = self.create_publisher(
            Odometry, "virtual_spacecraft/odom", 10
        )
        self.create_service(SetBool, "virtual_spacecraft/enable", self.enable_callback)
        self.create_service(Trigger, "virtual_spacecraft/reset", self.reset_callback)
        self.create_timer(1.0 / update_rate, self.update)
        self.get_logger().info(
            "Virtual spacecraft ready; reset from odometry, then enable explicitly."
        )

    def odometry_callback(self, message: Odometry) -> None:
        """Store the latest physical chassis state."""
        self.measured = PlanarState(
            x=message.pose.pose.position.x,
            y=message.pose.pose.position.y,
            yaw=yaw_from_quaternion(message.pose.pose.orientation),
            vx=message.twist.twist.linear.x,
            vy=message.twist.twist.linear.y,
            yaw_rate=message.twist.twist.angular.z,
        )
        self.last_odometry_time = self.get_clock().now()
        if not self.initialized:
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

    def enable_callback(self, request, response):
        """Enable or safely pause physical motion rendering."""
        if request.data and not self.initialized:
            response.success = False
            response.message = "Cannot enable before receiving odometry"
            return response
        self.enabled = request.data
        if not self.enabled:
            self.publish_stop()
        response.success = True
        response.message = "enabled" if self.enabled else "paused"
        return response

    def reset_callback(self, request, response):
        """Reset the virtual state to the latest physical pose at rest."""
        del request
        if self.last_odometry_time is None:
            response.success = False
            response.message = "No odometry received"
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
        self.initialized = True
        self.enabled = False
        self.last_update_time = self.get_clock().now()

    def update(self) -> None:
        """Advance virtual dynamics and publish reference and safe commands."""
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds * 1.0e-9
        self.last_update_time = now
        if not self.initialized:
            self.publish_stop()
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

        odometry_age = (now - self.last_odometry_time).nanoseconds * 1.0e-9
        if not self.enabled or odometry_age > self.odometry_timeout:
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

    def publish_reference(self, stamp) -> None:
        """Publish the observable virtual reference as odometry."""
        message = Odometry()
        message.header.stamp = stamp.to_msg()
        message.header.frame_id = "map"
        message.child_frame_id = "virtual_spacecraft"
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

    def publish_stop(self) -> None:
        """Publish a zero velocity command."""
        self.command_publisher.publish(Twist())


def main(args=None):
    """Run the virtual spacecraft node."""
    rclpy.init(args=args)
    node = VirtualSpacecraftNode()
    try:
        rclpy.spin(node)
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
