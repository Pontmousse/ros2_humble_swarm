"""Publish RViz markers for the bounded free-drift search experiment."""

import math

from geometry_msgs.msg import Point, Wrench
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

from .bounding_box_search import qos_profile_from_parameters
from .bounding_box_search import rotate_body_to_global
from .bounding_box_search import yaw_from_quaternion


class BoundingBoxVisualizerNode(Node):
    """Render the search box, spacecraft states, and applied wrench."""

    RECTANGLE_ID = 0
    VIRTUAL_ID = 1
    PHYSICAL_ID = 2
    FORCE_ID = 3
    TORQUE_ARC_ID = 4
    TORQUE_HEAD_ID = 5

    def __init__(self):
        super().__init__("bounding_box_visualizer")
        self.declare_parameter("timer_frequency", 0.05)
        self.declare_parameter("source_timeout", 0.25)
        self.declare_parameter("global_frame_id", "swarm_map")
        self.declare_parameter("x_min", -1.0)
        self.declare_parameter("x_max", 1.0)
        self.declare_parameter("y_min", -1.0)
        self.declare_parameter("y_max", 1.0)
        self.declare_parameter("triangle_size", 0.12)
        self.declare_parameter("maximum_force", 1.0)
        self.declare_parameter("maximum_torque", 0.2)
        self.declare_parameter("force_arrow_max_length", 1.0)
        self.declare_parameter("torque_arrow_max_radius", 0.25)
        self.declare_parameter("show_physical_robot", True)
        self.declare_parameter("qos_depth", 10)
        self.declare_parameter("qos_reliability", "RELIABLE")
        self.declare_parameter("qos_history", "KEEP_LAST")

        self.timer_frequency = float(self.get_parameter("timer_frequency").value)
        self.source_timeout = float(self.get_parameter("source_timeout").value)
        self.global_frame_id = str(
            self.get_parameter("global_frame_id").value
        ).strip("/")
        self.x_min = float(self.get_parameter("x_min").value)
        self.x_max = float(self.get_parameter("x_max").value)
        self.y_min = float(self.get_parameter("y_min").value)
        self.y_max = float(self.get_parameter("y_max").value)
        self.triangle_size = float(self.get_parameter("triangle_size").value)
        self.maximum_force = float(self.get_parameter("maximum_force").value)
        self.maximum_torque = float(self.get_parameter("maximum_torque").value)
        self.force_arrow_max_length = float(
            self.get_parameter("force_arrow_max_length").value
        )
        self.torque_arrow_max_radius = float(
            self.get_parameter("torque_arrow_max_radius").value
        )
        self.show_physical_robot = bool(
            self.get_parameter("show_physical_robot").value
        )
        self.validate_parameters()

        self.virtual_odometry = None
        self.physical_odometry = None
        self.wrench = Wrench()
        self.virtual_update_time = None
        self.physical_update_time = None
        self.wrench_update_time = None

        qos_profile = qos_profile_from_parameters(self)
        self.create_subscription(
            Odometry,
            "virtual_spacecraft/odom",
            self.virtual_callback,
            qos_profile,
        )
        self.create_subscription(
            Odometry,
            "localization/odom",
            self.physical_callback,
            qos_profile,
        )
        self.create_subscription(
            Wrench,
            "spacecraft_wrench",
            self.wrench_callback,
            qos_profile,
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray, "bounding_box_search/markers", qos_profile
        )
        self.create_timer(self.timer_frequency, self.publish_markers)
        self.get_logger().info("Bounding-box search visualizer ready")

    def validate_parameters(self) -> None:
        """Validate visualization geometry and scales."""
        values = (
            self.timer_frequency,
            self.source_timeout,
            self.x_min,
            self.x_max,
            self.y_min,
            self.y_max,
            self.triangle_size,
            self.maximum_force,
            self.maximum_torque,
            self.force_arrow_max_length,
            self.torque_arrow_max_radius,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("Visualizer parameters must be finite")
        if self.x_min >= self.x_max or self.y_min >= self.y_max:
            raise ValueError("Bounding-box minimums must be below maximums")
        positive = (
            self.timer_frequency,
            self.source_timeout,
            self.triangle_size,
            self.maximum_force,
            self.maximum_torque,
            self.force_arrow_max_length,
            self.torque_arrow_max_radius,
        )
        if not all(value > 0.0 for value in positive):
            raise ValueError("Visualizer timing, sizes, and limits must be positive")
        if not self.global_frame_id:
            raise ValueError("global_frame_id must not be empty")

    def virtual_callback(self, message: Odometry) -> None:
        """Store virtual spacecraft odometry for rendering."""
        self.virtual_odometry = message
        self.virtual_update_time = self.get_clock().now().nanoseconds

    def physical_callback(self, message: Odometry) -> None:
        """Store measured physical odometry for the optional overlay."""
        self.physical_odometry = message
        self.physical_update_time = self.get_clock().now().nanoseconds

    def wrench_callback(self, message: Wrench) -> None:
        """Store the most recently commanded body-frame wrench."""
        self.wrench = message
        self.wrench_update_time = self.get_clock().now().nanoseconds

    def source_is_fresh(self, now_nanoseconds: int, update_time) -> bool:
        """Return whether a source receipt timestamp is recent."""
        if update_time is None:
            return False
        age = (now_nanoseconds - update_time) * 1.0e-9
        return 0.0 <= age <= self.source_timeout

    def marker(self, marker_id: int, marker_type: int, stamp) -> Marker:
        """Create a marker with the experiment's shared identity and frame."""
        marker = Marker()
        marker.header.frame_id = self.global_frame_id
        marker.header.stamp = stamp
        marker.ns = "bounding_box_search"
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    @staticmethod
    def point(x: float, y: float, z: float = 0.0) -> Point:
        """Construct a marker point."""
        point = Point()
        point.x = x
        point.y = y
        point.z = z
        return point

    def rectangle_marker(self, stamp) -> Marker:
        """Create the fixed rectangular search-region marker."""
        marker = self.marker(self.RECTANGLE_ID, Marker.LINE_STRIP, stamp)
        marker.scale.x = 0.025
        marker.color.r = 0.25
        marker.color.g = 0.55
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.points = [
            self.point(self.x_min, self.y_min),
            self.point(self.x_max, self.y_min),
            self.point(self.x_max, self.y_max),
            self.point(self.x_min, self.y_max),
            self.point(self.x_min, self.y_min),
        ]
        return marker

    def triangle_marker(
        self,
        marker_id: int,
        odometry: Odometry,
        stamp,
        red: float,
        green: float,
        blue: float,
        alpha: float,
    ) -> Marker:
        """Create a heading triangle from planar odometry."""
        marker = self.marker(marker_id, Marker.LINE_STRIP, stamp)
        marker.scale.x = 0.035
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker.color.a = alpha

        x = odometry.pose.pose.position.x
        y = odometry.pose.pose.position.y
        yaw = yaw_from_quaternion(odometry.pose.pose.orientation)
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        local_points = (
            (self.triangle_size, 0.0),
            (-0.65 * self.triangle_size, 0.6 * self.triangle_size),
            (-0.65 * self.triangle_size, -0.6 * self.triangle_size),
            (self.triangle_size, 0.0),
        )
        marker.points = [
            self.point(
                x + cosine * local_x - sine * local_y,
                y + sine * local_x + cosine * local_y,
                0.03,
            )
            for local_x, local_y in local_points
        ]
        return marker

    def force_marker(self, stamp, odometry: Odometry, force_x, force_y) -> Marker:
        """Create a red force arrow with magnitude-proportional length."""
        marker = self.marker(self.FORCE_ID, Marker.ARROW, stamp)
        marker.scale.x = 0.025
        marker.scale.y = 0.08
        marker.scale.z = 0.10
        marker.color.r = 1.0
        marker.color.g = 0.05
        marker.color.b = 0.05
        magnitude = math.hypot(force_x, force_y)
        marker.color.a = 1.0 if magnitude > 0.0 else 0.0
        x = odometry.pose.pose.position.x
        y = odometry.pose.pose.position.y
        scale = self.force_arrow_max_length / self.maximum_force
        marker.points = [
            self.point(x, y, 0.04),
            self.point(x + scale * force_x, y + scale * force_y, 0.04),
        ]
        return marker

    def torque_markers(self, stamp, odometry: Odometry, torque: float):
        """Create a curved orange torque arrow and arrowhead."""
        arc = self.marker(self.TORQUE_ARC_ID, Marker.LINE_STRIP, stamp)
        head = self.marker(self.TORQUE_HEAD_ID, Marker.LINE_LIST, stamp)
        arc.scale.x = 0.025
        head.scale.x = 0.025
        for marker in (arc, head):
            marker.color.r = 1.0
            marker.color.g = 0.55
            marker.color.b = 0.0

        magnitude = abs(torque)
        alpha = 1.0 if magnitude > 0.0 else 0.0
        arc.color.a = alpha
        head.color.a = alpha
        x = odometry.pose.pose.position.x
        y = odometry.pose.pose.position.y
        yaw = yaw_from_quaternion(odometry.pose.pose.orientation)
        radius = self.torque_arrow_max_radius * magnitude / self.maximum_torque
        direction = 1.0 if torque >= 0.0 else -1.0
        span = 1.4 * math.pi
        start_angle = yaw - direction * 0.7 * math.pi
        point_count = 20
        arc.points = [
            self.point(
                x + radius * math.cos(start_angle + direction * span * i / point_count),
                y + radius * math.sin(start_angle + direction * span * i / point_count),
                0.05,
            )
            for i in range(point_count + 1)
        ]

        tip_angle = start_angle + direction * span
        tip = arc.points[-1]
        tangent_x = direction * -math.sin(tip_angle)
        tangent_y = direction * math.cos(tip_angle)
        normal_x = -tangent_y
        normal_y = tangent_x
        head_length = min(0.08, 0.5 * radius)
        left = self.point(
            tip.x - head_length * tangent_x + 0.5 * head_length * normal_x,
            tip.y - head_length * tangent_y + 0.5 * head_length * normal_y,
            tip.z,
        )
        right = self.point(
            tip.x - head_length * tangent_x - 0.5 * head_length * normal_x,
            tip.y - head_length * tangent_y - 0.5 * head_length * normal_y,
            tip.z,
        )
        head.points = [tip, left, tip, right]
        return arc, head

    def delete_marker(self, marker_id: int, stamp) -> Marker:
        """Delete a previously published dynamic marker."""
        marker = self.marker(marker_id, Marker.LINE_STRIP, stamp)
        marker.action = Marker.DELETE
        return marker

    def publish_markers(self) -> None:
        """Publish a complete visualization snapshot."""
        now = self.get_clock().now()
        stamp = now.to_msg()
        markers = [self.rectangle_marker(stamp)]
        virtual_valid = (
            self.source_is_fresh(now.nanoseconds, self.virtual_update_time)
            and self.virtual_odometry is not None
            and self.virtual_odometry.header.frame_id.lstrip("/")
            == self.global_frame_id
        )
        if not virtual_valid:
            markers.extend(
                self.delete_marker(marker_id, stamp)
                for marker_id in (
                    self.VIRTUAL_ID,
                    self.FORCE_ID,
                    self.TORQUE_ARC_ID,
                    self.TORQUE_HEAD_ID,
                )
            )
        else:
            markers.append(
                self.triangle_marker(
                    self.VIRTUAL_ID,
                    self.virtual_odometry,
                    stamp,
                    1.0,
                    1.0,
                    1.0,
                    1.0,
                )
            )
            wrench_valid = self.source_is_fresh(
                now.nanoseconds, self.wrench_update_time
            )
            body_force_x = self.wrench.force.x if wrench_valid else 0.0
            body_force_y = self.wrench.force.y if wrench_valid else 0.0
            torque = self.wrench.torque.z if wrench_valid else 0.0
            yaw = yaw_from_quaternion(
                self.virtual_odometry.pose.pose.orientation
            )
            force_x, force_y = rotate_body_to_global(
                body_force_x, body_force_y, yaw
            )
            markers.append(
                self.force_marker(
                    stamp, self.virtual_odometry, force_x, force_y
                )
            )
            markers.extend(
                self.torque_markers(stamp, self.virtual_odometry, torque)
            )

        physical_valid = (
            self.show_physical_robot
            and self.source_is_fresh(now.nanoseconds, self.physical_update_time)
            and self.physical_odometry is not None
            and self.physical_odometry.header.frame_id.lstrip("/")
            == self.global_frame_id
        )
        if physical_valid:
            markers.append(
                self.triangle_marker(
                    self.PHYSICAL_ID,
                    self.physical_odometry,
                    stamp,
                    0.1,
                    1.0,
                    0.9,
                    0.45,
                )
            )
        else:
            markers.append(self.delete_marker(self.PHYSICAL_ID, stamp))

        message = MarkerArray()
        message.markers = markers
        self.marker_publisher.publish(message)


def main(args=None):
    """Run the bounded-search marker visualizer."""
    rclpy.init(args=args)
    node = BoundingBoxVisualizerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
