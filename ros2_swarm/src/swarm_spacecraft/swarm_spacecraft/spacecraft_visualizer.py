"""Centralized per-robot RViz visualization for the virtual-spacecraft swarm.

Supersedes swarm_controller/bounding_box_visualizer.py. Everything that node
drew is reproduced here, in a single MarkerArray per robot:

  * the bounded search region (from the shared box params),
  * the virtual spacecraft and the physical robot as heading triangles,
  * the applied wrench as a force arrow and a curved torque arrow,
  * the raw Marvelmind fix and the chassis / Marvelmind IMU headings,

plus the perception overlays this node adds:

  * detected landmarks and targets, drawn as point clouds,
  * their normals (orientations), drawn as short sticks.

Frame discipline: the only yaw this node ever reads comes from odometry
QUATERNIONS, never from StatePos.theta (which is in degrees -- the standing trap
in this stack). Landmarks and targets arrive as BODY-frame Markers in
CENTIMETRES (the convention target_bridge, position_guidance3 and
cp_proportional_controller all share); they are lifted into the global frame
with the physical robot's own pose from localization/odom, so they are only
drawn while that pose is fresh and actually in the global frame.

Lives in swarm_spacecraft (not swarm_controller) and is self-contained: the two
helpers it needs are inlined rather than imported from bounding_box_search, so
the visualization has no build dependency on the controller package.
"""

import math

from geometry_msgs.msg import Point, Wrench
try:
    from marvelmind_ros2_msgs.msg import HedgePositionAddressed
except ImportError:
    # Absent on hardware-free dev machines (and under VIRTUAL_PLANT there is no
    # Marvelmind to publish mm_pos_unf anyway); the raw-fix overlay is skipped.
    HedgePositionAddressed = None
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

from swarm_interfaces.msg import Markers


def yaw_from_quaternion(quaternion) -> float:
    """Extract planar yaw from a geometry_msgs quaternion."""
    sin_yaw = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cos_yaw = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(sin_yaw, cos_yaw)


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


class SpacecraftVisualizerNode(Node):
    """Render the search box, spacecraft states, wrench, and perception."""

    RECTANGLE_ID = 0
    VIRTUAL_ID = 1
    PHYSICAL_ID = 2
    FORCE_ID = 3
    TORQUE_ARC_ID = 4
    TORQUE_HEAD_ID = 5
    INDOOR_GPS_ID = 6
    IMU_HEADING_ID = 7
    MM_IMU_HEADING_ID = 8
    VIRTUAL_LABEL_ID = 9
    LANDMARKS_ID = 10
    LANDMARK_NORMALS_ID = 11
    TARGETS_ID = 12
    TARGET_NORMALS_ID = 13
    PHYSICAL_LABEL_ID = 14
    # World-frame lab infrastructure (static, drawn in a shared "world_anchors" ns).
    AXIS_X_ID = 15
    AXIS_Y_ID = 16
    AXIS_X_LABEL_ID = 17
    AXIS_Y_LABEL_ID = 18
    ANCHOR_SPHERE_BASE = 20   # 20, 21, 22
    ANCHOR_LABEL_BASE = 23    # 23, 24, 25
    ARENA_ID = 26

    # RGB triples for the perception overlays.
    LANDMARK_COLOR = (0.2, 1.0, 0.3)
    TARGET_COLOR = (1.0, 0.2, 0.2)
    TARGET_NORMAL_COLOR = (1.0, 0.55, 0.0)

    # Marvelmind anchors (address, x, y) and the arena corner loop, in swarm_map.
    # Hand-transcribed lab layout; see plot_rm2_region.py / virtual_spacecraft.yaml.
    WORLD_ANCHORS = (("13", 0.0, 0.0), ("15", -5.610, 0.042), ("14", 0.360, -5.022))
    ARENA_CORNERS = ((0.0, 0.0), (-5.610, 0.042), (-5.6, -5.0), (0.360, -5.022))

    def __init__(self):
        super().__init__("spacecraft_visualizer")
        # Robot number from the namespace (e.g. "/RM6" -> "6") for the V#/R# labels.
        ns = self.get_namespace().strip("/")
        self.robot_label = "".join(c for c in ns if c.isdigit()) or ns
        self.declare_parameter("timer_frequency", 0.05)
        self.declare_parameter("source_timeout", 0.25)
        self.declare_parameter("global_frame_id", "swarm_map")
        self.declare_parameter("x_min", -1.0)
        self.declare_parameter("x_max", 1.0)
        self.declare_parameter("y_min", -1.0)
        self.declare_parameter("y_max", 1.0)
        self.declare_parameter("triangle_size", 0.12)
        # Real RoboMaster footprint (m): 0.40 long x 0.24 wide.
        self.declare_parameter("robot_length", 0.40)
        self.declare_parameter("robot_width", 0.24)
        self.declare_parameter("maximum_force", 1.0)
        self.declare_parameter("maximum_torque", 0.2)
        self.declare_parameter("force_arrow_max_length", 1.0)
        self.declare_parameter("torque_arrow_max_radius", 0.25)
        self.declare_parameter("show_physical_robot", True)
        self.declare_parameter("show_indoor_gps", True)
        self.declare_parameter("indoor_gps_timeout", 1.0)
        self.declare_parameter("indoor_gps_marker_size", 0.08)
        self.declare_parameter("show_imu_orientations", True)
        self.declare_parameter("imu_orientation_timeout", 0.5)
        self.declare_parameter("imu_arrow_length", 0.20)
        self.declare_parameter("mm_imu_arrow_length", 0.28)
        # Perception overlays.
        self.declare_parameter("show_landmarks", True)
        self.declare_parameter("show_targets", True)
        self.declare_parameter("show_marker_normals", True)
        # Markers come at the detector rate (~5-10 Hz), slower than odom, so they
        # get their own, looser freshness window.
        self.declare_parameter("perception_timeout", 0.5)
        self.declare_parameter("marker_point_size", 0.10)
        self.declare_parameter("normal_arrow_length", 0.20)
        # Static lab-frame overlay: X/Y axes + Marvelmind anchors + arena outline.
        self.declare_parameter("show_world_anchors", True)
        self.declare_parameter("world_axis_length", 1.0)
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
        self.robot_length = float(self.get_parameter("robot_length").value)
        self.robot_width = float(self.get_parameter("robot_width").value)
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
        self.show_indoor_gps = bool(self.get_parameter("show_indoor_gps").value)
        self.indoor_gps_timeout = float(
            self.get_parameter("indoor_gps_timeout").value
        )
        self.indoor_gps_marker_size = float(
            self.get_parameter("indoor_gps_marker_size").value
        )
        self.show_imu_orientations = bool(
            self.get_parameter("show_imu_orientations").value
        )
        self.imu_orientation_timeout = float(
            self.get_parameter("imu_orientation_timeout").value
        )
        self.imu_arrow_length = float(self.get_parameter("imu_arrow_length").value)
        self.mm_imu_arrow_length = float(
            self.get_parameter("mm_imu_arrow_length").value
        )
        self.show_landmarks = bool(self.get_parameter("show_landmarks").value)
        self.show_targets = bool(self.get_parameter("show_targets").value)
        self.show_marker_normals = bool(
            self.get_parameter("show_marker_normals").value
        )
        self.perception_timeout = float(
            self.get_parameter("perception_timeout").value
        )
        self.marker_point_size = float(
            self.get_parameter("marker_point_size").value
        )
        self.normal_arrow_length = float(
            self.get_parameter("normal_arrow_length").value
        )
        self.show_world_anchors = bool(
            self.get_parameter("show_world_anchors").value
        )
        self.world_axis_length = float(
            self.get_parameter("world_axis_length").value
        )
        self.validate_parameters()

        self.virtual_odometry = None
        self.physical_odometry = None
        self.indoor_gps_position = None
        self.indoor_gps_update_time = None
        self.imu_yaw = None
        self.imu_update_time = None
        self.mm_imu_yaw = None
        self.mm_imu_update_time = None
        self.wrench = Wrench()
        self.virtual_update_time = None
        self.physical_update_time = None
        self.wrench_update_time = None
        self.landmarks_msg = None
        self.landmarks_update_time = None
        self.targets_msg = None
        self.targets_update_time = None

        qos_profile = qos_profile_from_parameters(self)
        # Perception streams are read BEST_EFFORT so the subscription connects to
        # either a RELIABLE writer (aruco_filter) or a BEST_EFFORT one; a
        # RELIABLE reader would silently fail to match a BEST_EFFORT publisher.
        perception_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
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
            "virtual_spacecraft/applied_wrench",
            self.wrench_callback,
            qos_profile,
        )
        if HedgePositionAddressed is not None:
            self.create_subscription(
                HedgePositionAddressed,
                "mm_pos_unf",
                self.indoor_gps_callback,
                qos_profile,
            )
        if self.show_imu_orientations:
            self.create_subscription(
                Odometry,
                "localization/imu_odom",
                self.imu_orientation_callback,
                qos_profile,
            )
            self.create_subscription(
                Odometry,
                "localization/mm_imu_odom",
                self.mm_imu_orientation_callback,
                qos_profile,
            )
        if self.show_landmarks:
            self.create_subscription(
                Markers, "landmarks", self.landmarks_callback, perception_qos
            )
        if self.show_targets:
            self.create_subscription(
                Markers, "targets", self.targets_callback, perception_qos
            )
        self.marker_publisher = self.create_publisher(
            MarkerArray, "spacecraft_visualizer/markers", qos_profile
        )
        self.create_timer(self.timer_frequency, self.publish_markers)
        self.get_logger().info("Spacecraft visualizer ready")

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
            self.indoor_gps_timeout,
            self.indoor_gps_marker_size,
            self.imu_orientation_timeout,
            self.imu_arrow_length,
            self.mm_imu_arrow_length,
            self.perception_timeout,
            self.marker_point_size,
            self.normal_arrow_length,
            self.world_axis_length,
            self.robot_length,
            self.robot_width,
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
            self.indoor_gps_timeout,
            self.indoor_gps_marker_size,
            self.imu_orientation_timeout,
            self.imu_arrow_length,
            self.mm_imu_arrow_length,
            self.perception_timeout,
            self.marker_point_size,
            self.normal_arrow_length,
            self.world_axis_length,
            self.robot_length,
            self.robot_width,
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
        """Store measured physical odometry for the overlay and perception lift."""
        self.physical_odometry = message
        self.physical_update_time = self.get_clock().now().nanoseconds

    def indoor_gps_callback(self, message: HedgePositionAddressed) -> None:
        """Store the raw Marvelmind hedge position for the overlay."""
        self.indoor_gps_position = (float(message.x_m), float(message.y_m))
        self.indoor_gps_update_time = self.get_clock().now().nanoseconds

    def landmarks_callback(self, message: Markers) -> None:
        """Store the latest landmark detections (body-frame cm)."""
        self.landmarks_msg = message
        self.landmarks_update_time = self.get_clock().now().nanoseconds

    def targets_callback(self, message: Markers) -> None:
        """Store the latest target detections (body-frame cm)."""
        self.targets_msg = message
        self.targets_update_time = self.get_clock().now().nanoseconds

    def orientation_yaw(self, message: Odometry):
        """Extract a global-frame yaw from a comparison odometry source."""
        if message.header.frame_id.lstrip("/") != self.global_frame_id:
            return None
        return yaw_from_quaternion(message.pose.pose.orientation)

    def imu_orientation_callback(self, message: Odometry) -> None:
        """Store the chassis IMU yaw; the position is supplied by the beacon."""
        yaw = self.orientation_yaw(message)
        if yaw is None:
            return
        self.imu_yaw = yaw
        self.imu_update_time = self.get_clock().now().nanoseconds

    def mm_imu_orientation_callback(self, message: Odometry) -> None:
        """Store the Marvelmind IMU yaw; the position is supplied by the beacon."""
        yaw = self.orientation_yaw(message)
        if yaw is None:
            return
        self.mm_imu_yaw = yaw
        self.mm_imu_update_time = self.get_clock().now().nanoseconds

    def wrench_callback(self, message: Wrench) -> None:
        """Store the most recently commanded body-frame wrench."""
        self.wrench = message
        self.wrench_update_time = self.get_clock().now().nanoseconds

    def source_is_fresh(
        self, now_nanoseconds: int, update_time, timeout: float = None
    ) -> bool:
        """Return whether a source receipt timestamp is recent."""
        if update_time is None:
            return False
        age = (now_nanoseconds - update_time) * 1.0e-9
        return 0.0 <= age <= (self.source_timeout if timeout is None else timeout)

    def marker(self, marker_id: int, marker_type: int, stamp) -> Marker:
        """Create a marker with the experiment's shared identity and frame."""
        marker = Marker()
        marker.header.frame_id = self.global_frame_id
        marker.header.stamp = stamp
        marker.ns = f"spacecraft_viz{self.get_namespace()}"
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def world_marker(self, marker_id: int, marker_type: int, stamp) -> Marker:
        """Create a marker in the shared, robot-independent world ns.

        Same as marker() but ns="world_anchors" so every robot's visualizer
        writes the static lab overlay into one ns+id slot instead of a
        per-namespace one.
        """
        marker = self.marker(marker_id, marker_type, stamp)
        marker.ns = "world_anchors"
        return marker

    def build_world_markers(self, stamp):
        """Static lab-frame overlay: X/Y axes, numbered anchors, arena outline."""
        length = self.world_axis_length
        markers = []

        axes = (
            (self.AXIS_X_ID, (length, 0.0), (1.0, 0.1, 0.1)),
            (self.AXIS_Y_ID, (0.0, length), (0.1, 1.0, 0.1)),
        )
        for marker_id, (tip_x, tip_y), (red, green, blue) in axes:
            axis = self.world_marker(marker_id, Marker.ARROW, stamp)
            axis.scale.x = 0.03
            axis.scale.y = 0.09
            axis.scale.z = 0.12
            axis.color.r, axis.color.g, axis.color.b, axis.color.a = red, green, blue, 1.0
            axis.points = [self.point(0.0, 0.0, 0.02), self.point(tip_x, tip_y, 0.02)]
            markers.append(axis)

        labels = (
            (self.AXIS_X_LABEL_ID, "X", length, 0.0),
            (self.AXIS_Y_LABEL_ID, "Y", 0.0, length),
        )
        for marker_id, text, lx, ly in labels:
            label = self.world_marker(marker_id, Marker.TEXT_VIEW_FACING, stamp)
            label.text = text
            label.scale.z = 0.25
            label.color.r = label.color.g = label.color.b = 1.0
            label.color.a = 1.0
            label.pose.position.x = lx
            label.pose.position.y = ly
            label.pose.position.z = 0.15
            markers.append(label)

        for offset, (address, ax, ay) in enumerate(self.WORLD_ANCHORS):
            sphere = self.world_marker(
                self.ANCHOR_SPHERE_BASE + offset, Marker.CYLINDER, stamp)
            sphere.scale.x = sphere.scale.y = 0.12
            sphere.scale.z = 0.05
            sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = 0.1, 0.8, 1.0, 1.0
            sphere.pose.position.x = ax
            sphere.pose.position.y = ay
            sphere.pose.position.z = 0.025
            markers.append(sphere)

            tag = self.world_marker(
                self.ANCHOR_LABEL_BASE + offset, Marker.TEXT_VIEW_FACING, stamp)
            tag.text = address
            tag.scale.z = 0.3
            tag.color.r, tag.color.g, tag.color.b, tag.color.a = 1.0, 0.85, 0.1, 1.0
            tag.pose.position.x = ax
            tag.pose.position.y = ay
            tag.pose.position.z = 0.2
            markers.append(tag)

        arena = self.world_marker(self.ARENA_ID, Marker.LINE_STRIP, stamp)
        arena.scale.x = 0.02
        arena.color.r = arena.color.g = arena.color.b = 0.6
        arena.color.a = 1.0
        arena.points = [self.point(cx, cy) for cx, cy in self.ARENA_CORNERS]
        arena.points.append(self.point(*self.ARENA_CORNERS[0]))
        markers.append(arena)

        return markers

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
        half_length = 0.5 * self.robot_length
        half_width = 0.5 * self.robot_width
        local_points = (
            (half_length, 0.0),
            (-half_length, half_width),
            (-half_length, -half_width),
            (half_length, 0.0),
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

    def virtual_label_marker(self, odometry: Odometry, stamp) -> Marker:
        """Create a 'V<id>' text label at the virtual spacecraft, to tell it apart
        from the physical robot triangle."""
        marker = self.marker(self.VIRTUAL_LABEL_ID, Marker.TEXT_VIEW_FACING, stamp)
        marker.text = f"V{self.robot_label}"
        marker.scale.z = self.triangle_size
        marker.color.r = marker.color.g = marker.color.b = 1.0
        marker.color.a = 1.0
        marker.pose.position.x = odometry.pose.pose.position.x
        marker.pose.position.y = odometry.pose.pose.position.y
        marker.pose.position.z = 0.12
        return marker

    def physical_label_marker(self, odometry: Odometry, stamp) -> Marker:
        """Create a white 'R<id>' text label at the physical robot/plant triangle."""
        marker = self.marker(self.PHYSICAL_LABEL_ID, Marker.TEXT_VIEW_FACING, stamp)
        marker.text = f"R{self.robot_label}"
        marker.scale.z = self.triangle_size
        marker.color.r = marker.color.g = marker.color.b = 1.0
        marker.color.a = 1.0
        marker.pose.position.x = odometry.pose.pose.position.x
        marker.pose.position.y = odometry.pose.pose.position.y
        marker.pose.position.z = 0.10
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

    def indoor_gps_marker(self, stamp) -> Marker:
        """Create a position-only sphere at the raw Marvelmind fix."""
        marker = self.marker(self.INDOOR_GPS_ID, Marker.SPHERE, stamp)
        marker.scale.x = self.indoor_gps_marker_size
        marker.scale.y = self.indoor_gps_marker_size
        marker.scale.z = self.indoor_gps_marker_size
        marker.color.r = 1.0
        marker.color.g = 0.9
        marker.color.b = 0.1
        marker.color.a = 0.8
        marker.pose.position.x = self.indoor_gps_position[0]
        marker.pose.position.y = self.indoor_gps_position[1]
        marker.pose.position.z = 0.02
        return marker

    def heading_arrow_marker(
        self,
        marker_id: int,
        yaw: float,
        length: float,
        red: float,
        green: float,
        blue: float,
        stamp,
    ) -> Marker:
        """Create a heading arrow rooted at the indoor-GPS position."""
        marker = self.marker(marker_id, Marker.ARROW, stamp)
        marker.scale.x = 0.02
        marker.scale.y = 0.06
        marker.scale.z = 0.08
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker.color.a = 1.0
        x, y = self.indoor_gps_position
        marker.points = [
            self.point(x, y, 0.06),
            self.point(
                x + length * math.cos(yaw),
                y + length * math.sin(yaw),
                0.06,
            ),
        ]
        return marker

    def perception_markers(
        self, stamp, source: Markers, px, py, yaw, points_id, normals_id,
        point_color, normal_color,
    ):
        """Build (points, normals) markers for one perception stream.

        source.markers are BODY-frame centimetres and source.markers_normals are
        BODY-frame unit directions; both are rotated into the global frame by the
        physical robot's pose (px, py, yaw). Points collapse into one SPHERE_LIST
        and normals into one LINE_LIST so a variable detection count needs no
        per-marker id bookkeeping -- only the two fixed ids, deleted when empty.
        """
        cosine = math.cos(yaw)
        sine = math.sin(yaw)

        points = self.marker(points_id, Marker.SPHERE_LIST, stamp)
        points.scale.x = self.marker_point_size
        points.scale.y = self.marker_point_size
        points.scale.z = self.marker_point_size
        points.color.r, points.color.g, points.color.b = point_color
        points.color.a = 1.0

        normals = self.marker(normals_id, Marker.LINE_LIST, stamp)
        normals.scale.x = 0.015
        normals.color.r, normals.color.g, normals.color.b = normal_color
        normals.color.a = 1.0

        # markers_normals is optional and may be shorter/empty; only pair a
        # normal with a point when the arrays line up.
        have_normals = (
            self.show_marker_normals
            and len(source.markers_normals) == len(source.markers)
        )
        for i, point in enumerate(source.markers):
            body_x = point.x / 100.0
            body_y = point.y / 100.0
            world_x = px + cosine * body_x - sine * body_y
            world_y = py + sine * body_x + cosine * body_y
            points.points.append(self.point(world_x, world_y, 0.03))

            if have_normals:
                normal = source.markers_normals[i]
                magnitude = math.hypot(normal.x, normal.y)
                if magnitude <= 1.0e-6:
                    continue
                unit_x = -normal.y / magnitude
                unit_y = normal.x / magnitude
                global_x = cosine * unit_x - sine * unit_y
                global_y = sine * unit_x + cosine * unit_y
                normals.points.append(self.point(world_x, world_y, 0.03))
                normals.points.append(
                    self.point(
                        world_x + self.normal_arrow_length * global_x,
                        world_y + self.normal_arrow_length * global_y,
                        0.03,
                    )
                )
        return points, normals

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
                    self.VIRTUAL_LABEL_ID,
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
            markers.append(
                self.virtual_label_marker(self.virtual_odometry, stamp)
            )
            wrench_valid = self.source_is_fresh(
                now.nanoseconds, self.wrench_update_time
            )
            # Already the inertial-frame sum of every contributor, so it is
            # drawn as received rather than rotated out of the body frame.
            force_x = self.wrench.force.x if wrench_valid else 0.0
            force_y = self.wrench.force.y if wrench_valid else 0.0
            torque = self.wrench.torque.z if wrench_valid else 0.0
            markers.append(
                self.force_marker(
                    stamp, self.virtual_odometry, force_x, force_y
                )
            )
            markers.extend(
                self.torque_markers(stamp, self.virtual_odometry, torque)
            )

        # Physical pose: used both for the optional triangle and to lift the
        # perception overlays into the global frame, so its validity is computed
        # independently of show_physical_robot.
        physical_pose_valid = (
            self.source_is_fresh(now.nanoseconds, self.physical_update_time)
            and self.physical_odometry is not None
            and self.physical_odometry.header.frame_id.lstrip("/")
            == self.global_frame_id
        )
        if self.show_physical_robot and physical_pose_valid:
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
            markers.append(
                self.physical_label_marker(self.physical_odometry, stamp)
            )
        else:
            markers.append(self.delete_marker(self.PHYSICAL_ID, stamp))
            markers.append(self.delete_marker(self.PHYSICAL_LABEL_ID, stamp))

        indoor_gps_valid = (
            self.show_indoor_gps
            and self.indoor_gps_position is not None
            and self.source_is_fresh(
                now.nanoseconds,
                self.indoor_gps_update_time,
                self.indoor_gps_timeout,
            )
        )
        if indoor_gps_valid:
            markers.append(self.indoor_gps_marker(stamp))
        else:
            markers.append(self.delete_marker(self.INDOOR_GPS_ID, stamp))

        # Both arrows are anchored on the beacon position, so they are only
        # meaningful while that anchor is fresh.
        headings = (
            (
                self.IMU_HEADING_ID,
                self.imu_yaw,
                self.imu_update_time,
                self.imu_arrow_length,
                (1.0, 0.1, 0.9),
            ),
            (
                self.MM_IMU_HEADING_ID,
                self.mm_imu_yaw,
                self.mm_imu_update_time,
                self.mm_imu_arrow_length,
                (0.3, 1.0, 0.3),
            ),
        )
        for marker_id, yaw, update_time, length, color in headings:
            heading_valid = (
                self.show_imu_orientations
                and indoor_gps_valid
                and yaw is not None
                and self.source_is_fresh(
                    now.nanoseconds, update_time, self.imu_orientation_timeout
                )
            )
            if heading_valid:
                markers.append(
                    self.heading_arrow_marker(
                        marker_id, yaw, length, *color, stamp
                    )
                )
            else:
                markers.append(self.delete_marker(marker_id, stamp))

        # Perception overlays: landmarks and targets with their normals. Only
        # drawn while the physical pose that lifts them is fresh.
        if physical_pose_valid:
            px = self.physical_odometry.pose.pose.position.x
            py = self.physical_odometry.pose.pose.position.y
            yaw = yaw_from_quaternion(self.physical_odometry.pose.pose.orientation)
        else:
            px = py = yaw = None

        perception = (
            (
                self.show_landmarks,
                self.landmarks_msg,
                self.landmarks_update_time,
                self.LANDMARKS_ID,
                self.LANDMARK_NORMALS_ID,
                self.LANDMARK_COLOR,
                self.LANDMARK_COLOR,
            ),
            (
                self.show_targets,
                self.targets_msg,
                self.targets_update_time,
                self.TARGETS_ID,
                self.TARGET_NORMALS_ID,
                self.TARGET_COLOR,
                self.TARGET_NORMAL_COLOR,
            ),
        )
        for show, message, update_time, points_id, normals_id, pcol, ncol in perception:
            valid = (
                show
                and physical_pose_valid
                and message is not None
                and len(message.markers) > 0
                and self.source_is_fresh(
                    now.nanoseconds, update_time, self.perception_timeout
                )
            )
            if valid:
                points, normals = self.perception_markers(
                    stamp, message, px, py, yaw, points_id, normals_id,
                    pcol, ncol,
                )
                markers.append(points)
                markers.append(normals)
            else:
                markers.append(self.delete_marker(points_id, stamp))
                markers.append(self.delete_marker(normals_id, stamp))

        if self.show_world_anchors:
            markers.extend(self.build_world_markers(stamp))

        message = MarkerArray()
        message.markers = markers
        self.marker_publisher.publish(message)


def main(args=None):
    """Run the centralized spacecraft marker visualizer."""
    rclpy.init(args=args)
    node = SpacecraftVisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
