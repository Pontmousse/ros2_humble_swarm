"""Continuously send the virtual spacecraft pose to Nav2 as a one-pose path."""

from copy import deepcopy
import math
from typing import Optional

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


CONTROLLER_IDS = {
    "dwb": "DWBStandard",
    "dwb_ff": "DWBVelocityFF",
    "dwb_velocity_feedback": "DWBVelocityFeedback",
}


def controller_id_for_mode(controller_mode: str) -> str:
    """Return the configured Nav2 controller ID for an experiment mode."""
    try:
        return CONTROLLER_IDS[controller_mode]
    except KeyError as error:
        choices = ", ".join(sorted(CONTROLLER_IDS))
        raise ValueError(
            f"controller_mode must be one of: {choices}"
        ) from error


def reference_is_fresh(
    now_nanoseconds: int,
    update_nanoseconds: Optional[int],
    timeout: float,
) -> bool:
    """Return whether a virtual reference exists and is recent."""
    if update_nanoseconds is None:
        return False
    age = (now_nanoseconds - update_nanoseconds) * 1.0e-9
    return 0.0 <= age <= timeout


def make_single_pose_path(reference: Odometry, stamp) -> Path:
    """Create a one-pose path containing the current virtual pose."""
    path = Path()
    path.header.stamp = stamp
    path.header.frame_id = reference.header.frame_id.lstrip("/")
    pose = PoseStamped()
    pose.header = deepcopy(path.header)
    pose.pose = deepcopy(reference.pose.pose)
    path.poses.append(pose)
    return path


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


class Nav2DwbPathNode(Node):
    """Adapt virtual odometry to repeatedly preempted Nav2 FollowPath goals."""

    def __init__(self):
        super().__init__("nav2_dwb_path")
        self.declare_parameter("controller_mode", "dwb")
        self.declare_parameter("path_update_period", 0.1)
        self.declare_parameter("reference_timeout", 0.25)
        self.declare_parameter("goal_checker_id", "general_goal_checker")
        self.declare_parameter("qos_depth", 10)
        self.declare_parameter("qos_reliability", "RELIABLE")
        self.declare_parameter("qos_history", "KEEP_LAST")

        controller_mode = str(self.get_parameter("controller_mode").value)
        self.controller_id = controller_id_for_mode(controller_mode)
        self.path_update_period = float(
            self.get_parameter("path_update_period").value
        )
        self.reference_timeout = float(
            self.get_parameter("reference_timeout").value
        )
        self.goal_checker_id = str(
            self.get_parameter("goal_checker_id").value
        )
        if not (
            math.isfinite(self.path_update_period)
            and self.path_update_period > 0.0
            and math.isfinite(self.reference_timeout)
            and self.reference_timeout > 0.0
        ):
            raise ValueError("path period and reference timeout must be positive")

        self.latest_reference = None
        self.last_reference_time = None
        self.active_goal_handles = []
        self.goal_request_in_flight = False
        self.cancel_requests_in_flight = 0

        self.create_subscription(
            Odometry,
            "virtual_spacecraft/odom",
            self.reference_callback,
            qos_profile_from_parameters(self),
        )
        self.follow_path_client = ActionClient(self, FollowPath, "follow_path")
        self.create_timer(self.path_update_period, self.update_goal)
        self.get_logger().info(
            f"DWB moving-goal adapter ready for {self.controller_id}"
        )

    def reference_callback(self, message: Odometry) -> None:
        """Store the newest virtual pose and receipt time."""
        self.latest_reference = message
        self.last_reference_time = self.get_clock().now().nanoseconds

    def reference_is_valid(self, now_nanoseconds: int) -> bool:
        """Return whether the stored reference has a valid frame and age."""
        return (
            self.latest_reference is not None
            and bool(self.latest_reference.header.frame_id.lstrip("/"))
            and reference_is_fresh(
                now_nanoseconds,
                self.last_reference_time,
                self.reference_timeout,
            )
        )

    def update_goal(self) -> None:
        """Send the latest one-pose path, or cancel tracking when invalid."""
        now = self.get_clock().now()
        if not self.reference_is_valid(now.nanoseconds):
            self.cancel_active_goals()
            return
        if self.goal_request_in_flight or self.cancel_requests_in_flight:
            return
        if not self.follow_path_client.server_is_ready():
            return

        goal = FollowPath.Goal()
        goal.path = make_single_pose_path(self.latest_reference, now.to_msg())
        goal.controller_id = self.controller_id
        goal.goal_checker_id = self.goal_checker_id
        self.goal_request_in_flight = True
        future = self.follow_path_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future) -> None:
        """Remember an accepted goal and watch for its completion."""
        self.goal_request_in_flight = False
        try:
            goal_handle = future.result()
        except Exception as error:  # pragma: no cover - ROS transport failure
            self.get_logger().error(f"Failed to send FollowPath goal: {error}")
            return
        if not goal_handle.accepted:
            self.get_logger().warn("FollowPath goal was rejected")
            return
        self.active_goal_handles.append(goal_handle)
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result, handle=goal_handle: self.goal_result_callback(
                result, handle
            )
        )
        if not self.reference_is_valid(self.get_clock().now().nanoseconds):
            self.cancel_active_goals()

    def goal_result_callback(self, future, goal_handle) -> None:
        """Clear only the goal handle associated with this result."""
        del future
        if goal_handle in self.active_goal_handles:
            self.active_goal_handles.remove(goal_handle)

    def cancel_active_goals(self) -> None:
        """Cancel accepted FollowPath goals when reference data expires."""
        if not self.active_goal_handles or self.cancel_requests_in_flight:
            return
        goal_handles = self.active_goal_handles
        self.active_goal_handles = []
        self.cancel_requests_in_flight = len(goal_handles)
        for goal_handle in goal_handles:
            future = goal_handle.cancel_goal_async()
            future.add_done_callback(self.cancel_done_callback)

    def cancel_done_callback(self, future) -> None:
        """Allow later goals after cancellation finishes."""
        del future
        self.cancel_requests_in_flight -= 1


def main(args=None):
    """Run the moving-goal path adapter."""
    rclpy.init(args=args)
    node = Nav2DwbPathNode()
    try:
        rclpy.spin(node)
    finally:
        node.cancel_active_goals()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
