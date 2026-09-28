"""Hand off between bounded search and target capture on stable detection.

Watches the namespaced `targets` topic and, once a target has been seen for a
few consecutive frames, disables the bounded-search guidance and switches the
capture controller on. When the target is lost for longer than a timeout it
reverses the handoff. Manual `toggle_controller` / `all_capture` calls remain
usable and are not fought by this node.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_srvs.srv import SetBool
from swarm_interfaces.msg import Markers


class CaptureSupervisor(Node):
    """Switch cp_controller on and search off with detection hysteresis."""

    def __init__(self):
        super().__init__("capture_supervisor")

        # Require N consecutive non-empty frames before capturing, to reject
        # single-frame false positives.
        self.declare_parameter("detect_frames", 5)
        # Seconds the target may stay lost before we fall back to search.
        self.declare_parameter("lost_timeout", 2.0)
        self.declare_parameter("qos_reliability", "BEST_EFFORT")

        self.detect_frames = int(self.get_parameter("detect_frames").value)
        self.lost_timeout = float(self.get_parameter("lost_timeout").value)
        reliability = str(self.get_parameter("qos_reliability").value)

        qos = QoSProfile(
            depth=10,
            reliability=(ReliabilityPolicy.RELIABLE
                         if reliability == "RELIABLE"
                         else ReliabilityPolicy.BEST_EFFORT),
            history=HistoryPolicy.KEEP_LAST,
        )

        # State
        self.capturing = False          # True while in the capture phase
        self.hit_streak = 0             # consecutive frames with a target
        self.last_seen = None           # time of the last non-empty frame

        # Service clients. Relative names resolve under the launch namespace
        # into /RMx/toggle_controller and /RMx/search/enable.
        self.capture_client = self.create_client(SetBool, "toggle_controller")
        self.search_client = self.create_client(SetBool, "search/enable")

        self.create_subscription(Markers, "targets", self.on_targets, qos)
        # Poll for target loss at 5 Hz.
        self.create_timer(0.2, self.check_lost)
        self.get_logger().info(
            "Capture supervisor ready (manual override still works).")

    def on_targets(self, msg: Markers):
        """Count detections and enter capture once the streak is long enough."""
        if len(msg.markers) > 0:
            self.last_seen = self.get_clock().now()
            self.hit_streak += 1
            if not self.capturing and self.hit_streak >= self.detect_frames:
                self.enter_capture()
        else:
            self.hit_streak = 0

    def check_lost(self):
        """Fall back to search when the target has been gone past the timeout."""
        if not self.capturing or self.last_seen is None:
            return
        age = (self.get_clock().now() - self.last_seen).nanoseconds * 1e-9
        if age > self.lost_timeout:
            self.exit_capture()

    def enter_capture(self):
        """Stop search first, then enable the capture controller."""
        self.capturing = True
        self.hit_streak = 0
        self.get_logger().info("Target locked -> CAPTURE (search off).")
        # Order matters: silence the search wrench before cp_controller starts
        # commanding cmd_vel, so the two never fight over the same instant.
        self.call(self.search_client, False)
        self.call(self.capture_client, True)

    def exit_capture(self):
        """Disable capture, then resume search."""
        self.capturing = False
        self.get_logger().info("Target lost -> SEARCH (capture off).")
        self.call(self.capture_client, False)
        self.call(self.search_client, True)

    def call(self, client, state: bool):
        """Fire-and-forget SetBool request; warn if the service is absent."""
        if not client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn(f"{client.srv_name} not available.")
            return
        request = SetBool.Request()
        request.data = state
        client.call_async(request)


def main(args=None):
    rclpy.init(args=args)
    node = CaptureSupervisor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
