"""Hand off between bounded search and target capture on stable detection.

Watches the namespaced `targets` topic and, once a target has been seen for a
few consecutive frames, disables the bounded-search guidance and switches the
capture controller on. When the target is lost for longer than a timeout it
reverses the handoff. Manual `toggle_controller` / `all_capture` calls remain
usable and are not fought by this node.
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_srvs.srv import SetBool, Trigger
from nav2_msgs.srv import ManageLifecycleNodes
from swarm_interfaces.msg import Markers
from geometry_msgs.msg import Twist


class CaptureSupervisor(Node):
    """Switch cp_controller on and search off with detection hysteresis."""

    def __init__(self):
        super().__init__("capture_supervisor")

        # Require N consecutive non-empty frames before capturing, to reject
        # single-frame false positives.
        self.declare_parameter("detect_frames", 5)
        # Seconds the target may stay lost before we fall back to search.
        self.declare_parameter("lost_timeout", 2.0)
        # Consecutive EMPTY frames tolerated before the entry streak resets.
        # Lets a steadily-held marker that flickers out for a frame or two keep
        # accumulating hit_streak, instead of restarting the count on every drop
        # (fixes CAPTURE<->SEARCH oscillation caused by low-fps detection flicker).
        self.declare_parameter("miss_tolerance", 3)
        # Sticky capture: once a stable detection has latched us into CAPTURE,
        # never fall back to SEARCH automatically on target loss. Kills the
        # CAPTURE<->SEARCH oscillation outright (the lost->search edge is cut).
        self.declare_parameter("latch_capture", True)
        self.declare_parameter("qos_reliability", "BEST_EFFORT")

        self.detect_frames = int(self.get_parameter("detect_frames").value)
        self.lost_timeout = float(self.get_parameter("lost_timeout").value)
        self.miss_tolerance = int(self.get_parameter("miss_tolerance").value)
        self.latch_capture = bool(self.get_parameter("latch_capture").value)
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
        self.miss_streak = 0            # consecutive EMPTY frames (entry side)
        self.last_seen = None           # time of the last non-empty frame

        # Service clients. Relative names resolve under the launch namespace
        # into /RMx/toggle_controller and /RMx/search/enable.
        self.capture_client = self.create_client(SetBool, "toggle_controller")
        self.search_client = self.create_client(SetBool, "search/enable")
        # Trigger that re-seats the virtual spacecraft on the physical pose at
        # rest, killing any coasting velocity accumulated during search.
        self.reset_client = self.create_client(
            Trigger, "virtual_spacecraft/reset")
        # Lifecycle manager for velocity_smoother, the only node that actually
        # writes /RMx/cmd_vel on the search side. Pausing it hands cmd_vel
        # exclusively to cp_controller during capture.
        self.smoother_mgr = self.create_client(
            ManageLifecycleNodes,
            "velocity_smoother_lifecycle_manager/manage_nodes")

        # Zero-velocity publisher used to hard-stop the chassis across the
        # smoother pause. Relative "cmd_vel" resolves to /RMx/cmd_vel, the same
        # Twist topic the driver subscribes to (shared with velocity_smoother
        # and cp_controller). We only publish on it during the enter_capture
        # transition, so there is no steady-state contention with cp_controller.
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)

        self.create_subscription(Markers, "targets", self.on_targets, qos)
        # Poll for target loss at 5 Hz.
        self.create_timer(0.2, self.check_lost)
        self.get_logger().info(
            "Capture supervisor ready (manual override still works).")

    def on_targets(self, msg: Markers):
        """Count detections and enter capture once the streak is long enough."""
        if len(msg.markers) > 0:
            self.last_seen = self.get_clock().now()
            self.miss_streak = 0
            self.hit_streak += 1
            if not self.capturing and self.hit_streak >= self.detect_frames:
                self.enter_capture()
        else:
            # Tolerate a short run of empty frames (detection flicker) before
            # giving up the accumulated streak. Only a sustained gap resets it.
            self.miss_streak += 1
            if self.miss_streak > self.miss_tolerance:
                self.hit_streak = 0

    def check_lost(self):
        """Fall back to search when the target has been gone past the timeout.

        When ``latch_capture`` is set we never auto-fall back: once a stable
        detection has handed us into CAPTURE we stay there, so a steadily held
        (but low-fps / flickering) target can no longer bounce us to SEARCH.
        """
        if not self.capturing or self.last_seen is None:
            return
        if self.latch_capture:
            return
        age = (self.get_clock().now() - self.last_seen).nanoseconds * 1e-9
        if age > self.lost_timeout:
            self.exit_capture()

    def enter_capture(self):
        """Stop search first, then enable the capture controller.

        Order matters so the two sides never write /RMx/cmd_vel at the same
        instant: zero the search wrench, pause the velocity_smoother (the only
        search-side cmd_vel writer), then hand control to cp_controller.
        """
        self.capturing = True
        self.hit_streak = 0
        self.get_logger().info("Target locked -> CAPTURE (search off).")
        self.call(self.search_client, False)
        # Hard-stop the chassis around the smoother pause. Pausing the
        # velocity_smoother mid-motion otherwise leaves its last non-zero Twist
        # latched on the robot firmware, which keeps driving even though no node
        # publishes cmd_vel anymore (observed: a single false CAPTURE latch left
        # the robot arcing indefinitely with the smoother inactive and
        # cp_controller silent for lack of a fresh target). stop_chassis repeats
        # the zero across the async PAUSE so a zero lands *after* the pause takes
        # effect and becomes the driver's last command. cp_controller overrides
        # it as soon as it has a target to chase.
        self.stop_chassis()
        self.manage_smoother(ManageLifecycleNodes.Request.PAUSE)
        self.stop_chassis()
        self.call(self.capture_client, True)

    def exit_capture(self):
        """Disable capture, reset the sim, then resume search.

        Reverse order: stop cp_controller writing cmd_vel, re-seat the virtual
        spacecraft on the current physical pose at rest (so search restarts
        cleanly instead of chasing a coasted-away reference), re-enable the
        search wrench, then resume the velocity_smoother.
        """
        self.capturing = False
        self.get_logger().info("Target lost -> SEARCH (capture off).")
        self.call(self.capture_client, False)
        self.reset_spacecraft()
        self.call(self.search_client, True)
        self.manage_smoother(ManageLifecycleNodes.Request.RESUME)

    def stop_chassis(self, repeats: int = 5, period: float = 0.02):
        """Publish a zero Twist a few times to force the chassis to a stop.

        Repeated (with a short gap) because the smoother PAUSE is dispatched
        async: spacing the zeros over ~100 ms ensures at least one lands after
        the smoother has actually stopped publishing, so it sticks as the
        driver's last command instead of being overwritten by a final smoothed
        Twist. Brief blocking here is acceptable — this node's only other
        callbacks are on_targets and a 5 Hz check_lost timer.
        """
        zero = Twist()
        for _ in range(max(1, repeats)):
            self.cmd_vel_pub.publish(zero)
            time.sleep(period)

    def call(self, client, state: bool):
        """Fire-and-forget SetBool request; warn if the service is absent."""
        if not client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn(f"{client.srv_name} not available.")
            return
        request = SetBool.Request()
        request.data = state
        client.call_async(request)

    def reset_spacecraft(self):
        """Fire-and-forget Trigger to reset the virtual spacecraft."""
        if not self.reset_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn(
                f"{self.reset_client.srv_name} not available.")
            return
        self.reset_client.call_async(Trigger.Request())

    def manage_smoother(self, command: int):
        """Fire-and-forget lifecycle command (PAUSE/RESUME) to the smoother."""
        if not self.smoother_mgr.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn(
                f"{self.smoother_mgr.srv_name} not available.")
            return
        request = ManageLifecycleNodes.Request()
        request.command = command
        self.smoother_mgr.call_async(request)


def main(args=None):
    rclpy.init(args=args)
    node = CaptureSupervisor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
