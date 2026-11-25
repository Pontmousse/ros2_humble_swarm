"""Geometric stand-in for the contact limit switch, for virtual targets/plants.

Publishes `contact_detected` (std_msgs/Bool) when the closest `targets` marker is
within contact_distance and heading tolerance, debounced over a few ticks. It
uses only `targets`, never the plant state, so it runs whenever the target or
the plant is virtual. mode_supervisor ORs it with the Pi's contact_switch_node.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from swarm_interfaces.msg import Markers

from .virtual_spacecraft_node import qos_profile_from_parameters


class GeometricContact(Node):
    def __init__(self):
        super().__init__("geometric_contact")
        # ref_offset_x/y mirror cp_proportional_controller.py's closest-target
        # pick; contact_distance is kept in sync with it via the launch file.
        self.declare_parameter("ref_offset_x", 10.0)
        self.declare_parameter("ref_offset_y", 0.0)
        self.declare_parameter("contact_distance", 0.20)
        self.declare_parameter("contact_heading_tolerance", 0.26)
        self.declare_parameter("contact_debounce_frames", 3)
        self.declare_parameter("timer_frequency", 0.02)  # seconds per tick
        self.declare_parameter("qos_depth", 10)
        self.declare_parameter("qos_reliability", "RELIABLE")
        self.declare_parameter("qos_history", "KEEP_LAST")
        p = self.get_parameter
        self.offsets = np.array([float(p("ref_offset_x").value), float(p("ref_offset_y").value)])
        self.contact_distance = float(p("contact_distance").value)
        self.heading_tolerance = float(p("contact_heading_tolerance").value)
        self.debounce_frames = int(p("contact_debounce_frames").value)
        self.streak = 0
        self.latest_targets = None

        qos = qos_profile_from_parameters(self)
        self.pub = self.create_publisher(Bool, "contact_detected", qos)
        self.create_subscription(Markers, "targets", self._on_targets, qos)
        self.create_timer(max(float(p("timer_frequency").value), 1e-3), self.tick)

    def _on_targets(self, msg: Markers):
        self.latest_targets = msg

    def _contact_detected(self) -> bool:
        """Debounced check, replicating cp_proportional_controller.py's
        closest-target pick and heading formula so both agree on "contact"."""
        msg = self.latest_targets
        if msg is None or len(msg.markers) == 0:
            self.streak = 0
            return False
        closest = int(np.argmin([np.linalg.norm(np.array([m.x, m.y]) - self.offsets) for m in msg.markers]))
        distance = float(np.hypot(msg.markers[closest].x, msg.markers[closest].y)) / 100.0
        normal = msg.markers_normals[closest]
        heading_error = (-math.atan2(normal.x, normal.y) + math.pi) % (2.0 * math.pi) - math.pi
        in_contact = distance <= self.contact_distance and abs(heading_error) <= self.heading_tolerance
        self.streak = self.streak + 1 if in_contact else 0
        return self.streak >= self.debounce_frames

    def tick(self):
        self.pub.publish(Bool(data=self._contact_detected()))


def main(args=None):
    rclpy.init(args=args)
    node = GeometricContact()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
