"""Geometric stand-in for the contact limit switch, for virtual targets/plants. Plain class driven by the swarm node
(contact.source: geometric; formerly the geometric_contact node).

Publishes `contact_detected` (std_msgs/Bool) when the closest `targets` marker is
within contact_distance and heading tolerance, debounced over a few ticks. It
uses only `targets`, never the plant state, so it runs whenever the target or
the plant is virtual. mode_supervisor ORs it with any other contact source.
"""

import math

import numpy as np
from std_msgs.msg import Bool

from swarm_interfaces.msg import Markers


class GeometricContact:
    def __init__(self, p, pub):
        """p(name, default) reads contact.*; pub publishes contact_detected."""
        # ref_offset_x/y mirror capture_law.py's closest-target
        # pick; contact_distance is kept in sync with it via the launch file.
        self.offsets = np.array([float(p("ref_offset_x", 10.0)), float(p("ref_offset_y", 0.0))])
        self.contact_distance = float(p("contact_distance", 0.20))
        self.heading_tolerance = float(p("contact_heading_tolerance", 0.26))
        self.debounce_frames = int(p("contact_debounce_frames", 3))
        self.period = max(float(p("timer_frequency", 0.02)), 1e-3)   # seconds per tick
        self.streak = 0
        self.latest_targets = None
        self.pub = pub

    def _on_targets(self, msg: Markers):
        self.latest_targets = msg

    def _contact_detected(self) -> bool:
        """Debounced check, replicating capture_law.py's
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

