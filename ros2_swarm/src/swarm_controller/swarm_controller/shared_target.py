"""Shared target estimate: neighbours' landmark-memory centres on the flat /target_estimate topic.

Published by guidance (pointing) while a robot sees the target; read by pointing and bounding_box_search in SEARCH.
"""
from rclpy.qos import QoSProfile, ReliabilityPolicy

SHARE_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)  # flat broadcast, like /readiness
now_s = lambda node: node.get_clock().now().nanoseconds * 1e-9


class SharedTargets:
    """Neighbours' target-centre estimates, aged by RECEIVE time (HIL clocks are not synced)."""
    def __init__(self, me, timeout): self.me, self.timeout, self.est = me, timeout, {}

    def add(self, msg, now):
        if msg.robot != self.me and msg.count > 0: self.est[msg.robot] = (msg.x, msg.y, msg.count, now)

    def centre(self, now):
        """Count-weighted mean of the fresh estimates, or None."""
        e = [v for v in self.est.values() if now - v[3] <= self.timeout]
        if not e: return None
        w = sum(v[2] for v in e)
        return sum(v[0] * v[2] for v in e) / w, sum(v[1] * v[2] for v in e) / w
