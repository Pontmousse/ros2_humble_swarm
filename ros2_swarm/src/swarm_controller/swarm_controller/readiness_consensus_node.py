#!/usr/bin/env python3
"""Distributed synchronized-capture trigger via action-time consensus.

Once this agent has LOCKED an attachment point (from the auction), has that AP in
range (estimated `targets`), and is ACTUALLY seeing it right now (raw `targets_unf`
liveness, not estimator memory), it proposes an absolute capture timestamp
`now + action_lead` and runs a Metropolis-Hastings average over all ready in-range
neighbours on the flat `/readiness` topic. The update is snapshot-then-update, so
it is callback-order independent; convergence is measured from DISAGREEMENT
(max-min action_time < time_tol), not step size. When the swarm agrees (held for
`min_consensus_hold`) it latches the agreed time, and each agent fires its own
`capture_sync` gate when wall-clock reaches that time -> true simultaneity under
latency (chrony-synced clocks assumed).

Guards: requires >=1 live neighbour to fire (a comms-isolated agent holds, never
self-fires); withdraws its proposal and un-arms if it loses readiness before firing
(so the trigger never fires on remembered-but-unseen APs).

Pure coordination layer: identical under VIRTUAL_PLANT and real hardware.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from std_msgs.msg import Bool, Float64, Int32
from swarm_interfaces.msg import Markers, NeighbourStates, Readiness


class PeerReady:
    __slots__ = ("stamp", "action_time", "degree", "ready")

    def __init__(self, stamp, action_time, degree, ready):
        self.stamp = stamp
        self.action_time = action_time
        self.degree = degree
        self.ready = ready


class ReadinessConsensusNode(Node):
    def __init__(self):
        super().__init__("readiness_consensus")

        self.declare_parameter("robot_id", 1)
        self.declare_parameter("consensus_rate", 5.0)
        self.declare_parameter("approach_distance", 0.8)
        self.declare_parameter("perception_stale_timeout", 0.5)
        self.declare_parameter("action_lead", 2.0)
        self.declare_parameter("min_lead", 0.5)
        self.declare_parameter("time_tol", 0.05)
        self.declare_parameter("neighbor_timeout", 0.5)
        self.declare_parameter("min_consensus_hold", 0.3)

        self.robot_id = int(self.get_parameter("robot_id").value)
        self.approach_distance = float(self.get_parameter("approach_distance").value)
        self.perception_stale_timeout = float(self.get_parameter("perception_stale_timeout").value)
        self.action_lead = float(self.get_parameter("action_lead").value)
        self.min_lead = float(self.get_parameter("min_lead").value)
        self.time_tol = float(self.get_parameter("time_tol").value)
        self.neighbor_timeout = float(self.get_parameter("neighbor_timeout").value)
        self.min_consensus_hold = float(self.get_parameter("min_consensus_hold").value)

        # Auction outputs
        self.assigned_ap = -1
        self.locked = False
        # Latest estimated targets (range check)
        self.last_targets = None
        # Raw-perception liveness: last header stamp + per-id last-advance wall time,
        # tracked separately for the raw (targets_unf) and estimated (targets) streams.
        self._unf_ever = False
        self._last_stamp = {"unf": None, "est": None}
        self._id_live = {"unf": {}, "est": {}}
        # Neighbours
        self.live_ids = set()
        self.peers = {}
        # Consensus state
        self.my_t = 0.0
        self.armed = False
        self.fired = False
        self.hold = 0.0

        best_effort = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                 history=HistoryPolicy.KEEP_LAST)
        reliable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              history=HistoryPolicy.KEEP_LAST)
        bcast = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                           history=HistoryPolicy.KEEP_LAST)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST)

        self.create_subscription(Bool, "assignment_locked", self.on_locked, reliable)
        self.create_subscription(Int32, "assigned_ap", self.on_assigned, reliable)
        self.create_subscription(Markers, "targets", self.on_targets, best_effort)
        self.create_subscription(Markers, "targets_unf", self.on_targets_unf, best_effort)
        self.create_subscription(NeighbourStates, "comms/neighbours", self.on_neighbours, best_effort)
        self.create_subscription(Readiness, "/readiness", self.on_peer, bcast)

        self.ready_pub = self.create_publisher(Readiness, "/readiness", bcast)
        self.sync_pub = self.create_publisher(Bool, "capture_sync", latched)
        self.debug_pub = self.create_publisher(Float64, "consensus_debug", best_effort)

        self.dt = 1.0 / float(self.get_parameter("consensus_rate").value)
        self.create_timer(self.dt, self.round)
        self.get_logger().info(f"readiness_consensus robot_id={self.robot_id} up")

    # ------------------------------ inputs ---------------------------------
    def on_locked(self, msg: Bool):
        self.locked = bool(msg.data)

    def on_assigned(self, msg: Int32):
        self.assigned_ap = int(msg.data)

    def on_targets(self, msg: Markers):
        self.last_targets = msg
        self._track("est", msg)

    def on_targets_unf(self, msg: Markers):
        self._unf_ever = True
        self._track("unf", msg)

    def on_neighbours(self, msg: NeighbourStates):
        self.live_ids = {n.robot_id for n in msg.neighbours if n.valid}

    def on_peer(self, msg: Readiness):
        if msg.robot_id == self.robot_id:
            return
        self.peers[msg.robot_id] = PeerReady(
            self.stamp_sec(msg.header), float(msg.action_time),
            int(msg.degree), bool(msg.ready))

    def _track(self, which, msg: Markers):
        """Record per-id last-advance wall time on an advancing header stamp.

        A frozen camera republishes the same stamp, so a stream is 'live' for an
        id only while that id keeps arriving on frames whose stamp advances.
        """
        stamp = self.stamp_sec(msg.header)
        if self._last_stamp[which] is not None and stamp == self._last_stamp[which]:
            return
        self._last_stamp[which] = stamp
        now = self.now()
        live = self._id_live[which]
        for mid in msg.ids:
            live[int(mid)] = now

    # ------------------------------ readiness ------------------------------
    def ap_in_range(self):
        msg = self.last_targets
        if msg is None or len(msg.ids) != len(msg.markers):
            return False
        for i in range(len(msg.markers)):
            if int(msg.ids[i]) == self.assigned_ap:
                return (msg.markers[i].x ** 2 + msg.markers[i].y ** 2) ** 0.5 / 100.0 \
                    <= self.approach_distance
        return False

    def perception_live(self, now):
        # Raw stream when an estimator is in the loop; else `targets` is itself
        # the raw frozen-stamp stream (regime fallback, §0).
        live = self._id_live["unf" if self._unf_ever else "est"]
        t = live.get(self.assigned_ap)
        return t is not None and (now - t) < self.perception_stale_timeout

    def local_ready(self, now):
        return (self.locked and self.assigned_ap >= 0
                and self.ap_in_range() and self.perception_live(now))

    # ------------------------------ consensus ------------------------------
    def round(self):
        now = self.now()
        self.peers = {rid: pr for rid, pr in self.peers.items()
                      if now - pr.stamp < self.neighbor_timeout}
        my_degree = len(self.live_ids)
        live = {rid: pr for rid, pr in self.peers.items() if rid in self.live_ids}

        ready = self.local_ready(now)
        if ready and (self.my_t == 0.0 or self.my_t < now + self.min_lead):
            self.my_t = now + self.action_lead
        if not ready:
            self.my_t = 0.0
            if not self.fired:
                self.armed = False
                self.hold = 0.0

        t_old = self.my_t  # snapshot -> order-independent MH update
        if ready and t_old > 0.0:
            for pr in live.values():
                if pr.ready and pr.action_time > 0.0:
                    w = 1.0 / (1.0 + max(my_degree, pr.degree))
                    self.my_t += w * (pr.action_time - t_old)
            self.my_t = max(self.my_t, now + self.min_lead)

        self.publish_readiness(now, ready, my_degree)

        ready_peers = [pr.action_time for pr in live.values()
                       if pr.ready and pr.action_time > 0.0]
        all_ready = ready and len(ready_peers) == len(live) and len(live) >= 1
        if all_ready:
            times = [self.my_t] + ready_peers
            converged = (max(times) - min(times)) < self.time_tol
        else:
            converged = False
        self.hold = self.hold + self.dt if converged else 0.0
        if self.hold >= self.min_consensus_hold:
            self.armed = True
        if self.armed and not self.fired and now >= self.my_t:
            self.sync_pub.publish(Bool(data=True))
            self.fired = True
            self.get_logger().info(f"capture_sync FIRED at t={self.my_t:.3f}")

    def publish_readiness(self, now, ready, degree):
        msg = Readiness()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "swarm_map"
        msg.robot_id = self.robot_id
        msg.ready = ready
        msg.degree = min(degree, 255)
        msg.action_time = self.my_t
        self.ready_pub.publish(msg)
        self.debug_pub.publish(Float64(data=self.my_t))

    # ------------------------------ helpers --------------------------------
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def stamp_sec(self, header):
        return header.stamp.sec + header.stamp.nanosec * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = ReadinessConsensusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
