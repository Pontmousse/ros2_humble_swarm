#!/usr/bin/env python3
"""Distributed synchronized-capture trigger via action-time consensus.

Once this agent has LOCKED an attachment point (from the auction), has that AP in
range (estimated `targets`), and is ACTUALLY seeing it right now (raw `targets_unf`
liveness, not estimator memory), it proposes an absolute capture timestamp
`now + action_lead` and propagates the minimum proposal over `/readiness`.
Agreement must hold for `min_consensus_hold`, with at least `min_lead` remaining,
before the deadline is frozen. At that deadline each agent fires `capture_sync`
only if all its live neighbours have committed to the same round and agree within
`time_tol`. ROS clocks must be synchronized; timer scheduling limits simultaneity.

Readiness loss, membership changes, and missed deadlines cancel the round. Round
numbers propagate by maximum, preventing old proposals from entering a restarted
negotiation. This is neighbour-local coordination, not a global membership or
atomic-commit protocol under network partitions.

Pure coordination layer: identical under VIRTUAL_PLANT and real hardware.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from std_msgs.msg import Bool, Float64, Int32
from swarm_interfaces.msg import Markers, NeighbourStates, Readiness


class PeerReady:
    __slots__ = ("stamp", "action_time", "ready", "round_id", "armed")

    def __init__(self, stamp, action_time, ready, round_id, armed):
        self.stamp = stamp
        self.action_time = action_time
        self.ready = ready
        self.round_id = round_id
        self.armed = armed


class ReadinessConsensusNode(Node):
    def __init__(self):
        super().__init__("readiness_consensus")

        self.declare_parameter("robot_id", 1)
        self.declare_parameter("consensus_rate", 5.0)
        self.declare_parameter("approach_distance", 0.8)
        self.declare_parameter("perception_stale_timeout", 2.0)
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
        self.round_id = 0
        self.participants = frozenset()
        self.proposal_ap = -1
        self.armed_time = 0.0
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
        action_time = float(msg.action_time)
        if not math.isfinite(action_time) or action_time < 0.0:
            return
        stamp = self.stamp_sec(msg.header)
        previous = self.peers.get(msg.robot_id)
        if previous is not None and stamp <= previous.stamp:
            return
        self.peers[msg.robot_id] = PeerReady(
            stamp, action_time, bool(msg.ready), int(msg.round_id), bool(msg.armed))

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
    def reset_round(self, round_id):
        self.round_id = round_id
        self.my_t = self.armed_time = self.hold = 0.0
        self.armed = False

    def round(self):
        now = self.now()
        self.peers = {rid: pr for rid, pr in self.peers.items()
                      if now - pr.stamp < self.neighbor_timeout}
        live = {rid: pr for rid, pr in self.peers.items() if rid in self.live_ids}
        participants = frozenset(self.live_ids)
        ready = self.local_ready(now)

        if not self.fired:
            peer_round = max([self.round_id] + [pr.round_id for pr in live.values()])
            if peer_round > self.round_id:
                self.reset_round(peer_round)
            if self.my_t and (not ready or participants != self.participants
                              or self.assigned_ap != self.proposal_ap):
                self.reset_round(self.round_id + 1)
            self.participants = participants
            self.proposal_ap = self.assigned_ap

            peers = [pr for pr in live.values() if pr.round_id == self.round_id]
            all_ready = (ready and bool(participants) and len(peers) == len(participants)
                         and all(pr.ready and pr.action_time > 0.0 for pr in peers))
            if ready and self.my_t == 0.0:
                self.my_t = now + self.action_lead

            if self.armed:
                agreed = all_ready and all(
                    abs(pr.action_time - self.armed_time) < self.time_tol for pr in peers)
                if not agreed:
                    self.reset_round(self.round_id + 1)
                elif now >= self.armed_time:
                    if all(pr.armed for pr in peers):
                        self.sync_pub.publish(Bool(data=True))
                        self.fired = True
                        self.get_logger().info(f"capture_sync FIRED at t={self.armed_time:.3f}")
                    else:
                        self.reset_round(self.round_id + 1)
            elif ready:
                times = [self.my_t] + [pr.action_time for pr in peers
                                       if pr.ready and pr.action_time > 0.0]
                self.my_t = min(times)
                if self.my_t < now + self.min_lead:
                    # Withdraw for this tick; peers adopt the new round before retrying.
                    self.reset_round(self.round_id + 1)
                else:
                    converged = all_ready and max(times) - min(times) < self.time_tol
                    self.hold = self.hold + self.dt if converged else 0.0
                    if self.hold >= self.min_consensus_hold:
                        self.armed = True
                        self.armed_time = self.my_t

        self.publish_readiness(now, ready, len(participants))

    def publish_readiness(self, now, ready, degree):
        msg = Readiness()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "swarm_map"
        msg.robot_id = self.robot_id
        msg.ready = ready
        msg.degree = min(degree, 255)
        msg.action_time = self.my_t
        msg.round_id = self.round_id
        msg.armed = self.armed
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
