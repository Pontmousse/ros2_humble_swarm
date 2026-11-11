#!/usr/bin/env python3
"""Distributed attachment-point (AP) auction.

Each agent bids on every AP it can currently reach (from the ESTIMATED `targets`
stream, which is map-frame memory keyed by ArUco id so a claim survives a
perception blink). Bids are broadcast on the flat `/ap_bids` topic keyed by
`robot_id`; every agent deconflicts its own claim against all fresh in-range
peers using a deterministic `(bid, robot_id)` tuple compare (higher id wins
ties), so the distributed system converges to DISTINCT claims over a few rounds
without any central coordinator. The locked claim is published on `assigned_ap`
(consumed by cp_controller to pick its dock target) and `assignment_locked`
(consumed by the readiness consensus node).

Bids and geometry are purely body-frame local, so the node behaves identically
under VIRTUAL_PLANT and on real hardware.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Int32
from swarm_interfaces.msg import ApBids, Markers, NeighbourStates

EPS = 1e-6


class PeerBid:
    __slots__ = ("stamp", "bids", "claimed_ap_id", "locked")

    def __init__(self, stamp, bids, claimed_ap_id, locked):
        self.stamp = stamp
        self.bids = bids
        self.claimed_ap_id = claimed_ap_id
        self.locked = locked


class ApAuctionNode(Node):
    def __init__(self):
        super().__init__("ap_auction")

        self.declare_parameter("robot_id", 1)
        self.declare_parameter("bid_rate", 5.0)
        self.declare_parameter("w_proximity", 1.0)
        self.declare_parameter("w_velocity_align", 0.3)
        self.declare_parameter("w_normal_align", 0.3)
        self.declare_parameter("approach_distance", 0.8)
        self.declare_parameter("neighbor_bid_timeout", 0.5)
        self.declare_parameter("target_hold_timeout", 0.4)
        self.declare_parameter("lock_hysteresis", 0.1)
        self.declare_parameter("lock_rounds", 3)
        self.declare_parameter("allow_reassignment_on_dropout", True)

        self.robot_id = int(self.get_parameter("robot_id").value)
        self.w_prox = float(self.get_parameter("w_proximity").value)
        self.w_vel = float(self.get_parameter("w_velocity_align").value)
        self.w_norm = float(self.get_parameter("w_normal_align").value)
        self.approach_distance = float(self.get_parameter("approach_distance").value)
        self.neighbor_bid_timeout = float(self.get_parameter("neighbor_bid_timeout").value)
        self.target_hold_timeout = float(self.get_parameter("target_hold_timeout").value)
        self.lock_hysteresis = float(self.get_parameter("lock_hysteresis").value)
        self.lock_rounds = int(self.get_parameter("lock_rounds").value)

        # State
        self.last_targets = None
        self.last_targets_time = None
        self.vx = self.vy = 0.0
        self.live_ids = set()       # comms-valid neighbour robot_ids
        self.peers = {}             # robot_id -> PeerBid
        self.my_claim = -1
        self.lock_count = 0
        self.locked = False
        self.miss_rounds = 0

        best_effort = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                 history=HistoryPolicy.KEEP_LAST)
        bcast = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                           history=HistoryPolicy.KEEP_LAST)
        reliable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              history=HistoryPolicy.KEEP_LAST)

        self.create_subscription(Markers, "targets", self.on_targets, best_effort)
        self.create_subscription(Odometry, "localization/odom", self.on_odom, best_effort)
        self.create_subscription(NeighbourStates, "comms/neighbours", self.on_neighbours, best_effort)
        self.create_subscription(ApBids, "/ap_bids", self.on_peer_bids, bcast)

        self.bids_pub = self.create_publisher(ApBids, "/ap_bids", bcast)
        self.assigned_pub = self.create_publisher(Int32, "assigned_ap", reliable)
        self.locked_pub = self.create_publisher(Bool, "assignment_locked", reliable)

        self.create_timer(1.0 / float(self.get_parameter("bid_rate").value), self.round)
        self.get_logger().info(f"ap_auction robot_id={self.robot_id} up")

    # ------------------------------ inputs ---------------------------------
    def on_targets(self, msg: Markers):
        self.last_targets = msg
        self.last_targets_time = self.now()

    def on_odom(self, msg: Odometry):
        # twist is body-frame (child_frame_id); bearing to an AP is also body
        # frame, so velocity alignment needs no pose/rotation.
        self.vx = msg.twist.twist.linear.x
        self.vy = msg.twist.twist.linear.y

    def on_neighbours(self, msg: NeighbourStates):
        self.live_ids = {n.robot_id for n in msg.neighbours if n.valid}

    def on_peer_bids(self, msg: ApBids):
        if msg.robot_id == self.robot_id:
            return  # self-echo
        bids = {int(a): float(b) for a, b in zip(msg.ap_ids, msg.bids)
                if math.isfinite(b)}
        self.peers[msg.robot_id] = PeerBid(
            self.stamp_sec(msg.header), bids, int(msg.claimed_ap_id), bool(msg.locked))

    # ------------------------------ bidding --------------------------------
    def compute_bids(self, msg: Markers):
        """Body-frame bid per reachable AP id; {} if ids/markers mismatch."""
        n = len(msg.markers)
        if len(msg.ids) != n:
            return {}
        speed = math.hypot(self.vx, self.vy)
        out = {}
        for i in range(n):
            mx, my = msg.markers[i].x / 100.0, msg.markers[i].y / 100.0  # cm -> m
            dist = math.hypot(mx, my)
            if dist > self.approach_distance or dist < EPS:
                continue
            bid = self.w_prox / (EPS + dist)
            br = (mx / dist, my / dist)  # bearing unit (body)
            if speed > EPS:
                bid += self.w_vel * (self.vx * br[0] + self.vy * br[1]) / speed
            nx, ny = msg.markers_normals[i].x, msg.markers_normals[i].y
            nmag = math.hypot(nx, ny)
            if nmag > EPS:
                bid += self.w_norm * (br[0] * nx + br[1] * ny) / nmag
            if math.isfinite(bid):
                out[int(msg.ids[i])] = bid
        return out

    def outranked(self, ap_id, mybid, margin=0.0):
        """True if any fresh in-range peer should hold ap_id instead of me.

        An established peer lock is sticky (only dislodged by beating its bid by
        more than lock_hysteresis) so two robots within a hair of each other can't
        both keep the same AP; otherwise the deterministic (bid, robot_id) tuple
        decides, with `margin` defending my own lock.
        """
        for rid, pb in self.peers.items():
            if rid not in self.live_ids:
                continue
            pbid = pb.bids.get(ap_id)
            if pb.claimed_ap_id == ap_id and pb.locked:
                if pbid is None or (mybid, self.robot_id) <= (pbid + self.lock_hysteresis, rid):
                    return True
                continue
            if pbid is None:
                continue
            if (pbid, rid) > (mybid + margin, self.robot_id):
                return True
        return False

    def round(self):
        now = self.now()
        self.peers = {rid: pb for rid, pb in self.peers.items()
                      if now - pb.stamp < self.neighbor_bid_timeout}

        fresh = (self.last_targets_time is not None
                 and now - self.last_targets_time < self.target_hold_timeout)
        my_bids = self.compute_bids(self.last_targets) if fresh and self.last_targets else {}

        if not my_bids:
            # Perception blink: hold a locked claim for a few rounds before dropping.
            self.miss_rounds += 1
            if not (self.locked and self.miss_rounds <= self.lock_rounds):
                self.my_claim, self.locked, self.lock_count = -1, False, 0
            self.publish(my_bids)
            return
        self.miss_rounds = 0

        claim = -1
        for ap_id, mybid in sorted(my_bids.items(), key=lambda kv: kv[1], reverse=True):
            if self.locked and ap_id == self.my_claim:
                # Hysteresis: hold the lock unless beaten by a clear margin.
                if not self.outranked(ap_id, mybid, self.lock_hysteresis):
                    claim = ap_id
                    break
                continue
            if not self.outranked(ap_id, mybid):
                claim = ap_id
                break

        if claim == self.my_claim and claim != -1:
            self.lock_count += 1
            if self.lock_count >= self.lock_rounds:
                self.locked = True
        else:
            self.my_claim, self.lock_count, self.locked = claim, 0, False

        self.publish(my_bids)

    def publish(self, my_bids):
        msg = ApBids()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "swarm_map"
        msg.robot_id = self.robot_id
        msg.ap_ids = list(my_bids.keys())
        msg.bids = [my_bids[a] for a in msg.ap_ids]
        msg.claimed_ap_id = self.my_claim
        msg.locked = self.locked
        self.bids_pub.publish(msg)
        self.assigned_pub.publish(Int32(data=self.my_claim))
        self.locked_pub.publish(Bool(data=self.locked))

    # ------------------------------ helpers --------------------------------
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def stamp_sec(self, header):
        return header.stamp.sec + header.stamp.nanosec * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = ApAuctionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
