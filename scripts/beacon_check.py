#!/usr/bin/env python3
"""Tally the Marvelmind modem's TDMA cycle from /mm_all_pos.

Why not `ros2 topic hz`: every downstream positioning topic in this workspace
(mm_pos_unf, mm_imu, nbh_pos, pose, localization/odom) is republished by a
fixed-rate timer, so its topic rate is the timer's rate and carries no
information about whether the hedgehog is actually producing fixes. `hz` on
those topics reports a healthy 100 Hz for a beacon that has been dead the
whole run -- six of the ten bags in ~/rosbags are exactly that case.

/mm_all_pos is published one message per fix with no timer in between, so
counting *distinct* `timestamp_ms` per address is the real measurement. Three
failure modes are distinguished:

This sees only the addresses the robots' own hedgehogs carry. Each Pi has its
own hedgehog and a hedgehog reports only its own address (measured 2026-10-09:
55 s gave addr2 129 msgs / 129 distinct fixes, addr6 138 / 138, i.e. no
duplicates and so no cross-reporting, and addr5 absent while beacon 5 was alive
at 2.83 Hz). A beacon with no Pi behind it will read "silent" here however
healthy it is -- to see the whole TDMA cycle, read the modem on the dev host's
USB, as scripts/target_bridge.py does.

  silent    address never appears      -> hedgehog not in the TDMA cycle,
                                          OR no publisher carries that address
  stalled   appears, counter frozen    -> modem forwarding a cached fix
  frozen    counter moves, (x,y) does  -> tracking lost, position held
                                          (seen on RM1 bag 20260926 for 123 s)

Usage, from the repo root:

    source /opt/ros/humble/setup.bash
    source ros2_marvelmind/install/setup.bash
    python3 scripts/beacon_check.py --duration 60 --expect 2 6
"""
import argparse
import sys
from collections import defaultdict

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from marvelmind_ros2_msgs.msg import HedgePositionAddressed

# A fix whose position moved less than this is treated as the same place. The
# inlier-only Marvelmind noise floor measured over ~10 bags is 0.052 m std,
# p95 0.143 m, so 5 mm is far below anything the sensor can resolve: movement
# under it means the modem is repeating a held position, not measuring one.
FROZEN_EPS = 0.005


class BeaconCheck(Node):
    def __init__(self, duration):
        super().__init__("beacon_check")
        self.duration = duration
        # marvelmind_obs publishes RELIABLE/KEEP_LAST (qos_parameters() ignores
        # its arguments and always returns that). RELIABLE here matches it; a
        # BEST_EFFORT reader would also connect fine, so switch if the dev
        # host's WiFi link starts retransmitting.
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(HedgePositionAddressed, "/mm_all_pos",
                                 self.cb, qos)
        self.msgs = defaultdict(int)
        self.stamps = defaultdict(set)
        self.positions = defaultdict(list)
        self.t0 = self.get_clock().now()
        self.create_timer(0.1, self.tick)
        self.done = False

    def cb(self, msg):
        a = msg.address
        self.msgs[a] += 1
        self.stamps[a].add(msg.timestamp_ms)
        self.positions[a].append((msg.timestamp_ms, msg.x_m, msg.y_m, msg.z_m))

    def tick(self):
        if (self.get_clock().now() - self.t0).nanoseconds / 1e9 >= self.duration:
            self.done = True


def distinct_positions(samples):
    """Count position changes larger than the sensor's own noise floor."""
    moves = 0
    last = None
    for _, x, y, _z in samples:
        if last is None or abs(x - last[0]) > FROZEN_EPS or abs(y - last[1]) > FROZEN_EPS:
            moves += 1
            last = (x, y)
    return moves


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=60.0)
    ap.add_argument("--expect", type=int, nargs="*", default=[],
                    help="beacon addresses that must be alive; sets exit code")
    args = ap.parse_args()

    rclpy.init()
    node = BeaconCheck(args.duration)
    print(f"listening on /mm_all_pos for {args.duration:.0f} s ...", flush=True)
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.1)

    dur = (node.get_clock().now() - node.t0).nanoseconds / 1e9
    print(f"\n{dur:.1f} s observed\n")
    print(f"{'addr':>5} {'msgs':>7} {'fixes':>7} {'fix Hz':>7} {'moved':>7}  verdict")
    print("-" * 56)

    alive = set()
    for a in sorted(node.msgs):
        n_fix = len(node.stamps[a])
        rate = n_fix / dur if dur > 0 else 0.0
        moved = distinct_positions(node.positions[a])
        if n_fix <= 1:
            verdict = "STALLED (counter frozen)"
        elif moved <= 1:
            verdict = "FROZEN (counter moves, position held)"
        else:
            verdict = "ok"
            alive.add(a)
        print(f"{a:>5} {node.msgs[a]:>7} {n_fix:>7} {rate:>7.2f} {moved:>7}  {verdict}")

    missing = [a for a in args.expect if a not in node.msgs]
    for a in missing:
        print(f"{a:>5} {0:>7} {0:>7} {0:>7.2f} {0:>7}  SILENT (never in TDMA cycle)")

    # The healthy single-hedgehog rate measured from bags is 5.92 Hz. The modem
    # splits its cycle between hedgehogs, so with two alive expect roughly half
    # each -- that split has never actually been measured here, which is part
    # of what this run is for.
    if len(alive) > 1:
        print(f"\ntwo or more beacons alive; per-beacon rate above is the "
              f"TDMA-shared rate, compare against 5.92 Hz single-hedgehog")

    bad = [a for a in args.expect if a not in alive]
    if args.expect:
        print()
        if bad:
            print(f"FAIL: expected beacons not healthy: {bad}")
        else:
            print(f"PASS: all expected beacons healthy: {sorted(alive)}")

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
