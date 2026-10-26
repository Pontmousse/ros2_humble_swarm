#!/usr/bin/env python3
"""Publish a Marvelmind beacon as a vision target, so the capture chain can be
tested with no camera.

RM5 carries beacon 5 and runs only its RoboMaster driver, on ROS_DOMAIN_ID=5.
The swarm is domain 0 (swarm_ansible_deploy/vars.yml:15), so RM5 is invisible to
every node here and is driven by hand -- which is how it serves as a moving
target without also being a swarm member:
    ROS_DOMAIN_ID=5 ros2 topic pub -r 10 /RM5/cmd_vel geometry_msgs/msg/Twist \
        "{linear: {x: 0.03, y: 0.0, z: 0.0}}"

The beacon position is read from the Marvelmind MODEM on this host's USB, not
from /mm_all_pos. Measured 2026-10-09: each robot's Pi has its own hedgehog on
its own /dev/ttyACM*, and a hedgehog only ever reports its own address, so the
two marvelmind_obs publishers put exactly addresses 2 and 6 on /mm_all_pos and
nothing else -- 55 s of traffic gave 129 msgs / 129 distinct fixes for addr2
(no duplicates, so no cross-reporting) and zero for addr5. The modem is the
only device that hears the whole TDMA cycle, it is plugged into this host, and
reading it here also removes a WiFi hop from the target measurement.

This node turns that into the one topic the capture chain actually consumes.
Both consumers read /RMx/targets and nothing else from perception:
  mode_supervisor.py         only len(msg.markers) > 0, to switch into CAPTURE
  cp_proportional_controller.py:69  the coordinates, to drive the approach

Geometry is the exact inverse of position_guidance3.py:214-223, which does
    world = pose[:2] + R(theta) @ (markers / 100)
so here
    markers = 100 * R(-theta) @ (p_target - p_self)
with pose.x/y in metres and pose.theta in DEGREES (pose_publisher.py:1256
applies degrees(); feeding it to cos/sin unconverted is a known trap).

WHAT THIS DOES NOT TEST: calibration, aruco_scale, exposure, detection rate --
every perception failure mode. It validates guidance, the SEARCH/CAPTURE state
machine, and the approach law. A beacon target is not a cheaper camera.

The FOV and range gates are OFF by default. A beacon is omnidirectional and
always in the TDMA cycle, so gating it would discard the one advantage this
harness has over a camera. Pass --fov-deg / --max-range-m only to deliberately
emulate the camera cone later.

The marker normal is synthetic: n = (-sin b, cos b) for body-frame bearing b.
cp_proportional_controller:143-155 reads the normal as the direction to align
to, via trq_cmd = -degrees(atan2(n.x, n.y)), so this value makes trq_cmd ==
+degrees(b) and the robot turns to put the beacon dead ahead. At b = 0 it is
exactly (0, 1), the zero-torque value, which is also what the real detector
reports for a marker held square at 1 m: (-0.065, 0.998).

This repurposes the channel, and the alternative was tried first. A real ArUco
normal is the marker's FACING, which a beacon does not have, so emitting a
constant (0, 1) is more honest about the missing data. But it makes trq_cmd
identically zero, and a robot that cannot rotate does not stop approaching --
the lateral alignment term at cp_proportional_controller:128-130 saturates and
slides it sideways until the beacon is dead astern, so it REVERSES into the
target. Simulated from the geometry measured 2026-10-09, RM2 (0.97 m @
+135 deg) and RM6 (3.85 m @ +143 deg) both converged to 40 cm at bearing
180 deg, approaching backwards. Steering by bearing is the only rotation
command a point target can justify.

    source /opt/ros/humble/setup.bash
    source ros2_swarm/install/setup.bash
    python3 scripts/target_bridge.py --robots 2 6 --target-beacon 5 --dry-run
"""
import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from swarm_marvelmind.marvelmind import MarvelmindHedge
from swarm_interfaces.msg import CoordXY, Markers, StatePos


class TargetBridge(Node):

    def __init__(self, robots, beacon, tty, rate, fov_deg, max_range, timeout,
                 dry_run):
        super().__init__("target_bridge")
        self.robots = robots
        self.beacon = beacon
        self.half_fov = math.radians(fov_deg) / 2.0
        self.max_range = max_range if max_range > 0 else float("inf")
        self.timeout = timeout
        self.dry_run = dry_run

        self.target = None          # (x_m, y_m)
        self.target_stamp = None    # modem timestamp_ms of the last fix
        self.target_changed = None  # wall clock when timestamp_ms last moved
        self.poses = {}             # robot -> (x_m, y_m, theta_deg)

        # BEST_EFFORT on the way in: pose_publisher is RELIABLE (every node
        # here builds its profile from qos_parameters(), which ignores its
        # arguments and returns RELIABLE/depth 10), and a RELIABLE writer feeds
        # a BEST_EFFORT reader fine. Asking for RELIABLE would instead make the
        # dev host's lossy WiFi link retransmit, stalling exactly when the
        # robots are closing -- the same reason avoidance_monitor.py reads
        # forces BEST_EFFORT.
        qos_in = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                            history=HistoryPolicy.KEEP_LAST)
        # RELIABLE out, which satisfies both consumers: cp_controller subscribes
        # RELIABLE, mode_supervisor BEST_EFFORT.
        qos_out = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST)

        self.hedge = MarvelmindHedge(tty=tty, adr=None, debug=False)
        self.hedge.start()
        # 100 Hz against a ~8.6 Hz total fix rate (3 beacons x 2.87 Hz), so
        # position() is never overwritten between polls. Same polling shape as
        # marvelmind_obs.publish_pos_callback.
        self.create_timer(0.01, self.poll_modem)

        self.pubs = {}
        for r in robots:
            self.create_subscription(
                StatePos, f"/RM{r}/pose",
                lambda msg, r=r: self.poses.__setitem__(
                    r, (msg.x, msg.y, msg.theta)),
                qos_in)
            self.pubs[r] = self.create_publisher(
                Markers, f"/RM{r}/targets", qos_out)

        self.create_timer(1.0 / rate, self.tick)
        self.print_every = max(1, int(rate / 2))
        self.ticks = 0

    def poll_modem(self):
        if not self.hedge.positionUpdated:
            return
        self.hedge.positionUpdated = False
        address, x, y, _z, _angle, stamp_ms, _realtime = self.hedge.position()
        if address != self.beacon:
            return
        # A frozen timestamp_ms with live packets means the modem is forwarding
        # a cached fix -- the "stalled" failure mode in beacon_check.py. Tracking
        # arrival time alone would call that healthy and the robots would chase a
        # position the beacon left long ago.
        if stamp_ms != self.target_stamp:
            self.target_stamp = stamp_ms
            self.target_changed = self.get_clock().now()
        self.target = (x, y)

    def target_age(self):
        if self.target_changed is None:
            return None
        return (self.get_clock().now() - self.target_changed).nanoseconds * 1e-9

    def tick(self):
        age = self.target_age()
        fresh = age is not None and age <= self.timeout
        self.ticks += 1
        report = []

        for r in self.robots:
            pose = self.poses.get(r)
            msg = Markers()

            if not fresh or pose is None:
                report.append(f"RM{r}: " +
                              ("no pose" if pose is None else "target stale"))
            else:
                dx = self.target[0] - pose[0]
                dy = self.target[1] - pose[1]
                th = math.radians(pose[2])
                bx = math.cos(th) * dx + math.sin(th) * dy
                by = -math.sin(th) * dx + math.cos(th) * dy
                rng = math.hypot(bx, by)
                bearing = math.atan2(by, bx)

                if rng <= self.max_range and abs(bearing) <= self.half_fov:
                    m = CoordXY()
                    m.x, m.y = bx * 100.0, by * 100.0
                    n = CoordXY()
                    n.x, n.y = -math.sin(bearing), math.cos(bearing)
                    msg.markers = [m]
                    msg.markers_normals = [n]
                    mark = "SEE"
                else:
                    mark = "---"
                report.append(f"RM{r}: {mark} r={rng:5.2f}m "
                              f"b={math.degrees(bearing):+6.1f}deg")

            if not self.dry_run:
                self.pubs[r].publish(msg)

        if self.ticks % self.print_every == 0:
            age_s = f"{age:.2f}s" if age is not None else "never"
            print(f"beacon{self.beacon} age={age_s:>6} | " +
                  " | ".join(report), flush=True)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--robots", type=int, nargs="+", default=[2, 6],
                   help="chaser robot numbers (default: 2 6)")
    p.add_argument("--target-beacon", type=int, default=5,
                   help="Marvelmind address of the target beacon (default: 5)")
    p.add_argument("--modem-tty", default="/dev/ttyACM0",
                   help="serial port of the Marvelmind modem on this host. "
                        "Confirm with the manufacturer attribute: "
                        "cat /sys/class/tty/ttyACM0/device/../../manufacturer "
                        "(default: /dev/ttyACM0)")
    p.add_argument("--rate", type=float, default=10.0,
                   help="publish rate in Hz (default: 10)")
    p.add_argument("--fov-deg", type=float, default=360.0,
                   help="emulated horizontal field of view. Default 360 = no "
                        "gate. RM6's intrinsics give 2*atan(cx/fx) = 99 deg if "
                        "you want the camera cone (default: 360)")
    p.add_argument("--max-range-m", type=float, default=0.0,
                   help="emulated detection range; 0 = unlimited (default: 0)")
    p.add_argument("--target-timeout", type=float, default=1.0,
                   help="seconds without a NEW beacon fix before the target is "
                        "treated as lost (default: 1.0)")
    p.add_argument("--dry-run", action="store_true",
                   help="compute and print, publish nothing")
    args = p.parse_args()

    rclpy.init()
    node = TargetBridge(args.robots, args.target_beacon, args.modem_tty,
                        args.rate, args.fov_deg, args.max_range_m,
                        args.target_timeout, args.dry_run)
    if args.dry_run:
        print("DRY RUN - nothing is published", flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Hand back an empty target so mode_supervisor falls out of CAPTURE
        # instead of latching on the last frame this process ever sent.
        if not args.dry_run:
            for pub in node.pubs.values():
                pub.publish(Markers())
        node.hedge.stop()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
