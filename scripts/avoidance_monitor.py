#!/usr/bin/env python3
"""Live separation / avoidance-force monitor for the two-robot approach test.

Positions come off the UDP broadcast rather than ROS: the dev host loses a
large and time-varying fraction of DDS traffic over WiFi, so a ROS-sourced
separation would stall exactly when the robots are closing. Forces still have
to come from ROS, since avoidance_wrench is not on the wire -- they are read
BEST_EFFORT so a lost sample is skipped instead of blocking.

The expected force is recomputed from position_guidance3.py at startup, so
retuning the law cannot silently desynchronise this display from it.

    python3 scripts/avoidance_monitor.py            # robots 2 and 6
    python3 scripts/avoidance_monitor.py --robots 2 6 --port 9870
"""
import argparse
import math
import re
import socket
import sys
import threading
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "ros2_swarm/src/swarm_comms"))

from swarm_comms.packet import DEFAULT_PORT, decode  # noqa: E402

GUIDANCE_SRC = REPO / "ros2_swarm/src/swarm_controller/swarm_controller/position_guidance3.py"
# 2 x robot_radius from virtual_spacecraft.yaml: chassis contact.
CONTACT_DISTANCE = 0.44


def law_constants(path):
    src = Path(path).read_text()

    def grab(name):
        match = re.search(rf"^\s*self\.{name}\s*=\s*([0-9.]+)", src, re.M)
        if match is None:
            raise SystemExit(f"could not find self.{name} in {path}")
        return float(match.group(1))

    return grab("Rant"), grab("antflk_gain"), grab("max_nbh_force")


def expected_force(distance, r_ant, gain, f_max):
    if distance >= r_ant:
        return 0.0
    magnitude = gain * (1.0 / distance - 1.0 / r_ant) / (distance ** 2)
    return min(magnitude, f_max)


class PoseListener(threading.Thread):
    daemon = True

    def __init__(self, port):
        super().__init__()
        self.lock = threading.Lock()
        self.poses = {}
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("", port))
        self.sock.settimeout(0.2)
        self.running = True

    def run(self):
        while self.running:
            try:
                data, _ = self.sock.recvfrom(2048)
            except socket.timeout:
                continue
            packet = decode(data)
            if packet is None:
                continue
            with self.lock:
                self.poses[packet["robot_id"]] = (packet, time.monotonic())

    def snapshot(self):
        with self.lock:
            return dict(self.poses)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--robots", type=int, nargs=2, default=[2, 6])
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--rate", type=float, default=5.0)
    args = parser.parse_args()

    r_ant, gain, f_max = law_constants(GUIDANCE_SRC)
    saturation_d = None
    for i in range(1, 2001):
        d = r_ant * i / 2000.0
        if expected_force(d, r_ant, gain, f_max) >= f_max:
            saturation_d = d
    print(f"law from {GUIDANCE_SRC.name}: Rant={r_ant} gain={gain} max={f_max} N")
    print(f"  force onset {r_ant:.2f} m | saturates {saturation_d:.2f} m | contact {CONTACT_DISTANCE:.2f} m")
    print("separation is live immediately (UDP); forces need ~35 s of DDS discovery\n")

    import rclpy
    from geometry_msgs.msg import Wrench
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

    rclpy.init()
    node = Node("avoidance_monitor")
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                     history=HistoryPolicy.KEEP_LAST)
    wrenches = {}

    def make_cb(robot_id):
        def cb(msg: Wrench):
            wrenches[robot_id] = (msg.force.x, msg.force.y, time.monotonic())
        return cb

    for robot_id in args.robots:
        node.create_subscription(Wrench, f"/RM{robot_id}/avoidance_wrench",
                                 make_cb(robot_id), qos)

    listener = PoseListener(args.port)
    listener.start()

    min_sep = float("inf")
    peak = {r: 0.0 for r in args.robots}
    a_id, b_id = args.robots

    try:
        while True:
            # Pump rclpy from the main thread rather than a spin thread: Ctrl-C
            # otherwise reaches rclpy's handler and the thread races the context
            # teardown, which aborts the process instead of printing the summary.
            deadline = time.monotonic() + 1.0 / args.rate
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.02)
            now = time.monotonic()
            poses = listener.snapshot()

            if a_id not in poses or b_id not in poses:
                missing = [r for r in args.robots if r not in poses]
                print(f"\rwaiting for UDP from {missing}            ", end="", flush=True)
                continue

            (pa, ta), (pb, tb) = poses[a_id], poses[b_id]
            # An unlocalized robot broadcasts (0, 0) with pose_valid clear, so a
            # separation computed from it is a distance to the arena origin, not
            # to the neighbour. Reporting it would understate or overstate the
            # approach by metres.
            pose_ok = pa["pose_valid"] and pb["pose_valid"]
            if pose_ok:
                sep = math.hypot(pa["x"] - pb["x"], pa["y"] - pb["y"])
                min_sep = min(min_sep, sep)
                want = expected_force(sep, r_ant, gain, f_max)
                sep_text = f"sep {sep:5.3f}m  expect {want:5.3f}N"
            else:
                sep = None
                sep_text = "sep   --     expect   --  "

            cells = []
            for robot_id in args.robots:
                got = wrenches.get(robot_id)
                if got is None or now - got[2] > 1.0:
                    # A fresh rclpy node needs ~35 s to discover these publishers
                    # over the lab WiFi, so distinguish "not discovered yet" from
                    # "discovered but silent" -- they look identical otherwise.
                    seen = node.count_publishers(f"/RM{robot_id}/avoidance_wrench")
                    cells.append(f"RM{robot_id} {'silent' if seen else 'discov'}")
                    continue
                fx, fy, _ = got
                mag = math.hypot(fx, fy)
                peak[robot_id] = max(peak[robot_id], mag)
                cells.append(f"RM{robot_id} {mag:5.3f}N({fx:+.2f},{fy:+.2f})")

            if not pose_ok:
                bad = [r for r in args.robots if not poses[r][0]["pose_valid"]]
                flag = f" POSE_INVALID {bad}"
            elif sep < CONTACT_DISTANCE:
                flag = " *** CONTACT ***"
            elif sep < saturation_d:
                flag = " <<< SATURATED"
            elif sep < r_ant:
                # Crossing r_ant only means the force is no longer exactly zero;
                # at 0.695 m it is 0.02 N. Reporting that as "in range" reads as
                # "test point reached" and ended a run at 0.694 m with nothing
                # to show. Name the distance that actually exercises the cap.
                flag = f" in range - keep going to {saturation_d:.2f} m"
            else:
                flag = ""

            stale = max(now - ta, now - tb)
            print(f"\r{sep_text}  {'  '.join(cells)}"
                  f"  udp {stale:4.2f}s{flag}        ", end="", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        listener.running = False
        peaks = "  ".join(f"RM{r} {v:.3f}N" for r, v in peak.items())
        print(f"\n\nclosest approach {min_sep:.3f} m   peak force  {peaks}")
        # Ctrl-C reaches rclpy.spin in the helper thread first, which tears the
        # context down before this runs.
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
