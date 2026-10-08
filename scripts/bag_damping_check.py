#!/usr/bin/env python3
"""Decide from a bag whether the damping term is live, and what it bought.

Everything here is computed *within one robot*: `comms/neighbours` and
`avoidance_wrench` are published by the same Pi over the same link, so they can
be matched on recorder receive time without chrony. The cross-robot separation
in bag_report.py cannot do this -- differentiating two unsynchronised position
streams is what produced its 62 m/s closing speeds.

Two questions:

1. Is the deployed law the new one? The potential is a strict function of
   separation alone, so any sample whose force exceeds the potential evaluated
   at the same separation proves a second term is present. Below the curve
   proves nothing (saturation, staleness, a neighbour that just left), so the
   test is one-sided and the answer is read from the excess, not the residual.

2. What closing speeds did the law actually see? Taken from the broadcast
   velocities the guidance consumed, not from differentiated positions, so it
   is the number the controller acted on rather than one reconstructed later.
"""
import argparse
import bisect
import math
import sqlite3
import sys
from pathlib import Path

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

RANT = 0.70
GAIN = 0.78
DAMPING = 2.0
MAX_FORCE = 1.0
CONTACT = 0.44


def potential(distance):
    """The conservative term alone, in newtons."""
    if distance >= RANT:
        return 0.0
    return GAIN * (1.0 / distance - 1.0 / RANT) / (distance * distance)


def load(bag_dir, wanted):
    bag = Path(bag_dir)
    series = {topic: [] for topic in wanted}
    for db in sorted(bag.glob("*.db3")):
        connection = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
        types = {
            row[0]: (row[1], row[2])
            for row in connection.execute("SELECT id, name, type FROM topics")
        }
        classes = {}
        for topic_id, (name, type_name) in types.items():
            if name in series:
                classes[topic_id] = get_message(type_name)
        for topic_id, stamp, data in connection.execute(
            "SELECT topic_id, timestamp, data FROM messages ORDER BY timestamp"
        ):
            if topic_id in classes:
                series[types[topic_id][0]].append(
                    (stamp * 1e-9, deserialize_message(data, classes[topic_id]))
                )
        connection.close()
    return series


def nearest(times, values, t, tolerance=0.08):
    """Value whose receive time is closest to t, or None if nothing is close."""
    if not times:
        return None
    i = bisect.bisect_left(times, t)
    best = None
    for j in (i - 1, i):
        if 0 <= j < len(times) and abs(times[j] - t) <= tolerance:
            if best is None or abs(times[j] - t) < abs(times[best] - t):
                best = j
    return None if best is None else values[best]


def analyse(bag_dir, robot):
    topics = [
        f"/{robot}/avoidance_wrench",
        f"/{robot}/comms/neighbours",
        f"/{robot}/pose",
        f"/{robot}/localization/odom",
    ]
    series = load(bag_dir, topics)

    wrench = series[topics[0]]
    nbh_t = [t for t, _ in series[topics[1]]]
    nbh_v = [m for _, m in series[topics[1]]]
    pose_t = [t for t, _ in series[topics[2]]]
    pose_v = [m for _, m in series[topics[2]]]
    odom_t = [t for t, _ in series[topics[3]]]
    odom_v = [m for _, m in series[topics[3]]]

    rows = []
    for t, w in wrench:
        pose = nearest(pose_t, pose_v, t)
        table = nearest(nbh_t, nbh_v, t)
        odom = nearest(odom_t, odom_v, t)
        if pose is None or table is None:
            continue
        valid = [n for n in table.neighbours if n.valid]
        if len(valid) != 1:
            continue
        n = valid[0]
        dx, dy = pose.x - n.x, pose.y - n.y
        distance = math.hypot(dx, dy)
        if distance >= RANT or distance <= 0.0:
            continue

        own_vx = own_vy = 0.0
        if odom is not None:
            q = odom.pose.pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            c, s = math.cos(yaw), math.sin(yaw)
            tw = odom.twist.twist.linear
            own_vx = tw.x * c - tw.y * s
            own_vy = tw.x * s + tw.y * c

        ux, uy = dx / distance, dy / distance
        closing = -((own_vx - n.vx) * ux + (own_vy - n.vy) * uy)
        measured = math.hypot(w.force.x, w.force.y)
        rows.append((t, distance, closing, measured))
    return rows


def report(robot, rows):
    print(f"\n=== {robot}: {len(rows)} samples inside the influence radius ===")
    if not rows:
        print("  nothing inside Rant with exactly one valid neighbour")
        return

    # One-sided test: force above the potential curve at the same separation.
    excess = []
    for _, distance, closing, measured in rows:
        pot = min(potential(distance), MAX_FORCE)
        if measured > pot + 0.02:
            excess.append((measured - pot, closing, distance, measured, pot))

    print(f"  above the pure-potential curve: {len(excess)}/{len(rows)} "
          f"({100.0 * len(excess) / len(rows):.1f}%)")
    if excess:
        worst = max(excess)
        print(f"    largest excess {worst[0]:.3f} N at d={worst[2]:.3f} m "
              f"(measured {worst[3]:.3f} vs potential {worst[4]:.3f}), "
              f"closing {worst[1]:+.3f} m/s")
        # If the excess is damping it must track closing speed; if it is a
        # second wrench contributor leaking in, it will not.
        closing_positive = [e for e in excess if e[1] > 0.05]
        print(f"    of those, {len(closing_positive)} ({100.0*len(closing_positive)/len(excess):.0f}%)"
              f" were taken while actually closing")

    # What the law predicts, both ways, against what was published.
    err_pot = err_damp = 0.0
    for _, distance, closing, measured in rows:
        pot = min(potential(distance), MAX_FORCE)
        damp = min(potential(distance) + DAMPING * max(0.0, closing), MAX_FORCE)
        err_pot += (measured - pot) ** 2
        err_damp += (measured - damp) ** 2
    n = len(rows)
    print(f"  rms vs potential-only model : {math.sqrt(err_pot / n):.4f} N")
    print(f"  rms vs potential+damping    : {math.sqrt(err_damp / n):.4f} N")

    closings = sorted(c for _, _, c, _ in rows if c > 0)
    if closings:
        def pct(f):
            return closings[min(len(closings) - 1, int(f * len(closings)))]
        print(f"  closing speed (as the guidance saw it, broadcast velocities):")
        print(f"    p50={pct(0.5):.3f}  p90={pct(0.9):.3f}  p99={pct(0.99):.3f}  "
              f"max={closings[-1]:.3f} m/s   over 0.368: "
              f"{sum(1 for c in closings if c > 0.368)}")

    distances = sorted(d for _, d, _, _ in rows)
    contact = sum(1 for d in distances if d < CONTACT)
    print(f"  separation inside Rant: min={distances[0]:.3f}  "
          f"p01={distances[max(0, int(0.01*len(distances)))]:.3f} m   "
          f"below contact 0.44: {contact}/{len(distances)} "
          f"({100.0*contact/len(distances):.1f}%)")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--robots", nargs="+", default=["RM2", "RM6"])
    args = parser.parse_args()
    for robot in args.robots:
        report(robot, analyse(args.bag, robot))


if __name__ == "__main__":
    sys.exit(main())
