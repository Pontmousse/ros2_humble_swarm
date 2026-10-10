#!/usr/bin/env python3
"""Read a HIL rosbag and answer the questions the live probe cannot.

A bag recorded from the dev host carries *two* clocks for every message: the
receive time the recorder stamped it with, and the `header.stamp` the publisher
wrote. Every gap measurement taken live this session was ambiguous because it
only had the first one -- a 0.7 s hole in `localization/odom` looks identical
whether the publisher stalled or the WiFi did. Comparing the two distributions
settles it: if the header gaps are clean and the receive gaps are ragged, the
transport is at fault and the consumer inside the container never saw a gap.

Cross-robot timing is NOT comparable (the Pis have no chrony), so inter-robot
geometry is aligned on receive time instead and spatial conclusions are kept
separate from per-robot timing ones.

    python3 scripts/bag_report.py bags/hil_20261007_speedcap
"""
import argparse
import bisect
import math
import sqlite3
import sys
from pathlib import Path

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


# nav2_pidff drops to a zero command when either input is older than this, and
# velocity_smoother independently zeroes the chassis on its own 0.25 s timeout.
GATES = (0.25, 0.3, 0.5, 1.0)

# position_guidance3.py: influence radius, the distance where the repulsion
# saturates at max_nbh_force, and chassis contact (2 x robot_radius 0.22).
# Both radii moved on 2026-10-08 (0.70 -> 1.05, 0.55 -> 0.65). These stay at
# the old values on purpose: this script only bins separations for comparison
# across bags, and most of the bags on disk were flown at the old radius, so
# following the law would make two runs incomparable for no gain. Use
# bag_damping_check.py --rant when the question is about the law itself.
R_INFLUENCE = 0.70
R_SATURATE = 0.55
R_CONTACT = 0.44


def percentile(sorted_values, fraction):
    if not sorted_values:
        return float("nan")
    return sorted_values[min(len(sorted_values) - 1, int(fraction * len(sorted_values)))]


def load(bag_dir):
    """Return {topic: [(receive_s, header_s_or_None, msg)]} sorted by receive."""
    bag = Path(bag_dir)
    dbs = sorted(bag.glob("*.db3"))
    if not dbs:
        sys.exit(f"no .db3 found in {bag}")
    series = {}
    for db in dbs:
        connection = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
        types = {
            row[0]: (row[1], row[2])
            for row in connection.execute("SELECT id, name, type FROM topics")
        }
        classes = {}
        for topic_id, (name, type_name) in types.items():
            try:
                classes[topic_id] = get_message(type_name)
            except (ImportError, AttributeError, ValueError):
                print(f"  skipping {name}: cannot resolve {type_name}")
        for topic_id, receive_ns, blob in connection.execute(
            "SELECT topic_id, timestamp, data FROM messages ORDER BY timestamp"
        ):
            if topic_id not in classes:
                continue
            msg = deserialize_message(bytes(blob), classes[topic_id])
            stamp = getattr(getattr(msg, "header", None), "stamp", None)
            header_s = None
            if stamp is not None:
                header_s = stamp.sec + stamp.nanosec * 1e-9
                # An unstamped message carries 0, which is not a time.
                if header_s <= 0.0:
                    header_s = None
            series.setdefault(types[topic_id][0], []).append(
                (receive_ns * 1e-9, header_s, msg)
            )
        connection.close()
    return series


def gap_stats(times):
    gaps = sorted(b - a for a, b in zip(times, times[1:]))
    if not gaps:
        return None
    span = times[-1] - times[0]
    return {
        "rate": (len(times) - 1) / span if span > 0 else float("nan"),
        "p50": percentile(gaps, 0.50),
        "p99": percentile(gaps, 0.99),
        "worst": gaps[-1],
        "breach": {g: sum(1 for x in gaps if x > g) for g in GATES},
    }


def timing_table(series):
    print("\n=== per-topic timing: publisher clock vs recorder clock ===")
    print("    header = header.stamp (what the in-container consumer saw)")
    print("    recv   = recorder receive time (adds WiFi loss and jitter)\n")
    width = max(len(t) for t in series)
    for topic in sorted(series):
        rows = series[topic]
        print(f"{topic:{width}s}  {len(rows):6d} msgs")
        for label, times in (
            ("header", [h for _, h, _ in rows if h is not None]),
            ("recv  ", [r for r, _, _ in rows]),
        ):
            stats = gap_stats(sorted(times))
            if stats is None:
                print(f"{'':{width}s}    {label}  (no timestamps)")
                continue
            breaches = "  ".join(
                f">{g:g}s:{stats['breach'][g]}" for g in GATES)
            print(f"{'':{width}s}    {label}  {stats['rate']:6.2f} Hz  "
                  f"p50={stats['p50']:.3f}  p99={stats['p99']:.3f}  "
                  f"worst={stats['worst']:.3f}   {breaches}")


def speed_of(msg):
    return math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)


def position_of(msg):
    return msg.pose.pose.position.x, msg.pose.pose.position.y


def nearest(times, values, target):
    """Value whose time is closest to target, plus the mismatch in seconds."""
    i = bisect.bisect_left(times, target)
    best = None
    for j in (i - 1, i):
        if 0 <= j < len(times):
            error = abs(times[j] - target)
            if best is None or error < best[1]:
                best = (values[j], error)
    return best


def robot_report(series, robot):
    reference = series.get(f"/{robot}/virtual_spacecraft/odom")
    measured = series.get(f"/{robot}/localization/odom")
    if not reference or not measured:
        return
    print(f"\n=== {robot} ===")

    speeds = sorted(speed_of(m) for _, _, m in reference)
    print(f"  reference speed   p50={percentile(speeds, 0.50):.4f}  "
          f"p99={percentile(speeds, 0.99):.4f}  max={speeds[-1]:.4f} m/s"
          f"   (cap 0.25)")
    over = sum(1 for s in speeds if s > 0.2505)
    print(f"  samples over cap  {over}/{len(speeds)}")

    # Both streams are published by the same Pi, so their header stamps share a
    # clock and the tracking error is honest. This is the number the whole
    # session has been about.
    m_times = [h for _, h, _ in measured if h is not None]
    m_pos = [position_of(m) for _, h, m in measured if h is not None]
    errors = []
    for _, h, msg in reference:
        if h is None:
            continue
        hit = nearest(m_times, m_pos, h)
        if hit is None or hit[1] > 0.1:
            continue
        rx, ry = position_of(msg)
        errors.append(math.hypot(rx - hit[0][0], ry - hit[0][1]))
    if errors:
        errors.sort()
        print(f"  tracking error    p50={percentile(errors, 0.50):.3f}  "
              f"p90={percentile(errors, 0.90):.3f}  "
              f"p99={percentile(errors, 0.99):.3f}  "
              f"max={errors[-1]:.3f} m   (was 1.26 before the cap)")

    wrench = series.get(f"/{robot}/virtual_spacecraft/applied_wrench")
    if wrench:
        mags = sorted(math.hypot(m.force.x, m.force.y) for _, _, m in wrench)
        saturated = sum(1 for m in mags if m > 0.995)
        print(f"  applied |force|   p50={percentile(mags, 0.50):.3f}  "
              f"p99={percentile(mags, 0.99):.3f}  max={mags[-1]:.3f} N"
              f"   saturated {saturated}/{len(mags)}"
              f" ({100.0 * saturated / len(mags):.1f}%)")


def separation_report(series, robots):
    """Inter-robot geometry, aligned on receive time because the Pis' clocks
    are not synchronised. WiFi latency jitter (~80 ms) at a 0.5 m/s closing
    speed blurs this by about 4 cm, which is small against the 0.26 m that
    separates the influence radius from contact."""
    a, b = robots
    first = series.get(f"/{a}/localization/odom")
    second = series.get(f"/{b}/localization/odom")
    if not first or not second:
        return
    print(f"\n=== separation {a} <-> {b} ===")
    t2 = [r for r, _, _ in second]
    p2 = [position_of(m) for _, _, m in second]
    samples = []
    for r, _, msg in first:
        hit = nearest(t2, p2, r)
        if hit is None or hit[1] > 0.1:
            continue
        x, y = position_of(msg)
        samples.append((r, math.hypot(x - hit[0][0], y - hit[0][1])))
    if not samples:
        return
    distances = sorted(d for _, d in samples)
    print(f"  centre-to-centre  min={distances[0]:.3f}  "
          f"p01={percentile(distances, 0.01):.3f}  "
          f"p50={percentile(distances, 0.50):.3f} m")
    total = len(samples)
    for label, limit in (("influence  <0.70", R_INFLUENCE),
                         ("saturated  <0.55", R_SATURATE),
                         ("CONTACT    <0.44", R_CONTACT)):
        n = sum(1 for d in distances if d < limit)
        print(f"  {label}    {n:5d}/{total}  ({100.0 * n / total:.1f}% of the run)")

    # Closing speed matters more than distance: the influence radius buys
    # 0.26 m of braking room, and whether that is enough is a function of how
    # fast the pair is converging when the repulsion first switches on.
    closing = []
    for (t_prev, d_prev), (t_now, d_now) in zip(samples, samples[1:]):
        dt = t_now - t_prev
        if 0.0 < dt < 0.5:
            closing.append((d_prev - d_now) / dt)
    if closing:
        entries = sorted(
            rate for (_, d), rate in zip(samples[1:], closing)
            if d < R_INFLUENCE)
        if entries:
            print(f"  closing speed inside the influence radius: "
                  f"p50={percentile(entries, 0.50):+.3f}  "
                  f"p99={percentile(entries, 0.99):+.3f}  "
                  f"max={entries[-1]:+.3f} m/s")
            # Relative braking authority: each craft is 5 kg and the resultant
            # repulsion saturates at 1.0 N, but bounding_box can oppose it with
            # 0.5 N, so net is 0.5 N each -> 0.1 m/s^2 each, 0.2 m/s^2 relative.
            for name, net in (("1.0 N unopposed", 1.0), ("0.5 N net", 0.5)):
                decel = 2.0 * net / 5.0
                stoppable = math.sqrt(2.0 * decel * (R_INFLUENCE - R_CONTACT))
                print(f"    with {name:16s} relative decel {decel:.2f} m/s^2"
                      f" -> can absorb {stoppable:.3f} m/s over the 0.26 m runway")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--robots", default="RM2,RM6")
    args = parser.parse_args()
    robots = [r for r in args.robots.split(",") if r]

    series = load(args.bag)
    timing_table(series)
    for robot in robots:
        robot_report(series, robot)
    if len(robots) == 2:
        separation_report(series, robots)


if __name__ == "__main__":
    main()
