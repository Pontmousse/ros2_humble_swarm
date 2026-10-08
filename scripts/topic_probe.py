#!/usr/bin/env python3
"""Subscribe to several topics at once and report which ones actually publish.

A fresh rclpy node needs ~35 s to discover publishers over this lab WiFi, and
`ros2 topic hz` / `echo --once` create their own direct node every time -- so
checking N topics with the CLI costs N x 35 s and a one-shot echo usually
reports "does not appear to be published yet" before discovery has finished.
This does the discovery once and watches everything in parallel.

Types are resolved from the graph, so topics whose message package is missing
on this host are reported as skipped instead of aborting the run.

    python3 scripts/topic_probe.py /RM2/pose /RM2/comms/neighbours
    python3 scripts/topic_probe.py --listen 20 /RM2/pose
"""
import argparse
import importlib
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


# The fields worth seeing for the message types this stack uses. Printing a
# whole Odometry or Image drowns the one number you came for.
INTERESTING = (
    "x", "y", "yaw", "theta", "valid",
    "linear.x", "linear.y", "angular.z",
    "force.x", "force.y", "torque.z",
    "pose.pose.position.x", "pose.pose.position.y",
    "twist.twist.linear.x", "twist.twist.linear.y",
)


def numeric_leaves(msg):
    """Yield (path, float) for the INTERESTING fields this message has."""
    for path in INTERESTING:
        node = msg
        for part in path.split("."):
            node = getattr(node, part, None)
            if node is None:
                break
        if isinstance(node, (int, float)) and not isinstance(node, bool):
            yield path, float(node)
        elif isinstance(node, bool):
            yield path, float(node)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("topics", nargs="+")
    parser.add_argument("--discover", type=float, default=45.0,
                        help="seconds to wait for the graph to populate")
    parser.add_argument("--listen", type=float, default=15.0,
                        help="seconds to count messages for")
    # Consumers here gate on age, not on rate: nav2_pidff drops to a zero
    # command whenever localization/odom is older than its 0.25 s timeout. A
    # stream can average 10 Hz and still trip that gate on jitter, so the mean
    # rate alone cannot tell you whether a consumer stayed fed.
    # Comma-separated, because the useful question is not "how many gaps exceed
    # one threshold" but "how far would raising the threshold actually get me".
    # A single count cannot distinguish a stream that just misses the limit from
    # one that stalls for a second.
    parser.add_argument("--gap", default="0.25,0.3,0.5,1.0",
                        help="comma-separated gap thresholds to count breaches at")
    args = parser.parse_args()
    thresholds = sorted(float(g) for g in args.gap.split(",") if g.strip())

    rclpy.init()
    node = Node("topic_probe")

    counts = {}
    first = {}
    last = {}
    peak = {}
    final = {}
    gaps = {}
    skipped = {}
    graph = {}

    def try_subscribe(topic):
        """Subscribe if the topic's type is resolvable now. Return True if done."""
        entry = graph.get(topic)
        if entry is None:
            return False
        pkg, _, name = entry[0].split("/")
        try:
            msg_type = getattr(importlib.import_module(f"{pkg}.msg"), name)
        except (ImportError, AttributeError):
            skipped[topic] = f"{pkg} not installed here"
            return True
        counts[topic] = 0

        def cb(msg, topic=topic):
            counts[topic] += 1
            now = time.monotonic()
            first.setdefault(topic, now)
            if topic in last:
                gaps.setdefault(topic, []).append(now - last[topic][0])
            last[topic] = (now, msg)
            # Track the largest magnitude seen, not just the last sample: a
            # command stream that is publishing steadily at zero and one that
            # is oscillating look identical in a single snapshot, and that
            # distinction is the whole question when the robot will not move.
            slot = peak.setdefault(topic, {})
            # The last *signed* sample as well. The peak is an absolute value,
            # so comparing two topics' peaks cannot tell you whether they agree
            # or differ by twice the magnitude -- which is exactly the question
            # when checking a controller's reference against its measurement.
            tail = final.setdefault(topic, {})
            for path, value in numeric_leaves(msg):
                slot[path] = max(slot.get(path, 0.0), abs(value))
                tail[path] = value

        # BEST_EFFORT matches both BEST_EFFORT and RELIABLE publishers; the
        # reverse does not, so a RELIABLE subscription silently never matches
        # a BEST_EFFORT publisher and looks exactly like a dead topic.
        node.create_subscription(
            msg_type, topic, cb,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST))
        return True

    # Discovery is not a phase that completes -- it is a process that keeps
    # converging for as long as the node lives. An earlier version resolved all
    # types up front and reported whatever was still missing as NOT IN GRAPH;
    # on 2026-10-07 that declared five live topics dead after 50 s while a warm
    # ros2 daemon saw all 101. So subscribe to each topic the moment its type
    # appears and keep re-checking right through the listen window.
    def pump(seconds, stop_when_complete):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
            graph.update(node.get_topic_names_and_types())
            for topic in args.topics:
                if topic not in counts and topic not in skipped:
                    try_subscribe(topic)
            if stop_when_complete and len(counts) + len(skipped) == len(args.topics):
                return

    pump(args.discover, True)
    resolved = len(counts) + len(skipped)
    print(f"discovered {len(graph)} topics in the graph; "
          f"resolved {resolved}/{len(args.topics)} requested")
    print(f"\nlistening {args.listen:.0f} s (still discovering) ...\n")
    pump(args.listen, False)

    for topic in args.topics:
        if topic in skipped:
            print(f"{topic:34s} SKIP - {skipped[topic]}")
            continue
        if topic not in counts:
            # Never resolved. Say what the namespace does contain, so "the
            # robot is off the network" and "this one topic is absent" are
            # distinguishable without a second run.
            prefix = "/" + topic.lstrip("/").split("/")[0]
            siblings = sorted(t for t in graph if t.startswith(prefix + "/"))
            print(f"{topic:34s} NOT IN GRAPH  ({prefix}/* seen: "
                  f"{len(siblings)} topics)")
            continue
        n = counts[topic]
        pubs = node.count_publishers(topic)
        if n < 2:
            print(f"{topic:34s} {n:5d} msgs  SILENT  (publishers seen: {pubs})")
            continue
        span = last[topic][0] - first[topic]
        rate = (n - 1) / span if span > 0 else float("nan")
        series = sorted(gaps.get(topic, []))
        worst = series[-1] if series else 0.0
        print(f"{topic:34s} {n:5d} msgs  {rate:6.2f} Hz  "
              f"worst gap {worst:.3f}s")
        if series:
            def pct(p):
                return series[min(len(series) - 1, int(p * len(series)))]
            print(f"{'':34s}   gap p50={pct(0.50):.3f}s  p90={pct(0.90):.3f}s  "
                  f"p99={pct(0.99):.3f}s")
            breaches = "  ".join(
                f">{t:g}s:{sum(1 for g in series if g > t)}" for t in thresholds)
            print(f"{'':34s}   breaches  {breaches}")
        for path in peak[topic]:
            print(f"{'':34s}   {path:28s} last={final[topic][path]:+9.4f}"
                  f"   max|.|={peak[topic][path]:.4f}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
