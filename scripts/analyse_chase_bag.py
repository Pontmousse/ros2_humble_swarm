#!/usr/bin/env python3
"""Decode a chase rosbag without a ROS install and report search/chase behaviour.

The dev PC has no ROS and no built swarm_interfaces, so this parses the CDR
payloads straight out of the sqlite3 bag. All the types involved are flat
float64 aggregates, which makes that tractable:

    CoordXY   float64 x, y
    Markers   CoordXY[] markers, CoordXY[] markers_normals
    StatePos  float64 x, y, theta
    Twist     float64 linear[3], angular[3]
    Wrench    float64 force[3], torque[3]

CDR layout: 4 encapsulation bytes, then the body with each primitive aligned to
its own size *relative to the start of the body*. A sequence is a uint32 count
followed by its elements, so the first float64 after a count sits at the next
8-byte boundary, not immediately after it.
"""

import sqlite3
import struct
import sys
from collections import defaultdict

import numpy as np


def _align(off, size):
    return off + (-off % size)


def parse_markers(blob):
    """-> list of (x, y). Only the `markers` field; normals are ignored."""
    body = blob[4:]
    count = struct.unpack_from("<I", body, 0)[0]
    # Guard against a malformed record claiming an absurd length.
    if count > 1000:
        return []
    off = _align(4, 8)
    out = []
    for _ in range(count):
        x, y = struct.unpack_from("<dd", body, off)
        out.append((x, y))
        off += 16
    return out


def parse_floats(blob, n):
    return struct.unpack_from("<" + "d" * n, blob[4:], 0)


def load(path):
    con = sqlite3.connect(path)
    topics = {tid: name for tid, name in
              con.execute("SELECT id, name FROM topics")}
    data = defaultdict(list)
    for tid, ts, blob in con.execute(
            "SELECT topic_id, timestamp, data FROM messages ORDER BY timestamp"):
        data[topics[tid]].append((ts, blob))
    con.close()
    return data


def main(path):
    data = load(path)
    t0 = min(v[0][0] for v in data.values() if v)

    def rel(ts):
        return (ts - t0) * 1e-9

    print(f"bag: {path}")
    print(f"duration: {max(rel(v[-1][0]) for v in data.values() if v):.1f} s\n")

    # ---- detection ----------------------------------------------------
    for topic in ("/RM2/targets_unf", "/RM2/targets"):
        rows = [(rel(ts), parse_markers(b)) for ts, b in data[topic]]
        nonempty = [(t, m) for t, m in rows if m]
        pct = 100.0 * len(nonempty) / len(rows) if rows else 0.0
        print(f"{topic}: {len(rows)} msgs, {len(nonempty)} non-empty "
              f"({pct:.1f}%)")
        if nonempty:
            xs = np.array([m[0][0] for _, m in nonempty])
            ys = np.array([m[0][1] for _, m in nonempty])
            print(f"   x(cm) min {xs.min():7.1f}  p50 {np.percentile(xs,50):7.1f}  "
                  f"max {xs.max():7.1f}")
            print(f"   y(cm) min {ys.min():7.1f}  p50 {np.percentile(ys,50):7.1f}  "
                  f"max {ys.max():7.1f}")
            print(f"   first seen {nonempty[0][0]:.1f}s, "
                  f"last seen {nonempty[-1][0]:.1f}s")

    # ---- detection episodes (gap > 1 s splits) -------------------------
    rows = [(rel(ts), parse_markers(b)) for ts, b in data["/RM2/targets"]]
    nonempty = [t for t, m in rows if m]
    episodes = []
    if nonempty:
        start = prev = nonempty[0]
        for t in nonempty[1:]:
            if t - prev > 1.0:
                episodes.append((start, prev))
                start = t
            prev = t
        episodes.append((start, prev))
    print(f"\ndetection episodes (split on >1 s gap): {len(episodes)}")
    for s, e in episodes[:25]:
        print(f"   {s:7.1f}s -> {e:7.1f}s   ({e - s:5.1f}s)")

    # ---- cmd_vel -------------------------------------------------------
    cv = [(rel(ts), parse_floats(b, 6)) for ts, b in data["/RM2/cmd_vel"]]
    lin = np.array([v[0] for _, v in cv])
    ang = np.array([v[5] for _, v in cv])
    print(f"\n/RM2/cmd_vel: {len(cv)} msgs")
    print(f"   linear.x   min {lin.min():7.3f}  max {lin.max():7.3f}  "
          f"mean {lin.mean():7.3f}   nonzero {100*np.mean(np.abs(lin)>1e-6):.1f}%")
    print(f"   angular.z  min {ang.min():7.3f}  max {ang.max():7.3f}  "
          f"mean {ang.mean():7.3f}   nonzero {100*np.mean(np.abs(ang)>1e-6):.1f}%")

    # cmd_vel restricted to the windows where a target was visible: this is
    # the only slice where cp_controller, not the search guidance, is driving.
    if episodes:
        mask = np.zeros(len(cv), bool)
        times = np.array([t for t, _ in cv])
        for s, e in episodes:
            mask |= (times >= s) & (times <= e + 4.0)  # +lost_timeout
        if mask.any():
            print(f"\n   during detection episodes (+4 s lost_timeout), "
                  f"{mask.sum()} samples:")
            print(f"      linear.x  min {lin[mask].min():7.3f}  "
                  f"max {lin[mask].max():7.3f}  mean {lin[mask].mean():7.3f}")
            print(f"      forward {100*np.mean(lin[mask] > 1e-3):5.1f}%   "
                  f"reverse {100*np.mean(lin[mask] < -1e-3):5.1f}%")

    # ---- pose ----------------------------------------------------------
    ps = [(rel(ts), parse_floats(b, 3)) for ts, b in data["/RM2/pose"]]
    px = np.array([p[0] for _, p in ps])
    py = np.array([p[1] for _, p in ps])
    travel = np.sum(np.hypot(np.diff(px), np.diff(py)))
    print(f"\n/RM2/pose: {len(ps)} msgs")
    print(f"   x range [{px.min():.2f}, {px.max():.2f}]  "
          f"y range [{py.min():.2f}, {py.max():.2f}]")
    print(f"   path length {travel:.2f} m")


if __name__ == "__main__":
    main(sys.argv[1])
