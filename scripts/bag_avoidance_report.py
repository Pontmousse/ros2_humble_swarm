#!/usr/bin/env python3
"""Read one on-robot bag and report what the avoidance law actually did.

Separation is taken from comms/neighbours -- the table the guidance node
consumed -- not from the two robots' pose topics. The Pis have no common clock
(no chrony), so pairing poses across two independently recorded bags would
introduce an unknown offset; each robot's own view of its neighbour is
self-consistent by construction.

    python3 scripts/bag_avoidance_report.py ~/rosbags/onrobot/_RM2/rosbags/swarm_*
"""
import sqlite3
import sys
from pathlib import Path

import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def read_topic(db, topic):
    con = sqlite3.connect(db)
    row = con.execute("SELECT id, type FROM topics WHERE name = ?", (topic,)).fetchone()
    if row is None:
        con.close()
        return [], None
    topic_id, type_name = row
    msg_type = get_message(type_name)
    out = [(t / 1e9, deserialize_message(bytes(b), msg_type))
           for t, b in con.execute(
               "SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp",
               (topic_id,))]
    con.close()
    return out, type_name


def interval_stats(times):
    if len(times) < 2:
        return None
    d = np.diff(times)
    return dict(n=len(times), span=times[-1] - times[0], mean_hz=(len(times) - 1) / (times[-1] - times[0]),
                p50=np.median(d), p95=np.percentile(d, 95), worst=d.max())


def main():
    bag_dir = Path(sys.argv[1])
    db = next(bag_dir.glob("*.db3"))
    robot = next(p for p in bag_dir.name.split("_") if p.startswith("RM")) \
        if any(p.startswith("RM") for p in bag_dir.name.split("_")) else None
    if robot is None:
        # Bag names carry the tag, not the robot; recover it from a topic name.
        con = sqlite3.connect(db)
        names = [r[0] for r in con.execute("SELECT name FROM topics")]
        con.close()
        robot = next(n.split("/")[1] for n in names if n.startswith("/RM"))

    print(f"=== {robot}  {bag_dir.name} ===")

    wrench, _ = read_topic(db, f"/{robot}/avoidance_wrench")
    tbl, _ = read_topic(db, f"/{robot}/comms/neighbours")
    pose, _ = read_topic(db, f"/{robot}/pose")

    t0 = min(x[0][0] for x in (wrench, tbl, pose) if x)

    for label, data in (("avoidance_wrench", wrench), ("comms/neighbours", tbl), ("pose", pose)):
        s = interval_stats([t for t, _ in data])
        if s is None:
            print(f"{label:18s} <2 msgs")
            continue
        print(f"{label:18s} n={s['n']:4d} {s['mean_hz']:5.2f} Hz  "
              f"p50 {s['p50']*1000:6.1f} ms  p95 {s['p95']*1000:6.1f} ms  worst {s['worst']*1000:7.1f} ms")

    # Separation as this robot saw it, from the table it actually consumed.
    # NeighbourState.valid already folds in pose_valid (state_broadcaster
    # publishes valid = entry.valid and entry.pose_valid), so it is the single
    # usable-entry test.
    seps = []
    for t, msg in tbl:
        usable = [n for n in msg.neighbours if n.valid]
        seps.append((t - t0, (usable[0].x, usable[0].y) if usable else None,
                     usable[0].age if usable else None))

    # Pair each wrench sample with the pose/table nearest in time.
    pose_t = np.array([t for t, _ in pose])
    pose_xy = np.array([(m.x, m.y) for _, m in pose]) if pose else np.zeros((0, 2))
    tbl_t = np.array([t + t0 for t, _, _ in seps])

    rows = []
    for t, msg in wrench:
        mag = float(np.hypot(msg.force.x, msg.force.y))
        sep = None
        if len(pose_t) and len(tbl_t):
            i = int(np.argmin(np.abs(tbl_t - t)))
            j = int(np.argmin(np.abs(pose_t - t)))
            nb = seps[i][1]
            if nb is not None and abs(tbl_t[i] - t) < 0.3 and abs(pose_t[j] - t) < 0.5:
                sep = float(np.hypot(pose_xy[j][0] - nb[0], pose_xy[j][1] - nb[1]))
        rows.append((t - t0, mag, sep, msg.force.x, msg.force.y))

    nz = [r for r in rows if r[1] > 1e-9]
    print(f"\nwrench samples {len(rows)}   non-zero {len(nz)}   "
          f"peak {max(r[1] for r in rows):.3f} N")

    close = [r for r in rows if r[2] is not None and r[2] < 0.70]
    zero_close = [r for r in close if r[1] <= 1e-9]
    print(f"in-range samples (sep < 0.70 m) {len(close)}   "
          f"of which ZERO force {len(zero_close)}")
    if close:
        print(f"closest separation seen by {robot}: {min(r[2] for r in close):.3f} m")
    if zero_close:
        print("  zero-force-while-close at t = " +
              ", ".join(f"{r[0]:.2f}s(sep {r[2]:.3f})" for r in zero_close[:12]))

    # Longest run of consecutive zero force while a neighbour was in range.
    run = best = 0
    for r in rows:
        if r[2] is not None and r[2] < 0.70 and r[1] <= 1e-9:
            run += 1
            best = max(best, run)
        else:
            run = 0
    if close:
        dt = np.median(np.diff([r[0] for r in rows]))
        print(f"longest zero-force run while in range: {best} samples (~{best*dt:.2f} s)")

    # Neighbour-table health.
    empty = sum(1 for _, nb, _ in seps if nb is None)
    print(f"\nneighbour table: {len(seps)} publishes, {empty} with no usable neighbour "
          f"({100.0*empty/max(len(seps),1):.1f}%)")

    print("\n  t(s)   |F| N   sep m   fx      fy")
    for r in rows[::max(1, len(rows)//40)]:
        sep = f"{r[2]:6.3f}" if r[2] is not None else "   -- "
        print(f"{r[0]:7.2f} {r[1]:6.3f} {sep}  {r[3]:+6.3f} {r[4]:+6.3f}")


if __name__ == "__main__":
    main()
