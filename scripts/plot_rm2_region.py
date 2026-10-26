#!/usr/bin/env python3
"""Plot RM2's trajectory against the Marvelmind coverage area and the search box.

Answers one question: when RM2's filtered pose left the beacon-covered area in
the 2026-10-07 damping run, was that a sustained drift or an isolated spike?

A spike means the filter passed through a Marvelmind outlier it should have
gated. A sustained excursion means the filter itself diverged, or the pose is
not in the frame we think it is. The two have completely different fixes, and
an XY scatter alone cannot tell them apart -- hence the time series panels.

Beacon walls come from the comment block in
  ros2_swarm/src/swarm_bringup/config/virtual_spacecraft.yaml
because nothing in this repo stores the actual Marvelmind submap. They are
hand-transcribed from the Dashboard on the dev PC and are NOT verified by any
code path. Treat the drawn region as documentation, not as ground truth.

Usage:
    source /opt/ros/humble/setup.bash
    source ros2_marvelmind/install/setup.bash   # for HedgePositionAddressed
    source ros2_swarm/install/setup.bash        # for StatePos
    python3 scripts/plot_rm2_region.py
"""
import glob
import os
import sqlite3
import sys

import numpy as np

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
except ImportError:
    sys.exit("matplotlib is required: pip install matplotlib")

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# Transcribed from virtual_spacecraft.yaml's comment. Unverified by code.
BEACON_WALLS = dict(west=-5.61, east=0.36, north=0.0, south=-5.02)
# The /** block actually loaded by bounding_box_search and _visualizer.
BOX = dict(x_min=-4.5, x_max=-1.0, y_min=-4.0, y_max=-1.0)
BOUNDARY_MARGIN = 0.4

OUTDIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                      os.pardir, "docs", "plots")


def first_stamp(db_path, topics):
    """Earliest raw stamp across `topics`, as the common zero for one bag.

    Topics do not all start at the same instant, so normalising each one to its
    own first message silently shears them against each other -- which is how
    an odometry outage ends up shaded in the wrong place.
    """
    con = sqlite3.connect(db_path)
    try:
        stamps = []
        for topic in topics:
            row = con.execute(
                "select min(m.timestamp) from messages m join topics t "
                "on m.topic_id=t.id where t.name=?", (topic,)).fetchone()
            if row and row[0] is not None:
                stamps.append(row[0])
    finally:
        con.close()
    return min(stamps) if stamps else None


def read_topic(db_path, topic, t0=None):
    """Return (t_seconds, x, y) for one topic, or None if absent.

    `t0` is a raw nanosecond stamp used as the time origin; pass the same value
    for every topic in a bag so they stay comparable.
    """
    con = sqlite3.connect(db_path)
    try:
        row = con.execute(
            "select id, type from topics where name=?", (topic,)).fetchone()
        if row is None:
            return None
        tid, ttype = row
        msg_cls = get_message(ttype)
        times, xs, ys = [], [], []
        for stamp, data in con.execute(
                "select timestamp, data from messages where topic_id=? "
                "order by timestamp", (tid,)):
            msg = deserialize_message(data, msg_cls)
            if hasattr(msg, "x_m"):          # HedgePositionAddressed
                x, y = msg.x_m, msg.y_m
            else:                            # StatePos / Odometry / PoseStamped
                p = getattr(msg, "pose", msg)
                p = getattr(p, "pose", p)
                p = getattr(p, "position", p)
                x, y = p.x, p.y
            times.append(stamp)
            xs.append(x)
            ys.append(y)
    finally:
        con.close()
    if not times:
        return None
    origin = times[0] if t0 is None else t0
    t = (np.asarray(times, dtype=float) - origin) / 1e9
    return t, np.asarray(xs), np.asarray(ys)


def odom_outages(db_path, topic, t0, threshold=1.0):
    """Gaps in a topic's publication longer than `threshold` seconds.

    Use the *driver* odometry (`/RMx/odom`) here, not `/RMx/localization/odom`
    -- the latter is pose_publisher's own output republished as Odometry in the
    swarm_map frame, so its gaps describe the consumer, not the input.

    The pose is dead-reckoned from odometry between Marvelmind corrections, so
    an odometry blackout is the first thing to check when the pose leaves the
    room: on resume, `raw_odom - init_odom_pos` jumps by the whole blackout at
    once and the pose teleports.
    """
    con = sqlite3.connect(db_path)
    try:
        row = con.execute(
            "select id from topics where name=?", (topic,)).fetchone()
        if row is None:
            return []
        stamps = np.array(
            [r[0] for r in con.execute(
                "select timestamp from messages where topic_id=? "
                "order by timestamp", (row[0],))], dtype=float)
    finally:
        con.close()
    if len(stamps) < 2:
        return []
    stamps = (stamps - t0) / 1e9
    gaps = np.diff(stamps)
    return [(stamps[i], stamps[i + 1])
            for i in np.flatnonzero(gaps > threshold)]


def reconstruct_correction(db_path, robot, t0):
    """Recover pose_publisher's GPS `position_correction` from the bag.

    pose = init_pos + R(-init_angle) @ (odom - init_odom_pos) + position_correction

    `init_pos`, `init_odom_pos` and `init_angle` are all frozen at
    initialisation, so everything except `position_correction` is one fixed
    rigid transform of the driver odometry. Fit that transform over the whole
    run, subtract it, and what is left is the correction (plus One Euro lag).

    This is what makes "is the correction firing at all?" answerable without
    the node's logs: if it never fired, the leftover would be a constant.

    Returns (t, offset_x, offset_y) or None.
    """
    pose = read_topic(db_path, f"/{robot}/pose", t0)
    odom = read_topic(db_path, f"/{robot}/odom", t0)
    if pose is None or odom is None:
        return None
    tp, xp, yp = pose
    to, xo, yo = odom
    if len(to) < 2 or len(tp) < 2:
        return None

    source = np.c_[np.interp(tp, to, xo), np.interp(tp, to, yo)]
    target = np.c_[xp, yp]

    # Orthogonal Procrustes: rotation + translation, no scale. Scale must stay
    # fixed at 1 -- allowing it would absorb part of the correction and flatter
    # the dead-reckoning hypothesis.
    src_c, tgt_c = source.mean(0), target.mean(0)
    u, _, vt = np.linalg.svd((source - src_c).T @ (target - tgt_c))
    rot = vt.T @ u.T
    if np.linalg.det(rot) < 0:
        vt[-1] *= -1
        rot = vt.T @ u.T
    predicted = (source - src_c) @ rot.T + tgt_c

    offset = target - predicted
    # The fit's own translation is arbitrary; only the variation is meaningful.
    offset -= np.median(offset, axis=0)
    return tp, offset[:, 0], offset[:, 1]


def draw_region(ax):
    """Beacon coverage, the enforced box, and the braking margin inside it."""
    w, e = BEACON_WALLS["west"], BEACON_WALLS["east"]
    s, n = BEACON_WALLS["south"], BEACON_WALLS["north"]
    ax.add_patch(Rectangle((w, s), e - w, n - s, fill=False, lw=2.0,
                           edgecolor="0.35", linestyle="--",
                           label=f"beacon coverage {e-w:.2f}x{n-s:.2f} m"))
    ax.add_patch(Rectangle((BOX["x_min"], BOX["y_min"]),
                           BOX["x_max"] - BOX["x_min"],
                           BOX["y_max"] - BOX["y_min"],
                           fill=False, lw=2.0, edgecolor="tab:green",
                           label="search box (enforced)"))
    m = BOUNDARY_MARGIN
    ax.add_patch(Rectangle((BOX["x_min"] + m, BOX["y_min"] + m),
                           BOX["x_max"] - BOX["x_min"] - 2 * m,
                           BOX["y_max"] - BOX["y_min"] - 2 * m,
                           fill=False, lw=1.0, edgecolor="tab:green",
                           linestyle=":", alpha=0.7,
                           label=f"braking starts ({m} m margin)"))


def outside_coverage(x, y):
    """Boolean mask of samples outside the beacon rectangle."""
    return ((x < BEACON_WALLS["west"]) | (x > BEACON_WALLS["east"]) |
            (y < BEACON_WALLS["south"]) | (y > BEACON_WALLS["north"]))


def run_lengths(mask):
    """Lengths of each contiguous True run -- 1 means isolated spike."""
    runs, n = [], 0
    for flag in mask:
        if flag:
            n += 1
        elif n:
            runs.append(n)
            n = 0
    if n:
        runs.append(n)
    return runs


def summarise(label, data):
    t, x, y = data
    mask = outside_coverage(x, y)
    runs = run_lengths(mask)
    print(f"  {label}: n={len(x)}  span={t[-1]:.0f} s")
    print(f"      x [{x.min():7.3f}, {x.max():7.3f}]   "
          f"y [{y.min():7.3f}, {y.max():7.3f}]")
    if mask.any():
        longest = max(runs)
        dur = longest / len(t) * t[-1]
        print(f"      outside beacon coverage: {mask.sum()} samples "
              f"({100 * mask.mean():.1f}%) in {len(runs)} runs, "
              f"longest {longest} samples (~{dur:.2f} s)")
    else:
        print("      entirely inside beacon coverage")
    return mask


def main():
    bags = sorted(glob.glob(os.path.join("bags", "*", "*.db3")))
    if not bags:
        sys.exit("no bags found under bags/ -- run from the repo root")

    series = []
    outages = {}
    corrections = {}
    print("RM2 position extents\n")
    for bag in bags:
        name = os.path.basename(os.path.dirname(bag))
        print(name)
        t0 = first_stamp(bag, ["/RM2/pose", "/RM2/mm_pos_unf", "/RM2/odom",
                               "/RM6/pose", "/RM6/odom"])
        for topic, style in (("/RM2/pose", "filtered"),
                             ("/RM2/mm_pos_unf", "raw")):
            data = read_topic(bag, topic, t0)
            if data is None:
                print(f"  {topic}: absent")
                continue
            mask = summarise(f"{topic} ({style})", data)
            series.append((name, topic, style, data, mask))

        gaps = odom_outages(bag, "/RM2/odom", t0)
        outages[name] = gaps
        for lo, hi in gaps:
            print(f"  /RM2/odom (driver) outage {hi - lo:5.1f} s  "
                  f"t={lo:.1f} -> {hi:.1f} s")

        corrections[name] = {}
        for robot in ("RM2", "RM6"):
            rec = reconstruct_correction(bag, robot, t0)
            if rec is None:
                continue
            corrections[name][robot] = rec
            _, ox, oy = rec
            travel = float(np.hypot(np.ptp(ox), np.ptp(oy)))
            print(f"  {robot} reconstructed GPS correction: "
                  f"x [{ox.min():+.2f}, {ox.max():+.2f}] m  "
                  f"y [{oy.min():+.2f}, {oy.max():+.2f}] m  "
                  f"total excursion {travel:.2f} m")
        print()

    os.makedirs(OUTDIR, exist_ok=True)

    # --- Figure 1: XY, one panel per bag ------------------------------------
    bag_names = sorted({s[0] for s in series})
    fig, axes = plt.subplots(1, len(bag_names),
                             figsize=(7.5 * len(bag_names), 7.5),
                             squeeze=False)
    for ax, name in zip(axes[0], bag_names):
        draw_region(ax)
        for _, topic, style, (t, x, y), mask in series:
            if _ != name:
                continue
            if style == "raw":
                ax.plot(x, y, ".", ms=1.5, alpha=0.25, color="tab:orange",
                        label=f"{topic} (raw Marvelmind)")
            else:
                ax.plot(x, y, "-", lw=0.7, alpha=0.85, color="tab:blue",
                        label=f"{topic} (filtered)")
                if mask.any():
                    ax.plot(x[mask], y[mask], "x", ms=6, mew=1.4,
                            color="crimson",
                            label="filtered, outside coverage")
        ax.set_title(name)
        ax.set_xlabel("x [m], swarm_map")
        ax.set_ylabel("y [m], swarm_map")
        ax.set_aspect("equal")
        ax.grid(alpha=0.3)
        ax.legend(loc="upper left", fontsize=8)
    fig.suptitle("RM2 trajectory vs Marvelmind coverage and search box")
    fig.tight_layout()
    p1 = os.path.normpath(os.path.join(OUTDIR, "rm2_trajectory_region.png"))
    fig.savefig(p1, dpi=130)

    # --- Figure 2: x(t) and y(t), spike vs drift ----------------------------
    fig2, axes2 = plt.subplots(len(bag_names), 2,
                               figsize=(15, 4.0 * len(bag_names)),
                               squeeze=False, sharex="row")
    for row, name in enumerate(bag_names):
        for col, (axis, lo, hi, wall_lo, wall_hi) in enumerate((
                ("x", BOX["x_min"], BOX["x_max"],
                 BEACON_WALLS["west"], BEACON_WALLS["east"]),
                ("y", BOX["y_min"], BOX["y_max"],
                 BEACON_WALLS["south"], BEACON_WALLS["north"]))):
            ax = axes2[row][col]
            # Distinct names: `lo`/`hi` above are the box walls and reusing
            # them here silently drew the walls at the outage timestamps.
            for i, (gap_start, gap_end) in enumerate(outages.get(name, [])):
                ax.axvspan(gap_start, gap_end, color="tab:red", alpha=0.18,
                           lw=0,
                           label="RM2 odom outage" if i == 0 else None)
            for nm, topic, style, (t, x, y), mask in series:
                if nm != name:
                    continue
                v = x if axis == "x" else y
                if style == "raw":
                    ax.plot(t, v, ".", ms=1.0, alpha=0.3, color="tab:orange",
                            label="raw Marvelmind")
                else:
                    ax.plot(t, v, "-", lw=0.8, color="tab:blue",
                            label="filtered pose")
                    if mask.any():
                        ax.plot(t[mask], v[mask], "x", ms=5, mew=1.2,
                                color="crimson", label="outside coverage")
            for lim in (lo, hi):
                ax.axhline(lim, color="tab:green", lw=1.2)
            for lim in (wall_lo, wall_hi):
                ax.axhline(lim, color="0.35", lw=1.2, ls="--")
            # Raw Marvelmind spikes reach +-8 m; let them clip rather than
            # compress the room-scale detail we are here to read.
            ax.set_ylim(min(wall_lo, lo) - 1.5, max(wall_hi, hi) + 1.0)
            ax.set_title(f"{name}  --  {axis}(t)")
            ax.set_xlabel("t [s] from bag start")
            ax.set_ylabel(f"{axis} [m]")
            ax.grid(alpha=0.3)
            ax.legend(loc="upper right", fontsize=8)
    fig2.suptitle("Green = search box wall, dashed grey = beacon wall")
    fig2.tight_layout()
    p2 = os.path.normpath(os.path.join(OUTDIR, "rm2_xy_timeseries.png"))
    fig2.savefig(p2, dpi=130)

    # --- Figure 3: the reconstructed GPS correction -------------------------
    # A flat line here would mean the correction never fired and the pose was
    # pure dead reckoning. Anything that wanders means it is firing, and the
    # amount it wanders is the dead-reckoning error it had to absorb.
    fig3, axes3 = plt.subplots(len(bag_names), 1,
                               figsize=(13, 4.2 * len(bag_names)),
                               squeeze=False)
    for row, name in enumerate(bag_names):
        ax = axes3[row][0]
        for i, (gap_start, gap_end) in enumerate(outages.get(name, [])):
            ax.axvspan(gap_start, gap_end, color="tab:red", alpha=0.18, lw=0,
                       label="RM2 driver odom outage" if i == 0 else None)
        for robot, colour in (("RM2", "tab:blue"), ("RM6", "tab:purple")):
            rec = corrections.get(name, {}).get(robot)
            if rec is None:
                continue
            t, ox, oy = rec
            ax.plot(t, np.hypot(ox, oy), "-", lw=0.9, color=colour,
                    label=f"{robot} |correction|")
            ax.plot(t, ox, lw=0.5, alpha=0.45, color=colour, ls="--",
                    label=f"{robot} correction x")
            ax.plot(t, oy, lw=0.5, alpha=0.45, color=colour, ls=":",
                    label=f"{robot} correction y")
        ax.axhline(0.0, color="0.4", lw=1.0)
        # gps_correction_max_target_step caps how far ONE window may move the
        # target. Drawn as a magnitude, not accumulated: the cumulative ceiling
        # over a 1000 s run is ~170 m and tells you nothing.
        ax.axhline(0.5, color="k", lw=1.0, ls="-.", alpha=0.6,
                   label="gps_correction_max_target_step (0.5 m / 3 s window)")
        ax.set_title(f"{name}  --  GPS correction recovered from the bag "
                     f"(flat => correction never fired)")
        ax.set_xlabel("t [s] from bag start")
        ax.set_ylabel("correction [m]")
        ax.grid(alpha=0.3)
        ax.legend(loc="upper left", fontsize=8, ncol=3)
    fig3.suptitle("pose minus best-fit rigid transform of driver odometry")
    fig3.tight_layout()
    p3 = os.path.normpath(os.path.join(OUTDIR, "rm2_gps_correction.png"))
    fig3.savefig(p3, dpi=130)

    print(f"wrote {p1}\nwrote {p2}\nwrote {p3}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
