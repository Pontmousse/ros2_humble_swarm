#!/usr/bin/env python3
"""Plot a chase bag: detection, the approach law, and where the robot went.

No ROS install needed -- the CDR payloads are decoded by analyse_chase_bag.

The figure answers three questions that the printed summary can only assert:

  1. Does the detected range ever sit far enough outside buffer_radius for an
     approach to exist at all, or does the camera only ever see the marker
     from inside the keep-out circle?
  2. Is cp_controller executing linear.x = approach_gain * (range - buffer)?
     Plotted as commanded-vs-predicted, so saturation against the body-x cap
     shows up as a flat top rather than as a degraded correlation number.
  3. Did the robot drive the approach, or was the marker carried toward it?
     Only the pose track can separate those.

Usage:
    python3 scripts/plot_chase_bag.py bags/chase_222249/chase_222249_0.db3
"""

import os
import sys

import numpy as np

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle, Circle
except ImportError:
    sys.exit("matplotlib is required: pip install matplotlib")

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from analyse_chase_bag import load, parse_floats, parse_markers

APPROACH_GAIN = 0.9
BUFFER_RADIUS = 0.60
# cp_proportional_controller.max_translational_speed[0]
BODY_X_CAP = 0.05
LOST_TIMEOUT = 4.0
# The /** block loaded by bounding_box_search.
BOX = dict(x_min=-4.5, x_max=-1.0, y_min=-4.0, y_max=-1.0)

OUTDIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                      os.pardir, "docs", "plots")


def episodes(times, gap=1.0):
    """Contiguous runs of detection, split wherever the camera went blind."""
    if not len(times):
        return []
    out, start, prev = [], times[0], times[0]
    for t in times[1:]:
        if t - prev > gap:
            out.append((start, prev))
            start = t
        prev = t
    out.append((start, prev))
    return out


def shade(ax, eps, label="detection episode"):
    for i, (s, e) in enumerate(eps):
        ax.axvspan(s, e, color="tab:green", alpha=0.16, lw=0,
                   label=label if i == 0 else None)


def main(path):
    data = load(path)
    t0 = min(v[0][0] for v in data.values() if v)

    def rel(ts):
        return (ts - t0) * 1e-9

    rows = [(rel(ts), parse_markers(b)) for ts, b in data["/RM2/targets"]]
    det = [(t, m[0]) for t, m in rows if m]
    dt = np.array([t for t, _ in det])
    rng = np.array([m[0] for _, m in det]) / 100.0      # cm -> m
    lat = np.array([m[1] for _, m in det]) / 100.0

    cmd = [(rel(ts), parse_floats(b, 6)) for ts, b in data["/RM2/cmd_vel"]]
    ct = np.array([t for t, _ in cmd])
    cx = np.array([v[0] for _, v in cmd])
    cz = np.array([v[5] for _, v in cmd])

    pose = [(rel(ts), parse_floats(b, 3)) for ts, b in data["/RM2/pose"]]
    pt = np.array([t for t, _ in pose])
    px = np.array([p[0] for _, p in pose])
    py = np.array([p[1] for _, p in pose])

    eps = episodes(dt)
    # Nearest command sample to each detection. searchsorted can land one past
    # the end when the last detection outlives cmd_vel, hence the clip.
    idx = np.clip(np.searchsorted(ct, dt), 0, len(ct) - 1)
    actual = cx[idx]
    predicted = APPROACH_GAIN * (rng - BUFFER_RADIUS)

    fig = plt.figure(figsize=(16, 13))
    gs = fig.add_gridspec(4, 2, height_ratios=[1.0, 1.0, 1.0, 1.25],
                          hspace=0.34, wspace=0.22)

    # --- 1. range(t) ------------------------------------------------------
    ax = fig.add_subplot(gs[0, :])
    shade(ax, eps)
    ax.plot(dt, rng, ".", ms=3, color="tab:blue", label="reported range")
    ax.axhline(BUFFER_RADIUS, color="crimson", lw=1.6,
               label=f"buffer_radius = {BUFFER_RADIUS} m")
    ax.fill_between([0, max(pt)], 0, BUFFER_RADIUS, color="crimson",
                    alpha=0.07, lw=0)
    ax.set_xlim(0, max(pt))
    ax.set_ylim(0, 1.45)
    ax.set_ylabel("range [m]")
    ax.set_title("Reported target range. Red band = inside the keep-out "
                 "circle, where the law commands REVERSE "
                 f"({100*np.mean(rng < BUFFER_RADIUS):.0f}% of detections)")
    ax.grid(alpha=0.3)
    ax.legend(loc="upper left", fontsize=8, ncol=3)

    # --- 2. cmd_vel(t) ----------------------------------------------------
    ax = fig.add_subplot(gs[1, :])
    shade(ax, eps)
    ax.plot(ct, cx, lw=0.7, color="tab:blue", label="cmd_vel linear.x")
    ax.plot(ct, cz, lw=0.7, alpha=0.5, color="tab:purple",
            label="cmd_vel angular.z")
    for s in (+BODY_X_CAP, -BODY_X_CAP):
        ax.axhline(s, color="k", ls="-.", lw=1.0, alpha=0.7)
    ax.text(2, BODY_X_CAP + 0.015, "max_translational_speed[0] = 0.05 m/s",
            fontsize=8)
    ax.axhline(0, color="0.5", lw=0.8)
    ax.set_xlim(0, max(pt))
    ax.set_ylabel("cmd [m/s, rad/s]")
    ax.set_title("Commanded velocity. Outside the green bands the search "
                 "guidance is driving, not cp_controller")
    ax.grid(alpha=0.3)
    ax.legend(loc="upper left", fontsize=8, ncol=3)

    # --- 3. law check: predicted vs actual --------------------------------
    ax = fig.add_subplot(gs[2, 0])
    ax.scatter(predicted, actual, s=6, alpha=0.35, color="tab:blue")
    lim = max(np.abs(predicted).max(), BODY_X_CAP) * 1.1
    ax.plot([-lim, lim], [-lim, lim], "k--", lw=1.0, label="ideal (slope 1)")
    for s in (+BODY_X_CAP, -BODY_X_CAP):
        ax.axhline(s, color="crimson", ls="-.", lw=1.1)
    ax.axhline(0, color="0.5", lw=0.8)
    ax.axvline(0, color="0.5", lw=0.8)
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-0.09, 0.09)
    ax.set_xlabel(f"predicted {APPROACH_GAIN} x (range - {BUFFER_RADIUS}) [m/s]")
    ax.set_ylabel("actual linear.x [m/s]")
    ok = np.abs(actual) > 1e-6
    r = np.corrcoef(predicted[ok], actual[ok])[0, 1]
    ax.set_title(f"Approach law, whole bag: corr {r:+.3f}.\n"
                 "Flat top/bottom = the 0.05 m/s body-x cap, not a bad law")
    ax.grid(alpha=0.3)
    ax.legend(loc="upper left", fontsize=8)

    # --- 4. where the range actually lives --------------------------------
    ax = fig.add_subplot(gs[2, 1])
    ax.hist(rng, bins=50, color="tab:blue", alpha=0.75)
    ax.axvline(BUFFER_RADIUS, color="crimson", lw=1.6,
               label=f"buffer_radius = {BUFFER_RADIUS} m")
    ax.set_xlabel("reported range [m]")
    ax.set_ylabel("detections")
    ax.set_title(f"Detection range distribution. Only "
                 f"{100*np.mean(rng > BUFFER_RADIUS):.0f}% lies outside the "
                 f"keep-out\ncircle, so the approach corridor is "
                 f"{rng.max()-BUFFER_RADIUS:.2f} m wide at best")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)

    # --- 5. XY track ------------------------------------------------------
    ax = fig.add_subplot(gs[3, 0])
    ax.add_patch(Rectangle((BOX["x_min"], BOX["y_min"]),
                           BOX["x_max"] - BOX["x_min"],
                           BOX["y_max"] - BOX["y_min"],
                           fill=False, lw=1.8, edgecolor="tab:green",
                           label="search box"))
    ax.plot(px, py, "-", lw=0.7, alpha=0.6, color="0.45",
            label=f"pose track ({np.hypot(np.diff(px),np.diff(py)).sum():.1f} m)")
    seen = np.zeros(len(pt), bool)
    for s, e in eps:
        seen |= (pt >= s) & (pt <= e)
    ax.plot(px[seen], py[seen], ".", ms=3.5, color="tab:red",
            label="target visible")
    ax.plot(px[0], py[0], "o", ms=9, mfc="none", mec="k", label="start")
    ax.set_aspect("equal")
    ax.set_xlabel("x [m], swarm_map")
    ax.set_ylabel("y [m], swarm_map")
    ax.set_title("Where RM2 was when it could see the marker")
    ax.grid(alpha=0.3)
    ax.legend(loc="upper left", fontsize=8)

    # --- 6. the two long episodes, zoomed ---------------------------------
    ax = fig.add_subplot(gs[3, 1])
    longest = sorted(eps, key=lambda p: p[1] - p[0], reverse=True)[:2]
    for (s, e), colour in zip(longest, ("tab:blue", "tab:orange")):
        m = (dt >= s) & (dt <= e)
        ax.plot(dt[m] - s, rng[m], "-", lw=1.0, color=colour,
                label=f"episode {s:.0f}-{e:.0f}s")
    ax.axhline(BUFFER_RADIUS, color="crimson", lw=1.6, label="buffer_radius")
    # A handful of detections report ~0.01 m -- a decode artefact, not a pose.
    # Left in the statistics, clipped out of the view so it cannot compress
    # the 0.5-0.8 m band that the convergence actually happens in.
    ax.set_ylim(0.3, 1.0)
    ax.set_xlabel("t since lock [s]")
    ax.set_ylabel("range [m]")
    ax.set_title("The two long locks, aligned at lock-on:\n"
                 "range converges onto buffer_radius and stays")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)

    fig.suptitle(f"chase bag: {os.path.basename(path)}   "
                 f"{len(det)}/{len(rows)} detections "
                 f"({100*len(det)/len(rows):.1f}%), {len(eps)} episodes",
                 fontsize=13)

    os.makedirs(OUTDIR, exist_ok=True)
    out = os.path.normpath(os.path.join(OUTDIR, "chase_overview.png"))
    fig.savefig(out, dpi=120, bbox_inches="tight")
    print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1]))
