#!/usr/bin/env python3
"""Plot what is wrong with the marker DETECTION itself, separately from control.

The chase summary says detection ran at 27.8%, which sounds like a detector
that keeps losing the marker. It is not. Inside a lock the detector holds
90-98%, and the real defects are three quieter ones:

  1. A fixed ~-10 deg bearing offset. Isolated from the control loop by
     conditioning on heading_error: when the robot is square-on to the marker
     face the marker should sit on the camera bore, and it does not. That
     makes it an intrinsics/mounting constant, not a loop equilibrium.
  2. Outliers survive targets_filter. A single 0.01 m range reaches the
     approach law as 0.9 * (0.01 - 0.60) = -0.53 m/s, i.e. full reverse.
  3. Max range 1.31 m from a 3.556 cm marker, which is what caps the whole
     approach corridor at buffer_radius + 0.71 m.

Usage:
    python3 scripts/plot_chase_detection.py bags/chase_222249/chase_222249_0.db3
"""

import os
import struct
import sys

import numpy as np

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError:
    sys.exit("matplotlib is required: pip install matplotlib")

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from analyse_chase_bag import load, _align

BUFFER_RADIUS = 0.60
REFERENCE_FX_AT_640 = 317.03
OUTDIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                      os.pardir, "docs", "plots")


def parse_both(blob):
    """Markers AND markers_normals. The base parser drops the normals, but the
    torque law is driven entirely by them, so they are what decides whether a
    bearing offset is the detector's or the controller's."""
    b = blob[4:]
    n = struct.unpack_from("<I", b, 0)[0]
    if n > 1000 or n == 0:
        return None
    off = _align(4, 8)
    mk = []
    for _ in range(n):
        mk.append(struct.unpack_from("<dd", b, off))
        off += 16
    off = _align(off, 4)
    n2 = struct.unpack_from("<I", b, off)[0]
    off += 4
    off = _align(off, 8)
    nm = []
    for _ in range(n2):
        nm.append(struct.unpack_from("<dd", b, off))
        off += 16
    return mk, nm


def series(data, topic, rel):
    t, r, y, he = [], [], [], []
    for ts, blob in data[topic]:
        p = parse_both(blob)
        if not p:
            continue
        (mx, my), (nx, ny) = p[0][0], p[1][0]
        t.append(rel(ts))
        r.append(mx / 100.0)
        y.append(my / 100.0)
        he.append(np.degrees((-np.arctan2(nx, ny) + np.pi) % (2 * np.pi) - np.pi))
    return (np.array(t), np.array(r), np.array(y), np.array(he))


def episodes(times, gap=1.0):
    out, start, prev = [], times[0], times[0]
    for x in times[1:]:
        if x - prev > gap:
            out.append((start, prev))
            start = x
        prev = x
    out.append((start, prev))
    return out


def main(path):
    data = load(path)
    t0 = min(v[0][0] for v in data.values() if v)

    def rel(ts):
        return (ts - t0) * 1e-9

    t, r, y, he = series(data, "/RM2/targets", rel)
    tu, ru, yu, _ = series(data, "/RM2/targets_unf", rel)
    bearing = np.degrees(np.arctan2(y, r))
    eps = episodes(t)

    # Frames published vs frames with a marker, for the duty-cycle panel.
    all_t = np.array([rel(ts) for ts, _ in data["/RM2/targets"]])

    fig = plt.figure(figsize=(16, 12))
    gs = fig.add_gridspec(3, 2, hspace=0.42, wspace=0.22)

    # --- 1. duty cycle per episode ---------------------------------------
    ax = fig.add_subplot(gs[0, 0])
    duty, labels, widths = [], [], []
    for s, e in eps:
        m = (all_t >= s) & (all_t <= e)
        hit = np.sum((t >= s) & (t <= e))
        duty.append(100.0 * hit / max(1, m.sum()))
        labels.append(f"{s:.0f}s")
        widths.append(e - s)
    colours = ["tab:red" if w < 1.0 else "tab:blue" for w in widths]
    ax.bar(range(len(duty)), duty, color=colours)
    ax.axhline(100, color="0.5", lw=0.8)
    ax.set_xticks(range(len(duty)))
    ax.set_xticklabels(labels, rotation=45, fontsize=7)
    ax.set_ylim(0, 110)
    ax.set_ylabel("frames with a marker [%]")
    ax.set_title("Inside a lock the detector is 90-98% reliable.\n"
                 "27.8% overall just means the marker was out of frame.")
    ax.grid(alpha=0.3, axis="y")

    # --- 2. bearing(t): never crosses zero -------------------------------
    ax = fig.add_subplot(gs[0, 1])
    ax.plot(t, bearing, ".", ms=2.5, color="tab:red", label="bearing to marker")
    ax.plot(t, he, ".", ms=2.0, alpha=0.35, color="tab:blue",
            label="heading_error (what trq_cmd regulates)")
    ax.axhline(0, color="k", lw=1.2)
    ax.axhline(bearing.mean(), color="tab:red", ls="--", lw=1.2,
               label=f"bearing mean {bearing.mean():+.1f} deg")
    ax.set_ylim(-45, 45)
    ax.set_xlabel("t [s]")
    ax.set_ylabel("angle [deg]")
    nx_cross = int(np.sum(np.diff(np.sign(bearing)) != 0))
    nh_cross = int(np.sum(np.diff(np.sign(he)) != 0))
    ax.set_title(f"heading_error crosses zero {nh_cross}x (closed loop);\n"
                 f"bearing crosses {nx_cross}x in 929 detections (NOT regulated)")
    ax.grid(alpha=0.3)
    ax.legend(loc="lower left", fontsize=7)

    # --- 3. the discriminator --------------------------------------------
    ax = fig.add_subplot(gs[1, 0])
    ax.scatter(he, bearing, s=7, alpha=0.3, color="tab:blue")
    bins = [(0, 2), (2, 5), (5, 10), (10, 20), (20, 90)]
    for lo, hi in bins:
        m = (np.abs(he) >= lo) & (np.abs(he) < hi)
        if m.sum() > 5:
            ax.plot([-hi, hi], [bearing[m].mean()] * 2, color="crimson", lw=2.0)
    m0 = np.abs(he) < 2
    ax.axhline(0, color="k", lw=1.0)
    ax.axvline(0, color="k", lw=1.0)
    ax.set_xlim(-50, 50)
    ax.set_ylim(-30, 20)
    ax.set_xlabel("heading_error [deg]  (0 = robot square-on to marker face)")
    ax.set_ylabel("bearing to marker [deg]")
    px = REFERENCE_FX_AT_640 * np.tan(np.radians(abs(bearing[m0].mean())))
    ax.set_title(f"At heading_error ~ 0 ({m0.sum()} samples) the bearing is still "
                 f"{bearing[m0].mean():+.1f} deg.\n"
                 f"A loop equilibrium would go to 0 here -> this is a FIXED "
                 f"offset,\n~{px:.0f} px of principal point at fx={REFERENCE_FX_AT_640}")
    ax.grid(alpha=0.3)

    # --- 4. outliers survive the filter ----------------------------------
    ax = fig.add_subplot(gs[1, 1])
    ax.plot(tu, ru, ".", ms=4, alpha=0.5, color="tab:orange",
            label="targets_unf (raw)")
    ax.plot(t, r, ".", ms=2.5, alpha=0.8, color="tab:blue",
            label="targets (filtered)")
    bad = r < 0.20
    ax.plot(t[bad], r[bad], "x", ms=9, mew=2.0, color="crimson",
            label=f"range < 0.20 m surviving the filter ({bad.sum()})")
    ax.axhline(BUFFER_RADIUS, color="crimson", lw=1.3, ls="--",
               label="buffer_radius")
    ax.set_ylim(0, 1.5)
    ax.set_xlabel("t [s]")
    ax.set_ylabel("range [m]")
    ax.set_title("targets_filter changes 929 detections into 929 detections.\n"
                 "A 0.01 m reading becomes 0.9*(0.01-0.60) = -0.53 m/s: "
                 "full reverse.")
    ax.grid(alpha=0.3)
    ax.legend(loc="upper left", fontsize=7)

    # --- 5. noise at a held standoff -------------------------------------
    ax = fig.add_subplot(gs[2, 0])
    w = (t > 185) & (t < 215)
    ax.plot(t[w], r[w], "-", lw=0.8, color="tab:blue")
    ax.plot(t[w], r[w], ".", ms=3, color="tab:blue")
    ax.axhline(r[w].mean(), color="k", ls="--", lw=1.0,
               label=f"mean {r[w].mean():.3f} m")
    for k in (1, -1):
        ax.axhline(r[w].mean() + k * r[w].std(), color="0.5", ls=":", lw=1.0)
    ax.set_xlabel("t [s]")
    ax.set_ylabel("range [m]")
    ax.set_title(f"Range noise while holding station, 185-215 s:\n"
                 f"std {r[w].std()*1000:.0f} mm, peak-to-peak "
                 f"{(r[w].max()-r[w].min())*1000:.0f} mm on a stationary target")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)

    # --- 6. lateral vs range ---------------------------------------------
    ax = fig.add_subplot(gs[2, 1])
    good = (r > 0.3) & (r < 1.2)
    ax.scatter(r[good], y[good], s=7, alpha=0.3, color="tab:blue")
    sl, ic = np.polyfit(r[good], y[good], 1)
    xs = np.linspace(0, 1.3, 10)
    ax.plot(xs, sl * xs + ic, "crimson", lw=1.6,
            label=f"fit  y = {sl:+.3f}·range {ic:+.3f}")
    ax.plot(xs, np.tan(np.radians(bearing.mean())) * xs, "k--", lw=1.2,
            label=f"pure {bearing.mean():+.1f} deg angle (through origin)")
    ax.axhline(y[good].mean(), color="tab:green", ls=":", lw=1.4,
               label=f"pure mounting offset ({y[good].mean():+.3f} m)")
    ax.axhline(0, color="0.5", lw=0.8)
    ax.set_xlim(0, 1.3)
    ax.set_xlabel("range [m]")
    ax.set_ylabel("lateral y [m]")
    ax.set_title("Offset grows with range and extrapolates through the origin\n"
                 "=> an ANGLE (optics), not a translation (mounting)")
    ax.grid(alpha=0.3)
    ax.legend(loc="lower left", fontsize=7)

    fig.suptitle("Marker detection defects -- "
                 f"{os.path.basename(path)}", fontsize=14)
    os.makedirs(OUTDIR, exist_ok=True)
    out = os.path.normpath(os.path.join(OUTDIR, "chase_detection.png"))
    fig.savefig(out, dpi=120, bbox_inches="tight")
    print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1]))
