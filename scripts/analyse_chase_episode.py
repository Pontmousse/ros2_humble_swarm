#!/usr/bin/env python3
"""Check whether cp_controller's approach law is being executed as designed.

Separates two very different failures that look identical from outside:
a controller that is misbehaving, versus a controller that is doing exactly
what it was told with a wrong setpoint or a wrong distance scale.

The law under test (cp_proportional_controller):
    linear.x = approach_gain * (range - buffer_radius)
with approach_gain = 0.9 s^-1 and buffer_radius = 0.60 m.
"""

import sys

import numpy as np

from analyse_chase_bag import load, parse_floats, parse_markers

APPROACH_GAIN = 0.9
BUFFER_RADIUS = 0.60


def main(path, *windows):
    data = load(path)
    t0 = min(v[0][0] for v in data.values() if v)

    def rel(ts):
        return (ts - t0) * 1e-9

    tgt = [(rel(ts), parse_markers(b)) for ts, b in data["/RM2/targets"]]
    tgt = [(t, m[0]) for t, m in tgt if m]
    cmd = [(rel(ts), parse_floats(b, 6)) for ts, b in data["/RM2/cmd_vel"]]

    ct = np.array([t for t, _ in cmd])
    cx = np.array([v[0] for _, v in cmd])
    cz = np.array([v[5] for _, v in cmd])

    for win in windows:
        s, e = (float(v) for v in win.split(":"))
        seg = [(t, m) for t, m in tgt if s <= t <= e]
        if not seg:
            print(f"\n=== {s}-{e}s: no detections ===")
            continue

        tt = np.array([t for t, _ in seg])
        rng = np.array([m[0] for _, m in seg]) / 100.0   # cm -> m
        lat = np.array([m[1] for _, m in seg]) / 100.0

        print(f"\n=== episode {s:.0f}-{e:.0f}s "
              f"({len(seg)} detections, {tt[-1]-tt[0]:.1f}s) ===")
        print(f"  range  start {rng[0]:.2f}  end {rng[-1]:.2f}  "
              f"min {rng.min():.2f}  max {rng.max():.2f}  m")
        print(f"  lateral start {lat[0]:+.2f}  end {lat[-1]:+.2f}  "
              f"|mean| {np.abs(lat).mean():.2f}  m")

        # Did range converge toward buffer_radius?
        err0 = abs(rng[0] - BUFFER_RADIUS)
        err1 = abs(rng[-1] - BUFFER_RADIUS)
        print(f"  |range - buffer_radius|  start {err0:.3f} -> end {err1:.3f} m")

        # Fraction of time inside the keep-out radius: that is where the law
        # commands reverse, so a high number explains a robot that backs off
        # instead of approaching.
        inside = 100.0 * np.mean(rng < BUFFER_RADIUS)
        print(f"  inside buffer_radius: {inside:.0f}% of detections")

        # Compare the commanded speed against what the law prescribes for the
        # range reported at (nearly) the same instant.
        idx = np.searchsorted(ct, tt)
        idx = np.clip(idx, 0, len(ct) - 1)
        actual = cx[idx]
        predicted = APPROACH_GAIN * (rng - BUFFER_RADIUS)
        ok = np.abs(actual) > 1e-6
        if ok.sum() > 5:
            r = np.corrcoef(predicted[ok], actual[ok])[0, 1]
            slope = np.polyfit(predicted[ok], actual[ok], 1)[0]
            print(f"  law check: corr(predicted, actual linear.x) = {r:+.3f}, "
                  f"slope {slope:.2f}")
            print(f"    predicted  min {predicted.min():+.3f} "
                  f"max {predicted.max():+.3f}")
            print(f"    actual     min {actual.min():+.3f} "
                  f"max {actual.max():+.3f}  m/s")
            agree = 100.0 * np.mean(
                np.sign(predicted[ok]) == np.sign(actual[ok]))
            print(f"    sign agreement {agree:.0f}%")

        step = max(1, len(seg) // 12)
        print("    t       range   lat    cmd.x   cmd.z")
        for i in range(0, len(seg), step):
            print(f"   {tt[i]:6.1f}  {rng[i]:6.2f}  {lat[i]:+5.2f}  "
                  f"{actual[i]:+6.3f}  {cz[idx[i]]:+6.3f}")


if __name__ == "__main__":
    main(sys.argv[1], *sys.argv[2:])
