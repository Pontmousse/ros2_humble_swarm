#!/usr/bin/env python3
"""Animate one campaign bag to a GIF: robots, modes, landmarks seen, locked capture points and the
shared /target_estimate (per-sender estimates + fused centre), with a mode timeline per robot.

  source /opt/ros/humble/setup.bash && source ros2_swarm/install/setup.bash
  python3 scripts/animate_run.py ~/rosbags/sil_share/swarm_test_campaign_SIL_5agents_20261126_130448 --focus RM5
"""
import argparse, json, math, re, sys
from pathlib import Path

import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.lines import Line2D
from matplotlib.patches import Circle, Patch, Polygon

sys.path.insert(0, str(Path(__file__).parent))
import campaign_extract as ce

ce.WANT = re.compile(r"^(/run_config|/target_estimate|/RM\d+/(ground_truth/pose|current_mode|assigned_ap|assignment_locked|landmarks))$")
CAT = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#6250d6", "#e34948"]
INK, MUTED, GRID = "#1a1a19", "#6b6a64", "#e4e3de"
MODES = {"search": ("SEARCH", "#c3c2bd"), "encapsulation": ("ENCAP", "#86b6ef"), "capture": ("CAPTURE", "#2a78d6"), "dock": ("DOCK", "#104281")}
RADIUS, SHARE_TIMEOUT = 0.2, 2.0


def step(rows, t, f, fill=None):  # value of the last message at or before each t
    ts = np.array([r for r, _ in rows]); i = np.searchsorted(ts, t, "right") - 1
    return [f(rows[j][1]) if j >= 0 else fill for j in np.atleast_1d(i)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag"); ap.add_argument("--focus", default=None, help="robot to annotate (landmarks, pull to shared centre)")
    ap.add_argument("--speed", type=float, default=10.0); ap.add_argument("--fps", type=int, default=12)
    ap.add_argument("--start", type=float, default=None); ap.add_argument("--end", type=float, default=None)
    ap.add_argument("--trail", type=float, default=30.0); ap.add_argument("--out", default=None)
    a = ap.parse_args()
    S = ce.load(a.bag)
    t0 = min(rows[0][0] for rows in S.values())
    robots = sorted({k.split("/")[1] for k in S if k.endswith("ground_truth/pose")}, key=lambda r: int(r[2:]))
    _, _, _, scene, _ = ce.config([json.loads(rows[0][1].data) for k, rows in S.items() if k.endswith("run_config")], [])
    g = lambda r, s: S.get(f"/{r}/{s}", [])
    pose = {r: np.array([(t - t0, m.x, m.y, math.radians(m.theta)) for t, m in g(r, "ground_truth/pose")]) for r in robots}
    rel = lambda rows: [(t - t0, m) for t, m in rows]
    mode, apr, lk, lm = ({r: rel(g(r, s)) for r in robots} for s in ("current_mode", "assigned_ap", "assignment_locked", "landmarks"))
    est = [(t - t0, m) for t, m in S.get("/target_estimate", [])]
    est_t = np.array([t for t, _ in est])
    arm = min(p[0, 0] for p in pose.values())
    t_a = a.start if a.start is not None else max(0.0, arm - 3)
    t_b = a.end if a.end is not None else min(p[-1, 0] for p in pose.values())
    frames = np.arange(t_a, t_b, a.speed / a.fps)
    pos = lambda r, t: np.array([np.interp(t, pose[r][:, 0], pose[r][:, k]) for k in (1, 2)])
    yaw = lambda r, t: float(np.interp(t, pose[r][:, 0], np.unwrap(pose[r][:, 3])))
    first = lambda rows, pred: next((t for t, m in rows if pred(m)), None)
    ev = {r: dict(encap=first(mode[r], lambda m: m.data == "encapsulation"), lock=first(lk[r], lambda m: m.data),
                  capture=first(mode[r], lambda m: m.data == "capture"), dock=first(mode[r], lambda m: m.data == "dock")) for r in robots}
    rj = Path(a.bag).parent / "processed" / Path(a.bag).name / "run.json"  # show times on campaign_extract's clock when available
    off = next((e["events"]["encap"] - ev[r]["encap"] for r, e in json.loads(rj.read_text())["robots"].items()
                if e["events"]["encap"] and ev[r]["encap"]), 0.0) if rj.exists() else 0.0
    tick = lambda v, _: f"{v + off:.0f}"
    ticks = np.arange(np.ceil((t_a + off) / 25) * 25, t_b + off, 25) - off

    # ---- figure: map (left), mode timeline (right) ----------------------------------------------
    fig = plt.figure(figsize=(11, 6.2), dpi=90, facecolor="white")
    axm = fig.add_axes([0.04, 0.08, 0.52, 0.84]); axt = fig.add_axes([0.63, 0.40, 0.34, 0.48]); axi = fig.add_axes([0.63, 0.06, 0.34, 0.26])
    allxy = np.vstack([p[:, 1:3] for p in pose.values()])
    lo, hi = allxy.min(0) - 0.6, allxy.max(0) + 0.6; c, h = (lo + hi) / 2, max(hi - lo) / 2
    axm.set_xlim(c[0] - h, c[0] + h); axm.set_ylim(c[1] - h, c[1] + h); axm.set_aspect("equal")
    axm.grid(color=GRID, lw=0.6); axm.tick_params(colors=MUTED, labelsize=8); [s.set_color(GRID) for s in axm.spines.values()]
    axm.set_xlabel("x [m]", color=MUTED, fontsize=8); axm.set_ylabel("y [m]", color=MUTED, fontsize=8)
    if scene:
        for o in scene["outline"]: axm.add_patch(Polygon(o, closed=True, fc="#f2f1ec", ec=MUTED, lw=1.2, zorder=1))
        for k, (x, y, nx, ny) in scene["aps"].items():
            axm.plot(x, y, "o", ms=3.5, color=MUTED, zorder=2); axm.text(x + 0.12 * nx, y + 0.12 * ny, str(k % 1000), fontsize=6.5, color=MUTED, ha="center", va="center")
    trail = {r: axm.plot([], [], "-", lw=1.2, color=CAT[i], alpha=0.45, zorder=3)[0] for i, r in enumerate(robots)}
    body = {r: axm.add_patch(Circle((0, 0), RADIUS, fc=CAT[i], ec=CAT[i], lw=2, zorder=5)) for i, r in enumerate(robots)}
    head = {r: axm.plot([], [], "-", lw=2, color=INK, zorder=6)[0] for r in robots}
    lbl = {r: axm.text(0, 0, "", fontsize=7.5, color=INK, zorder=7) for r in robots}
    aplink = {r: axm.plot([], [], ":", lw=1.4, color=CAT[i], zorder=4)[0] for i, r in enumerate(robots)}
    f = a.focus or robots[-1]
    seen = axm.scatter([], [], s=14, marker="s", fc="none", ec=INK, lw=0.8, zorder=4)
    sender = axm.scatter([], [], s=40, marker="x", lw=1.5, zorder=6)
    fused, = axm.plot([], [], "*", ms=15, mfc="#ffffff", mec=INK, mew=1.5, zorder=8)
    pull, = axm.plot([], [], "--", lw=1.6, color=INK, zorder=7)
    L = lambda **k: Line2D([], [], ls="none", **k)
    axm.legend([L(marker="o", ms=8, mfc="white", mec=MUTED, mew=2), L(marker="s", ms=5, mfc="none", mec=INK), L(marker="x", ms=7, mec=MUTED, mew=1.5),
                L(marker="*", ms=12, mfc="white", mec=INK), Line2D([], [], ls="--", color=INK), Line2D([], [], ls=":", color=MUTED)],
               ["hollow = SEARCH", f"{f} landmark memory", "shared estimate (colour = sender)", "fused centre (what the blind robot uses)",
                f"{f} blind: pull toward fused centre", "locked capture point"], loc="upper left", fontsize=7, framealpha=0.9, edgecolor=GRID)
    title = axm.set_title("", fontsize=10, color=INK, loc="left")

    # timeline: one row per robot, coloured by mode; tick at lock
    for i, r in enumerate(robots):
        segs = [(t, m.data) for t, m in mode[r]] + [(t_b, None)]
        for (ta, md), (tb, _) in zip(segs, segs[1:]):
            axt.barh(i, max(0, tb - max(ta, t_a)), left=max(ta, t_a), height=0.6, color=MODES.get(md, ("", GRID))[1], edgecolor="white", lw=1)
        on = [(t, m.data) for t, m in lk[r]] + [(t_b, False)]
        for (ta, v), (tb, _) in zip(on, on[1:]):
            if v: axt.plot([max(ta, t_a), tb], [i, i], "-", lw=2.5, color=INK, solid_capstyle="butt")
    axt.set_yticks(range(len(robots))); axt.set_yticklabels(robots, fontsize=8); axt.invert_yaxis(); axt.set_xlim(t_a, t_b)
    axt.tick_params(colors=MUTED, labelsize=8); [s.set_visible(False) for s in axt.spines.values()]
    axt.set_xlabel("time [s]", fontsize=8, color=MUTED); axt.set_xticks(ticks); axt.xaxis.set_major_formatter(tick)
    axt.legend([Patch(color=c) for _, c in MODES.values()] + [Line2D([], [], color=INK, lw=2.5)], [n for n, _ in MODES.values()] + ["AP locked"],
               ncol=5, fontsize=7, loc="lower center", bbox_to_anchor=(0.5, 1.0), frameon=False, handlelength=1.2)
    cur = axt.axvline(t_a, color=INK, lw=1)
    # focus panel: landmark count and distance to the fused centre
    lm_t = np.array([t for t, _ in lm[f]]); lm_n = np.array([len(m.markers) for _, m in lm[f]])
    axi.step(lm_t, lm_n, where="post", color=CAT[robots.index(f)], lw=1.5); axi.set_xlim(t_a, t_b)
    axi.set_xticks(ticks); axi.xaxis.set_major_formatter(tick); axi.set_ylabel(f"{f} landmarks seen", fontsize=8, color=MUTED); axi.tick_params(colors=MUTED, labelsize=8)
    [axi.spines[s].set_visible(False) for s in ("top", "right")]
    cur2 = axi.axvline(t_a, color=INK, lw=1)

    def shared(t, me):  # fresh estimates from others (by receive time) + count-weighted centre
        j = np.searchsorted(est_t, t, "right"); last = {}
        for tt, m in est[max(0, np.searchsorted(est_t, t - SHARE_TIMEOUT)):j]:
            if m.robot != me and m.count > 0: last[m.robot] = (m.x, m.y, m.count)
        if not last: return last, None
        w = sum(v[2] for v in last.values())
        return last, (sum(v[0] * v[2] for v in last.values()) / w, sum(v[1] * v[2] for v in last.values()) / w)

    def draw(t):
        for i, r in enumerate(robots):
            p, th = pos(r, t), yaw(r, t)
            md = step(mode[r], t, lambda m: m.data, "search")[0]
            body[r].center = p; body[r].set_facecolor("white" if md == "search" else CAT[i])
            head[r].set_data([p[0], p[0] + 0.32 * math.cos(th)], [p[1], p[1] + 0.32 * math.sin(th)])
            sel = (pose[r][:, 0] <= t) & (pose[r][:, 0] >= t - a.trail)
            trail[r].set_data(pose[r][sel, 1], pose[r][sel, 2])
            k = step(apr[r], t, lambda m: m.data, -1)[0]; locked = step(lk[r], t, lambda m: m.data, False)[0]
            n = step(lm[r], t, lambda m: len(m.markers), 0)[0]
            lbl[r].set_position((p[0] + 0.24, p[1] + 0.2)); lbl[r].set_text(f"{r} {MODES.get(md, (md,))[0]}" + (f" →{k % 1000}" if locked and k >= 0 else "") + (" (blind)" if n == 0 else ""))
            if scene and locked and k in scene["aps"]: aplink[r].set_data([p[0], scene["aps"][k][0]], [p[1], scene["aps"][k][1]])
            else: aplink[r].set_data([], [])
        p, th = pos(f, t), yaw(f, t)
        ms = step(lm[f], t, lambda m: m.markers, [])[0]
        R = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
        seen.set_offsets(p + (R @ (np.array([(m.x, m.y) for m in ms]) / 100.0).T).T if len(ms) else np.empty((0, 2)))
        last, cf = shared(t, f)
        sender.set_offsets(np.array([v[:2] for v in last.values()]) if last else np.empty((0, 2)))
        sender.set_color([CAT[robots.index(s)] for s in last] if last else [])
        fused.set_data(*([cf[0]], [cf[1]]) if cf else ([], []))
        blind = step(mode[f], t, lambda m: m.data, "search")[0] == "search" and len(ms) == 0 and cf is not None
        pull.set_data(*([p[0], cf[0]], [p[1], cf[1]]) if blind else ([], []))
        title.set_text(f"t = {t + off:6.1f} s   ({a.speed:g}× speed)   {Path(a.bag).name[-15:]}")
        cur.set_xdata([t, t]); cur2.set_xdata([t, t])
        return []

    out = a.out or str(Path(a.bag).name) + ".gif"
    FuncAnimation(fig, draw, frames=frames, blit=False).save(out, writer=PillowWriter(fps=a.fps))
    print(f"{out}: {len(frames)} frames, t {t_a:.1f}..{t_b:.1f} s; events", {r: {k: round(v, 1) for k, v in e.items() if v} for r, e in ev.items()})


if __name__ == "__main__":
    main()
