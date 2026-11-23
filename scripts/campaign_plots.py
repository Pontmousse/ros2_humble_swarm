#!/usr/bin/env python3
"""Paper figures and tables from campaign_extract.py output (no ROS needed).

Groups runs by (mode, N); warns when a group mixes config ids; drops QC-DEGRADED
runs unless --include-degraded. The representative run per (mode, N) is the
QC-OK run with the median mission time (override with --rep). Colour follows the
entity: robots RM1..RM6 take categorical slots 1..6, SIL/HIL take slots 1/2.

    python3 scripts/campaign_plots.py                      # ~/rosbags/processed -> ~/rosbags/processed/paper
    python3 scripts/campaign_plots.py --include-degraded --rep swarm_test_campaign_SIL_3agents_20261116_161402
"""
import argparse
import csv
import json
import math
from collections import defaultdict
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

PHASES = ("formation", "negotiation", "approach", "docked")
CAT = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#6250d6", "#e34948"]
MODE_C = {"SIL": CAT[0], "HIL": CAT[1]}
LAYER_C = dict(flocking=CAT[0], avoidance=CAT[1], preparation=CAT[2], capture=CAT[3], spacecraft=CAT[4])
LAYER_NAME = dict(spacecraft="bounding box")
PHASE_SHADE = ["#f0efec", "#e1e0db", "#d0cfc8", "#bdbcb4"]
EVENTS = (("first_lock", "^"), ("sync", "o"), ("capture", "|"), ("dock", "s"))
INK, INK2 = "#0b0b0b", "#52514e"
plt.rcParams.update({"font.size": 8, "axes.edgecolor": INK2, "axes.labelcolor": INK, "xtick.color": INK2, "ytick.color": INK2,
                     "axes.grid": True, "grid.color": "#e6e5e0", "grid.linewidth": 0.6, "axes.spines.top": False,
                     "axes.spines.right": False, "lines.linewidth": 1.4, "legend.frameon": False, "figure.dpi": 100,
                     "savefig.dpi": 300, "savefig.bbox": "tight", "figure.constrained_layout.use": True, "figure.facecolor": "white", "axes.facecolor": "#fcfcfb"})
robot_c = lambda r: CAT[(int(r[2:]) - 1) % len(CAT)]
fnum = lambda v: float(v) if v not in ("", None, "None") else math.nan


def wilson(successes, count):  # copy of swarm_controller/consensus_benchmark.py:wilson (that module needs ROS)
    if not count: return (math.nan, math.nan)
    z = 1.959963984540054; p = successes / count; den = 1 + z * z / count
    c, rad = (p + z * z / (2 * count)) / den, z * math.sqrt(p * (1 - p) / count + z * z / (4 * count * count)) / den
    return max(0.0, c - rad), min(1.0, c + rad)


def mstd(v):
    v = np.array([x for x in v if x == x], float)
    return "-" if not len(v) else f"{v.mean():.3g}" if len(v) == 1 else f"{v.mean():.3g} ± {v.std(ddof=1):.2g}"


def table(name, header, rows, out, md):
    with open(out / f"{name}.csv", "w", newline="") as fh: csv.writer(fh).writerows([header] + rows)
    md.append(f"\n### {name}\n\n| " + " | ".join(header) + " |\n|" + "---|" * len(header) + "\n"
              + "".join("| " + " | ".join(map(str, r)) + " |\n" for r in rows))


def save(fig, out, name, md, caption):
    fig.savefig(out / f"{name}.png"); plt.close(fig); md.append(f"\n**{name}** — {caption}\n\n![{name}]({name}.png)\n")


def shade(ax, bounds, t0=0.0, labels=False):
    for k in range(4):
        ax.axvspan(bounds[k] - t0, bounds[k + 1] - t0, color=PHASE_SHADE[k], lw=0, zorder=0)
        if labels: ax.text((bounds[k] + bounds[k + 1]) / 2 - t0, 1.0, PHASES[k], transform=ax.get_xaxis_transform(),
                           ha="center", va="bottom", fontsize=6, color=INK2)


def by_n(ax, rows, key, ylabel, hline=None):
    """Dots per run/robot at x = N (jittered by mode), mean as a bar tick."""
    for i, mode in enumerate(sorted({r["mode"] for r in rows})):
        g = defaultdict(list)
        for r in rows:
            if r["mode"] == mode and fnum(r.get(key)) == fnum(r.get(key)): g[int(r["N"])].append(fnum(r[key]))
        for n, v in g.items():
            x = n + (i - 0.5) * 0.25 * (len({r["mode"] for r in rows}) > 1)
            ax.scatter(np.full(len(v), x) + np.linspace(-0.04, 0.04, len(v)), v, s=14, color=MODE_C[mode], zorder=3,
                       edgecolor="white", lw=0.5, label=mode if n == min(g) else None)
            ax.hlines(np.mean(v), x - 0.12, x + 0.12, color=INK, lw=1.2, zorder=4)
    if hline: [ax.axhline(y, color=INK2, ls=":", lw=0.9) for y in hline]
    ax.set_xticks(sorted({int(r["N"]) for r in rows})); ax.set_xlabel("N agents"); ax.set_ylabel(ylabel)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--in", dest="src", default="~/rosbags/processed")
    ap.add_argument("--out", default=None)
    ap.add_argument("--include-degraded", action="store_true")
    ap.add_argument("--rep", nargs="*", default=[])
    a = ap.parse_args()
    src = Path(a.src).expanduser(); out = Path(a.out).expanduser() if a.out else src / "paper"; out.mkdir(parents=True, exist_ok=True)
    runs = list(csv.DictReader(open(src / "runs.csv"))); robots = list(csv.DictReader(open(src / "robots.csv")))
    notes, md = [], ["# Capture campaign — figures and tables\n"]
    kept = [r for r in runs if a.include_degraded or r["qc"] == "OK"]
    dropped = [r["bag"] for r in runs if r not in kept]
    if dropped: notes.append(f"excluded as QC-DEGRADED: {', '.join(dropped)}")
    groups = defaultdict(list)
    for r in kept: groups[(r["mode"], int(r["N"]))].append(r)
    for (mode, n), g in sorted(groups.items()):
        if len({r["config_id"] for r in g}) > 1: notes.append(f"{mode} N={n} mixes config ids {sorted({r['config_id'] for r in g})}")
    ns = sorted({int(r["N"]) for r in kept})
    if len({r["config_id"] for r in kept if r["mode"] == "SIL"}) > 1 or len({r["config_id"] for r in kept if r["mode"] == "HIL"}) > 1:
        notes.append("the sweep over N mixes config ids: " + ", ".join(f"{m} N={n}: {sorted({r['config_id'] for r in g})}" for (m, n), g in sorted(groups.items())))
    keep = {r["bag"] for r in kept}
    rob = [r for r in robots if r["bag"] in keep]
    # Representative runs: median mission time per (mode, N), unless overridden.
    reps = {}
    for key, g in sorted(groups.items()):
        forced = [r for r in g if r["bag"] in a.rep]
        ok = sorted([r for r in g if fnum(r["mission_time"]) == fnum(r["mission_time"])], key=lambda r: fnum(r["mission_time"])) or g
        reps[key] = (forced or [ok[len(ok) // 2]])[0]["bag"]
    load = lambda bag: (json.loads((src / bag / "run.json").read_text()), np.load(src / bag / "series.npz"))
    R = {k: load(b) for k, b in reps.items()}
    print("representative runs:", {f"{m} N={n}": b for (m, n), b in reps.items()})

    # F1 timeline ------------------------------------------------------------------
    fig, axs = plt.subplots(len(R), 1, figsize=(7.2, 0.5 + 0.55 * sum(r[0]["meta"]["N"] for r in R.values()) + 0.3 * len(R)), squeeze=False)
    for ax, ((mode, n), (run, s)) in zip(axs[:, 0], R.items()):
        rb = run["meta"]["robots"]
        for i, r in enumerate(rb):
            b, e = run["robots"][r]["bounds"], run["robots"][r]["events"]
            for k in range(4): ax.barh(i, b[k + 1] - b[k], left=b[k], color=PHASE_SHADE[k], edgecolor="white", lw=1, height=0.7)
            for key, mk in EVENTS:
                if e.get(key) is not None: ax.plot(e[key], i, mk, color=robot_c(r), ms=6 if mk != "|" else 12, mew=1.5)
        c = run["consensus"]
        if c.get("all_armed"): ax.axvline(c["all_armed"], color=INK2, ls=":", lw=0.9)
        ax.set_yticks(range(len(rb))); ax.set_yticklabels(rb); ax.invert_yaxis(); ax.grid(axis="y", visible=False)
        ax.set_title(f"{mode} N={n}  ({run['meta']['bag'][-15:]})", loc="left", fontsize=8, color=INK)
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch
    fig.legend(handles=[Patch(color=PHASE_SHADE[k], label=p) for k, p in enumerate(PHASES)] +
               [Line2D([], [], ls="", marker=mk, color=INK2, ms=5 if mk != "|" else 9, mew=1.5, label=k.replace("_", " ")) for k, mk in EVENTS] +
               [Line2D([], [], ls=":", color=INK2, label="all armed")], ncol=9, loc="lower center", bbox_to_anchor=(0.5, 1.0), fontsize=6)
    axs[-1, 0].set_xlabel("time since bag start [s]")
    save(fig, out, "fig01_timeline", md, "phase windows per robot (formation → negotiation → approach → docked); ▲ first AP lock, ● capture_sync, | capture mode, ■ dock; dotted = all armed.")

    # F2 trajectories --------------------------------------------------------------
    fig, axs = plt.subplots(1, len(R), figsize=(2.6 * len(R), 2.8), squeeze=False, sharex=True, sharey=True)
    for ax, ((mode, n), (run, s)) in zip(axs[0], R.items()):
        for k in [k for k in s.files if k.startswith("outline/")]:
            V = s[k]; ax.fill(V[:, 0], V[:, 1], color="#e1e0db", ec=INK2, lw=0.8)
        if "aps" in s.files: ax.plot(s["aps"][:, 1], s["aps"][:, 2], "x", color=INK2, ms=3, mew=0.8)
        for r in run["meta"]["robots"]:
            c = robot_c(r); ax.plot(s[f"{r}/vx"], s[f"{r}/vy"], color=c, lw=0.9, ls="--", alpha=0.8)
            ax.plot(s[f"{r}/x"], s[f"{r}/y"], color=c, lw=1.2, label=r)
            ok = np.isfinite(s[f"{r}/x"]); ax.plot(s[f"{r}/x"][ok][0], s[f"{r}/y"][ok][0], "o", color=c, ms=4, mec="white")
            d = run["robots"][r]["events"]["dock"]
            if d is not None:
                i = min(np.searchsorted(s["t"], d), len(s["t"]) - 1); ax.plot(s[f"{r}/x"][i], s[f"{r}/y"][i], "s", color=c, ms=5, mec="white")
        ax.set_aspect("equal"); ax.set_title(f"{mode} N={n}", loc="left", fontsize=8); ax.set_xlabel("x [m]")
    axs[0, 0].set_ylabel("y [m]")
    h = {l.get_label(): l for ax in axs[0] for l in ax.get_lines() if l.get_label().startswith("RM")}
    fig.legend(handles=[h[k] for k in sorted(h, key=lambda r: int(r[2:]))], ncol=6, loc="lower center", bbox_to_anchor=(0.5, 1.0), fontsize=7)
    save(fig, out, "fig02_trajectories", md, f"physical path (solid, {R[next(iter(R))][0]['meta']['position_source']}) and virtual spacecraft (dashed); ○ start, ■ dock, × capture points.")

    # F3-F6 time series for representative runs ------------------------------------
    def ts_fig(name, caption, key_fn, ylabel, hlines=(), ylim=None):
        fig, axs = plt.subplots(len(R), 1, figsize=(7.2, 1.5 * len(R) + 0.3), sharex=False, squeeze=False)
        for ax, ((mode, n), (run, s)) in zip(axs[:, 0], R.items()):
            shade(ax, run["swarm_bounds"], labels=ax is axs[0, 0])
            for lbl, y, c in key_fn(run, s): ax.plot(s["t"], y, color=c, lw=1.1, label=lbl)
            for y, txt in hlines: ax.axhline(y, color=INK2, ls=":", lw=0.9); ax.text(1.0, y, f" {txt}", transform=ax.get_yaxis_transform(), fontsize=6, color=INK2, va="center")
            ax.set_ylabel(ylabel); ax.set_title(f"{mode} N={n}", loc="left", fontsize=8, pad=10 if ax is axs[0, 0] else 3)
            if ylim: ax.set_ylim(*ylim)
            if len(ax.get_lines()) > len(hlines) + 1: ax.legend(fontsize=6, ncol=6, loc="upper left")
        axs[-1, 0].set_xlabel("time since bag start [s]")
        save(fig, out, name, md, caption)
    per_robot = lambda key, scale=1.0: (lambda run, s: [(r, s[f"{r}/{key}"] * scale, robot_c(r)) for r in run["meta"]["robots"]])
    ts_fig("fig03_tracking", "virtual-vs-physical tracking error (virtual spacecraft vs KF estimate, matched on header stamps); shading = swarm phase.",
           per_robot("trk", 1e3), "tracking err [mm]")
    P = next(iter(R.values()))[0]["params"]
    if any(run["meta"]["N"] > 1 for run, _ in R.values()):
        def seps(run, s): return [(k[4:], s[k], CAT[i]) for i, k in enumerate(sorted(x for x in s.files if x.startswith("sep/")))]
        R_all = R; R = {k: v for k, v in R_all.items() if v[0]["meta"]["N"] > 1}
        ts_fig("fig04_separation", "pairwise centre distance vs avoidance radii; contact = 2 × robot radius.", seps, "separation [m]",
               [(P["R_influence"], "influence"), (P["R_saturate"], "saturation"), (P["R_contact"], "contact")])
        R = R_all
    ts_fig("fig05_pointing", "pointing error: virtual spacecraft yaw minus bearing to the target centroid (pointing law active in formation/negotiation only).",
           lambda run, s: [(r, np.degrees(np.abs(s[f"{r}/pointing"])), robot_c(r)) for r in run["meta"]["robots"]], "|pointing err| [deg]")
    ts_fig("fig05b_yawrate", "virtual spacecraft yaw rate (ring-up check).", per_robot("yawrate", 1.0), "yaw rate [rad/s]")

    # F6 range to assigned AP aligned at dock (all kept runs) -------------------------
    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(7.2, 2.6), gridspec_kw=dict(width_ratios=[2, 1]))
    tg = np.arange(-30, 3, 0.1)
    for mode in sorted({r["mode"] for r in kept}):
        curves = []
        for r in [r for r in kept if r["mode"] == mode]:
            run, s = load(r["bag"])
            for rb in run["meta"]["robots"]:
                d = run["robots"][rb]["events"]["dock"]
                if d is None: continue
                y = np.interp(tg, s["t"] - d, s[f"{rb}/range"], left=np.nan, right=np.nan); curves.append(y)
                ax.plot(tg, y, color=MODE_C[mode], lw=0.5, alpha=0.35)
        if curves:
            C = np.array(curves); full = np.isfinite(C).sum(0) >= 0.8 * len(C)  # mean only where most curves exist
            m, sd = np.where(full, np.nanmean(C, 0), np.nan), np.where(full, np.nanstd(C, 0), np.nan)
            ax.plot(tg, m, color=MODE_C[mode], lw=2, label=f"{mode} mean (n={len(C)})"); ax.fill_between(tg, m - sd, m + sd, color=MODE_C[mode], alpha=0.15, lw=0)
    for y, txt in ((P["contact_distance"], "contact gate"), (0.20, "approach equilibrium")):
        ax.axhline(y, color=INK2, ls=":", lw=0.9); ax.text(-29.5, y, txt, fontsize=6, color=INK2, va="bottom")
    ax.set_xlabel("time relative to dock [s]"); ax.set_ylabel("range to assigned AP [m]"); ax.set_ylim(0, None); ax.legend(fontsize=7)
    for mode in sorted({r["mode"] for r in rob}):
        f = [(fnum(r["final_range"]), math.degrees(fnum(r["final_face"]))) for r in rob if r["mode"] == mode and r["phase"] == "docked" and r["docked"] == "True"]
        if f: ax2.scatter(*zip(*f), s=16, color=MODE_C[mode], edgecolor="white", lw=0.5, label=mode, zorder=3)
    ax2.axvline(P["contact_distance"], color=INK2, ls=":", lw=0.9); ax2.axhline(math.degrees(P["contact_heading_tolerance"]), color=INK2, ls=":", lw=0.9)
    ax2.set_xlabel("final range [m]"); ax2.set_ylabel("final |face heading err| [deg]"); ax2.legend(fontsize=7, loc="upper left")
    save(fig, out, "fig06_capture_range", md, "left: range to the assigned capture point, all robots of all kept runs aligned at DOCK (mean ± std); right: dock geometry vs the contact gate (true pose in SIL — the stack judges contact on the KF pose).")

    # F7 consensus / F8 auction / F11 success: run-level dots vs N -----------------
    multi = [r for r in kept if int(r["N"]) > 1]
    if multi:
        fig, axs = plt.subplots(1, 4, figsize=(7.2, 2.0))
        for ax, (k, lbl, sc) in zip(axs, (("cons_action_spread", "agreed action-time spread [ms]", 1e3), ("cons_sync_spread", "capture_sync spread [ms]", 1e3),
                                          ("cons_capture_spread", "capture-entry spread [ms]", 1e3), ("cons_rounds", "consensus rounds", 1))):
            by_n(ax, [dict(r, **{k: fnum(r[k]) * sc}) for r in multi], k, lbl)
        axs[0].legend(fontsize=6)
        save(fig, out, "fig07_consensus", md, "capture-time consensus vs N (dot = run, tick = mean). Agreed action-time spread is clock-free; observed spreads use recorder receive time (on HIL they include WiFi jitter).")
    fig, axs = plt.subplots(1, 4, figsize=(7.2, 2.0))
    for ax, (k, lbl, sc) in zip(axs, (("auc_lock_time", "time to all-locked [s]", 1), ("auc_churn", "claim changes", 1),
                                      ("auc_gap_bid", "bid optimality gap [%]", 100), ("auc_gap_dist", "distance gap [%]", 100))):
        by_n(ax, [dict(r, **{k: fnum(r[k]) * sc}) for r in kept], k, lbl)
    axs[0].legend(fontsize=6)
    save(fig, out, "fig08_auction", md, "AP auction vs N: time from formation start to every robot holding its final lock, claim churn, and gap of the final assignment to the Hungarian optimum on the recorded bids and on true distance.")

    # F3b/F4b/F5b/F9 per-phase dots ----------------------------------------------------
    def phase_fig(name, caption, key, ylabel, sc=1.0, rows=rob, hline=None, phases=PHASES):
        fig, axs = plt.subplots(1, len(phases), figsize=(7.2, 2.0), sharey=True)
        for ax, p in zip(np.atleast_1d(axs), phases):
            by_n(ax, [dict(r, **{key: fnum(r[key]) * sc}) for r in rows if r["phase"] == p], key, ylabel if p == phases[0] else "", hline)
            ax.set_title(p, fontsize=8, loc="left")
        np.atleast_1d(axs)[0].legend(fontsize=6)
        save(fig, out, name, md, caption)
    phase_fig("fig03b_tracking_by_phase", "tracking error p95 per robot per run, by phase and N.", "trk_p95", "tracking p95 [mm]", 1e3)
    phase_fig("fig05c_pointing_by_phase", "pointing error p95 per robot per run.", "pnt_p95", "pointing p95 [deg]", 180 / math.pi, phases=PHASES[:2])
    swarm_rows = [dict(r, phase=p, min_sep=fnum(r[f"{p}_min_sep"])) for r in multi for p in PHASES]
    if multi: phase_fig("fig04b_minsep_by_phase", "minimum pairwise separation per run, by swarm phase.", "min_sep", "min separation [m]",
                        rows=swarm_rows, hline=(P["R_saturate"], P["R_contact"]))
    phase_fig("fig09_dv_by_phase", "delta-v per robot per phase, ∫|F_applied|/m dt.", "dv", "Δv [mm/s]", 1e3)
    phase_fig("fig09b_saturation", "fraction of samples with the applied force at the actuator cap.", "sat_F", "force saturated [%]", 100)
    # Layer demand share (pre-clamp |F| of each additive contributor), stacked per phase.
    fig, axs = plt.subplots(1, 4, figsize=(7.2, 2.2), sharey=True)
    keys = sorted({(r["mode"], int(r["N"])) for r in rob})
    for ax, p in zip(axs, PHASES):
        bottom = np.zeros(len(keys))
        for L, c in LAYER_C.items():
            v = np.array([np.nanmean([fnum(r[f"{L}_F"]) for r in rob if (r["mode"], int(r["N"])) == k and r["phase"] == p] or [math.nan]) for k in keys])
            tot = np.array([np.nansum([np.nanmean([fnum(r[f"{L2}_F"]) for r in rob if (r["mode"], int(r["N"])) == k and r["phase"] == p] or [0]) for L2 in LAYER_C]) for k in keys])
            sh = np.nan_to_num(v / np.where(tot > 0, tot, np.nan)) * 100
            ax.bar(range(len(keys)), sh, bottom=bottom, color=c, edgecolor="white", lw=1, width=0.7, label=LAYER_NAME.get(L, L) if p == PHASES[0] else None); bottom += sh
        ax.set_xticks(range(len(keys))); ax.set_xticklabels([f"{m}\nN={n}" for m, n in keys], fontsize=6); ax.set_title(p, loc="left", fontsize=8); ax.grid(axis="x", visible=False)
    axs[0].set_ylabel("share of layer force demand [%]"); fig.legend(ncol=5, loc="upper center", bbox_to_anchor=(0.5, 1.08), fontsize=7)
    save(fig, out, "fig09c_layer_share", md, "which behaviour layer dominates the (pre-clamp) force demand in each phase; the sum is clamped to the actuator cap.")

    # F10 SIL vs HIL tracking CDF per N ----------------------------------------------
    fig, axs = plt.subplots(1, len(ns), figsize=(2.4 * len(ns), 2.0), sharey=True, squeeze=False)
    for ax, n in zip(axs[0], ns):
        for mode in sorted({r["mode"] for r in kept}):
            v = np.concatenate([np.ravel([s[f"{rb}/trk"] for rb in run["meta"]["robots"]]) for run, s in
                                [load(r["bag"]) for r in kept if r["mode"] == mode and int(r["N"]) == n]] or [np.zeros(0)])
            v = np.sort(v[np.isfinite(v)]) * 1e3
            if len(v): ax.plot(v, np.linspace(0, 1, len(v)), color=MODE_C[mode], label=mode)
        ax.set_title(f"N={n}", loc="left", fontsize=8); ax.set_xlabel("tracking err [mm]")
    axs[0, 0].set_ylabel("CDF"); axs[0, 0].legend(fontsize=7)
    hil = any(r["mode"] == "HIL" for r in kept)
    save(fig, out, "fig10_sil_vs_hil_tracking", md, "tracking-error CDF pooled over robots and runs, SIL vs HIL" + ("" if hil else " (no HIL runs yet — SIL only)") + ".")

    # F11 success + time to dock -----------------------------------------------------
    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(5.0, 2.0))
    for i, mode in enumerate(sorted({r["mode"] for r in kept})):
        for n in ns:
            rr = [r for r in rob if r["mode"] == mode and int(r["N"]) == n and r["phase"] == "docked"]
            if not rr: continue
            k = sum(r["success"] == "True" for r in rr); lo, hi = wilson(k, len(rr)); x = n + (i - 0.5) * 0.2 * hil
            ax.errorbar(x, 100 * k / len(rr), yerr=[[100 * (k / len(rr) - lo)], [100 * (hi - k / len(rr))]], fmt="o", color=MODE_C[mode], ms=4, capsize=2,
                        label=mode if n == ns[0] else None)
    ax.set_ylim(-5, 105); ax.set_xticks(ns); ax.set_xlabel("N agents"); ax.set_ylabel("robots docked at assigned AP [%]"); ax.legend(fontsize=6)
    by_n(ax2, [r for r in rob if r["phase"] == "docked"], "time_to_dock", "time to dock [s]")
    save(fig, out, "fig11_success", md, "per-robot capture success (docked by the stack at its assigned AP) with Wilson 95% interval; time from formation start to dock.")

    # Tables -------------------------------------------------------------------------
    md.append("\n## Tables\n")
    t1 = []
    for (mode, n), g in sorted(groups.items()):
        rr = [r for r in rob if r["bag"] in {x["bag"] for x in g} and r["phase"] == "docked"]
        tot = sum(1 for r in runs if r["mode"] == mode and int(r["N"]) == n)
        t1.append([mode, n, f"{len(g)}/{tot}", f"{sum(x['success'] == 'True' for x in g)}/{len(g)}", f"{sum(r['success'] == 'True' for r in rr)}/{len(rr)}",
                   f"{sum(r['geom_ok'] == 'True' for r in rr)}/{len(rr)}", mstd([fnum(x["mission_time"]) for x in g]), " ".join(sorted({x["config_id"] for x in g}))])
    table("T1_campaign", ["mode", "N", "runs kept/total", "runs all-docked", "robots docked@AP", "robots in true gate", "mission time [s]", "config"], t1, out, md)
    t2, cols = [], [("trk_p95", 1e3), ("trk_true_p95", 1e3), ("pnt_p95", 180 / math.pi), ("dv", 1e3), ("sat_F", 100), ("range_min", 1), ("lateral_p95", 1)]
    for (mode, n), g in sorted(groups.items()):
        bags = {x["bag"] for x in g}
        for p in PHASES:
            rr = [r for r in rob if r["bag"] in bags and r["phase"] == p]
            t2.append([mode, n, p] + [mstd([fnum(r[c]) * s for r in rr]) for c, s in cols] + [mstd([fnum(x[f"{p}_min_sep"]) for x in g])])
    table("T2_per_phase", ["mode", "N", "phase", "trk p95 [mm]", "trk vs truth p95 [mm]", "pointing p95 [deg]", "Δv [mm/s]", "force sat [%]",
                           "min range [m]", "lateral p95 [m]", "min sep [m]"], t2, out, md)
    t3, cols3 = [], [("auc_lock_time", 1), ("auc_churn", 1), ("auc_gap_bid", 100), ("auc_gap_dist", 100), ("auc_conflict_s", 1), ("cons_rounds", 1),
                     ("cons_ready_to_armed", 1), ("cons_action_spread", 1e3), ("cons_sync_spread", 1e3), ("cons_capture_spread", 1e3)]
    for (mode, n), g in sorted(groups.items()):
        bags = {x["bag"] for x in g}; rr = [r for r in rob if r["bag"] in bags and r["phase"] == "docked" and r["docked"] == "True"]
        t3.append([mode, n] + [mstd([fnum(x[c]) * s for x in g]) for c, s in cols3] + [mstd([fnum(r["final_range"]) for r in rr]), mstd([math.degrees(fnum(r["final_face"])) for r in rr])])
    table("T3_capture", ["mode", "N", "lock time [s]", "churn", "bid gap [%]", "dist gap [%]", "conflict [s]", "rounds", "ready→armed [s]",
                         "action spread [ms]", "sync spread [ms]", "capture spread [ms]", "final range [m]", "final face err [deg]"], t3, out, md)
    table("T4_manifest", ["bag", "mode", "N", "commit", "dirty", "config", "QC", "trigger", "docked", "kept"],
          [[r["bag"], r["mode"], r["N"], r["commit"], r["dirty"], r["config_id"], r["qc"], r["trigger"], f"{r['n_docked']}/{r['N']}", r["bag"] in keep] for r in runs], out, md)
    md.insert(1, "\n## Warnings\n\n" + ("".join(f"- {n}\n" for n in notes) or "- none\n") +
              "\nRepresentative runs: " + ", ".join(f"{m} N={n} `{b}`" for (m, n), b in reps.items()) + "\n")
    (out / "summary.md").write_text("".join(md))
    for n in notes: print("!", n)
    print(f"-> {out}/summary.md")


if __name__ == "__main__":
    main()
