#!/usr/bin/env python3
"""Turn capture-campaign bags into phase-sliced metrics for the paper.

Per bag: <out>/<bag>/run.json (meta, QC, events, phases, metrics) and series.npz
(20 Hz grid); across bags: runs.csv (one row per run) and robots.csv (one row per
run x robot x phase). The bags are only read.

Phases per robot: formation [arm/encapsulation -> first AP lock), negotiation
(-> capture mode), approach (-> dock), docked. Pair metrics use swarm windows:
formation ends at the first lock anywhere, negotiation at the first capture
entry, approach at the last dock.

Clocks: SIL is one host, so every stamp agrees. On HIL the Pis have no chrony, so
per-robot pairs (virtual vs KF, same Pi) match on header stamps and cross-robot
quantities use the recorder's receive time. Positions are ground truth in SIL
(ground_truth/pose, else /tf swarm_map->RMn/base_link from fake_plant) and the
KF estimate on HIL. Target geometry is the snapshotted scene when the target is
virtual, else the estimator's `targets` (body frame, cm).

    source ros2_swarm/install/setup.bash        # swarm_interfaces, marvelmind msgs, swarm_target
    python3 scripts/campaign_extract.py                       # all ~/rosbags/swarm_test_campaign_*
    python3 scripts/campaign_extract.py ~/rosbags/<bag> --force
"""
import argparse
import csv
import hashlib
import json
import math
import re
import sqlite3
import warnings
from pathlib import Path
warnings.filterwarnings("ignore")  # all-NaN slices (N=1, undocked robots) are expected, as is scipy's numpy-version notice

import numpy as np
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.optimize import linear_sum_assignment
from swarm_target.virtual_scene import Object

PHASES = ("formation", "negotiation", "approach", "docked")
LAYERS = ("flocking", "avoidance", "pointing", "preparation", "capture", "spacecraft")  # spacecraft = bounding box
DT = 0.05
# Fallbacks for values the run_config snapshot lacks (bags recorded before the
# launch files were snapshotted). The avoidance/flocking radii are code constants
# in position_guidance3.py (influence Rant, force saturation, 2 x robot_radius,
# Rflk); the contact gate is CONTACT_DISTANCE + CONTACT_MARGIN.
DEFAULTS = dict(R_influence=1.05, R_saturate=0.65, R_contact=0.44, R_flock=0.6, contact_distance=0.25,
                contact_heading_tolerance=0.26, approach_distance=0.8, mass=5.0, yaw_inertia=0.05,
                maximum_force=0.02, maximum_torque=0.003, translation_response_scale=1.0,
                rotation_response_scale=1.0, odometry_timeout=0.25)
WANT = re.compile(r"^(/run_config|/ap_bids|/readiness|/tf|/mode_cmd|/RM\d+/(run_config|localization/odom|"
                  r"virtual_spacecraft/(odom|applied_wrench)|\w+_wrench|current_mode|capture_sync|assigned_ap|"
                  r"assignment_locked|mode_cmd|contact_detected|ground_truth/pose|targets))$")

hs = lambda m: m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
qyaw = lambda q: math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
wrap = lambda a: (np.asarray(a) + np.pi) % (2 * np.pi) - np.pi
first = lambda rows, pred=lambda m: True: next((t for t, m in rows if pred(m)), None)
nan = float("nan")


def load(bag):
    out = {}
    for p in sorted(Path(bag).glob("*.db3")):
        db = sqlite3.connect(f"file:{p}?mode=ro", uri=True)
        ids = {i: (n, get_message(t)) for i, n, t in db.execute("SELECT id, name, type FROM topics") if WANT.match(n)}
        for i, ts, d in db.execute(f"SELECT topic_id, timestamp, data FROM messages WHERE topic_id IN "
                                   f"({','.join(map(str, ids)) or -1}) ORDER BY timestamp"):
            out.setdefault(ids[i][0], []).append((ts * 1e-9, deserialize_message(d, ids[i][1])))
        db.close()
    return out


def interp(tq, t, v):
    return np.interp(tq, t, v, left=nan, right=nan) if len(t) > 1 else np.full(np.shape(tq), nan)


def yinterp(tq, t, yaw):
    return wrap(interp(tq, t, np.unwrap(yaw))) if len(t) > 1 else np.full(np.shape(tq), nan)


def hold(tq, t, v, fill):
    i = np.searchsorted(t, tq, "right") - 1
    return np.where(i >= 0, np.asarray(v)[np.clip(i, 0, None)], fill) if len(t) else np.full(np.shape(tq), fill)


def stats(x, prefix):
    x = np.abs(np.asarray(x, float)); x = x[np.isfinite(x)]
    if not len(x): return {f"{prefix}_{k}": nan for k in ("p50", "p95", "max", "rms")}
    return {f"{prefix}_p50": np.percentile(x, 50), f"{prefix}_p95": np.percentile(x, 95),
            f"{prefix}_max": x.max(), f"{prefix}_rms": math.sqrt((x * x).mean())}


def seg_dist(P, V):
    """Distance from points P (n,2) to the closed polygon V (m,2)."""
    A, AB = V, np.roll(V, -1, 0) - V
    t = np.clip(((P[:, None] - A) * AB).sum(-1) / (AB * AB).sum(-1), 0, 1)
    return np.linalg.norm(P[:, None] - (A + t[..., None] * AB), axis=-1).min(1)


def odom(rows):  # columns: recv, header, x, y, yaw, yaw rate
    return np.array([(r, hs(m), m.pose.pose.position.x, m.pose.pose.position.y, qyaw(m.pose.pose.orientation),
                      m.twist.twist.angular.z) for r, m in rows]).reshape(-1, 6)


def config(rcs, warn):
    files = {Path(k).name: v.get("text", "") for rc in rcs for k, v in rc.get("files", {}).items()}
    p, src = dict(DEFAULTS), "defaults"
    vs = yaml.safe_load(files.get("virtual_spacecraft.yaml", "") or "{}") or {}
    p.update({k: v for k, v in vs.get("/**", {}).get("ros__parameters", {}).items() if k in p})
    p.update({k: v for k, v in vs.get("/**/virtual_spacecraft", {}).get("ros__parameters", {}).items() if k in p})
    launch = "\n".join(v for k, v in files.items() if k.startswith("launch_"))
    if launch:
        src, g = "launch", lambda pat: float(re.search(pat, launch).group(1)) if re.search(pat, launch) else None
        cd, cm = g(r"CONTACT_DISTANCE\s*=\s*([\d.]+)"), g(r"CONTACT_MARGIN\s*=\s*([\d.]+)")
        if cd is not None: p["contact_distance"] = cd + (cm or 0.0)
        for k in ("contact_heading_tolerance", "approach_distance"):
            v = g(rf"[\"']{k}[\"']\s*:\s*([\d.]+)"); p[k] = v if v is not None else p[k]
    # config_id = the YAMLs that shape the mission (spacecraft + scene); commit is reported separately.
    shas = sorted({(Path(k).name, v.get("sha256", "")) for rc in rcs for k, v in rc.get("files", {}).items()})
    cid = hashlib.sha1(repr([s for s in shas if s[0].endswith(".yaml") and s[0] != "swarm_config.yaml"]).encode()).hexdigest()[:6]
    scene = None
    if files.get("default_scene.yaml") or any("scene" in k for k in files):
        objs = [Object(o) for o in (yaml.safe_load(next(v for k, v in files.items() if "scene" in k)) or {}).get("objects", [])]
        if any(o.vel != [0.0, 0.0] or o.omega for o in objs): warn.append("moving target: truth geometry assumes it static")
        caps = [(x, y, nx, ny) for o in objs for (x, y), (nx, ny) in o.world_captures()]
        scene = dict(aps={1000 + i: c for i, c in enumerate(caps)}, outline=[np.array(o.world_outline()) for o in objs],
                     centroid=np.mean([pt for o in objs for pt, _ in o.world_landmarks()], 0))
    beacons = {r["name"]: int(r["beacon"]) for r in (yaml.safe_load(files.get("swarm_config.yaml", "") or "{}") or {}).get("robots", [])}
    return p, src, cid, scene, beacons


def extract(bag):
    warn, S = [], load(bag)
    if not any(S.values()): raise SystemExit(f"{bag}: empty bag (recorder never discovered the topics)")
    t0 =min(rows[0][0] for rows in S.values())
    rel = lambda t: None if t is None else round(t - t0, 3)
    rcs = [json.loads(rows[0][1].data) for k, rows in sorted(S.items()) if k.endswith("run_config")]
    if not rcs: raise SystemExit(f"{bag}: no run_config, cannot classify the run")
    rc = rcs[0]; env = rc.get("env", {})
    robots = sorted({k.split("/")[1] for k in S if re.match(r"/RM\d+/localization/odom$", k)}, key=lambda r: int(r[2:]))
    named = sorted({n for x in rcs for n in x.get("robot_names", [])})
    if named and named != sorted(robots): warn.append(f"run_config robots {named} != recorded {robots}")
    P, psrc, cid, scene, beacons = config(rcs, warn)
    sil, vtarget = env.get("VIRTUAL_PLANT") == "1", env.get("VIRTUAL_TARGET") == "1"
    if not vtarget: scene = None
    N = len(robots)
    tf = {}
    for t, m in S.get("/tf", []) if sil else []:
        for tr in m.transforms:
            if tr.header.frame_id == "swarm_map" and tr.child_frame_id.endswith("/base_link"):
                tf.setdefault(tr.child_frame_id.split("/")[0], []).append(
                    (t, hs(tr), tr.transform.translation.x, tr.transform.translation.y, qyaw(tr.transform.rotation), nan))

    R, ev = {}, {}
    for r in robots:
        g = lambda s: S.get(f"/{r}/{s}", [])
        kf, vs = odom(g("localization/odom")), odom(g("virtual_spacecraft/odom"))
        truth = np.array([(t, t, m.x, m.y, math.radians(m.theta), nan) for t, m in g("ground_truth/pose")]
                         or tf.get(r, [])).reshape(-1, 6)
        modes = g("current_mode"); ap_rows = g("assigned_ap"); lk = g("assignment_locked")
        t_cap = first(modes, lambda m: m.data == "capture")
        apt, apv = np.array([t for t, _ in ap_rows]), np.array([m.data for _, m in ap_rows])
        lkt, lkv = np.array([t for t, _ in lk]), np.array([m.data for _, m in lk], bool)
        final_ap = int(hold([t_cap], apt, apv, -1)[0]) if t_cap else (int(apv[-1]) if len(apv) else -1)
        # final_lock = start of the last unbroken locked claim on final_ap before capture
        same = (apv == final_ap) & hold(apt, lkt, lkv, False).astype(bool) & (apt <= (t_cap or np.inf)) if len(apv) else np.zeros(0, bool)
        on = np.where(same)[0]; off = np.where(~same[:on[-1] + 1])[0] if len(on) else []
        e = dict(arm=vs[0, 0] if len(vs) else None, encap=first(modes, lambda m: m.data == "encapsulation"),
                 first_lock=first(lk, lambda m: m.data), final_lock=apt[off[-1] + 1 if len(off) else 0] if len(on) else None,
                 sync=first(g("capture_sync"), lambda m: m.data), capture=t_cap, dock=first(modes, lambda m: m.data == "dock"),
                 contact=first(g("contact_detected"), lambda m: m.data))
        ev[r] = e
        R[r] = dict(kf=kf, vs=vs, truth=truth, pos=truth if sil and len(truth) else kf, apt=apt, apv=apv, lkt=lkt, lkv=lkv,
                    final_ap=final_ap, churn=int((np.diff(apv) != 0).sum()) if len(apv) > 1 else 0, targets=g("targets"),
                    applied=np.array([(t, math.hypot(m.force.x, m.force.y), abs(m.torque.z)) for t, m in
                                      g("virtual_spacecraft/applied_wrench")]).reshape(-1, 3),
                    layers={L: np.array([(t, math.hypot(m.force.x, m.force.y), abs(m.torque.z)) for t, m in
                                         g(f"{L}_wrench")]).reshape(-1, 3) for L in LAYERS})
    if any(len(R[r]["kf"]) < 2 for r in robots): raise SystemExit(f"{bag}: a robot has no localization/odom")
    ts, te = max(R[r]["kf"][0, 0] for r in robots), min(R[r]["kf"][-1, 0] for r in robots)
    grid = np.arange(ts, te, DT)
    end = te
    trigger = "operator" if any(m.data for k, v in S.items() if k.endswith("mode_cmd") for _, m in v) else \
        ("operator" if any(e["capture"] for e in ev.values()) and not any(e["sync"] for e in ev.values()) else "consensus")

    # -- phase windows ------------------------------------------------------------
    bounds = {}
    for r, e in ev.items():
        start = max([x for x in (e["arm"], e["encap"]) if x is not None] or [ts])
        b = [start, min(x for x in (e["first_lock"], e["capture"], end) if x is not None), e["capture"] or end, e["dock"] or end, end]
        bounds[r] = np.maximum.accumulate(b)
    docks = [e["dock"] for e in ev.values()]
    sb = np.maximum.accumulate([max(b[0] for b in bounds.values()), min(b[1] for b in bounds.values()),
                                min(b[2] for b in bounds.values()), max(docks) if all(docks) else end, end])
    phase_of = lambda b, t: np.searchsorted(b, t, "right") - 1

    # -- per-robot series + metrics ----------------------------------------------
    series, rows, robot_out = {"t": grid - t0}, [], {}
    for r in robots:
        d, e, b = R[r], ev[r], bounds[r]
        pos, kf, vs = d["pos"], d["kf"], d["vs"]
        x, y, yaw = interp(grid, pos[:, 0], pos[:, 2]), interp(grid, pos[:, 0], pos[:, 3]), yinterp(grid, pos[:, 0], pos[:, 4])
        vx, vy, vyaw = (interp(grid, vs[:, 0], vs[:, 2]), interp(grid, vs[:, 0], vs[:, 3]), yinterp(grid, vs[:, 0], vs[:, 4])) \
            if len(vs) > 1 else (np.full_like(grid, nan),) * 3
        # Tracking: virtual vs KF matched on header (same clock even on HIL), labelled by receive time.
        trk_t = vs[:, 0] if len(vs) else np.zeros(0)
        trk = np.hypot(vs[:, 2] - interp(vs[:, 1], kf[:, 1], kf[:, 2]), vs[:, 3] - interp(vs[:, 1], kf[:, 1], kf[:, 3])) if len(vs) else trk_t
        trk_yaw = wrap(vs[:, 4] - yinterp(vs[:, 1], kf[:, 1], kf[:, 4])) if len(vs) else trk_t
        tt = d["truth"]
        trk_true = np.hypot(vs[:, 2] - interp(vs[:, 1], tt[:, 1], tt[:, 2]), vs[:, 3] - interp(vs[:, 1], tt[:, 1], tt[:, 3])) \
            if len(vs) and len(tt) > 1 else np.full(len(trk_t), nan)
        ap = hold(grid, d["apt"], d["apv"], -1)
        # Range to the assigned AP: estimated from `targets` (body frame cm, relative to the KF pose) ...
        est = [(t, next((math.hypot(mk.x, mk.y) / 100.0 for mk, i in zip(m.markers, m.ids) if i == a), nan),
                next((wrap(-math.atan2(n.x, n.y)) for n, i in zip(m.markers_normals, m.ids) if i == a), nan))
               for t, m in d["targets"] for a in [int(hold([t], d["apt"], d["apv"], -1)[0])]]
        est = np.array(est).reshape(-1, 3)
        rng_est, face_est = interp(grid, est[:, 0], est[:, 1]), interp(grid, est[:, 0], est[:, 2])
        # ... and from the true scene geometry when the target is virtual.
        if scene:
            A = np.array([scene["aps"].get(int(a), (nan,) * 4) for a in ap])
            dx, dy = x - A[:, 0], y - A[:, 1]
            rng, lat, face = np.hypot(dx, dy), np.abs(dx * A[:, 3] - dy * A[:, 2]), wrap(yaw - np.arctan2(-A[:, 3], -A[:, 2]))
            c = scene["centroid"]
        else:
            rng, lat, face = rng_est, np.full_like(grid, nan), face_est
            c = None
        pnt = wrap(vyaw - np.arctan2(c[1] - y, c[0] - x)) if c is not None else np.full_like(grid, nan)
        yawrate = interp(grid, vs[:, 0], vs[:, 5]) if len(vs) > 1 else np.full_like(grid, nan)
        standoff = np.min([seg_dist(np.c_[x, y], V) for V in scene["outline"]], 0) if scene else np.full_like(grid, nan)
        ph = phase_of(b, grid)
        series.update({f"{r}/{k}": v for k, v in dict(x=x, y=y, yaw=yaw, vx=vx, vy=vy, vyaw=vyaw, ap=ap, range=rng,
                       range_est=rng_est, lateral=lat, face=face, pointing=pnt, yawrate=yawrate, standoff=standoff, phase=ph,
                       trk=interp(grid, trk_t, trk), trk_true=interp(grid, trk_t, trk_true)).items()})
        # Dock quality at the moment the stack declared DOCK.
        fin = dict(docked=e["dock"] is not None, final_ap=d["final_ap"], time_to_dock=(e["dock"] - b[0]) if e["dock"] else nan,
                   search_time=max(0.0, (e["encap"] or end) - e["arm"]) if e["arm"] else nan)  # blind time: arm -> first ENCAPSULATION
        if e["dock"]:
            i = min(np.searchsorted(grid, e["dock"]), len(grid) - 1)
            fin.update(final_range=rng[i], final_range_est=rng_est[i], final_face=abs(face[i]), final_lateral=lat[i])
            if scene:
                fin["correct_ap"] = bool(min(scene["aps"], key=lambda a: math.hypot(x[i] - scene["aps"][a][0], y[i] - scene["aps"][a][1])) == d["final_ap"])
        else: fin.update(final_range=nan, final_range_est=nan, final_face=nan, final_lateral=nan)
        # Success = the stack's own DOCK at the assigned AP, the one criterion SIL and HIL share. geom_ok re-judges
        # the contact gate on the true pose (SIL), so it mostly measures localization error, not docking.
        fin["success"] = bool(fin["docked"] and fin.get("correct_ap", True))
        fin["geom_ok"] = bool(fin["docked"] and fin["final_range"] <= P["contact_distance"] and fin["final_face"] <= P["contact_heading_tolerance"])
        a = d["applied"]
        fmax, tmax = P["maximum_force"] * P["translation_response_scale"], P["maximum_torque"] * P["rotation_response_scale"]
        per = {}
        for k, name in enumerate(PHASES):
            sel = lambda t: phase_of(b, t) == k
            m = ph == k
            row = dict(duration=b[k + 1] - b[k], **stats(trk[sel(trk_t)], "trk"), **stats(trk_true[sel(trk_t)], "trk_true"),
                       **{f"yaw_{k2}": v for k2, v in stats(trk_yaw[sel(trk_t)], "trk").items() if k2.endswith("p95")},
                       **stats(pnt[m], "pnt"), standoff_p50=np.nanmedian(standoff[m]) if m.any() and scene else nan,
                       range_start=rng[m][0] if m.any() else nan, range_min=np.nanmin(rng[m]) if m.any() and np.isfinite(rng[m]).any() else nan,
                       lateral_p95=np.nanpercentile(lat[m], 95) if m.any() and np.isfinite(lat[m]).any() else nan,
                       face_p95=np.nanpercentile(np.abs(face[m]), 95) if m.any() and np.isfinite(face[m]).any() else nan)
            yr = yawrate[m][np.isfinite(yawrate[m])]
            sg = np.sign(yr[np.abs(yr) > 0.005])
            row.update(yawrate_rms=math.sqrt((yr ** 2).mean()) if len(yr) else nan,
                       yawrate_flips_per_min=(np.diff(sg) != 0).sum() / max(row["duration"], 1e-9) * 60 if len(sg) > 1 else nan)
            s = sel(a[:, 0]) if len(a) else np.zeros(0, bool)
            dt = np.minimum(np.diff(a[:, 0], append=a[-1, 0]), 0.1)[s] if len(a) else np.zeros(0)
            row.update(dv=(a[s, 1] * dt).sum() / P["mass"] if len(a) else nan, dw=(a[s, 2] * dt).sum() / P["yaw_inertia"] if len(a) else nan,
                       sat_F=(a[s, 1] >= 0.98 * fmax).mean() if s.any() else nan, sat_T=(a[s, 2] >= 0.98 * tmax).mean() if s.any() else nan)
            for L, w in d["layers"].items():
                q = sel(w[:, 0]) if len(w) else np.zeros(0, bool)
                row.update({f"{L}_F": w[q, 1].mean() if q.any() else nan, f"{L}_T": w[q, 2].mean() if q.any() else nan,
                            f"{L}_on": ((w[q, 1] > 1e-6) | (w[q, 2] > 1e-6)).mean() if q.any() else nan})
            per[name] = row
            rows.append(dict(robot=r, phase=name, **fin, churn=d["churn"], **row))
        robot_out[r] = dict(events={k: rel(v) for k, v in e.items()}, bounds=list(b - t0), final=fin, churn=d["churn"], phases=per)

    # -- pairs / swarm --------------------------------------------------------------
    sph = phase_of(sb, grid); series["swarm_phase"] = sph
    pairs, msep = {}, np.full_like(grid, np.inf)
    for i, a in enumerate(robots):
        for bname in robots[i + 1:]:
            dd = np.hypot(series[f"{a}/x"] - series[f"{bname}/x"], series[f"{a}/y"] - series[f"{bname}/y"])
            series[f"sep/{a}-{bname}"] = dd; msep = np.fmin(msep, dd)
            j = np.nanargmin(dd)
            pairs[f"{a}-{bname}"] = dict(min=dd[j], t_min=grid[j] - t0, phase=PHASES[sph[j]] if 0 <= sph[j] < 4 else "pre")
    msep[np.isinf(msep)] = nan; series["minsep"] = msep
    swarm = {}
    for k, name in enumerate(PHASES):
        m = (sph == k) & np.isfinite(msep)
        swarm[name] = dict(duration=sb[k + 1] - sb[k], min_sep=msep[m].min() if m.any() else nan,
                           **{f"frac_{n}": (msep[m] < P[f"R_{n}"]).mean() if m.any() else nan for n in ("influence", "saturate", "contact")})
    if scene and N >= 2:
        cc = scene["centroid"]
        ang = np.sort(np.stack([np.arctan2(series[f"{r}/y"] - cc[1], series[f"{r}/x"] - cc[0]) for r in robots], 1), 1)
        gap = np.max(np.diff(np.c_[ang, ang[:, :1] + 2 * np.pi], axis=1), 1) / (2 * np.pi / N)
        series["angular_gap"] = gap; m = sph == 0
        swarm["formation"]["angular_gap_p50"] = np.nanmedian(gap[m]) if m.any() else nan

    # -- consensus -----------------------------------------------------------------
    rd = {r: [(t, m) for t, m in S.get("/readiness", []) if m.robot_id == beacons.get(r, int(r[2:]))] for r in robots}
    rnd = max([m.round_id for v in rd.values() for _, m in v] or [0])
    fr = max([m.round_id for v in rd.values() for _, m in v if m.armed] or [rnd])
    alls = lambda pred: nan if N < 2 or None in (v := [first(rd[r], pred) for r in robots]) else max(v)
    act = [next((m.action_time for _, m in reversed(rd[r]) if m.armed), nan) for r in robots]
    syncs, caps = [ev[r]["sync"] for r in robots if ev[r]["sync"]], [ev[r]["capture"] for r in robots if ev[r]["capture"]]
    t_ready, t_armed = alls(lambda m: m.ready and m.round_id == fr), alls(lambda m: m.armed and m.round_id == fr)
    consensus = dict(rounds=int(rnd), all_ready=rel(t_ready) if t_ready == t_ready else nan, all_armed=rel(t_armed) if t_armed == t_armed else nan,
                     ready_to_armed=t_armed - t_ready, armed_to_fire=(np.mean(syncs) - t_armed) if syncs else nan,
                     action_spread=np.nanmax(act) - np.nanmin(act) if np.isfinite(act).any() else nan,
                     sync_spread=max(syncs) - min(syncs) if len(syncs) == N and N > 1 else nan,
                     capture_spread=max(caps) - min(caps) if len(caps) == N and N > 1 else nan,
                     fire_lag=np.mean(syncs) - np.nanmean(act) if syncs and np.isfinite(act).any() else nan, n_synced=len(syncs))

    # -- auction -------------------------------------------------------------------
    fl = [ev[r]["final_lock"] for r in robots]
    t_lock = max(fl) if all(fl) else None
    locks = [ev[r]["first_lock"] for r in robots if ev[r]["first_lock"]]
    auction = dict(t_first_lock=rel(min(locks)) if locks else None, t_all_locked=rel(t_lock),
                   lock_time=(t_lock - sb[0]) if t_lock else nan, churn=sum(R[r]["churn"] for r in robots), gap_bid=nan, gap_dist=nan)
    claims = np.stack([np.where(hold(grid, R[r]["lkt"], R[r]["lkv"], False).astype(bool), series[f"{r}/ap"], -1) for r in robots])
    auction["conflict_s"] = float(sum(((claims[i] == claims[j]) & (claims[i] >= 0)).sum() for i in range(N) for j in range(i + 1, N)) * DT)
    if t_lock:
        fap = [R[r]["final_ap"] for r in robots]
        last = [next((m for t, m in reversed(S.get("/ap_bids", [])) if t <= t_lock + 1e-3 and m.robot_id == beacons.get(r, int(r[2:]))), None) for r in robots]
        if all(last):
            ids = sorted({i for m in last for i in m.ap_ids} | set(fap))
            B = np.full((N, len(ids)), -1e3)
            for k, m in enumerate(last): B[k, [ids.index(i) for i in m.ap_ids]] = m.bids
            ri, ci = linear_sum_assignment(-B)
            opt, got = B[ri, ci].sum(), sum(B[k, ids.index(a)] for k, a in enumerate(fap))
            auction["gap_bid"] = (opt - got) / opt if opt > 0 and got > -1e2 else nan
        if scene:
            i = min(np.searchsorted(grid, t_lock), len(grid) - 1); ids = sorted(scene["aps"])
            D = np.array([[math.hypot(series[f"{r}/x"][i] - scene["aps"][a][0], series[f"{r}/y"][i] - scene["aps"][a][1]) for a in ids] for r in robots])
            ri, ci = linear_sum_assignment(D)
            auction["gap_dist"] = sum(D[k, ids.index(a)] for k, a in enumerate(fap) if a in ids) / D[ri, ci].sum() - 1 if all(a in ids for a in fap) else nan

    # -- QC ------------------------------------------------------------------------
    lag = {r: R[r]["kf"][:, 0] - R[r]["kf"][:, 1] for r in robots}
    qc = dict(loc_rate={r: (len(R[r]["kf"]) - 1) / (R[r]["kf"][-1, 0] - R[r]["kf"][0, 0]) for r in robots},
              jitter={r: float(np.median(v) - np.percentile(v, 1)) for r, v in lag.items()},
              gaps={r: int((np.diff(R[r]["kf"][:, 1]) > P["odometry_timeout"]).sum()) for r in robots},
              tf_rate=len(S.get("/tf", [])) / max(te - ts, 1e-9))
    # SIL host starvation shows as /tf collapse (>=100 Hz/robot healthy, <=12 starved) before jitter rises
    qc["verdict"] = "DEGRADED" if max(qc["jitter"].values()) > 0.05 or sum(qc["gaps"].values()) or (sil and qc["tf_rate"] / N < 50) else "OK"

    meta = dict(bag=Path(bag).name, mode="SIL" if sil else "HIL", N=N, robots=robots, config_id=cid, commit=rc.get("git", {}).get("commit", "")[:7],
                dirty=rc.get("git", {}).get("dirty"), source_launch=rc.get("source_launch"), trigger=trigger, params_source=psrc,
                virtual_target=vtarget, position_source="truth" if sil and all(len(R[r]["truth"]) for r in robots) else "kf",
                duration=te - ts, t_start=ts - t0)
    succ = [robot_out[r]["final"]["success"] for r in robots]
    run = dict(meta=meta, params=P, qc=qc, warnings=warn, swarm_bounds=list(sb - t0), robots=robot_out, pairs=pairs, swarm=swarm,
               consensus=consensus, auction=auction,
               summary=dict(n_success=sum(succ), n_docked=sum(robot_out[r]["final"]["docked"] for r in robots), success=all(succ),
                            mission_time=(max(docks) - sb[0]) if all(docks) else nan,
                            search_time_max=np.nanmax([robot_out[r]["final"]["search_time"] for r in robots]), min_sep=np.nanmin(msep) if N > 1 else nan,
                            min_sep_approach=swarm["approach"]["min_sep"],
                            trk_p95=np.nanmax([robot_out[r]["phases"][p]["trk_p95"] for r in robots for p in PHASES[:3]])))
    if scene: series.update(aps=np.array([(a, *v) for a, v in sorted(scene["aps"].items())]), centroid=scene["centroid"],
                            **{f"outline/{i}": V for i, V in enumerate(scene["outline"])})
    return run, series, rows


def jsonable(o):
    if isinstance(o, dict): return {k: jsonable(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)): return [jsonable(v) for v in o]
    if isinstance(o, (np.floating, float)): return None if not math.isfinite(o) else round(float(o), 6)
    if isinstance(o, np.integer): return int(o)
    if isinstance(o, np.bool_): return bool(o)
    return o


def verdict(run):
    m, s, c = run["meta"], run["summary"], run["consensus"]
    f = lambda v, fmt: "-" if v is None or v != v else format(v, fmt)
    return (f"{m['mode']} N={m['N']} cfg={m['config_id']} {run['qc']['verdict']:8s} {s['n_success']}/{m['N']} ok "
            f"({s['n_docked']} docked) t={f(s['mission_time'], '.1f')}s minsep={f(s['min_sep'], '.2f')} "
            f"trk95={f(s['trk_p95'] * 1e3 if s['trk_p95'] == s['trk_p95'] else nan, '.0f')}mm sync={f(c['sync_spread'] * 1e3 if c['sync_spread'] == c['sync_spread'] else nan, '.0f')}ms "
            f"trig={m['trigger']}" + "".join(f"\n    ! {w}" for w in run["warnings"]))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("bags", nargs="*")
    ap.add_argument("--out", default="~/rosbags/processed")
    ap.add_argument("--force", action="store_true")
    a = ap.parse_args()
    out = Path(a.out).expanduser(); out.mkdir(parents=True, exist_ok=True)
    bags = [Path(b).expanduser() for b in a.bags] or sorted(Path("~/rosbags").expanduser().glob("swarm_test_campaign_*"))
    for bag in bags:
        dst = out / bag.name
        dbs = list(bag.glob("*.db3"))
        if not dbs: print(f"{bag.name}: no .db3, skipped"); continue
        if not a.force and (dst / "run.json").exists() and (dst / "run.json").stat().st_mtime > max(p.stat().st_mtime for p in dbs):
            print(f"{bag.name}: up to date"); continue
        run, series, rows = extract(bag)
        dst.mkdir(exist_ok=True)
        (dst / "run.json").write_text(json.dumps(jsonable(run), indent=1))
        np.savez_compressed(dst / "series.npz", **series)
        with open(dst / "robots_rows.json", "w") as fh: json.dump(jsonable(rows), fh)
        print(f"{bag.name}: {verdict(run)}")
    # Rebuild the flat tables from every processed run.
    runs, robots = [], []
    for rj in sorted(out.glob("*/run.json")):
        run = json.loads(rj.read_text()); m = run["meta"]
        key = dict(bag=m["bag"], mode=m["mode"], N=m["N"], config_id=m["config_id"], commit=m["commit"], dirty=m["dirty"],
                   qc=run["qc"]["verdict"], trigger=m["trigger"])
        runs.append(dict(key, **run["summary"], **{f"cons_{k}": v for k, v in run["consensus"].items()},
                         **{f"auc_{k}": v for k, v in run["auction"].items()},
                         **{f"{p}_{k}": v for p, d in run["swarm"].items() for k, v in d.items()}))
        robots += [dict(key, **r) for r in json.loads((rj.parent / "robots_rows.json").read_text())]
    for name, rows in (("runs.csv", runs), ("robots.csv", robots)):
        cols = list(dict.fromkeys(k for r in rows for k in r))
        with open(out / name, "w", newline="") as fh:
            w = csv.DictWriter(fh, cols); w.writeheader(); w.writerows(rows)
    print(f"{len(runs)} runs -> {out}/runs.csv, {out}/robots.csv")


if __name__ == "__main__":
    main()
