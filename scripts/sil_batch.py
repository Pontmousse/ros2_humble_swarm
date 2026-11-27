#!/usr/bin/env python3
"""Unattended SIL campaign: real simulated capture runs, N agents x reps, one at a time.

Each run starts launch_centralized_swarm.py (VIRTUAL_PLANT/TARGET/SPACECRAFT) plus launch_rosbag.py,
arms every virtual spacecraft (as arm_all does) after --arm-delay s, waits until every robot reports DOCK
(or --timeout), keeps recording --settle s of the docked phase, then stops the recorder first and the
swarm second. N=1 cannot reach CAPTURE by consensus, so the runner plays the operator: one /mode_cmd
"capture" once RM1 has been ready for --trigger-hold s. Runs headless (GUIDANCE_VIZ=false).

Host capacity: the whole swarm runs on this one machine, so large N starves the CPU. On the 8-core dev PC
(2026-11-23) N<=2 is clean, N=3-4 come out DEGRADED even headless (/tf collapse; N=3 was clean on 11-16,
so other desktop load matters) and N>=5 never docks: run larger N on
a bigger host, and trust campaign_extract.py's QC verdict over the runner's "docked".

Isolation from the lab: ROS_LOCALHOST_ONLY=1 + own ROS_DOMAIN_ID, UDP comms on loopback broadcast and
its own port (the receiver binds every interface, so the lab port 9870 would hear the real robots).
Resumable: bags already present for the tag count as done. A crashed/interrupted/never-armed run is renamed
ABORTED_<bag> so neither the resume count nor campaign_extract.py picks it up; a timeout is kept
(it is a real mission failure).

  source /opt/ros/humble/setup.bash && source ros2_marvelmind/install/setup.bash && source ros2_swarm/install/setup.bash
  python3 scripts/sil_batch.py --n 1 2 3 --reps 30 --extract
"""
import argparse, faulthandler, json, os, signal, socket, subprocess, sys, time
from datetime import datetime
from pathlib import Path

ap = argparse.ArgumentParser()
ap.add_argument("--n", type=int, nargs="+", default=[1, 2, 3, 4, 5, 6])
ap.add_argument("--reps", type=int, default=30)
ap.add_argument("--timeout", type=float, default=180.0, help="s from launch until all robots docked")
ap.add_argument("--settle", type=float, default=10.0, help="s kept recording after the last dock")
ap.add_argument("--arm-delay", type=float, default=8.0, help="s after launch before arming (retried until localization is fresh)")
ap.add_argument("--arm-timeout", type=float, default=90.0, help="s after --arm-delay before a never-armed run is aborted (N>=5 startup takes ~30 s)")
ap.add_argument("--trigger-hold", type=float, default=2.0, help="N=1: s of continuous ready before capture")
ap.add_argument("--tag", default="test_campaign_SIL")
ap.add_argument("--bag-dir", default="~/rosbags/sil_batch", help="bags + logs/ (BAG_DIR for launch_rosbag.py)")
ap.add_argument("--domain", default="77")
ap.add_argument("--comms-port", type=int, default=9871)
ap.add_argument("--cooldown", type=float, default=5.0)
ap.add_argument("--extract", action="store_true", help="run campaign_extract.py on each finished bag")
ap.add_argument("--dry-run", action="store_true", help="print the schedule and exit")
a = ap.parse_args()

os.environ.update(ROS_LOCALHOST_ONLY="1", ROS_DOMAIN_ID=a.domain)  # before rclpy.init: monitor joins the same graph
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import Trigger
from swarm_interfaces.msg import Readiness

ROOT = Path(a.bag_dir).expanduser()
LOGS = ROOT / "logs"; LOGS.mkdir(parents=True, exist_ok=True)
REPO = Path(__file__).resolve().parent.parent
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
ARM = {}  # robot -> arm client, shared by all runs
faulthandler.register(signal.SIGUSR1, all_threads=True)  # kill -USR1 <pid> dumps the stacks of a hung runner
bag_glob = lambda n: f"swarm_{a.tag}_{n}{'agent' if n == 1 else 'agents'}_*"
done = lambda n: len(list(ROOT.glob(bag_glob(n))))


def lab_is_broadcasting(wait=1.5):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("", a.comms_port)); s.settimeout(0.2); t = time.time() + wait
    try:
        while time.time() < t:
            try:
                _, (src, _) = s.recvfrom(2048)
                if not src.startswith("127."): return src
            except socket.timeout: pass
    finally: s.close()


def stop(p, grace=20.0):
    """SIGINT the whole process group, escalate if needed. True = it shut down cleanly on SIGINT."""
    for sig, wait in ((signal.SIGINT, grace), (signal.SIGTERM, 5.0), (signal.SIGKILL, 5.0)):
        try: os.killpg(p.pid, sig)
        except ProcessLookupError: return True
        t = time.time() + wait
        while time.time() < t:
            p.poll()
            try: os.killpg(p.pid, 0)
            except ProcessLookupError: return sig == signal.SIGINT
            time.sleep(0.2)
    return False


def run(node, n, rep):
    names = [f"RM{i}" for i in range(1, n + 1)]
    env = dict(os.environ, ROBOT_IDX=",".join(str(i) for i in range(1, n + 1)), BAG_TAG=a.tag, BAG_DIR=str(ROOT),
               VIRTUAL_PLANT="1", VIRTUAL_TARGET="1", VIRTUAL_SPACECRAFT="1",
               DIRECT_CMD_VEL="0", SWARM_BROADCAST_ADDR="127.255.255.255", SWARM_COMMS_PORT=str(a.comms_port), GUIDANCE_VIZ="false",
               OMP_NUM_THREADS="1", OPENBLAS_NUM_THREADS="1", OPENCV_FOR_THREADS_NUM="1")  # as on the Pi (lib_threads)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log = open(LOGS / f"N{n}_rep{rep:02d}_{stamp}.log", "w")
    before = set(ROOT.iterdir())
    mode, ready_since, rec, swarm = {}, [None], None, None
    subs = [node.create_subscription(String, f"/{r}/current_mode", lambda m, r=r: mode.__setitem__(r, m.data), LATCHED) for r in names]
    subs.append(node.create_subscription(Readiness, "/readiness", lambda m: m.robot_id == 1 and ready_since.__setitem__(
        0, (ready_since[0] or time.time()) if m.ready else None), qos_profile_sensor_data))
    pub = node.create_publisher(String, "/mode_cmd", LATCHED) if n == 1 else None
    # Created once and reused: destroying a client with a request still pending hung rcl (N=6, 11-25).
    arm = {r: ARM.setdefault(r, node.create_client(Trigger, f"/{r}/virtual_spacecraft/arm")) for r in names}
    futs, armed, t_arm = {}, set(), None
    pop = lambda f: subprocess.Popen(["ros2", "launch", "swarm_bringup", f], env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    t0, t_all, trig, outcome = time.time(), None, None, None
    try:
        swarm, rec = pop("launch_centralized_swarm.py"), pop("launch_rosbag.py")
        while outcome is None:
            rclpy.spin_once(node, timeout_sec=0.1); now = time.time()
            for r, cli in arm.items():  # call each robot's arm, retry about 1 s later until it accepts
                if r in armed or now - t0 < a.arm_delay: continue
                if r in futs and futs[r].done(): armed.update([r] if futs.pop(r).result().success else [])
                elif r in futs and now - cli.last > 3.0: futs.pop(r)  # reply lost under load ("failed to send response"); arm is idempotent
                elif r not in futs and cli.service_is_ready() and now - getattr(cli, "last", 0) > 1.0: cli.last = now; futs[r] = cli.call_async(Trigger.Request())
            if t_arm is None and len(armed) == n: t_arm = now - t0
            if pub and trig is None and ready_since[0] and now - ready_since[0] >= a.trigger_hold:
                pub.publish(String(data="capture")); trig = now - t0
            if t_all is None and all(mode.get(r) == "dock" for r in names): t_all = now
            if t_all and now - t_all >= a.settle: outcome = "docked"
            elif t_arm is None and now - t0 > a.arm_delay + a.arm_timeout: outcome = "arm_failed"  # stack never came up: not a mission result
            elif now - t0 > a.timeout: outcome = "timeout"
            elif swarm.poll() is not None or rec.poll() is not None: outcome = "crash"
    except KeyboardInterrupt: outcome = "interrupted"
    finally:
        clean = all(stop(p) for p in (rec, swarm) if p)  # recorder first: the bag ends on the docked phase
        for s in subs: node.destroy_subscription(s)
        if pub: node.destroy_publisher(pub)  # a latched capture must not leak into the next run
        log.close()
    bag = next(iter(sorted(set(ROOT.iterdir()) - before - {LOGS})), None)
    if bag and outcome in ("crash", "interrupted", "arm_failed"): bag = bag.rename(bag.with_name("ABORTED_" + bag.name))
    res = dict(N=n, rep=rep, bag=bag and bag.name, outcome=outcome, armed=f"{len(armed)}/{n}", t_arm=t_arm and round(t_arm, 1), t_all_docked=t_all and round(t_all - t0, 1),
               trigger=trig and round(trig, 1), modes=mode, clean_shutdown=clean, log=log.name)
    with open(LOGS / "batch.jsonl", "a") as f: f.write(json.dumps(res) + "\n")
    print(f"N={n} rep {rep}/{a.reps}: {outcome:<11} docked={sum(v == 'dock' for v in mode.values())}/{n} "
          f"armed={res['armed']}@{res['t_arm']}s t={res['t_all_docked']} bag={res['bag']}{'' if clean else '  (needed SIGTERM/KILL)'}", flush=True)
    if a.extract and bag and outcome in ("docked", "timeout"):
        subprocess.run([sys.executable, str(REPO / "scripts/campaign_extract.py"), str(bag)])
    if outcome == "interrupted": raise KeyboardInterrupt
    return outcome


todo = [(n, rep) for n in a.n for rep in range(done(n) + 1, a.reps + 1)]
print(f"domain {a.domain}, tag {a.tag}: {len(todo)} runs to do "
      + ", ".join(f"N={n}: {done(n)}/{a.reps} done" for n in a.n))
if a.dry_run or not todo: sys.exit(0)
rclpy.init(); node = rclpy.create_node("sil_batch")
streak = 0
try:
    for n, rep in todo:
        if (src := lab_is_broadcasting()): sys.exit(f"{src} broadcasts on the SIL comms port {a.comms_port}: pick another --comms-port")
        streak = streak + 1 if run(node, n, rep) in ("crash", "arm_failed") else 0
        if streak >= 3: sys.exit("3 crashed/unarmed runs in a row (CPU-starved at this N?): stopping, see the logs in " + str(LOGS))
        time.sleep(a.cooldown)
except KeyboardInterrupt: print("interrupted; the partial bag was renamed ABORTED_*")
finally: node.destroy_node(); rclpy.try_shutdown()
