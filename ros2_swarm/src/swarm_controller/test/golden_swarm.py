"""Shared scenario driver for the Phase 5 golden tests (swarm logic).

The same driver runs the OLD nodes (recorded once from commit b0d3085 into golden_swarm.json) and the NEW
classes (in test_golden_swarm.py), so both see identical inputs, including peers that react to what the robot
publishes. A World stands in for rclpy: fake clock, publishers that log, service clients, and a scheduler that
fires timers created at runtime (the consensus one-shot) at their due time, as ROS would.
"""
import math
import random

from builtin_interfaces.msg import Time as TimeMsg
from rclpy.clock import ClockType
from rclpy.time import Time
from std_msgs.msg import Bool, Int32, String
from std_srvs.srv import SetBool
from nav_msgs.msg import Odometry
from swarm_interfaces.msg import ApBids, CoordXY, Markers, NeighbourState, NeighbourStates, Readiness

# Launch values (launch_decentralized_agent.py at b0d3085) for the components under test.
AUCTION = dict(robot_id=1, bid_rate=5.0, approach_distance=0.8, w_proximity=0.3, w_velocity_align=0.3,
               velocity_min_speed=0.05, w_normal_align=0.3, neighbor_bid_timeout=0.5, target_hold_timeout=0.4,
               lock_hysteresis=0.1, lock_rounds=3, allow_reassignment_on_dropout=True)
CONSENSUS = dict(robot_id=1, consensus_rate=5.0, approach_distance=0.8, perception_stale_timeout=2.0, action_lead=5.0,
                 min_lead=0.5, time_tol=0.05, neighbor_timeout=0.5, min_consensus_hold=0.3)
SUPERVISOR = dict(detect_frames=3, lost_timeout=4.0, miss_tolerance=3, latch_capture=False, auto_capture=True,
                  require_capture_sync=True, initial_mode='search', qos_reliability='BEST_EFFORT',
                  disengage_wheels_on_dock=True)
GEOMETRIC = dict(contact_distance=0.25, contact_heading_tolerance=0.26, contact_debounce_frames=3, timer_frequency=0.02)


def ser(m):
    if isinstance(m, ApBids):
        return ['bids', m.robot_id, list(m.ap_ids), list(m.bids), m.claimed_ap_id, m.locked, stamp_ns(m.header.stamp)]
    if isinstance(m, Readiness):
        return ['ready', m.robot_id, m.ready, m.degree, m.action_time, m.round_id, m.armed, stamp_ns(m.header.stamp)]
    if isinstance(m, (Int32, Bool, String)):
        return [type(m).__name__, m.data]
    raise TypeError(type(m))


def stamp_ns(s): return s.sec * 10**9 + s.nanosec


class Timer:
    def __init__(self, world, period, cb, live):
        self.world, self.period, self.cb, self.live, self.cancelled = world, period, cb, live, False
        self.due = world.t_ns + int(round(period * 1e9))
    def cancel(self): self.cancelled = True


class Client:
    def __init__(self, world, name): self.world, self.srv_name = world, name
    def service_is_ready(self): return self.world.ready(self.srv_name)
    def call_async(self, req):
        self.world.log.append(['call', self.srv_name, req.data])
        world, name = self.world, self.srv_name
        class Future:
            def add_done_callback(self, cb):
                if not world.lose(name): cb(self)
            def exception(self): return None
            def result(self): return SetBool.Response(success=True)
        return Future()


class World:
    """rclpy stand-in. ready(name)/lose(name) script service availability and lost replies."""
    def __init__(self, t0=0.0):
        self.t_ns, self.log, self.timers, self.unready, self.losses = int(t0 * 1e9), [], [], {}, {}

    def now(self): return Time(nanoseconds=self.t_ns, clock_type=ClockType.ROS_TIME)
    def stamp(self): return TimeMsg(sec=self.t_ns // 10**9, nanosec=self.t_ns % 10**9)
    def pub(self, topic):
        class P:
            def publish(_, m): self.log.append([topic, ser(m)])
        return P()
    def create_timer(self, period, cb, live=True):
        t = Timer(self, period, cb, live); self.timers.append(t); return t
    def destroy_timer(self, t): t.cancel()
    def client(self, name): return Client(self, name)
    def ready(self, name): return not any(a <= self.t_ns * 1e-9 < b for a, b in self.unready.get(name, ()))
    def lose(self, name):
        n = self.losses.get(name, 0)
        if n: self.losses[name] = n - 1
        return bool(n)

    def advance(self, t):
        """Fire due live timers in time order up to t, then set the clock to t."""
        target = int(round(t * 1e9))
        while True:
            due = [x for x in self.timers if x.live and not x.cancelled and x.due <= target]
            if not due: break
            x = min(due, key=lambda x: x.due)
            self.t_ns = x.due; x.due += int(round(x.period * 1e9)); x.cb()
        self.t_ns = target


def run(world, obj, events):
    """events: [(t, fn(obj))]; returns per-event outputs (what was published / called)."""
    out = []
    for t, fn in sorted(events, key=lambda e: e[0]):
        world.advance(t); world.log = []
        fn(obj)
        out.append(world.log)
    return out


def markers(world, items, stamp=None):
    """items: [(id, x_cm, y_cm, normal_right, normal_fwd)]."""
    m = Markers(); m.header.stamp = stamp or world.stamp()
    for i, x, y, nr, nf in items:
        m.ids.append(i); m.markers.append(CoordXY(x=float(x), y=float(y))); m.markers_normals.append(CoordXY(x=float(nr), y=float(nf)))
    return m


def neighbours(ids_valid):
    m = NeighbourStates()
    for rid, valid in ids_valid:
        n = NeighbourState(); n.robot_id, n.valid = rid, valid; m.neighbours.append(n)
    return m


def ticks(t0, t1, period, rng, jitter=0.01):
    t, out = t0, []
    while t < t1:
        out.append(round(t, 6)); t += period + rng.uniform(-jitter, jitter)
    return out


# ------------------------------------------------------------------ auction
def auction_events(world):
    rng, ev, ours = random.Random(11), [], {}
    for t in ticks(0.05, 14.0, 0.05, rng, 0.005):
        if 8.0 <= t < 9.2: continue                                           # perception blink: > lock_rounds misses
        d = 70.0 - 2.0 * t                                                     # closing on AP 7
        items = [(7, d, 5.0, 0.0, -1.0), (8, 60.0, -30.0, 0.3, -0.9), (9, 120.0, 0.0, 0.0, -1.0)]
        if 11.0 <= t < 11.06: items = items[:2] + [(9, 50.0, 0.0, 0.0, -1.0)]
        def deliver(a, items=items, bad=abs(t - 5.0) < 0.03):
            m = markers(world, items)
            if bad: m.ids.pop()                                                # ids/markers mismatch: no bids
            a.on_targets(m)
        ev.append((t, deliver))
    for t in ticks(0.02, 14.0, 0.05, rng, 0.005):
        o = Odometry(); o.twist.twist.linear.x, o.twist.twist.linear.y = 0.05 + 0.02 * math.sin(t), 0.01
        ev.append((t, lambda a, o=o: a.on_odom(o)))
    for t in ticks(0.03, 14.0, 0.1, rng, 0.005):
        ev.append((t, lambda a, t=t: a.on_neighbours(neighbours([(2, True), (3, not (9.0 <= t < 10.0)), (4, 10.2 <= t < 11.5)]))))
    for t in ticks(0.07, 14.0, 0.2, rng, 0.02):
        def peers(a, t=t):
            if 2.0 <= t < 6.0:                                                 # RM2 outbids us on AP 7, then leaves
                a.on_peer_bids(bids(world, 2, {7: 2.0, 8: 0.5}, 7 if t > 2.6 else -1, t > 3.0))
            if t >= 1.0:                                                       # RM3 holds AP 8, close to our bid
                a.on_peer_bids(bids(world, 3, {8: 0.62 if t < 3 else (0.85 if t < 7 else 0.95), 7: 0.3}, 8, t > 1.6))  # 0.85: within lock_hysteresis of ours
            if 10.2 <= t < 11.5 and 7 in ours:                                 # RM4, unlocked, just above our lock on 7
                a.on_peer_bids(bids(world, 4, {7: ours[7] + 0.05}, 7, False))
            a.on_peer_bids(bids(world, 1, {7: 9.0}, 7, True))                  # self-echo: ignored
            a.on_peer_bids(bids(world, 5, {7: float('nan')}, -1, False))     # non-finite bid dropped
        ev.append((t, peers))
    def round_(a):
        a.round()
        for topic, m in world.log:
            if topic == '/ap_bids': ours.clear(); ours.update(zip(m[2], m[3]))
    ev += [(t, round_) for t in ticks(0.1, 14.0, 0.2, rng, 0.01)]
    return ev


def bids(world, rid, d, claim, locked):
    m = ApBids(); m.header.stamp = world.stamp(); m.robot_id, m.ap_ids = rid, list(d)
    m.bids, m.claimed_ap_id, m.locked = [float(v) for v in d.values()], claim, locked
    return m


# ------------------------------------------------------------------ consensus
def consensus_events(world):
    """Peers echo what the robot last published on /readiness (round, action time, armed)."""
    rng, ev = random.Random(12), []
    last = {}
    def mine(c):
        for topic, msg in world.log:
            if topic == '/readiness': last.update(t=msg[4], r=msg[5], armed=msg[6])
    ev += [(1.0, lambda c: (c.on_assigned(Int32(data=5)), c.on_locked(Bool(data=True))))]
    ev += [(25.0, lambda c: c.on_locked(Bool(data=False)))]
    for t in ticks(0.05, 30.0, 0.05, rng, 0.004):
        items = [(5, 40.0, 0.0, 0.0, -1.0), (6, 60.0, 30.0, 0.0, -1.0)] if not (14.0 <= t < 14.3) else []
        ev.append((t, lambda c, items=items: c.on_targets(markers(world, items))))
    for t in ticks(0.08, 30.0, 0.1, rng, 0.01):
        if 17.0 <= t < 19.5: continue                                          # raw camera stalls: liveness lost
        ev.append((t, lambda c: c.on_targets_unf(markers(world, [(5, 41.0, 1.0, 0.0, -1.0)]))))
    for t in ticks(0.03, 30.0, 0.1, rng, 0.005):
        ev.append((t, lambda c, t=t: c.on_neighbours(neighbours([(2, True), (3, not (6.0 <= t < 6.4))]))))
    for t in ticks(0.13, 30.0, 0.2, rng, 0.02):
        def peers(c, t=t):
            for rid in (2, 3):
                ready = t >= 1.5 and not (rid == 3 and 4.0 <= t < 4.5)
                at = last.get('t', 0.0) or (world.t_ns * 1e-9 + 5.0)
                m = Readiness(robot_id=rid, ready=ready, degree=2, action_time=float(at if ready else 0.0),
                              round_id=int(last.get('r', 0)), armed=bool(last.get('armed', False)))
                m.header.stamp = world.stamp()
                c.on_peer(m)
            c.on_peer(Readiness(robot_id=1, ready=True, action_time=1.0))      # self-echo: ignored
            bad = Readiness(robot_id=4, ready=True, action_time=-1.0); bad.header.stamp = world.stamp()
            c.on_peer(bad)                                                      # negative action time: ignored
        ev.append((t, peers))
    ev += [(t, lambda c: (c.round(), mine(c))) for t in ticks(0.1, 30.0, 0.2, rng, 0.01)]
    return ev


# ------------------------------------------------------------------ supervisor
def supervisor_events(world):
    rng, ev = random.Random(13), []
    world.unready = {'engage_wheels': [(0.0, 9.5)], 'search/enable': [(0.0, 0.5)]}
    world.losses = {'toggle_dock': 1, 'toggle_controller': 1}
    for t in ticks(0.05, 24.0, 0.05, rng, 0.004):
        lm = [(1, 100.0, 0.0, 0.0, -1.0)] if t >= 1.0 and not (2.0 <= t < 2.12 or 15.0 <= t < 20.0) else []
        tg = [(5, 60.0, 0.0, 0.0, -1.0)] if t >= 3.0 and not (15.0 <= t < 20.0) else []
        ev.append((t, lambda s, lm=lm, tg=tg: (s.on_landmarks(markers(world, lm)), s.on_targets(markers(world, tg)))))
    ev += [(5.0, lambda s: s.on_capture_sync(Bool(data=False))), (6.0, lambda s: s.on_capture_sync(Bool(data=True))),
           (8.0, lambda s: s.on_contact(Bool(data=False))), (9.0, lambda s: s.on_contact(Bool(data=True))),
           (11.0, lambda s: s.on_mode_cmd(String(data='bogus'))), (12.0, lambda s: s.on_mode_cmd(String(data='EP'))),
           (13.0, lambda s: s.on_mode_cmd(String(data=''))), (13.5, lambda s: s.on_mode_cmd(String(data='capture'))),
           (14.0, lambda s: s.on_contact(Bool(data=True))), (21.0, lambda s: s.on_mode_cmd(String(data='auto')))]
    ev += [(t, lambda s: s.tick()) for t in ticks(0.2, 24.0, 0.2, rng, 0.01)]
    return ev


# ------------------------------------------------------------------ geometric contact
def geometric_events(world):
    rng, ev = random.Random(14), []
    for t in ticks(0.03, 6.0, 0.05, rng, 0.004):
        d = max(10.0, 100.0 - 30.0 * t)                                         # approach to 10 cm
        heading = 0.0 if t < 4.0 else 0.4                                       # then turn past the tolerance
        items = [(5, d, 2.0, -math.sin(heading), math.cos(heading)), (6, d + 40.0, -30.0, 0.0, 1.0)]
        items = items if not (2.4 <= t < 2.5) else []
        ev.append((t, lambda g, items=items: g._on_targets(markers(world, items))))
    ev += [(t, lambda g: g.tick()) for t in ticks(0.0, 6.0, 0.02, rng, 0.002)]
    return ev
