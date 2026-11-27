"""control (Phase 4) wiring: the real __init__ against the real yaml (flattened as ROS does), with rclpy.Node's
DDS calls recorded instead of made, and a fake clock. The classes themselves are pinned by the golden tests."""
import math
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml
from geometry_msgs.msg import Wrench
from nav_msgs.msg import Odometry
from rclpy.clock import ClockType
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import SetBool, Trigger

from swarm_spacecraft.control_node import ControlNode

YAML = Path(__file__).resolve().parents[2] / 'swarm_bringup' / 'config' / 'virtual_spacecraft.yaml'


def flatten(d, prefix=''):
    out = {}
    for k, v in d.items():
        out.update(flatten(v, f'{prefix}{k}.') if isinstance(v, dict) else {f'{prefix}{k}': v})
    return out


def yaml_params():
    doc = yaml.safe_load(YAML.read_text())
    return {**flatten(doc['/**']['ros__parameters']), **flatten(doc['/**/control']['ros__parameters'])}


class Pub:
    def __init__(self): self.msgs = []
    def publish(self, m): self.msgs.append(m)


@pytest.fixture
def make(monkeypatch):
    def build(**over):
        params = {**yaml_params(), **over}
        rec = SimpleNamespace(subs={}, pubs={}, srvs={}, timers=[], ns=0, declared=set())
        def declare(self, n, d):
            assert n not in rec.declared, f'{n} declared twice'
            rec.declared.add(n)
            return SimpleNamespace(value=params.get(n, d))
        for name, fn in (('__init__', lambda self, name: None), ('declare_parameter', declare),
                         ('get_clock', lambda self: SimpleNamespace(now=lambda: Time(nanoseconds=rec.ns, clock_type=ClockType.ROS_TIME))),
                         ('get_logger', lambda self: SimpleNamespace(info=lambda *a, **k: None, warn=lambda *a, **k: None,
                                                                     error=lambda *a, **k: None)),
                         ('get_namespace', lambda self: '/RM1'),
                         ('create_publisher', lambda self, t, topic, q: rec.pubs.setdefault(topic, Pub())),
                         ('create_subscription', lambda self, t, topic, cb, q: rec.subs.__setitem__(topic, cb)),
                         ('create_service', lambda self, t, name, cb: rec.srvs.__setitem__(name, cb)),
                         ('create_timer', lambda self, period, cb: rec.timers.append((period, cb)))):
            monkeypatch.setattr(Node, name, fn)
        n = ControlNode()
        n.rec, n.params = rec, params
        return n
    return build


def odom(x, y, yaw=0.0, vx=0.0):
    m = Odometry(); m.header.frame_id = 'swarm_map'
    m.pose.pose.position.x, m.pose.pose.position.y = x, y
    m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    m.twist.twist.linear.x = vx
    return m


def run(n, t0, t1, odom_until=None, x=-4.0):
    """20 Hz ticks from t0 to t1 with odom just before each tick (until odom_until)."""
    k = 0
    while t0 + 0.05 * k < t1 - 1e-9:
        t = t0 + 0.05 * k
        if odom_until is None or t < odom_until:
            n.rec.ns = int((t - 0.01) * 1e9); n.rec.subs['localization/odom'](odom(x, -2.5))
        n.rec.ns = int(t * 1e9); n.rec.timers[0][1]()
        k += 1


def test_one_subscription_per_input_services_and_one_timer(make):
    n = make()
    assert sorted(n.rec.subs) == sorted(['localization/odom', '/target_estimate', 'flocking_wrench', 'preparation_wrench',
                                         'capture_wrench', 'avoidance_wrench', 'pointing_wrench'])   # not spacecraft_wrench
    assert sorted(n.rec.srvs) == ['search/enable', 'virtual_spacecraft/arm', 'virtual_spacecraft/reset']
    assert sorted(n.rec.pubs) == sorted(['virtual_spacecraft/odom', 'virtual_spacecraft/applied_wrench', 'cmd_vel',
                                         'cmd_vel_raw', 'spacecraft_wrench'])
    assert [p for p, _ in n.rec.timers] == [0.05]


def test_nested_yaml_reaches_every_class(make):
    n = make()
    assert (n.craft.maximum_force, n.craft.mass, n.craft.wrench_frames['flocking']) == (0.02, 5.0, 'inertial')
    assert (n.tracker.position_gain, n.tracker.odometry_timeout, n.tracker.enable_pid) == (1.5, 0.3, True)
    assert (n.box.x_min, n.box.boundary_margin, n.box.share_target) == (-7.0, 0.4, True)       # /** and own block
    assert (n.smoother.smoothing_frequency, n.smoother.max_accels) == (20.0, [1.5, 1.5, 3.0])
    assert n.smoother.velocity_timeout_ns == 250_000_000 and n.smoother.scale_velocities


def test_pre_arm_streams_zero_cmd_vel_and_no_reference(make):
    n = make()
    run(n, 0.05, 1.0)
    assert n.rec.pubs['virtual_spacecraft/odom'].msgs == []
    cmds = n.rec.pubs['cmd_vel'].msgs
    assert len(cmds) == 19 and all(m.linear.x == m.linear.y == m.angular.z == 0.0 for m in cmds)


def test_armed_chain_runs_in_one_tick_and_feeds_box_and_tracker(make):
    n = make(**{'virtual_spacecraft.maximum_force': 1.0, 'virtual_spacecraft.maximum_speed': 0.5})
    run(n, 0.05, 0.5)
    assert n.rec.srvs['virtual_spacecraft/arm'](Trigger.Request(), Trigger.Response()).success
    flock = Wrench(); flock.force.x = 0.8
    for k in range(40):
        n.rec.ns = int((0.5 + 0.05 * k - 0.02) * 1e9); n.rec.subs['flocking_wrench'](flock)
        run(n, 0.5 + 0.05 * k, 0.5 + 0.05 * (k + 1))
        assert n.tracker.last_reference_time == n.rec.ns and n.box.last_reference_time == n.rec.ns   # same tick
    ref = n.rec.pubs['virtual_spacecraft/odom'].msgs[-1]
    assert ref.twist.twist.linear.x > 0.1 and ref.pose.pose.position.x > -4.0
    assert n.rec.pubs['cmd_vel'].msgs[-1].linear.x > 0.1 and len(n.rec.pubs['cmd_vel'].msgs) == len(n.rec.pubs['cmd_vel_raw'].msgs)
    # the box's wrench is the craft's bounding_box source, in-process (initial burn is 0 in the yaml -> exact zero)
    assert n.craft.wrench_values['bounding_box'] == (0.0, 0.0, 0.0) and n.craft.wrench_update_times['bounding_box'] == n.rec.ns
    assert n.rec.srvs['search/enable'](SetBool.Request(data=False), SetBool.Response()).success and not n.box.enabled


def test_stale_odom_freezes_stops_and_ramps_down(make):
    n = make(**{'virtual_spacecraft.maximum_force': 1.0, 'virtual_spacecraft.maximum_speed': 0.5})
    run(n, 0.05, 0.5)
    n.rec.srvs['virtual_spacecraft/arm'](Trigger.Request(), Trigger.Response())
    flock = Wrench(); flock.force.x = 1.0
    for k in range(60):
        n.rec.ns = int((0.5 + 0.05 * k - 0.02) * 1e9); n.rec.subs['flocking_wrench'](flock)
        run(n, 0.5 + 0.05 * k, 0.5 + 0.05 * (k + 1))
    v0 = n.rec.pubs['cmd_vel'].msgs[-1].linear.x
    assert v0 > 0.3
    before = len(n.rec.pubs['cmd_vel'].msgs)
    n_refs = len(n.rec.pubs['virtual_spacecraft/odom'].msgs)
    run(n, 3.5, 5.0, odom_until=3.5)                     # last odom at 3.44 s; ticks at 3.50 + 0.05 i
    raw = [m.linear.x for m in n.rec.pubs['cmd_vel_raw'].msgs[before:]]
    out = [m.linear.x for m in n.rec.pubs['cmd_vel'].msgs[before:]]
    refs = [r.twist.twist.linear.x for r in n.rec.pubs['virtual_spacecraft/odom'].msgs[n_refs:]]
    assert refs[3] > 0.0 and refs[4:] == [0.0] * (len(refs) - 4)    # odom > 0.25 s old from 3.70 s: frozen, zero twist
    assert raw[4] != 0.0 and raw[5:] == [0.0] * (len(raw) - 5)      # odom > 0.3 s old from 3.75 s: tracker stops
    drops = [a - b for a, b in zip(out[4:], out[5:]) if a > 0.0]
    assert drops and all(d == pytest.approx(0.1) or b == 0.0 for d, b in zip(drops, out[5:])) and out[-1] == 0.0  # 2 m/s^2


def test_direct_cmd_vel_publishes_cmd_vel_nav(make):
    n = make(direct_cmd_vel=True)
    assert 'cmd_vel_nav' in n.rec.pubs and 'cmd_vel' not in n.rec.pubs
