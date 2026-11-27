"""swarm (Phase 5) wiring: the real __init__ with rclpy.Node's DDS calls recorded, a fake clock and timers.
The classes themselves are pinned by test_golden_swarm.py."""
import json
from types import SimpleNamespace

import pytest
from rclpy.clock import ClockType
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool, Int32, String
from swarm_interfaces.msg import CoordXY, Markers, NeighbourState, NeighbourStates

from swarm_controller.swarm_node import SwarmNode

LAUNCH = {'robot_id': 3, 'consensus.consensus_rate': 5.0, 'supervisor.detect_frames': 3,
          'supervisor.require_capture_sync': True, 'supervisor.latch_capture': False}


class Client:  # hashable, like rclpy's: the supervisor keys pending calls by client
    def __init__(self, name): self.srv_name, self.calls = name, []
    def service_is_ready(self): return True
    def call_async(self, req): self.calls.append(req.data); return SimpleNamespace(add_done_callback=lambda cb: None)


class Pub:
    def __init__(self, topic, qos): self.topic, self.qos, self.msgs = topic, qos, []
    def publish(self, m): self.msgs.append(m)


@pytest.fixture
def make(monkeypatch, tmp_path):
    def build(**over):
        params = {**LAUNCH, **over}
        rec = SimpleNamespace(subs={}, pubs={}, timers=[], clients={}, ns=10**9, declared=set())
        def declare(self, n, d):
            assert n not in rec.declared, f'{n} declared twice'
            rec.declared.add(n)
            return SimpleNamespace(value=params.get(n, d))
        def sub(self, t, topic, cb, q): rec.subs.setdefault(topic, []).append(cb)
        def timer(self, period, cb):
            t = SimpleNamespace(period=period, cb=cb, cancelled=False); t.cancel = lambda: setattr(t, 'cancelled', True)
            rec.timers.append(t); return t
        client = lambda self, t, name: rec.clients.setdefault(name, Client(name))  # noqa: E731
        for name, fn in (('__init__', lambda self, name: None), ('declare_parameter', declare),
                         ('get_clock', lambda self: SimpleNamespace(now=lambda: Time(nanoseconds=rec.ns, clock_type=ClockType.ROS_TIME))),
                         ('get_logger', lambda self: SimpleNamespace(info=lambda *a, **k: None, warn=lambda *a, **k: None,
                                                                     warning=lambda *a, **k: None, error=lambda *a, **k: None)),
                         ('get_namespace', lambda self: '/RM3'),
                         ('create_publisher', lambda self, t, topic, q: rec.pubs.setdefault(topic, Pub(topic, q))),
                         ('create_subscription', sub), ('create_timer', timer), ('destroy_timer', lambda self, t: t.cancel()),
                         ('create_client', client)):
            monkeypatch.setattr(Node, name, fn)
        n = SwarmNode()
        n.rec = rec
        return n
    return build


def markers(ns, items, fwd=-1.0):
    m = Markers(); m.header.stamp.sec, m.header.stamp.nanosec = ns // 10**9, ns % 10**9
    for i, x, y in items:
        m.ids.append(i); m.markers.append(CoordXY(x=float(x), y=float(y))); m.markers_normals.append(CoordXY(x=0.0, y=fwd))
    return m


def test_one_subscription_per_input_and_the_outward_topics(make):
    n = make()
    assert sorted(n.rec.subs) == sorted(['targets', 'landmarks', 'targets_unf', 'localization/odom', 'comms/neighbours',
                                         '/ap_bids', '/readiness', 'contact_detected', 'mode_cmd', '/mode_cmd'])
    assert all(len(cbs) == 1 for cbs in n.rec.subs.values())
    assert sorted(n.rec.pubs) == sorted(['current_mode', '/readiness', 'capture_sync', '/ap_bids', 'assigned_ap',
                                         'assignment_locked', 'contact_detected'])
    assert [t.period for t in n.rec.timers] == [0.2]                      # contact.source none: just the tick
    assert n.rec.pubs['current_mode'].msgs[-1].data == 'search'
    assert n.auction.robot_id == n.consensus.robot_id == 3 and n.supervisor.require_capture_sync


def test_auction_result_reaches_consensus_in_the_same_tick(make):
    n = make()
    for k in range(5):                                                     # 1 s of 20 Hz targets + 5 Hz ticks
        for j in range(4):
            n.rec.ns += 50_000_000
            n.rec.subs['targets'][0](markers(n.rec.ns, [(7, 40.0, 0.0)]))
        n.rec.timers[0].cb()
        assert (n.consensus.assigned_ap, n.consensus.locked) == (n.rec.pubs['assigned_ap'].msgs[-1].data,
                                                                 n.rec.pubs['assignment_locked'].msgs[-1].data)
    assert n.consensus.assigned_ap == 7 and n.consensus.locked                 # locked after lock_rounds
    assert len(n.rec.pubs['/ap_bids'].msgs) == len(n.rec.pubs['/readiness'].msgs) == 5


def test_capture_sync_and_contact_drive_the_supervisor_in_process(make):
    n = make()
    for _ in range(4):
        n.rec.ns += 50_000_000
        n.rec.subs['targets'][0](markers(n.rec.ns, [(7, 40.0, 0.0)]))
        n.rec.subs['landmarks'][0](markers(n.rec.ns, [(1, 100.0, 0.0)]))
    n.supervisor.tick()
    assert n.supervisor.mode.value == 'search' or n.supervisor.mode.value == 'encapsulation'
    n.consensus.sync_pub.publish(Bool(data=True))                          # what consensus does when it fires
    assert n.supervisor.mode.value == 'capture' and n.rec.pubs['capture_sync'].msgs[-1].data
    n.supervisor.contact = False
    n.rec.subs['contact_detected'][0](Bool(data=True))                     # remote switch (centralized HIL)
    n.supervisor.tick()
    assert n.supervisor.mode.value == 'dock'
    assert n.rec.clients['toggle_dock'].calls == [True]


def test_geometric_contact_source(make):
    n = make(**{'contact.source': 'geometric', 'contact.contact_distance': 0.25, 'contact.timer_frequency': 0.02})
    assert sorted(t.period for t in n.rec.timers) == [0.02, 0.2]
    tick = next(t for t in n.rec.timers if t.period == 0.02).cb
    for _ in range(3):
        n.rec.subs['targets'][0](markers(n.rec.ns, [(7, 15.0, 0.0)], fwd=1.0))   # 15 cm, head-on
        tick()
    assert n.rec.pubs['contact_detected'].msgs[-1].data
    n.supervisor.mode = n.supervisor.mode.__class__('capture')
    tick()
    assert n.supervisor.contact                                            # fed in-process


def test_run_config_is_latched_once(make, tmp_path):
    f = tmp_path / 'cfg.yaml'; f.write_text('a: 1\n')
    n = make(**{'run_config.config_files': [str(f)], 'run_config.env_vars': ['HOME'], 'run_config.source_launch': 'centralized',
                'run_config.robot_names': ['RM3']})
    [msg] = n.rec.pubs['run_config'].msgs
    doc = json.loads(msg.data)
    assert doc['source_launch'] == 'centralized' and doc['robot_names'] == ['RM3'] and doc['files'][str(f)]['text'] == 'a: 1\n'
    assert str(n.rec.pubs['run_config'].qos.durability).endswith('TRANSIENT_LOCAL')


def test_unknown_contact_source_is_refused(make):
    with pytest.raises(ValueError):
        make(**{'contact.source': 'lever'})
