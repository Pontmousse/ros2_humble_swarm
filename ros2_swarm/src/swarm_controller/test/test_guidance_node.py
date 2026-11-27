"""guidance (Phase 3) wiring: one subscription per input fans out to the laws, one tick publishes every wrench,
each mode service reaches its law. GuidanceNode.__init__ runs against a recording stub, so no DDS."""
from types import SimpleNamespace

import pytest
from rclpy.node import Node
from rclpy.clock import ClockType
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import SetBool
from swarm_interfaces.msg import CoordXY, Markers
from swarm_interfaces.msg import StatePos as State

from swarm_controller.guidance_node import GuidanceNode


class Pub:
    def __init__(self, topic): self.topic, self.msgs = topic, []
    def publish(self, m): self.msgs.append(m)


@pytest.fixture
def node(monkeypatch):
    """A GuidanceNode built by its real __init__, with Node's DDS calls recorded instead."""
    rec = SimpleNamespace(subs={}, pubs={}, srvs={}, timers=[], t=Time(seconds=100.0, clock_type=ClockType.ROS_TIME))
    params = {"landmarks_freshness_timeout": 1e9}  # the fake clock is not wall time
    monkeypatch.setattr(Node, "__init__", lambda self, name: None)
    monkeypatch.setattr(GuidanceNode, "declare_parameter", lambda self, n, d: SimpleNamespace(value=params.get(n, d)),
                        raising=False)
    monkeypatch.setattr(GuidanceNode, "get_clock", lambda self: SimpleNamespace(now=lambda: rec.t), raising=False)
    monkeypatch.setattr(GuidanceNode, "get_logger", lambda self: SimpleNamespace(
        info=lambda *a, **k: None, warn=lambda *a, **k: None), raising=False)
    monkeypatch.setattr(GuidanceNode, "get_namespace", lambda self: "/RM2", raising=False)
    monkeypatch.setattr(GuidanceNode, "create_publisher", lambda self, t, topic, q: rec.pubs.setdefault(topic, Pub(topic)),
                        raising=False)
    monkeypatch.setattr(GuidanceNode, "create_subscription",
                        lambda self, t, topic, cb, q: rec.subs.__setitem__(topic, cb), raising=False)
    monkeypatch.setattr(GuidanceNode, "create_service", lambda self, t, name, cb: rec.srvs.__setitem__(name, cb),
                        raising=False)
    monkeypatch.setattr(GuidanceNode, "create_timer", lambda self, period, cb: rec.timers.append((period, cb)),
                        raising=False)
    n = GuidanceNode()
    n.rec = rec
    return n


def test_one_subscription_per_input_and_the_old_services(node):
    assert sorted(node.rec.subs) == sorted([
        'pose', 'landmarks', 'localization/odom', 'current_mode', 'comms/neighbours', 'targets', 'assigned_ap',
        'assignment_locked', 'virtual_spacecraft/odom', '/target_estimate'])
    assert sorted(node.rec.srvs) == ['encapsulation/enable', 'toggle_controller', 'toggle_dock']
    assert sorted(node.rec.pubs) == sorted(['avoidance_wrench', 'flocking_wrench', 'pointing_wrench', 'capture_wrench',
                                            'preparation_wrench', '/target_estimate'])
    assert [p for p, _ in node.rec.timers] == [0.5, 0.05]   # share at 2 Hz, then the 20 Hz tick


def test_inputs_fan_out_to_every_law_that_reads_them(node):
    node.rec.subs['pose'](State(x=1.0, y=2.0, theta=90.0))
    assert list(node.flocking.pose) == [1.0, 2.0, 90.0] and list(node.pointing.pose) == [1.0, 2.0, 90.0]
    assert (node.capture.pose_x, node.capture.pose_y, node.capture.pose_theta) == (1.0, 2.0, 90.0)
    node.rec.subs['current_mode'](String(data='ENCAPSULATION'))
    assert node.pointing.current_mode == node.capture.current_mode == 'encapsulation'
    m = Markers()
    m.header.stamp = node.rec.t.to_msg()
    m.markers.append(CoordXY(x=100.0, y=0.0))
    node.rec.subs['landmarks'](m)
    assert node.pointing.target_lcd == pytest.approx(90.0)          # 1 m ahead of a robot facing +y
    assert node.flocking.last_landmarks_time == node.rec.t


def test_services_reach_their_law_and_one_tick_publishes_all_wrenches(node):
    call = lambda name, on: node.rec.srvs[name](SetBool.Request(data=on), SetBool.Response())  # noqa: E731
    assert call('encapsulation/enable', True).success and node.flocking.encapsulation_enabled
    assert call('toggle_controller', True).success and node.capture.capture_mode
    assert call('toggle_dock', True).success and node.capture.dock_mode
    node.rec.subs['pose'](State(x=0.0, y=0.0, theta=0.0))
    tick = node.rec.timers[-1][1]
    node.rec.t = Time(seconds=103.0, clock_type=ClockType.ROS_TIME)   # past every law's 2 * init_period start-up hold
    tick()
    tick()   # zeros are sent once (zero_gate), so a second tick adds nothing
    for topic in ('avoidance_wrench', 'flocking_wrench', 'pointing_wrench', 'capture_wrench', 'preparation_wrench'):
        assert len(node.rec.pubs[topic].msgs) == 1, topic


def test_direct_cmd_vel_swaps_capture_wrench_for_the_cmd_vel_mux(monkeypatch, node):
    params = {"capture.direct_cmd_vel": True}
    monkeypatch.setattr(GuidanceNode, "declare_parameter", lambda self, n, d: SimpleNamespace(value=params.get(n, d)),
                        raising=False)
    node.rec.subs.clear(), node.rec.pubs.clear()
    n = GuidanceNode()
    assert 'cmd_vel_nav' in node.rec.subs and 'cmd_vel' in node.rec.pubs and 'capture_wrench' not in node.rec.pubs
    assert n.capture.direct_cmd_vel
