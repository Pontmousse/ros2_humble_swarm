"""VirtualCraft must reproduce the old virtual_spacecraft node exactly.

golden_virtual_craft.json was recorded from VirtualSpacecraftNode at commit da5ae53 (pre-Phase-4), driven
through its real __init__/callbacks/update with a fake clock: arm before odom, odom gaps across the 0.25 s
freeze, timer stalls past the 0.1 s dt clamp, six wrench sources going stale, a NaN wrench, reset. Two
parameter sets: the yaml one (20 mN clamp, CHW on) and a wide one (1 N, response scales < 1, CHW off).
"""
import json
import math
from pathlib import Path
from types import SimpleNamespace

import pytest
from geometry_msgs.msg import Wrench
from nav_msgs.msg import Odometry
from rclpy.clock import ClockType
from rclpy.time import Time
from std_srvs.srv import Trigger

from swarm_spacecraft.virtual_craft import VirtualCraft

FIXTURE = json.loads((Path(__file__).parent / 'golden_virtual_craft.json').read_text())


def odom(x, y, yaw, bvx=0.0, bvy=0.0, wz=0.0, frame='swarm_map'):
    m = Odometry(); m.header.frame_id = frame
    m.pose.pose.position.x, m.pose.pose.position.y = float(x), float(y)
    m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.angular.z = float(bvx), float(bvy), float(wz)
    return m


def ser(m):
    if isinstance(m, Odometry):
        p, q, t = m.pose.pose.position, m.pose.pose.orientation, m.twist.twist
        return ['odom', m.header.frame_id, m.child_frame_id, m.header.stamp.sec * 10**9 + m.header.stamp.nanosec,
                p.x, p.y, q.z, q.w, t.linear.x, t.linear.y, t.angular.z]
    return ['wrench', m.force.x, m.force.y, m.force.z, m.torque.x, m.torque.y, m.torque.z]


@pytest.mark.parametrize('case', sorted(FIXTURE))
def test_matches_the_old_node(case):
    params, events, expected = FIXTURE[case]['params'], FIXTURE[case]['events'], FIXTURE[case]['outputs']
    clock, log = SimpleNamespace(ns=0), []
    rec = lambda topic: SimpleNamespace(publish=lambda m: log.append([topic, ser(m)]))  # noqa: E731
    node = SimpleNamespace(get_clock=lambda: SimpleNamespace(now=lambda: Time(nanoseconds=clock.ns, clock_type=ClockType.ROS_TIME)),
                           get_logger=lambda: SimpleNamespace(info=print, warn=print, error=print),
                           get_namespace=lambda: '/RM1')
    craft = VirtualCraft(node, lambda n, d: params.get(n, d), rec('virtual_spacecraft/odom'),
                         rec('virtual_spacecraft/applied_wrench'))
    for (t, kind, *a), want in zip(events, expected):
        clock.ns, log[:] = int(round(t * 1e9)), []
        resp = None
        if kind == 'tick': craft.update()
        elif kind == 'odom': craft.odometry_callback(odom(*a))
        elif kind == 'wrench':
            w = Wrench(); w.force.x, w.force.y, w.torque.z = (float(v) for v in a[1:])
            craft.wrench_callback(a[0], w)
        else: resp = getattr(craft, f'{kind}_callback')(Trigger.Request(), Trigger.Response())
        got = log + ([['resp', resp.success, resp.message]] if resp is not None else [])
        assert got == want, (case, t, kind)
    assert sum(len(o) for o in expected) > 450
