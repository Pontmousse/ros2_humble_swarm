"""BoxSearch must reproduce the old bounding_box_search node exactly.

golden_box_search.json was recorded from BoundingBoxSearchNode at commit da5ae53 (pre-Phase-4), driven through
its real __init__/callbacks/update with a fake clock and random_seed 7: reference in the wrong frame first,
wall approaches and bounces (lateral kick resampled per bounce), a reference gap, search disabled and
re-enabled, shared-target drift (own and count-0 estimates ignored). The wide set adds a ramped initial burn.
Every tick's raw wrench is compared (the zero gate now sits in the control node).
"""
import json
import math
from pathlib import Path
from types import SimpleNamespace

import pytest
from nav_msgs.msg import Odometry
from rclpy.clock import ClockType
from rclpy.time import Time
from std_srvs.srv import SetBool
from swarm_interfaces.msg import TargetEstimate

from swarm_controller.bounding_box_search import BoxSearch
from swarm_controller.shared_target import now_s

FIXTURE = json.loads((Path(__file__).parent / 'golden_box_search.json').read_text())


def odom(x, y, yaw, bvx, bvy, wz, frame):
    m = Odometry(); m.header.frame_id = frame
    m.pose.pose.position.x, m.pose.pose.position.y = float(x), float(y)
    m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.angular.z = float(bvx), float(bvy), float(wz)
    return m


@pytest.mark.parametrize('case', sorted(FIXTURE))
def test_matches_the_old_node(case):
    params, events, expected = FIXTURE[case]['params'], FIXTURE[case]['events'], FIXTURE[case]['outputs']
    clock, log = SimpleNamespace(ns=0), []
    node = SimpleNamespace(get_clock=lambda: SimpleNamespace(now=lambda: Time(nanoseconds=clock.ns, clock_type=ClockType.ROS_TIME)),
                           get_logger=lambda: SimpleNamespace(info=lambda *a: None), get_namespace=lambda: '/RM1')
    box = BoxSearch(node, lambda n, d: params.get(n, d), SimpleNamespace(publish=lambda w: log.append(
        ['spacecraft_wrench', ['wrench', w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z]])))
    for (t, kind, *a), want in zip(events, expected):
        clock.ns, log[:] = int(round(t * 1e9)), []
        resp = None
        if kind == 'tick': box.update()
        elif kind == 'ref': box.reference_callback(odom(*a))
        elif kind == 'enable': resp = box.handle_enable(SetBool.Request(data=a[0]), SetBool.Response())
        else: box.shared.add(TargetEstimate(robot=a[0], x=float(a[1]), y=float(a[2]), count=a[3]), now_s(node))
        got = log + ([['resp', resp.success, resp.message]] if resp is not None else [])
        assert got == want, (case, t, kind)
    assert sum(1 for o in expected for x in o if x[0] == 'spacecraft_wrench' and any(x[1][1:])) > 100
