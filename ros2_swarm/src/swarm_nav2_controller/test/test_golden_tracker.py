"""Tracker must reproduce the old nav2_pidff node exactly.

golden_tracker.json was recorded from Nav2PidFfNode at commit da5ae53 (pre-Phase-4), driven through its real
__init__/callbacks/update with a fake clock: yaw wrapping through +pi, a 2 m reference jump (saturation and
conditional-integration anti-windup), a frame mismatch, odom and reference gaps past the 0.3 s timeouts, a
timer stall. Two parameter sets: the yaml PID one and P-only (enable_pid false).
"""
import json
import math
from pathlib import Path
from types import SimpleNamespace

import pytest
from nav_msgs.msg import Odometry
from rclpy.clock import ClockType
from rclpy.time import Time

from swarm_nav2_controller.tracking import Tracker

FIXTURE = json.loads((Path(__file__).parent / 'golden_tracker.json').read_text())


def odom(x, y, yaw, bvx=0.0, bvy=0.0, wz=0.0, frame='swarm_map'):
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
                           get_logger=lambda: SimpleNamespace(info=lambda *a: None, warn=lambda *a: None))
    tracker = Tracker(node, lambda n, d: params.get(n, d), SimpleNamespace(publish=lambda m: log.append(
        ['cmd_vel', ['twist', m.linear.x, m.linear.y, m.linear.z, m.angular.x, m.angular.y, m.angular.z]])),
        params['timer_frequency'])
    for (t, kind, *a), want in zip(events, expected):
        clock.ns, log[:] = int(round(t * 1e9)), []
        if kind == 'tick': tracker.update()
        elif kind == 'odom': tracker.odometry_callback(odom(*a))
        else: tracker.reference_callback(odom(*a[:6], frame=a[6]))
        assert log == want, (case, t, kind)
    assert sum(1 for o in expected for x in o if any(x[1][1:])) > 250
