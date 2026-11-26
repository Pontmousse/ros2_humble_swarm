"""Shared target estimate: receiver fusion (pointing_guidance2) and blind drift firing (bounding_box_search).

Pure functions/classes only: no node is constructed (the dev host shares a DDS domain with live robots).
"""

import pytest

from swarm_controller.bounding_box_search import drift_firing
from swarm_controller.pointing_guidance2 import SharedTargets

from swarm_interfaces.msg import TargetEstimate


def est(robot, x, y, count):
    return TargetEstimate(robot=robot, x=x, y=y, count=count)


def test_fusion_count_weighted_and_drops_own_and_blind():
    s = SharedTargets("RM1", timeout=2.0)
    s.add(est("RM1", 100.0, 100.0, 50), 0.0)  # own broadcast loops back: ignored
    s.add(est("RM2", 0.0, 0.0, 1), 0.0)
    s.add(est("RM3", 4.0, 2.0, 3), 0.0)
    s.add(est("RM4", 9.0, 9.0, 0), 0.0)       # sender with no landmarks: ignored
    assert s.centre(1.0) == pytest.approx((3.0, 1.5))


def test_fusion_ages_by_receive_time():
    s = SharedTargets("RM1", timeout=2.0)
    s.add(est("RM2", 1.0, 1.0, 1), 0.0)
    s.add(est("RM3", 5.0, 5.0, 1), 1.5)
    assert s.centre(2.5) == pytest.approx((5.0, 5.0))  # RM2 is 2.5 s old
    assert s.centre(4.0) is None
    s.add(est("RM2", 3.0, 3.0, 1), 4.0)                 # a newer message replaces the old one
    assert s.centre(4.0) == pytest.approx((3.0, 3.0))


def test_drift_fires_toward_centre_below_speed():
    assert drift_firing(0, 0, 0, 0, 3, 4, 0.03, 0.5, 1.0) == pytest.approx((0.3, 0.4))
    assert drift_firing(0, 0, -0.1, 0, 3, 4, 0.03, 0.5, 1.0) == pytest.approx((0.3, 0.4))  # receding: fire


def test_drift_coasts_at_speed_and_inside_stop():
    assert drift_firing(0, 0, 0.03, 0.04, 3, 4, 0.03, 0.5, 1.0) == (0.0, 0.0)  # closing at 0.05 >= 0.03
    assert drift_firing(0, 0, 0, 0, 0.6, 0.0, 0.03, 0.5, 1.0) == (0.0, 0.0)    # within share_drift_stop
