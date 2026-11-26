"""Check yaw handoff without starting ROS nodes."""
from types import MethodType, SimpleNamespace

import numpy as np
import pytest
from rclpy.time import Time
from std_msgs.msg import String

from swarm_controller.pointing_guidance2 import GuidanceNode


def pointing(mode):
    node = SimpleNamespace(
        current_mode='', seconds=5., init_start_time=Time(), init_duration=1.,
        pose=np.array([0., 0., 0.]), target_lcd=45., craft_wz=.1,
        heading_torque_gain=.6, yaw_damping_gain=.15, maximum_torque=.2,
        torque_slew_rate=.4, _last_torque=.2, _last_pub_time=Time(seconds=4.99),
        wrenches=[], landmarks_expired=lambda: False,
    )
    node.get_clock = lambda: SimpleNamespace(now=lambda: Time(nanoseconds=round(node.seconds * 1e9)))
    node.wrench_pub = SimpleNamespace(publish=node.wrenches.append)
    node._slew = MethodType(GuidanceNode._slew, node)
    GuidanceNode.mode_callback(node, String(data=mode))
    return node


@pytest.mark.parametrize('mode', ['capture', 'dock'])
def test_cp_ownership_immediately_clears_centroid_torque(mode):
    node = pointing(mode)
    GuidanceNode.publish_target_orientation(node)
    assert node.wrenches[-1].torque.z == 0.
    assert node._last_torque == 0.
    GuidanceNode.publish_target_orientation(node)
    assert node.wrenches[-1].torque.z == 0.


@pytest.mark.parametrize('mode', ['search', 'encapsulation'])
def test_centroid_pointing_stays_active_before_capture(mode):
    node = pointing(mode)
    GuidanceNode.publish_target_orientation(node)
    assert node.wrenches[-1].torque.z > 0.


def test_return_to_encapsulation_slews_from_zero():
    node = pointing('capture')
    GuidanceNode.publish_target_orientation(node)
    node.seconds += .01
    GuidanceNode.mode_callback(node, String(data='encapsulation'))
    GuidanceNode.publish_target_orientation(node)
    assert 0. < node.wrenches[-1].torque.z <= .004 + 1e-12
