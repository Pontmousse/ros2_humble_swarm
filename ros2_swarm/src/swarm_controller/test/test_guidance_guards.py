"""Tests that each guidance callback only ever clears its own outputs.

The two callbacks share a shape -- bail out when the pose is not initialised,
and drop the force you are responsible for -- and `neighbours_callback`'s
version of that guard was a copy of `landmarks_callback`'s that still named the
other callback's output. It cleared the landmark force and left the avoidance
force standing, the exact inversion of what it reads as.

Nothing caught it because `self.pose` is only ever assigned, never reset to the
uninitialised sentinel, so the branch cannot run after either force has become
non-zero. That is a property of today's `pose_callback`, not of the guard, and
it stops holding the moment a localization timeout invalidates the pose -- the
change these tests exist to survive. So the ownership is asserted directly
instead of through any observable the current wiring happens to make constant.

The callbacks are called unbound against a stub rather than against a real
node: constructing `GuidanceNode` joins the DDS graph, and the dev host shares
a domain with live robots.
"""

import numpy as np

import pytest

from swarm_controller.position_guidance3 import GuidanceNode

from swarm_interfaces.msg import Markers, NeighbourState, NeighbourStates


# What an uninitialised pose looks like: `None in self.pose` is the guard, and
# an object-dtype array is what the constructor leaves behind.
UNINITIALISED = np.array([None, None, None], dtype=object)

# Sentinels distinguishable from both 0 and each other, so a test that fails
# says which force was clobbered.
LANDMARK_FORCE = np.array([1.5, -2.5])
AVOIDANCE_FORCE = np.array([-3.5, 4.5])


class SilentLogger:

    def warn(self, *args, **kwargs):
        pass


class StubNode:
    """Just the state the two guards read and write."""

    def __init__(self, pose=UNINITIALISED):
        self.pose = pose
        self.vel1 = LANDMARK_FORCE.copy()
        self.vel2 = AVOIDANCE_FORCE.copy()
        self.nbh_Fx, self.nbh_Fy = 0.9, -0.3
        self.last_nbh_time = None
        self.nbh_hold_timeout = 0.5

    def get_logger(self):
        return SilentLogger()


def neighbour_table(*neighbours):
    msg = NeighbourStates()
    for x, y, valid in neighbours:
        n = NeighbourState()
        n.x, n.y, n.valid = x, y, valid
        msg.neighbours.append(n)
    return msg


def test_the_neighbour_guard_leaves_the_landmark_force_alone():
    node = StubNode()
    GuidanceNode.neighbours_callback(node, neighbour_table())

    assert node.vel1 == pytest.approx(LANDMARK_FORCE)
    assert node.vel2 == pytest.approx([0.0, 0.0])
    assert (node.nbh_Fx, node.nbh_Fy) == (0.0, 0.0)


def test_the_landmark_guard_leaves_the_avoidance_force_alone():
    # The mirror image, so the pair pins the ownership rather than one side of
    # it -- this is the guard the broken one was copied from.
    node = StubNode()
    GuidanceNode.landmarks_callback(node, Markers())

    assert node.vel1 == pytest.approx([0.0, 0.0])
    assert node.vel2 == pytest.approx(AVOIDANCE_FORCE)
    assert (node.nbh_Fx, node.nbh_Fy) == (0.9, -0.3)


def test_an_empty_landmark_list_is_not_an_uninitialised_pose():
    # Same output, different reason: `landmarks_callback` has a second guard
    # for "pose is fine, there is nothing to see". Both must clear vel1 only,
    # or one of them becomes a way back into the bug.
    node = StubNode(pose=np.array([1.0, 2.0, 0.0]))
    GuidanceNode.landmarks_callback(node, Markers())

    assert node.vel1 == pytest.approx([0.0, 0.0])
    assert node.vel2 == pytest.approx(AVOIDANCE_FORCE)


def test_an_emptied_neighbour_table_holds_rather_than_zeroes():
    # A single dropped UDP packet empties the table, so "no entries" is not
    # "no neighbour" until the hold expires. Nothing may be cleared meanwhile,
    # including -- especially -- vel1.
    node = StubNode(pose=np.array([1.0, 2.0, 0.0]))
    node.last_nbh_time = object()   # non-None, so the hold is consulted
    node.held_force_expired = lambda: False

    GuidanceNode.neighbours_callback(node, neighbour_table((5.0, 5.0, False)))

    assert node.vel1 == pytest.approx(LANDMARK_FORCE)
    assert node.vel2 == pytest.approx(AVOIDANCE_FORCE)
    assert (node.nbh_Fx, node.nbh_Fy) == (0.9, -0.3)


def test_an_expired_hold_clears_the_avoidance_force_only():
    node = StubNode(pose=np.array([1.0, 2.0, 0.0]))
    node.held_force_expired = lambda: True

    GuidanceNode.neighbours_callback(node, neighbour_table((5.0, 5.0, False)))

    assert node.vel1 == pytest.approx(LANDMARK_FORCE)
    assert node.vel2 == pytest.approx([0.0, 0.0])
    assert (node.nbh_Fx, node.nbh_Fy) == (0.0, 0.0)
