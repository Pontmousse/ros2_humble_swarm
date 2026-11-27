"""Small offline checks for the preparation source; no DDS participants."""
from types import MethodType

import numpy as np
import pytest
from std_msgs.msg import Bool, Int32

from swarm_controller.capture_law import CaptureLaw
from test_capture_approach import markers, stub


def preparation_node():
    node = stub()
    node.seconds = 10.0
    node.current_mode = 'encapsulation'
    node.capture_mode = node.dock_mode = False
    node.assignment_locked = True
    node.assigned_ap_id = 42
    node.assigned_ap_time = node.assignment_locked_time = node.get_clock().now()
    node.preparation_tangent = None
    node.preparation_target_id = -1
    node.preparation_target_time = None
    node.preparation_max_force = 0.002
    node.velocity_force_gain = 6.0
    node.craft_vx = node.craft_vy = 0.0
    node.preparation_wrench = MethodType(CaptureLaw.preparation_wrench, node)
    return node


def observe(node, positions=((60., 30.),), ids=(42,)):
    msg = markers(*positions)
    msg.ids = list(ids)
    CaptureLaw.target_callback(node, msg)


def force(node):
    msg = node.preparation_wrench(node.get_clock().now())
    assert (msg.force.z, msg.torque.x, msg.torque.y, msg.torque.z) == (0., 0., 0., 0.)
    return np.array([msg.force.x, msg.force.y])


def test_preparation_is_tangential_capped_and_does_not_brake_radially():
    node = preparation_node()
    observe(node)
    radial = np.array([.6, .3]) / np.hypot(.6, .3)
    first = force(node)
    assert np.linalg.norm(first) == pytest.approx(.002)
    assert first @ radial == pytest.approx(0., abs=1e-12)
    node.craft_vx, node.craft_vy = 3.0 * radial
    assert force(node) == pytest.approx(first)
    node.preparation_max_force = 0.
    assert force(node) == pytest.approx([0., 0.])


@pytest.mark.parametrize('mode,capture,dock,locked', [
    ('search', False, False, True), ('capture', False, False, True),
    ('dock', False, False, True), ('encapsulation', True, False, True),
    ('encapsulation', False, True, True), ('encapsulation', False, False, False),
])
def test_preparation_gate(mode, capture, dock, locked):
    node = preparation_node()
    observe(node)
    node.current_mode, node.capture_mode, node.dock_mode = mode, capture, dock
    node.assignment_locked = locked
    assert force(node) == pytest.approx([0., 0.])


@pytest.mark.parametrize('missing', ['assignment', 'lock', 'target', 'absent', 'empty', 'clock'])
def test_stale_or_missing_assigned_geometry_stops_preparation(missing):
    node = preparation_node()
    observe(node)
    node.seconds += .6
    if missing != 'assignment':
        node.assigned_ap_time = node.get_clock().now()
    if missing != 'lock':
        node.assignment_locked_time = node.get_clock().now()
    if missing not in ('assignment', 'lock', 'target'):
        observe(node, ids=(99,) if missing == 'absent' else (42,))
    if missing == 'empty':
        observe(node, positions=(), ids=())
    if missing == 'clock':
        node.seconds -= 1.0
    assert force(node) == pytest.approx([0., 0.])


def test_alignment_damping_is_shared_and_identity_changes_reset_it():
    node = preparation_node()
    node.preparation_max_force = 2.
    observe(node, positions=((60., 10.),))
    node.seconds += .1
    observe(node, positions=((60., 20.),))
    assert node._bearing_rate_filt > 0.
    tangent = node.preparation_tangent
    assert np.asarray(node.frc_cmd, dtype=float) @ tangent == pytest.approx(node.preparation_speed)
    assert force(node) @ tangent == pytest.approx(6.0 * node.preparation_speed)
    # Same array index, different AP: no derivative or velocity history may cross identities.
    CaptureLaw.assigned_ap_callback(node, Int32(data=43))
    CaptureLaw.assignment_locked_callback(node, Bool(data=True))
    assert force(node) == pytest.approx([0., 0.])
    node.seconds += .1
    observe(node, positions=((60., -30.),), ids=(43,))
    assert node._bearing_rate_filt == 0.
    assert node.preparation_speed == pytest.approx(node.align_gain * np.arctan2(-.3, .6))


@pytest.mark.parametrize('direction', [-1., 1.])
def test_bearing_rate_brakes_as_alignment_error_shrinks(direction):
    node = preparation_node()
    node.preparation_max_force = 2.
    node.align_rate_filter_alpha = 1.
    observe(node, positions=((60. * np.cos(.2), direction * 60. * np.sin(.2)),))
    node.seconds += .1
    observe(node, positions=((60. * np.cos(.1), direction * 60. * np.sin(.1)),))
    assert direction * node._bearing_rate_filt < 0.
    assert direction * node.preparation_speed < 0.  # Requests braking, not more sidling.
    assert node.preparation_speed == pytest.approx(
        node.align_gain * direction * .1 + node.align_damping_gain * node._bearing_rate_filt)
    assert np.asarray(node.frc_cmd, dtype=float) @ node.preparation_tangent \
        == pytest.approx(node.preparation_speed)
