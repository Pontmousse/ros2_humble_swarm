"""Tests for the body-to-map rotation the damping term depends on."""

import math

import pytest

from nav_msgs.msg import Odometry

from swarm_controller.position_guidance3 import map_frame_velocity


def odometry(body_vx, body_vy, yaw):
    msg = Odometry()
    msg.pose.pose.orientation.z = math.sin(0.5 * yaw)
    msg.pose.pose.orientation.w = math.cos(0.5 * yaw)
    msg.twist.twist.linear.x = body_vx
    msg.twist.twist.linear.y = body_vy
    return msg


def test_zero_yaw_is_the_identity():
    assert map_frame_velocity(odometry(0.3, -0.1, 0.0)) == pytest.approx((0.3, -0.1))


def test_driving_forward_while_facing_minus_x_moves_in_minus_x():
    # The case the avoidance law actually sees: two robots nose to nose, each
    # driving forward. Taking the body twist as map-frame would report +0.4 and
    # the damping term would brake the pair apart instead of together.
    vx, vy = map_frame_velocity(odometry(0.4, 0.0, math.pi))
    assert vx == pytest.approx(-0.4)
    assert vy == pytest.approx(0.0, abs=1e-12)


@pytest.mark.parametrize("yaw", [0.0, 0.7, math.pi / 2, 2.5, -1.3])
def test_rotation_preserves_speed(yaw):
    speed = math.hypot(*map_frame_velocity(odometry(0.3, 0.4, yaw)))
    assert speed == pytest.approx(0.5)


def test_yaw_is_read_as_radians_not_degrees():
    # StatePos.theta is in degrees and is the obvious place to get yaw from.
    # 90 degrees as radians is 1.57 rad, so a degrees/radians mix-up here is a
    # whole-quadrant error, not a rounding one. Pinned by comparing against the
    # quaternion, which is unambiguously radians.
    vx, vy = map_frame_velocity(odometry(1.0, 0.0, math.radians(90.0)))
    assert (vx, vy) == pytest.approx((0.0, 1.0), abs=1e-12)


def test_yaw_comes_from_the_full_quaternion_not_just_z():
    # A quaternion with x/y components still has a well-defined yaw. Reading
    # 2*atan2(z, w) would be right only for the pure-yaw case this filter
    # happens to publish today, and would break silently if it ever tilts.
    msg = odometry(0.5, 0.0, 0.0)
    roll = 0.2
    q = msg.pose.pose.orientation
    q.w, q.x, q.y, q.z = math.cos(0.5 * roll), math.sin(0.5 * roll), 0.0, 0.0
    assert map_frame_velocity(msg) == pytest.approx((0.5, 0.0))
