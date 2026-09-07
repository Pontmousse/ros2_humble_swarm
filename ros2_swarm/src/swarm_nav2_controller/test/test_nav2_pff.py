"""Tests for the P+FF virtual-spacecraft tracker."""

import math

import pytest

from swarm_nav2_controller.nav2_pff import inputs_are_fresh
from swarm_nav2_controller.nav2_pff import frames_match
from swarm_nav2_controller.nav2_pff import MeasuredPose
from swarm_nav2_controller.nav2_pff import ReferenceState
from swarm_nav2_controller.nav2_pff import tracking_command
from swarm_nav2_controller.nav2_pff import validate_controller_parameters


def test_feedforward_is_transformed_between_body_frames():
    reference = ReferenceState(yaw=math.pi / 2.0, body_vx=1.0)
    measured = MeasuredPose(yaw=0.0)
    x, y, _, _ = tracking_command(reference, measured, 0.0, 0.0, 2.0, 1.0)
    assert x == pytest.approx(0.0, abs=1.0e-12)
    assert y == pytest.approx(1.0)


def test_position_error_is_expressed_in_measured_body_frame():
    reference = ReferenceState(x=1.0)
    measured = MeasuredPose(yaw=math.pi / 2.0)
    x, y, _, _ = tracking_command(reference, measured, 1.0, 0.0, 2.0, 1.0)
    assert x == pytest.approx(0.0, abs=1.0e-12)
    assert y == pytest.approx(-1.0)


def test_tracking_saturation_preserves_direction_and_limits_yaw():
    reference = ReferenceState(body_vx=3.0, body_vy=4.0, yaw_rate=2.0)
    x, y, yaw_rate, saturated = tracking_command(
        reference, MeasuredPose(), 0.0, 0.0, 1.0, 0.5
    )
    assert x == pytest.approx(0.6)
    assert y == pytest.approx(0.8)
    assert yaw_rate == pytest.approx(0.5)
    assert saturated


def test_yaw_error_wraps_across_pi():
    reference = ReferenceState(yaw=-math.pi + 0.1)
    measured = MeasuredPose(yaw=math.pi - 0.1)
    _, _, yaw_rate, _ = tracking_command(
        reference, measured, 0.0, 1.0, 1.0, 1.0
    )
    assert yaw_rate == pytest.approx(0.2)


def test_inputs_must_both_exist_and_be_fresh():
    now = 1_000_000_000
    assert not inputs_are_fresh(now, None, None, 0.25, 0.25)
    assert not inputs_are_fresh(now, 700_000_000, 900_000_000, 0.25, 0.25)
    assert inputs_are_fresh(now, 800_000_000, 900_000_000, 0.25, 0.25)


def test_input_frames_must_be_nonempty_and_equal():
    assert frames_match("swarm_map", "swarm_map")
    assert not frames_match("", "swarm_map")
    assert not frames_match("swarm_map", "RM1/odom")


@pytest.mark.parametrize(
    "values",
    [
        (0.0, 1.0, 1.0, 1.0, 1.0, 0.25, 0.25),
        (100.0, -1.0, 1.0, 1.0, 1.0, 0.25, 0.25),
        (100.0, 1.0, float("nan"), 1.0, 1.0, 0.25, 0.25),
        (100.0, 1.0, 1.0, 0.0, 1.0, 0.25, 0.25),
    ],
)
def test_invalid_controller_parameters_are_rejected(values):
    with pytest.raises(ValueError):
        validate_controller_parameters(*values)
