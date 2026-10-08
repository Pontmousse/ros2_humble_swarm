"""Tests for the neighbour avoidance force and its closing-speed budget."""

import math

import pytest

from swarm_controller.avoidance import closest_approach
from swarm_controller.avoidance import neighbour_repulsion


# position_guidance3 settings: influence radius, potential gain, damping gain,
# envelope, virtual mass, and chassis contact at 2 x the 0.22 m robot_radius.
RANT = 0.70
GAIN = 0.78
DAMPING = 2.0
MAX_FORCE = 1.0
MASS = 5.0
CONTACT = 0.44


def critical_closing_speed(damping_gain, opposing_force=0.0):
    """Fastest head-on approach that still stops short of contact."""
    low, high = 0.0, 2.0
    for _ in range(40):
        middle = 0.5 * (low + high)
        clears = closest_approach(
            middle, RANT, GAIN, damping_gain, MAX_FORCE, MASS,
            opposing_force=opposing_force,
        ) > CONTACT
        low, high = (middle, high) if clears else (low, middle)
    return low


def test_no_force_outside_the_influence_radius():
    force = neighbour_repulsion(
        (0.0, 0.0), (1.0, 0.0), [(RANT + 0.01, 0.0, 0.0, 0.0)],
        RANT, GAIN, DAMPING, MAX_FORCE,
    )
    assert force == (0.0, 0.0)


def test_damping_acts_only_while_closing():
    # Same geometry, opposite relative velocity. Approaching must be braked
    # harder than the potential alone; separating must not be braked at all,
    # or a pair that has just cleared each other gets glued back together.
    neighbour = (0.0, 0.0, 0.0, 0.0)
    approaching, _ = neighbour_repulsion(
        (0.6, 0.0), (-0.4, 0.0), [neighbour], RANT, GAIN, DAMPING, MAX_FORCE)
    separating, _ = neighbour_repulsion(
        (0.6, 0.0), (0.4, 0.0), [neighbour], RANT, GAIN, DAMPING, MAX_FORCE)
    undamped, _ = neighbour_repulsion(
        (0.6, 0.0), (-0.4, 0.0), [neighbour], RANT, GAIN, 0.0, MAX_FORCE)
    assert approaching > undamped
    assert separating == pytest.approx(undamped)


def test_damping_reads_relative_not_absolute_velocity():
    # Two craft travelling together at 0.4 m/s are not closing, so the pair
    # must feel only the potential. Differentiating our own position instead
    # of using the broadcast neighbour velocity would brake here for nothing.
    convoy, _ = neighbour_repulsion(
        (0.6, 0.0), (0.4, 0.0), [(0.0, 0.0, 0.4, 0.0)],
        RANT, GAIN, DAMPING, MAX_FORCE)
    stationary, _ = neighbour_repulsion(
        (0.6, 0.0), (0.0, 0.0), [(0.0, 0.0, 0.0, 0.0)],
        RANT, GAIN, DAMPING, MAX_FORCE)
    assert convoy == pytest.approx(stationary)


def test_resultant_is_saturated_not_each_term():
    # Three neighbours converging from the same side. Capping per neighbour
    # would let the sum reach 3x the validated envelope.
    neighbours = [
        (0.0, 0.0, 0.0, 0.0), (0.0, 0.05, 0.0, 0.0), (0.0, -0.05, 0.0, 0.0),
    ]
    force = neighbour_repulsion(
        (0.5, 0.0), (-0.5, 0.0), neighbours, RANT, GAIN, DAMPING, MAX_FORCE)
    assert math.hypot(*force) == pytest.approx(MAX_FORCE)


def test_damping_gain_saturates_the_envelope_at_the_capped_closing_speed():
    # How the gain is chosen rather than guessed: two references capped at
    # virtual_spacecraft.maximum_speed = 0.25 m/s can close at 0.5 m/s, and at
    # exactly that speed the damping term alone should spend the whole 1.0 N.
    assert DAMPING * (2 * 0.25) == pytest.approx(MAX_FORCE)


def test_a_conservative_potential_alone_cannot_stop_the_measured_approach():
    # The regression. On 2026-10-07 RM2 and RM6 converged at 0.445 m/s and
    # touched; the potential saturated at 1.0 N and still lost. A conservative
    # field can only do a fixed amount of work between RANT and contact, so it
    # has one critical speed and 0.445 m/s is above it.
    assert critical_closing_speed(0.0) == pytest.approx(0.368, abs=0.005)
    assert closest_approach(
        0.445, RANT, GAIN, 0.0, MAX_FORCE, MASS) < CONTACT

    # Dissipating instead of storing recovers the runway the potential wastes
    # near RANT, where it is still only 0.087 N.
    assert critical_closing_speed(DAMPING) > 0.45
    assert closest_approach(
        0.445, RANT, GAIN, DAMPING, MAX_FORCE, MASS) > CONTACT


def test_damping_cannot_beat_the_force_times_runway_ceiling():
    # Necessary but not sufficient, and this is the number that says so.
    # Holding MAX_FORCE across the whole RANT-to-contact runway absorbs
    # sqrt(2 * (2F/m) * runway); no law bounded by MAX_FORCE does better, so
    # arbitrarily large damping must converge to it and never exceed it.
    ceiling = math.sqrt(2.0 * (2.0 * MAX_FORCE / MASS) * (RANT - CONTACT))
    assert ceiling == pytest.approx(0.456, abs=0.001)
    assert critical_closing_speed(50.0) <= ceiling + 1e-3

    # Two references at the 0.25 m/s cap can still close at 0.5 m/s, which is
    # above that ceiling. Damping alone does not make the envelope sufficient:
    # maximum_speed or the influence radius has to move too.
    assert 2 * 0.25 > ceiling
