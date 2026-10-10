"""Tests for the neighbour avoidance force and its closing-speed budget."""

import math

import pytest

from swarm_controller.avoidance import closest_approach
from swarm_controller.avoidance import neighbour_repulsion


# position_guidance3 settings: influence radius, potential gain, damping gain,
# envelope, virtual mass, and chassis contact at 2 x the 0.22 m robot_radius.
RANT = 1.05
GAIN = 0.78
DAMPING = 2.0
MAX_FORCE = 1.0
MASS = 5.0
CONTACT = 0.44

# The radius in force up to 2026-10-08. The 2026-10-07 flight was flown under
# it, so the regression below keeps testing it rather than following the
# current value -- otherwise the evidence that motivated both the damping term
# and the widening silently stops being checked.
RANT_BEFORE = 0.70

# Peak closing speed on the 2026-10-07 bag, read from the broadcast velocities
# the guidance actually consumed rather than from differentiated positions.
MEASURED_PEAK_CLOSING = 0.660


def absorbable_ceiling(influence_radius):
    """Fastest approach any law bounded by MAX_FORCE can stop in this runway.

    Holding MAX_FORCE across the whole influence-radius-to-contact runway
    absorbs sqrt(2 * (2F/m) * runway) in the separation coordinate, where the
    reduced mass of two equal masses is m / 2. No shaping of the force does
    better, so this bounds every candidate law at a given radius -- which is
    what makes the radius, not the gains, the thing to move.
    """
    return math.sqrt(
        2.0 * (2.0 * MAX_FORCE / MASS) * (influence_radius - CONTACT)
    )


def critical_closing_speed(
    damping_gain, influence_radius=RANT, opposing_force=0.0
):
    """Fastest head-on approach that still stops short of contact."""
    low, high = 0.0, 2.0
    for _ in range(40):
        middle = 0.5 * (low + high)
        clears = closest_approach(
            middle, influence_radius, GAIN, damping_gain, MAX_FORCE, MASS,
            start=influence_radius + 0.5,
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


# Far enough out that the potential alone is unsaturated (0.153 N), so the
# damping tests below compare forces rather than two clamped 1.0 N values.
# At RANT = 0.70 the old 0.6 m probe was unsaturated; at 1.05 it is not.
UNSATURATED = 0.9


def test_damping_acts_only_while_closing():
    # Same geometry, opposite relative velocity. Approaching must be braked
    # harder than the potential alone; separating must not be braked at all,
    # or a pair that has just cleared each other gets glued back together.
    neighbour = (0.0, 0.0, 0.0, 0.0)
    approaching, _ = neighbour_repulsion(
        (UNSATURATED, 0.0), (-0.4, 0.0), [neighbour],
        RANT, GAIN, DAMPING, MAX_FORCE)
    separating, _ = neighbour_repulsion(
        (UNSATURATED, 0.0), (0.4, 0.0), [neighbour],
        RANT, GAIN, DAMPING, MAX_FORCE)
    undamped, _ = neighbour_repulsion(
        (UNSATURATED, 0.0), (-0.4, 0.0), [neighbour],
        RANT, GAIN, 0.0, MAX_FORCE)
    assert approaching > undamped
    assert separating == pytest.approx(undamped)


def test_damping_reads_relative_not_absolute_velocity():
    # Two craft travelling together at 0.4 m/s are not closing, so the pair
    # must feel only the potential. Differentiating our own position instead
    # of using the broadcast neighbour velocity would brake here for nothing.
    convoy, _ = neighbour_repulsion(
        (UNSATURATED, 0.0), (0.4, 0.0), [(0.0, 0.0, 0.4, 0.0)],
        RANT, GAIN, DAMPING, MAX_FORCE)
    stationary, _ = neighbour_repulsion(
        (UNSATURATED, 0.0), (0.0, 0.0), [(0.0, 0.0, 0.0, 0.0)],
        RANT, GAIN, DAMPING, MAX_FORCE)
    assert convoy == pytest.approx(stationary)
    # And it is the potential alone, not a clamped value that would compare
    # equal no matter what the damping term did.
    assert math.hypot(convoy, 0.0) < MAX_FORCE


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
    # The regression, pinned at the radius it was observed under. On
    # 2026-10-07 RM2 and RM6 converged at 0.445 m/s and touched; the potential
    # saturated at 1.0 N and still lost. A conservative field can only do a
    # fixed amount of work between the influence radius and contact, so it has
    # one critical speed and 0.445 m/s is above it.
    assert critical_closing_speed(
        0.0, RANT_BEFORE) == pytest.approx(0.368, abs=0.005)
    assert closest_approach(
        0.445, RANT_BEFORE, GAIN, 0.0, MAX_FORCE, MASS) < CONTACT

    # Dissipating instead of storing recovers the runway the potential wastes
    # near the influence radius, where it was still only 0.087 N.
    assert critical_closing_speed(DAMPING, RANT_BEFORE) > 0.45
    assert closest_approach(
        0.445, RANT_BEFORE, GAIN, DAMPING, MAX_FORCE, MASS) > CONTACT


def test_damping_cannot_beat_the_force_times_runway_ceiling():
    # Necessary but not sufficient, and this is the number that says so.
    # No law bounded by MAX_FORCE does better than the ceiling, so arbitrarily
    # large damping must converge to it and never exceed it -- at either
    # radius. This is why the fix was the radius and not a bigger gain.
    for radius, expected in ((RANT_BEFORE, 0.456), (RANT, 0.699)):
        ceiling = absorbable_ceiling(radius)
        assert ceiling == pytest.approx(expected, abs=0.001)
        assert critical_closing_speed(50.0, radius) <= ceiling + 1e-3


def test_the_old_radius_could_not_cover_the_envelope_but_the_new_one_can():
    # Two references at the 0.25 m/s cap can close at 0.5 m/s. At RANT_BEFORE
    # that was above the ceiling, so damping could not have been made
    # sufficient by any amount of tuning; at RANT it is below it. This is the
    # whole argument for the change, and it is one comparison.
    assert 2 * 0.25 > absorbable_ceiling(RANT_BEFORE)
    assert 2 * 0.25 < absorbable_ceiling(RANT)


def test_the_new_radius_clears_the_measured_peak_closing_speed():
    # The envelope bound above is about the reference generator. This is the
    # number the robots actually produced, which is higher because each
    # tracker may run at nav2_pidff.maximum_speed = 0.5 m/s to catch up.
    assert MEASURED_PEAK_CLOSING < absorbable_ceiling(RANT)
    assert closest_approach(
        MEASURED_PEAK_CLOSING, RANT_BEFORE, GAIN, DAMPING, MAX_FORCE, MASS,
    ) < CONTACT
    assert closest_approach(
        MEASURED_PEAK_CLOSING, RANT, GAIN, DAMPING, MAX_FORCE, MASS,
        start=RANT + 0.5,
    ) > CONTACT


def test_damping_is_still_load_bearing_at_the_wider_radius():
    # The wider runway alone lifts the pure potential to 0.524 m/s, which could
    # read as "the potential is enough now". It is not, because the avoidance
    # wrench is not the only contributor: bounding_box guidance can demand up
    # to 0.5 N of its own pushing the pair together, and the sum is what gets
    # clamped. Under that load the potential collapses and damping carries it.
    assert critical_closing_speed(0.0, RANT) == pytest.approx(0.524, abs=0.005)
    assert critical_closing_speed(
        0.0, RANT, opposing_force=0.3) == pytest.approx(0.091, abs=0.01)
    assert critical_closing_speed(
        DAMPING, RANT, opposing_force=0.3) == pytest.approx(0.471, abs=0.01)


def test_the_widened_band_is_a_gentle_onset_not_a_wall():
    # The cost side of the change. Separation fell in [0.70, 1.05) 9.7% of the
    # 2026-10-07 bag, so whatever the law does there it does often. It must be
    # a nudge: at 1.0 m under a tenth of the envelope, and still under four
    # fifths of it where the old radius used to begin.
    def magnitude(distance):
        force = neighbour_repulsion(
            (distance, 0.0), (0.0, 0.0), [(0.0, 0.0, 0.0, 0.0)],
            RANT, GAIN, 0.0, MAX_FORCE)
        return math.hypot(*force)

    assert magnitude(1.00) < 0.1 * MAX_FORCE
    assert magnitude(0.90) < 0.2 * MAX_FORCE
    assert magnitude(RANT_BEFORE) < 0.8 * MAX_FORCE
    # Monotone inwards across the whole new band, so a neighbour drifting
    # closer never feels a step down.
    distances = [1.00, 0.90, 0.80, RANT_BEFORE]
    assert all(
        magnitude(a) < magnitude(b) for a, b in zip(distances, distances[1:])
    )
    # Inside the old radius it is saturated, which is where it should be: the
    # ceiling is derived by holding MAX_FORCE across the runway.
    assert magnitude(0.65) == pytest.approx(MAX_FORCE)
    assert magnitude(CONTACT) == pytest.approx(MAX_FORCE)
