import math

import pytest

from swarm_spacecraft.dynamics import PlanarState
from swarm_spacecraft.dynamics import hold_planar_state
from swarm_spacecraft.dynamics import integrate_constant_wrench
from swarm_spacecraft.dynamics import limit_planar_velocity


def test_constant_force_uses_exact_kinematics():
    state = integrate_constant_wrench(
        PlanarState(), 1.0, 0.0, 0.0, mass=5.0, inertia=1.0, dt=1.0
    )
    assert state.x == pytest.approx(0.1)
    assert state.vx == pytest.approx(0.2)


def test_zero_wrench_preserves_momentum():
    initial = PlanarState(vx=0.2, vy=-0.3, yaw_rate=0.4)
    state = integrate_constant_wrench(
        initial, 0.0, 0.0, 0.0, mass=5.0, inertia=1.0, dt=2.0
    )
    assert state.vx == initial.vx
    assert state.vy == initial.vy
    assert state.yaw_rate == initial.yaw_rate
    assert state.x == pytest.approx(0.4)
    assert state.y == pytest.approx(-0.6)


@pytest.mark.parametrize("mass,inertia", [(0.0, 1.0), (1.0, 0.0)])
def test_nonphysical_parameters_are_rejected(mass, inertia):
    with pytest.raises(ValueError):
        integrate_constant_wrench(PlanarState(), 0.0, 0.0, 0.0, mass, inertia, 0.1)


def test_velocity_limit_preserves_heading():
    # 3-4-5 triangle: speed 0.5 along a heading that is not axis-aligned, so a
    # per-axis clamp would visibly rotate the velocity and this would fail.
    state = limit_planar_velocity(
        PlanarState(vx=0.3, vy=0.4), maximum_speed=0.25, maximum_yaw_rate=1.0
    )
    assert math.hypot(state.vx, state.vy) == pytest.approx(0.25)
    heading = math.atan2(state.vy, state.vx)
    assert heading == pytest.approx(math.atan2(0.4, 0.3))


def test_velocity_limit_leaves_slower_states_untouched():
    initial = PlanarState(
        x=1.0, y=2.0, yaw=0.3, vx=0.1, vy=-0.05, yaw_rate=0.2
    )
    state = limit_planar_velocity(initial, 0.25, 0.5)
    assert state == initial


def test_velocity_limit_does_not_move_the_spacecraft():
    # Position and attitude are integrator outputs; the cap must only remove
    # momentum, never teleport the reference the tracker is chasing.
    state = limit_planar_velocity(
        PlanarState(x=-2.5, y=-3.6, yaw=1.2, vx=5.0, vy=5.0, yaw_rate=9.0),
        0.25,
        0.5,
    )
    assert (state.x, state.y, state.yaw) == (-2.5, -3.6, 1.2)


@pytest.mark.parametrize(
    "yaw_rate,expected", [(9.0, 0.5), (-9.0, -0.5), (0.4, 0.4)]
)
def test_yaw_rate_is_clamped_symmetrically(yaw_rate, expected):
    state = limit_planar_velocity(PlanarState(yaw_rate=yaw_rate), 0.25, 0.5)
    assert state.yaw_rate == pytest.approx(expected)


def test_bounded_force_still_needs_a_velocity_cap():
    # The regression this cap exists for: maximum_force bounds the input, not
    # the state. Integrating 1.0 N on 5 kg for 30 s reaches 6 m/s, far past the
    # 0.5 m/s the chassis can render, so the reference outruns its tracker.
    state = PlanarState()
    for _ in range(600):
        state = integrate_constant_wrench(
            state, 1.0, 0.0, 0.0, mass=5.0, inertia=1.0, dt=0.05
        )
    assert state.vx == pytest.approx(6.0)

    capped = PlanarState()
    for _ in range(600):
        capped = integrate_constant_wrench(
            capped, 1.0, 0.0, 0.0, mass=5.0, inertia=1.0, dt=0.05
        )
        capped = limit_planar_velocity(capped, 0.25, 0.5)
    assert capped.vx == pytest.approx(0.25)


def test_hold_stops_the_trajectory_but_keeps_the_pose():
    held = hold_planar_state(
        PlanarState(x=1.5, y=-2.5, yaw=0.75, vx=0.2, vy=-0.15, yaw_rate=0.4)
    )
    assert (held.x, held.y, held.yaw) == (1.5, -2.5, 0.75)
    assert (held.vx, held.vy, held.yaw_rate) == (0.0, 0.0, 0.0)


def test_velocity_cap_alone_does_not_bound_a_localization_outage():
    # The regression freezing exists for. RM6 lost localization for 14.2 s on
    # 2026-10-07 while nav2_pidff held the chassis stopped. A capped reference
    # still coasts at the cap for the whole outage, so the error it opens is
    # bounded only by how long the outage lasts -- which is not a bound.
    reference = PlanarState(vx=0.25)
    coasting = reference
    for _ in range(284):  # 14.2 s of the 50 Hz timer
        coasting = integrate_constant_wrench(
            coasting, 0.0, 0.0, 0.0, mass=5.0, inertia=1.0, dt=0.05
        )
        coasting = limit_planar_velocity(coasting, 0.25, 0.5)
    assert coasting.x == pytest.approx(3.55)

    # Freezing is the absence of that loop: update() returns before it
    # integrates, so the outage costs nothing in position however long it runs
    # and the momentum survives to resume with. What the tracker is shown
    # meanwhile is stationary, which is what the reference actually is.
    published = hold_planar_state(reference)
    assert published.x == pytest.approx(0.0)
    assert published.vx == pytest.approx(0.0)
    assert reference.vx == pytest.approx(0.25)
