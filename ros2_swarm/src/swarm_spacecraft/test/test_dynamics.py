import math

import pytest

from swarm_spacecraft.dynamics import PlanarState
from swarm_spacecraft.dynamics import integrate_constant_wrench
from swarm_spacecraft.dynamics import tracking_command


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


def test_tracking_uses_measured_yaw_for_body_command():
    reference = PlanarState(vx=1.0)
    measured = PlanarState(yaw=math.pi / 2.0)
    x, y, _, _ = tracking_command(reference, measured, 0.0, 0.0, 2.0, 1.0)
    assert x == pytest.approx(0.0, abs=1e-12)
    assert y == pytest.approx(-1.0)


def test_tracking_saturation_preserves_direction():
    reference = PlanarState(vx=3.0, vy=4.0, yaw_rate=2.0)
    x, y, yaw_rate, saturated = tracking_command(
        reference, PlanarState(), 0.0, 0.0, 1.0, 0.5
    )
    assert x == pytest.approx(0.6)
    assert y == pytest.approx(0.8)
    assert yaw_rate == pytest.approx(0.5)
    assert saturated


@pytest.mark.parametrize("mass,inertia", [(0.0, 1.0), (1.0, 0.0)])
def test_nonphysical_parameters_are_rejected(mass, inertia):
    with pytest.raises(ValueError):
        integrate_constant_wrench(PlanarState(), 0.0, 0.0, 0.0, mass, inertia, 0.1)
