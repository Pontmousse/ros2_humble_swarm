import pytest

from swarm_spacecraft.dynamics import PlanarState
from swarm_spacecraft.dynamics import integrate_constant_wrench


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
