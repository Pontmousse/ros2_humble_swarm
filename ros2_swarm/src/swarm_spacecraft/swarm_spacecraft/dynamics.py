"""ROS-independent planar rigid-body dynamics and tracking helpers."""

from dataclasses import dataclass
import math
from typing import Tuple


@dataclass
class PlanarState:
    """Planar spacecraft state expressed in the inertial laboratory frame."""

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    yaw_rate: float = 0.0


def wrap_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def rotate_body_to_inertial(x: float, y: float, yaw: float) -> Tuple[float, float]:
    """Rotate a planar vector from body coordinates to inertial coordinates."""
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return cosine * x - sine * y, sine * x + cosine * y


def rotate_inertial_to_body(x: float, y: float, yaw: float) -> Tuple[float, float]:
    """Rotate a planar vector from inertial coordinates to body coordinates."""
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return cosine * x + sine * y, -sine * x + cosine * y


def mean_motion(altitude: float, gravity: float, earth_radius: float) -> float:
    """Orbital mean motion for a circular orbit at the given altitude."""
    if altitude < 0.0 or gravity <= 0.0 or earth_radius <= 0.0:
        raise ValueError("altitude must be non-negative, gravity and radius positive")
    gravitational_parameter = gravity * earth_radius * earth_radius
    semi_major_axis = earth_radius + altitude
    return math.sqrt(gravitational_parameter / semi_major_axis**3)


def chw_force(state: PlanarState, motion: float, mass: float) -> Tuple[float, float]:
    """Planar Clohessy-Wiltshire force about an LVLH origin at (0, 0).

    x is radial and y is along-track, so the state is read as an LVLH
    offset directly with no frame change.
    """
    if mass <= 0.0:
        raise ValueError("mass must be positive")
    radial = 3.0 * motion * motion * state.x + 2.0 * motion * state.vy
    along_track = -2.0 * motion * state.vx
    return mass * radial, mass * along_track


def limit_planar_velocity(
    state: PlanarState,
    maximum_speed: float,
    maximum_yaw_rate: float,
) -> PlanarState:
    """Cap translational speed and yaw rate, preserving velocity direction.

    This is deliberately non-physical. A spacecraft has no top speed, and an
    undamped double integrator driven by repeated boundary firings accumulates
    velocity without bound -- which is exactly what happened: on 2026-10-07 RM2
    was chasing a reference coasting at 0.461 m/s against its own 0.5 m/s
    actuator limit, leaving 8% of authority to correct a 1.26 m tracking error.
    The error could not converge, and no amount of CPU tuning would have fixed
    it, because the reference was simply faster than the tracker.

    The cap belongs here rather than in the guidance law because every wrench
    contributor feeds the same integrator; clamping one source still lets the
    others run the velocity up.

    Scale the translational pair together rather than clamping each axis, so a
    capped spacecraft keeps travelling along its current heading instead of
    being deflected toward whichever axis saturated first.
    """
    if maximum_speed <= 0.0 or maximum_yaw_rate <= 0.0:
        raise ValueError("maximum_speed and maximum_yaw_rate must be positive")

    vx, vy = state.vx, state.vy
    speed = math.hypot(vx, vy)
    if speed > maximum_speed:
        scale = maximum_speed / speed
        vx *= scale
        vy *= scale

    yaw_rate = max(-maximum_yaw_rate, min(maximum_yaw_rate, state.yaw_rate))

    return PlanarState(
        x=state.x,
        y=state.y,
        yaw=state.yaw,
        vx=vx,
        vy=vy,
        yaw_rate=yaw_rate,
    )


def hold_planar_state(state: PlanarState) -> PlanarState:
    """Return the same pose at rest, for publishing a frozen reference.

    Used when physical localization goes stale and the virtual reference stops
    advancing. The trajectory it then describes genuinely has zero velocity,
    even though the simulator retains the momentum it will resume with, so the
    published twist must read zero: an Odometry message is a statement about
    the reference trajectory, and nav2_pidff feeds reference velocity forward.
    Reporting the retained coast velocity would command motion the reference is
    not making.
    """
    return PlanarState(x=state.x, y=state.y, yaw=state.yaw)


def integrate_constant_wrench(
    state: PlanarState,
    force_x: float,
    force_y: float,
    torque: float,
    mass: float,
    inertia: float,
    dt: float,
) -> PlanarState:
    """Integrate a constant inertial-frame wrench exactly over one interval."""
    if mass <= 0.0 or inertia <= 0.0:
        raise ValueError("mass and inertia must be positive")
    if dt < 0.0:
        raise ValueError("dt must be non-negative")

    ax = force_x / mass
    ay = force_y / mass
    angular_acceleration = torque / inertia
    half_dt_squared = 0.5 * dt * dt
    return PlanarState(
        x=state.x + dt * state.vx + half_dt_squared * ax,
        y=state.y + dt * state.vy + half_dt_squared * ay,
        yaw=wrap_angle(
            state.yaw + dt * state.yaw_rate + half_dt_squared * angular_acceleration
        ),
        vx=state.vx + dt * ax,
        vy=state.vy + dt * ay,
        yaw_rate=state.yaw_rate + dt * angular_acceleration,
    )
