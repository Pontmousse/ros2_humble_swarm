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


def limit_vector(x: float, y: float, maximum: float) -> Tuple[float, float, bool]:
    """Apply direction-preserving magnitude saturation to a planar vector."""
    if maximum < 0.0:
        raise ValueError("maximum must be non-negative")
    magnitude = math.hypot(x, y)
    if magnitude <= maximum or magnitude == 0.0:
        return x, y, False
    scale = maximum / magnitude
    return scale * x, scale * y, True


def tracking_command(
    reference: PlanarState,
    measured: PlanarState,
    position_gain: float,
    yaw_gain: float,
    maximum_speed: float,
    maximum_yaw_rate: float,
) -> Tuple[float, float, float, bool]:
    """Calculate a saturated body-frame velocity tracking command."""
    inertial_x = reference.vx + position_gain * (reference.x - measured.x)
    inertial_y = reference.vy + position_gain * (reference.y - measured.y)
    body_x, body_y = rotate_inertial_to_body(inertial_x, inertial_y, measured.yaw)
    body_x, body_y, linear_saturated = limit_vector(body_x, body_y, maximum_speed)
    yaw_command = reference.yaw_rate + yaw_gain * wrap_angle(
        reference.yaw - measured.yaw
    )
    limited_yaw = max(-maximum_yaw_rate, min(maximum_yaw_rate, yaw_command))
    return (
        body_x,
        body_y,
        limited_yaw,
        linear_saturated or limited_yaw != yaw_command,
    )
