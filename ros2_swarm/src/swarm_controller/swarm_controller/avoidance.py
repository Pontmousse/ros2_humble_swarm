"""ROS-independent neighbour avoidance force.

Kept out of the node so the closing-speed budget can be tested by integrating
the law directly, which is the only way to answer the question that matters:
how fast may two craft converge and still stop short of contact.
"""
import math


def neighbour_repulsion(
    position,
    velocity,
    neighbours,
    influence_radius,
    gain,
    damping_gain,
    maximum_force,
    epsilon=1e-4,
):
    """Return the resultant (fx, fy) pushing `position` away from neighbours.

    `neighbours` holds (x, y, vx, vy) in the same map frame as `position` and
    `velocity`. A neighbour may be given as (x, y) alone, in which case it
    contributes no damping.

    Two terms, with different jobs:

    The position term is the original potential, gradient of
    0.5 * gain * (1/d - 1/R)^2. Being conservative it can only trade kinetic
    energy for potential energy, so the work it can do between R and contact is
    a fixed quantity no matter how fast the pair is closing. That gives a single
    critical speed: below it the craft stops, above it the craft hits, and the
    excess is untouched. Worse, the term vanishes as d approaches R -- measured
    2026-10-07 it was still only 0.087 N at d = 0.676 m, so the first 0.1 m of
    an 0.26 m braking runway contributed almost nothing.

    The damping term is dissipative: it removes energy from the relative motion
    rather than storing it, and it scales with how fast the pair is actually
    converging, so there is no single critical speed to be above. It also acts
    at full strength the instant a fast approach enters the influence radius,
    which is where the position term is weakest. In control terms the virtual
    craft is an undamped double integrator and a pure potential is a pure P
    law, so the loop had no damping anywhere in it and overshoot was structural.

    Damping is applied only while closing. A symmetric term would also resist
    separation and glue a pair together after a near miss.
    """
    if influence_radius <= 0.0 or maximum_force <= 0.0:
        raise ValueError("influence_radius and maximum_force must be positive")
    if gain < 0.0 or damping_gain < 0.0:
        raise ValueError("gains must be non-negative")

    x, y = position[0], position[1]
    force_x = 0.0
    force_y = 0.0
    for neighbour in neighbours:
        dx = x - neighbour[0]
        dy = y - neighbour[1]
        distance = math.hypot(dx, dy) + epsilon
        if distance >= influence_radius:
            continue
        unit_x = dx / distance
        unit_y = dy / distance
        magnitude = (
            gain
            * (1.0 / distance - 1.0 / influence_radius)
            / (distance * distance)
        )
        if damping_gain > 0.0 and velocity is not None and len(neighbour) >= 4:
            # d/dt of the separation is u . (v_self - v_neighbour); closing is
            # its negation, positive while the gap shrinks.
            closing = -(
                (velocity[0] - neighbour[2]) * unit_x
                + (velocity[1] - neighbour[3]) * unit_y
            )
            if closing > 0.0:
                magnitude += damping_gain * closing
        force_x += magnitude * unit_x
        force_y += magnitude * unit_y

    # Saturate the resultant, not each term: with three robots closing in,
    # per-neighbour caps would still sum to 3x the envelope.
    total = math.hypot(force_x, force_y)
    if total > maximum_force:
        force_x *= maximum_force / total
        force_y *= maximum_force / total
    return force_x, force_y


def closest_approach(
    closing_speed,
    influence_radius,
    gain,
    damping_gain,
    maximum_force,
    mass,
    start=1.2,
    dt=0.0005,
    opposing_force=0.0,
):
    """Minimum separation reached by two identical craft closing head-on.

    Integrated in the separation coordinate, where the reduced mass of two
    equal masses is mass / 2 -- the interaction force does work on the gap, so
    counting each craft's work separately double-counts it.

    `opposing_force` models a second wrench contributor pushing the pair
    together, which is the realistic case here: bounding_box guidance can
    demand up to 0.5 N of its own and the sum is what gets clamped.
    """
    reduced_mass = mass / 2.0
    separation = float(start)
    rate = -float(closing_speed)
    minimum = separation
    for _ in range(int(60.0 / dt)):
        force_x, _ = neighbour_repulsion(
            (separation, 0.0),
            (rate, 0.0),
            [(0.0, 0.0, 0.0, 0.0)],
            influence_radius,
            gain,
            damping_gain,
            maximum_force,
        )
        rate += ((force_x - opposing_force) / reduced_mass) * dt
        separation += rate * dt
        minimum = min(minimum, separation)
        if separation <= 0.0 or (rate > 0.0 and separation > influence_radius):
            break
    return minimum
