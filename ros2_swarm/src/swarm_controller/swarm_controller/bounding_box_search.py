"""Bounded free-drift search: an initial burn, then finite firings near the box walls (and a blind drift toward
the shared target estimate), as the spacecraft_wrench contributor. Plain class driven by the control node."""

import math
import random

from geometry_msgs.msg import Wrench
from nav_msgs.msg import Odometry
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from .shared_target import SharedTargets, now_s


def yaw_from_quaternion(quaternion) -> float:
    """Extract planar yaw from a geometry_msgs quaternion."""
    sin_yaw = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cos_yaw = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(sin_yaw, cos_yaw)


def rotate_body_to_global(x: float, y: float, yaw: float):
    """Rotate a planar vector from the spacecraft body into the global frame."""
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return cosine * x - sine * y, sine * x + cosine * y


def rotate_global_to_body(x: float, y: float, yaw: float):
    """Rotate a planar vector from the global frame into the spacecraft body."""
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return cosine * x + sine * y, -sine * x + cosine * y


def limit_vector(x: float, y: float, maximum: float):
    """Limit a planar vector while preserving its direction."""
    magnitude = math.hypot(x, y)
    if magnitude <= maximum or magnitude == 0.0:
        return x, y
    scale = maximum / magnitude
    return scale * x, scale * y


def burn_ramp_scale(elapsed: float, duration: float, ramp: float) -> float:
    """Return the raised-cosine throttle scale in [0, 1] for the initial burn.

    A step in commanded force demands infinite chassis acceleration to render,
    so the thrust rises and falls over `ramp` seconds instead. Symmetric ends
    keep the burn centred; ramp is clamped to half the duration so the two
    halves cannot overlap.
    """
    if elapsed < 0.0 or elapsed > duration:
        return 0.0

    ramp = min(ramp, 0.5 * duration)
    if ramp <= 0.0:
        return 1.0

    if elapsed < ramp:
        phase = elapsed / ramp
    elif elapsed > duration - ramp:
        phase = (duration - elapsed) / ramp
    else:
        return 1.0

    return 0.5 * (1.0 - math.cos(math.pi * phase))


def boundary_firing(
    x: float,
    y: float,
    velocity_x: float,
    velocity_y: float,
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
    boundary_margin: float,
    firing_force: float,
    rebound_speed: float,
):
    """Return an inward global force and whether a boundary region is active."""
    force_x = 0.0
    force_y = 0.0
    near_boundary = False

    if x >= x_max - boundary_margin:
        near_boundary = True
        if velocity_x > -rebound_speed:
            force_x = -firing_force
    elif x <= x_min + boundary_margin:
        near_boundary = True
        if velocity_x < rebound_speed:
            force_x = firing_force

    if y >= y_max - boundary_margin:
        near_boundary = True
        if velocity_y > -rebound_speed:
            force_y = -firing_force
    elif y <= y_min + boundary_margin:
        near_boundary = True
        if velocity_y < rebound_speed:
            force_y = firing_force

    force_x, force_y = limit_vector(force_x, force_y, firing_force)
    return force_x, force_y, near_boundary


def drift_firing(x, y, velocity_x, velocity_y, centre_x, centre_y, drift_speed, firing_force, stop_distance):
    """Fire toward a shared target centre until the closing speed reaches drift_speed, then coast.
    No firing within stop_distance: the robot's own camera takes over from there."""
    dx, dy = centre_x - x, centre_y - y
    distance = math.hypot(dx, dy)
    if distance <= stop_distance or (velocity_x * dx + velocity_y * dy) / distance >= drift_speed: return 0.0, 0.0
    return firing_force * dx / distance, firing_force * dy / distance



def qos_profile_from_parameters(node) -> QoSProfile:
    """Build a configurable QoS profile with reliable defaults."""
    depth = int(node.get_parameter("qos_depth").value)
    reliability = str(node.get_parameter("qos_reliability").value)
    history = str(node.get_parameter("qos_history").value)
    if depth <= 0:
        raise ValueError("qos_depth must be positive")
    if reliability not in ("RELIABLE", "BEST_EFFORT"):
        raise ValueError(f"Invalid qos_reliability: {reliability}")
    if history not in ("KEEP_LAST", "KEEP_ALL"):
        raise ValueError(f"Invalid qos_history: {history}")
    return QoSProfile(
        depth=depth,
        reliability=(
            ReliabilityPolicy.RELIABLE
            if reliability == "RELIABLE"
            else ReliabilityPolicy.BEST_EFFORT
        ),
        history=(
            HistoryPolicy.KEEP_LAST
            if history == "KEEP_LAST"
            else HistoryPolicy.KEEP_ALL
        ),
    )


class BoxSearch:
    """Fire briefly at startup and near search-box boundaries."""

    def __init__(self, node, p, wrench_publisher):
        """p(name, default) reads bounding_box_search.* and the shared /** box and share_* values."""
        self.get_clock, self.get_logger = node.get_clock, node.get_logger
        self.reference_timeout = float(p("reference_timeout", 0.25))
        self.global_frame_id = str(p("global_frame_id", "swarm_map")).strip("/")
        self.x_min, self.x_max = float(p("x_min", -1.0)), float(p("x_max", 1.0))
        self.y_min, self.y_max = float(p("y_min", -1.0)), float(p("y_max", 1.0))
        self.boundary_margin = float(p("boundary_margin", 0.25))
        self.boundary_force = float(p("boundary_force", 1.0))
        self.rebound_speed = float(p("rebound_speed", 0.15))
        self.maximum_force = float(p("maximum_force", 1.0))
        self.maximum_torque = float(p("maximum_torque", 0.2))
        self.initial_burn_duration = float(p("initial_burn_duration", 1.0))
        self.initial_force_x = float(p("initial_force_x", 0.8))
        self.initial_force_y = float(p("initial_force_y", 0.4))
        self.initial_burn_ramp = float(p("initial_burn_ramp", 0.0))
        self.initial_torque = float(p("initial_torque", 0.1))
        self.lateral_kick_fraction = float(p("lateral_kick_fraction", 0.15))
        seed = int(p("random_seed", 0))
        self.validate_parameters()

        # Seed 0 means a fresh sequence each run; any other value is reproducible.
        self.generator = random.Random(seed or None)
        self.lateral_sign = 1.0
        self.was_near_boundary = False
        self.latest_reference = None
        self.last_reference_time = None
        self.initial_burn_start = None

        # Zero outside SEARCH, before arm and mid-box (the node zero-gates the debug topic).
        self.wrench_publisher = wrench_publisher
        # Runtime on/off switch so a supervisor can hand control to the capture
        # controller. Disabled means we keep publishing an explicit zero wrench.
        self.enabled = True
        # Drift toward the neighbours' shared target estimate while blind (free drift, not flocking)
        self.share_target = bool(p("share_target", True))
        self.drift_speed, self.drift_force, self.drift_stop = (
            float(p(n, d)) for n, d in (("share_drift_speed", 0.03), ("share_drift_force", 0.5), ("share_drift_stop", 1.0)))
        self.shared = SharedTargets(node.get_namespace().strip("/"), float(p("share_timeout", 2.0)))


    def handle_enable(self, request, response):
        """Enable or disable the search wrench at runtime."""
        self.enabled = request.data
        response.success = True
        response.message = f"search {'enabled' if request.data else 'disabled'}"
        self.get_logger().info(response.message)
        return response

    def validate_parameters(self) -> None:
        """Reject inconsistent search geometry and firing parameters."""
        values = (
            self.reference_timeout,
            self.x_min,
            self.x_max,
            self.y_min,
            self.y_max,
            self.boundary_margin,
            self.boundary_force,
            self.rebound_speed,
            self.maximum_force,
            self.maximum_torque,
            self.initial_burn_duration,
            self.initial_burn_ramp,
            self.initial_force_x,
            self.initial_force_y,
            self.initial_torque,
            self.lateral_kick_fraction,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("Bounding-box parameters must be finite")
        if not 0.0 <= self.lateral_kick_fraction < 1.0:
            raise ValueError("lateral_kick_fraction must be within [0, 1)")
        if self.initial_burn_ramp < 0.0:
            raise ValueError("initial_burn_ramp must be non-negative")
        if self.x_min >= self.x_max or self.y_min >= self.y_max:
            raise ValueError("Bounding-box minimums must be below maximums")
        if not 0.0 < self.boundary_margin < 0.5 * min(
            self.x_max - self.x_min, self.y_max - self.y_min
        ):
            raise ValueError("boundary_margin is inconsistent with the box")
        positive = (
            self.reference_timeout,
            self.boundary_force,
            self.rebound_speed,
            self.maximum_force,
            self.maximum_torque,
            self.initial_burn_duration,
        )
        if not all(value > 0.0 for value in positive):
            raise ValueError("Timing, limits, and firing values must be positive")

    def reference_callback(self, message: Odometry) -> None:
        """Store the latest virtual state and its receipt time."""
        now = self.get_clock().now()
        self.latest_reference = message
        self.last_reference_time = now.nanoseconds
        if (
            self.initial_burn_start is None
            and message.header.frame_id.lstrip("/") == self.global_frame_id
        ):
            self.initial_burn_start = now

    def reference_is_valid(self, now) -> bool:
        """Return whether virtual odometry is fresh and in the expected frame."""
        if self.latest_reference is None or self.last_reference_time is None:
            return False
        age = (now.nanoseconds - self.last_reference_time) * 1.0e-9
        return (
            0.0 <= age <= self.reference_timeout
            and self.latest_reference.header.frame_id.lstrip("/")
            == self.global_frame_id
        )

    def update(self) -> None:
        """Publish the current finite firing or an explicit zero wrench."""
        # Disabled by the supervisor outside SEARCH: drop out of the spacecraft
        # wrench sum by publishing an explicit zero (gating by zero).
        if not self.enabled:
            self.wrench_publisher.publish(Wrench())
            return

        now = self.get_clock().now()
        if not self.reference_is_valid(now) or self.initial_burn_start is None:
            self.wrench_publisher.publish(Wrench())
            return

        reference = self.latest_reference
        yaw = yaw_from_quaternion(reference.pose.pose.orientation)
        global_velocity_x, global_velocity_y = rotate_body_to_global(
            reference.twist.twist.linear.x,
            reference.twist.twist.linear.y,
            yaw,
        )
        global_force_x, global_force_y, near_boundary = boundary_firing(
            reference.pose.pose.position.x,
            reference.pose.pose.position.y,
            global_velocity_x,
            global_velocity_y,
            self.x_min,
            self.x_max,
            self.y_min,
            self.y_max,
            self.boundary_margin,
            min(self.boundary_force, self.maximum_force),
            self.rebound_speed,
        )

        # Resample once per bounce, not per tick, or the kick averages away.
        if near_boundary and not self.was_near_boundary:
            self.lateral_sign = self.generator.choice((-1.0, 1.0))
        self.was_near_boundary = near_boundary

        # Kick along the free axis so bounces do not collapse into one line.
        kick = (
            self.lateral_sign
            * self.lateral_kick_fraction
            * min(self.boundary_force, self.maximum_force)
        )
        if global_force_x != 0.0 and global_force_y == 0.0:
            global_force_y = kick
        elif global_force_y != 0.0 and global_force_x == 0.0:
            global_force_x = kick

        torque = 0.0
        elapsed = (now - self.initial_burn_start).nanoseconds * 1.0e-9
        if not near_boundary and elapsed <= self.initial_burn_duration:
            throttle = burn_ramp_scale(
                elapsed,
                self.initial_burn_duration,
                self.initial_burn_ramp,
            )
            global_force_x = throttle * self.initial_force_x
            global_force_y = throttle * self.initial_force_y
            torque = throttle * self.initial_torque
        elif not near_boundary and self.share_target and (centre := self.shared.centre(now_s(self))) is not None:
            global_force_x, global_force_y = drift_firing(
                reference.pose.pose.position.x, reference.pose.pose.position.y, global_velocity_x, global_velocity_y,
                *centre, self.drift_speed, min(self.drift_force, self.maximum_force), self.drift_stop)

        global_force_x, global_force_y = limit_vector(
            global_force_x, global_force_y, self.maximum_force
        )
        torque = max(-self.maximum_torque, min(self.maximum_torque, torque))
        body_force_x, body_force_y = rotate_global_to_body(
            global_force_x, global_force_y, yaw
        )

        wrench = Wrench()
        wrench.force.x = body_force_x
        wrench.force.y = body_force_y
        wrench.torque.z = torque
        self.wrench_publisher.publish(wrench)
