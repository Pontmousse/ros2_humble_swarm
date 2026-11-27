"""PID position-error feedback plus velocity-feedforward tracking of the virtual spacecraft (formerly the
nav2_pidff node). Plain class driven by the control node."""

from dataclasses import dataclass
import math
from typing import Optional, Tuple

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


@dataclass
class ReferenceState:
    """Virtual pose with body-frame velocity feedforward."""

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    body_vx: float = 0.0
    body_vy: float = 0.0
    yaw_rate: float = 0.0


@dataclass
class MeasuredState:
    """Measured planar chassis pose and body-frame velocity."""

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    body_vx: float = 0.0
    body_vy: float = 0.0
    yaw_rate: float = 0.0


@dataclass
class IntegralState:
    """Integrated tracking errors in the inertial frame."""

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


def yaw_from_quaternion(quaternion) -> float:
    """Extract planar yaw from a geometry_msgs quaternion."""
    sin_yaw = 2.0 * (
        quaternion.w * quaternion.z
        + quaternion.x * quaternion.y
    )
    cos_yaw = 1.0 - 2.0 * (
        quaternion.y * quaternion.y
        + quaternion.z * quaternion.z
    )
    return math.atan2(sin_yaw, cos_yaw)


def wrap_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def rotate_body_to_inertial(
    x: float,
    y: float,
    yaw: float,
) -> Tuple[float, float]:
    """Rotate a planar vector from body coordinates to inertial coordinates."""
    cosine = math.cos(yaw)
    sine = math.sin(yaw)

    return (
        cosine * x - sine * y,
        sine * x + cosine * y,
    )


def rotate_inertial_to_body(
    x: float,
    y: float,
    yaw: float,
) -> Tuple[float, float]:
    """Rotate a planar vector from inertial coordinates to body coordinates."""
    cosine = math.cos(yaw)
    sine = math.sin(yaw)

    return (
        cosine * x + sine * y,
        -sine * x + cosine * y,
    )


def limit_vector(
    x: float,
    y: float,
    maximum: float,
) -> Tuple[float, float, bool]:
    """Apply direction-preserving magnitude saturation to a planar vector."""
    if maximum < 0.0:
        raise ValueError("maximum must be non-negative")

    magnitude = math.hypot(x, y)

    if magnitude <= maximum or magnitude == 0.0:
        return x, y, False

    scale = maximum / magnitude

    return (
        scale * x,
        scale * y,
        True,
    )


def tracking_errors(
    reference: ReferenceState,
    measured: MeasuredState,
) -> Tuple[float, float, float]:
    """Return position and wrapped yaw tracking errors."""
    return (
        reference.x - measured.x,
        reference.y - measured.y,
        wrap_angle(reference.yaw - measured.yaw),
    )


def tracking_command(
    reference: ReferenceState,
    measured: MeasuredState,
    integral: IntegralState,
    enable_pid: bool,
    position_gain: float,
    position_integral_gain: float,
    velocity_error_gain: float,
    yaw_gain: float,
    yaw_integral_gain: float,
    yaw_rate_error_gain: float,
    maximum_speed: float,
    maximum_yaw_rate: float,
) -> Tuple[float, float, float, bool, bool]:
    """
    Calculate the saturated chassis-body-frame velocity command.

    PID disabled:
        v_cmd = v_ref + Kp * position_error

    PID enabled:
        v_cmd = v_ref
              + Kp * position_error
              + Ki * integral(position_error)
              + Kd * velocity_error

    Yaw follows the corresponding scalar form.
    """

    position_error_x, position_error_y, yaw_error = tracking_errors(
        reference,
        measured,
    )

    # ---------------------------------------------------------------
    # Reference velocity:
    # virtual-spacecraft Odometry twist is body-frame velocity.
    # Convert it to the common inertial/global frame before applying
    # position feedback.
    # ---------------------------------------------------------------

    reference_vx, reference_vy = rotate_body_to_inertial(
        reference.body_vx,
        reference.body_vy,
        reference.yaw,
    )

    # Base controller: identical conceptually to the existing P+FF node.
    inertial_vx = (
        reference_vx
        + position_gain * position_error_x
    )

    inertial_vy = (
        reference_vy
        + position_gain * position_error_y
    )

    yaw_command = (
        reference.yaw_rate
        + yaw_gain * yaw_error
    )

    # ---------------------------------------------------------------
    # PID additions
    # ---------------------------------------------------------------

    if enable_pid:

        # Physical velocity from localization/odom.
        # Convert measured body-frame velocity to the inertial frame.
        measured_vx, measured_vy = rotate_body_to_inertial(
            measured.body_vx,
            measured.body_vy,
            measured.yaw,
        )

        velocity_error_x = reference_vx - measured_vx
        velocity_error_y = reference_vy - measured_vy
        yaw_rate_error = reference.yaw_rate - measured.yaw_rate

        inertial_vx += (
            position_integral_gain * integral.x
            + velocity_error_gain * velocity_error_x
        )

        inertial_vy += (
            position_integral_gain * integral.y
            + velocity_error_gain * velocity_error_y
        )

        yaw_command += (
            yaw_integral_gain * integral.yaw
            + yaw_rate_error_gain * yaw_rate_error
        )

    # ---------------------------------------------------------------
    # Convert global/inertial velocity command into the physical
    # RoboMaster body frame.
    # ---------------------------------------------------------------

    body_x, body_y = rotate_inertial_to_body(
        inertial_vx,
        inertial_vy,
        measured.yaw,
    )

    body_x, body_y, linear_saturated = limit_vector(
        body_x,
        body_y,
        maximum_speed,
    )

    limited_yaw = max(
        -maximum_yaw_rate,
        min(maximum_yaw_rate, yaw_command),
    )

    yaw_saturated = limited_yaw != yaw_command

    return (
        body_x,
        body_y,
        limited_yaw,
        linear_saturated,
        yaw_saturated,
    )


def inputs_are_fresh(
    now_nanoseconds: int,
    odometry_time: Optional[int],
    reference_time: Optional[int],
    odometry_timeout: float,
    reference_timeout: float,
) -> bool:
    """Return whether both controller inputs exist and are recent."""

    if odometry_time is None or reference_time is None:
        return False

    odometry_age = (
        now_nanoseconds - odometry_time
    ) * 1.0e-9

    reference_age = (
        now_nanoseconds - reference_time
    ) * 1.0e-9

    return (
        0.0 <= odometry_age <= odometry_timeout
        and
        0.0 <= reference_age <= reference_timeout
    )


def frames_match(
    odometry_frame: str,
    reference_frame: str,
) -> bool:
    """Return whether both inputs name the same non-empty global frame."""
    return (
        bool(odometry_frame)
        and odometry_frame == reference_frame
    )


def validate_controller_parameters(
    timer_frequency: float,
    position_gain: float,
    position_integral_gain: float,
    velocity_error_gain: float,
    yaw_gain: float,
    yaw_integral_gain: float,
    yaw_rate_error_gain: float,
    maximum_speed: float,
    maximum_yaw_rate: float,
    odometry_timeout: float,
    reference_timeout: float,
) -> None:
    """Reject unsafe or non-finite controller parameters."""

    positive_values = (
        timer_frequency,
        maximum_speed,
        maximum_yaw_rate,
        odometry_timeout,
        reference_timeout,
    )

    if not all(
        math.isfinite(value) and value > 0.0
        for value in positive_values
    ):
        raise ValueError(
            "period, timeout, and limit parameters must be positive"
        )

    gains = (
        position_gain,
        position_integral_gain,
        velocity_error_gain,
        yaw_gain,
        yaw_integral_gain,
        yaw_rate_error_gain,
    )

    if not all(
        math.isfinite(value) and value >= 0.0
        for value in gains
    ):
        raise ValueError(
            "PID gains must be finite and non-negative"
        )


class Tracker:
    """Track virtual spacecraft odometry using PID feedback plus feedforward."""

    def __init__(self, node, p, command_publisher, timer_frequency):
        """p(name, default) reads nav2_pidff.*; timer_frequency is the control loop period."""
        self.get_clock, self.get_logger = node.get_clock, node.get_logger
        self.timer_frequency = float(timer_frequency)
        # PID gate: false -> original P + velocity feedforward, true -> PID + velocity feedforward.
        self.enable_pid = bool(p("enable_pid", False))
        # position_gain / yaw_gain are Kp; the integral and error-rate gains are the PID additions.
        self.position_gain = float(p("position_gain", 1.0))
        self.position_integral_gain = float(p("position_integral_gain", 0.0))
        self.velocity_error_gain = float(p("velocity_error_gain", 0.0))
        self.yaw_gain = float(p("yaw_gain", 2.0))
        self.yaw_integral_gain = float(p("yaw_integral_gain", 0.0))
        self.yaw_rate_error_gain = float(p("yaw_rate_error_gain", 0.0))
        self.maximum_speed = float(p("maximum_speed", 0.5))
        self.maximum_yaw_rate = float(p("maximum_yaw_rate", 1.0))
        self.odometry_timeout = float(p("odometry_timeout", 0.25))
        self.reference_timeout = float(p("reference_timeout", 0.25))
        validate_controller_parameters(
            self.timer_frequency,
            self.position_gain,
            self.position_integral_gain,
            self.velocity_error_gain,
            self.yaw_gain,
            self.yaw_integral_gain,
            self.yaw_rate_error_gain,
            self.maximum_speed,
            self.maximum_yaw_rate,
            self.odometry_timeout,
            self.reference_timeout,
        )

        # ---------------------------------------------------------------
        # State
        # ---------------------------------------------------------------

        self.reference = ReferenceState()
        self.measured = MeasuredState()
        self.integral = IntegralState()

        self.odometry_frame_id = ""
        self.reference_frame_id = ""

        self.last_odometry_time = None
        self.last_reference_time = None
        self.last_control_time = None

        self.saturation_count = 0

        self.command_publisher = command_publisher
        self.get_logger().info("PID+FF spacecraft tracking controller ready" if self.enable_pid
                               else "P+FF spacecraft tracking controller ready (PID disabled)")

    def odometry_callback(
        self,
        message: Odometry,
    ) -> None:
        """Store the latest physical chassis state."""

        self.measured = MeasuredState(
            x=message.pose.pose.position.x,
            y=message.pose.pose.position.y,
            yaw=yaw_from_quaternion(
                message.pose.pose.orientation
            ),
            body_vx=message.twist.twist.linear.x,
            body_vy=message.twist.twist.linear.y,
            yaw_rate=message.twist.twist.angular.z,
        )

        self.odometry_frame_id = (
            message.header.frame_id.lstrip("/")
        )

        self.last_odometry_time = (
            self.get_clock().now().nanoseconds
        )

    def reference_callback(
        self,
        message: Odometry,
    ) -> None:
        """Store the latest virtual spacecraft reference."""

        self.reference = ReferenceState(
            x=message.pose.pose.position.x,
            y=message.pose.pose.position.y,
            yaw=yaw_from_quaternion(
                message.pose.pose.orientation
            ),
            body_vx=message.twist.twist.linear.x,
            body_vy=message.twist.twist.linear.y,
            yaw_rate=message.twist.twist.angular.z,
        )

        self.reference_frame_id = (
            message.header.frame_id.lstrip("/")
        )

        self.last_reference_time = (
            self.get_clock().now().nanoseconds
        )

    def reset_integral(self) -> None:
        """Clear PID integral state."""

        self.integral = IntegralState()
        self.last_control_time = None

    def update(self) -> None:
        """Publish tracking command, or stop when an input is stale."""

        now_nanoseconds = (
            self.get_clock().now().nanoseconds
        )

        inputs_fresh = inputs_are_fresh(
            now_nanoseconds,
            self.last_odometry_time,
            self.last_reference_time,
            self.odometry_timeout,
            self.reference_timeout,
        )

        if (
            not inputs_fresh
            or not frames_match(
                self.odometry_frame_id,
                self.reference_frame_id,
            )
        ):
            self.reset_integral()
            self.publish_stop()
            return

        # ---------------------------------------------------------------
        # Calculate actual controller dt.
        #
        # Do not assume the timer callback executes at exactly the requested
        # period.
        # ---------------------------------------------------------------

        if self.last_control_time is None:
            dt = 0.0
        else:
            dt = (
                now_nanoseconds
                - self.last_control_time
            ) * 1.0e-9

        self.last_control_time = now_nanoseconds

        position_error_x, position_error_y, yaw_error = tracking_errors(
            self.reference,
            self.measured,
        )

        # ---------------------------------------------------------------
        # Candidate integral update.
        #
        # The candidate is tested against actuator saturation before being
        # accepted. This provides conditional-integration anti-windup.
        # ---------------------------------------------------------------

        candidate_integral = IntegralState(
            x=self.integral.x,
            y=self.integral.y,
            yaw=self.integral.yaw,
        )

        if (
            self.enable_pid
            and dt > 0.0
            and math.isfinite(dt)
        ):
            candidate_integral.x += (
                position_error_x * dt
            )

            candidate_integral.y += (
                position_error_y * dt
            )

            candidate_integral.yaw += (
                yaw_error * dt
            )

        # First calculate using the candidate integral.
        (
            body_x,
            body_y,
            yaw_rate,
            linear_saturated,
            yaw_saturated,
        ) = tracking_command(
            self.reference,
            self.measured,
            candidate_integral,
            self.enable_pid,
            self.position_gain,
            self.position_integral_gain,
            self.velocity_error_gain,
            self.yaw_gain,
            self.yaw_integral_gain,
            self.yaw_rate_error_gain,
            self.maximum_speed,
            self.maximum_yaw_rate,
        )

        # ---------------------------------------------------------------
        # Anti-windup:
        #
        # If the translational command saturated, reject the new x/y
        # integral contribution.
        #
        # If yaw saturated, reject the new yaw integral contribution.
        # ---------------------------------------------------------------

        if self.enable_pid:

            recompute = False

            accepted_integral = IntegralState(
                x=candidate_integral.x,
                y=candidate_integral.y,
                yaw=candidate_integral.yaw,
            )

            if linear_saturated:
                accepted_integral.x = (
                    self.integral.x
                )
                accepted_integral.y = (
                    self.integral.y
                )
                recompute = True

            if yaw_saturated:
                accepted_integral.yaw = (
                    self.integral.yaw
                )
                recompute = True

            self.integral = accepted_integral

            if recompute:
                (
                    body_x,
                    body_y,
                    yaw_rate,
                    linear_saturated,
                    yaw_saturated,
                ) = tracking_command(
                    self.reference,
                    self.measured,
                    self.integral,
                    self.enable_pid,
                    self.position_gain,
                    self.position_integral_gain,
                    self.velocity_error_gain,
                    self.yaw_gain,
                    self.yaw_integral_gain,
                    self.yaw_rate_error_gain,
                    self.maximum_speed,
                    self.maximum_yaw_rate,
                )

        # ---------------------------------------------------------------
        # Publish
        # ---------------------------------------------------------------

        command = Twist()

        command.linear.x = body_x
        command.linear.y = body_y
        command.angular.z = yaw_rate

        self.command_publisher.publish(
            command
        )

        if linear_saturated or yaw_saturated:
            self.saturation_count += 1

            if self.saturation_count % 100 == 1:
                self.get_logger().warn(
                    "Physical velocity command is saturated"
                )

    def publish_stop(self) -> None:
        """Publish a zero velocity command."""
        self.command_publisher.publish(
            Twist()
        )
