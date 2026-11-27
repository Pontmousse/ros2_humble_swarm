"""Velocity smoother: a line-for-line Python port of nav2 Humble's VelocitySmoother (smootherTimer,
findEtaConstraint, applyConstraints), OPEN_LOOP feedback only. Plain class driven by the control node, which
steps it once per control tick, so smoothing_frequency is the loop rate: the m/s and m/s^2 limits are the
same as the C++ node's, applied in smaller steps.
"""

import math

from geometry_msgs.msg import Twist


def clamp(value, low, high):
    return min(max(value, low), high)


class VelocitySmoother:
    def __init__(self, node, p, publisher, smoothing_frequency):
        """p(name, default) reads velocity_smoother.*; defaults are nav2's."""
        self.get_clock, self.get_logger = node.get_clock, node.get_logger
        self.smoothing_frequency = float(smoothing_frequency)
        self.scale_velocities = bool(p("scale_velocities", False))
        if str(p("feedback", "OPEN_LOOP")) != "OPEN_LOOP":
            raise ValueError("Only OPEN_LOOP feedback is ported (the swarm config uses it).")
        self.max_velocities = [float(v) for v in p("max_velocity", [0.50, 0.0, 2.5])]
        self.min_velocities = [float(v) for v in p("min_velocity", [-0.50, 0.0, -2.5])]
        self.max_accels = [float(v) for v in p("max_accel", [2.5, 0.0, 3.2])]
        self.max_decels = [float(v) for v in p("max_decel", [-2.5, 0.0, -3.2])]
        self.deadband_velocities = [float(v) for v in p("deadband_velocity", [0.0, 0.0, 0.0])]
        self.velocity_timeout_ns = int(round(float(p("velocity_timeout", 1.0)) * 1e9))
        for limits in (self.max_velocities, self.min_velocities, self.max_accels, self.max_decels,
                       self.deadband_velocities):
            if len(limits) != 3:
                raise ValueError("Invalid setting of kinematic and/or deadband limits! "
                                 "All limits must be size of 3 representing (x, y, theta).")
        for i in range(3):
            if self.max_decels[i] > 0.0:
                raise ValueError("Positive values set of deceleration! These should be negative to slow down!")
            if self.max_accels[i] < 0.0:
                raise ValueError("Negative values set of acceleration! These should be positive to speed up!")
            if self.min_velocities[i] > self.max_velocities[i]:
                raise ValueError("Min velocities are higher than max velocities!")
        self.publisher = publisher
        self.command = None                   # [vx, vy, wz], nav2's command_ (mutated in place, as there)
        self.last_command_time = None
        self.last_cmd = [0.0, 0.0, 0.0]       # nav2's last_cmd_ (pre-deadband)
        self.stopped = False

    def command_callback(self, msg: Twist) -> None:
        """nav2 inputCommandCallback: keep the latest finite command."""
        cmd = [msg.linear.x, msg.linear.y, msg.angular.z]
        if not all(math.isfinite(v) for v in (*cmd, msg.linear.z, msg.angular.x, msg.angular.y)):
            self.get_logger().error("Velocity message contains NaNs or Infs! Ignoring as invalid!")
            return
        self.command = cmd
        self.last_command_time = self.get_clock().now().nanoseconds

    def find_eta_constraint(self, v_curr, v_cmd, accel, decel):
        dv = v_cmd - v_curr
        if abs(v_cmd) >= abs(v_curr) and v_curr * v_cmd >= 0.0:
            v_component_max, v_component_min = accel / self.smoothing_frequency, -accel / self.smoothing_frequency
        else:
            v_component_max, v_component_min = -decel / self.smoothing_frequency, decel / self.smoothing_frequency
        if dv > v_component_max:
            return v_component_max / dv
        if dv < v_component_min:
            return v_component_min / dv
        return -1.0

    def apply_constraints(self, v_curr, v_cmd, accel, decel, eta):
        dv = v_cmd - v_curr
        if abs(v_cmd) >= abs(v_curr) and v_curr * v_cmd >= 0.0:
            v_component_max, v_component_min = accel / self.smoothing_frequency, -accel / self.smoothing_frequency
        else:
            v_component_max, v_component_min = -decel / self.smoothing_frequency, decel / self.smoothing_frequency
        return v_curr + clamp(eta * dv, v_component_min, v_component_max)

    def update(self) -> None:
        """nav2 smootherTimer."""
        if self.command is None:                                  # wait until the first command is received
            return
        # Velocity timeout: if nothing received, decelerate to zero, then stop publishing.
        if self.get_clock().now().nanoseconds - self.last_command_time > self.velocity_timeout_ns:
            if self.last_cmd == [0.0, 0.0, 0.0] or self.stopped:
                self.stopped = True
                return
            self.command[:] = [0.0, 0.0, 0.0]
        self.stopped = False
        current = self.last_cmd                                   # OPEN_LOOP
        for i in range(3):                                        # absolute velocity limits
            self.command[i] = clamp(self.command[i], self.min_velocities[i], self.max_velocities[i])
        # One eta for all axes, so the command keeps its direction within the change-of-velocity bounds.
        eta = 1.0
        if self.scale_velocities:
            for i in range(3):
                curr_eta = self.find_eta_constraint(current[i], self.command[i], self.max_accels[i], self.max_decels[i])
                if curr_eta > 0.0 and abs(1.0 - curr_eta) > abs(1.0 - eta):
                    eta = curr_eta
        out = [self.apply_constraints(current[i], self.command[i], self.max_accels[i], self.max_decels[i], eta)
               for i in range(3)]
        self.last_cmd = out
        cmd_vel = Twist()                                         # deadband, then publish
        cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z = (
            0.0 if abs(v) < d else v for v, d in zip(out, self.deadband_velocities))
        self.publisher.publish(cmd_vel)
