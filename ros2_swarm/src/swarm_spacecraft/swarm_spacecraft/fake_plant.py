"""Kinematic stand-in for a RoboMaster + its localization, for hardware-free runs.

Under VIRTUAL_PLANT the centralized launch drops the robomaster driver, the
Marvelmind chain and pose_publisher, and starts one of these per robot instead.
It subscribes to the same `cmd_vel` the real chassis would and republishes the
exact two topics pose_publisher produced -- `pose` (StatePos, theta in DEGREES)
and `localization/odom` (Odometry, body-frame twist) -- so the whole mission loop
(virtual_scene -> guidance -> mode_supervisor -> virtual_spacecraft -> nav2 ->
velocity_smoother -> cmd_vel) closes with no hardware.

The model is a rate/accel-limited holonomic integrator: the commanded body twist
is reached through a bounded acceleration and capped speed, so motion resembles a
chassis rather than a teleporting setpoint. It is NOT real chassis dynamics, so it
validates logic and flow, not controller robustness to actuator behaviour.
"""

import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node

from swarm_interfaces.msg import StatePos

from .dynamics import rotate_body_to_inertial, wrap_angle
from .virtual_spacecraft_node import (
    quaternion_from_yaw,
    qos_profile_from_parameters,
)


def _degrees_wrapped(angle: float) -> float:
    """Radians -> degrees wrapped to [-180, 180), matching pose_publisher."""
    return math.degrees(wrap_angle(angle))


def _step_toward(current: float, target: float, max_delta: float) -> float:
    """Move `current` toward `target` by at most `max_delta`."""
    delta = target - current
    if delta > max_delta:
        return current + max_delta
    if delta < -max_delta:
        return current - max_delta
    return target


class FakePlant(Node):
    def __init__(self):
        super().__init__("fake_plant")

        self.declare_parameter("init_pose", [0.0, 0.0, 0.0])  # x m, y m, yaw deg
        self.declare_parameter("timer_frequency", 0.02)       # seconds per tick
        self.declare_parameter("global_frame_id", "swarm_map")
        self.declare_parameter("base_frame_id", "")           # "" -> "<ns>/base_link"
        self.declare_parameter("max_speed", 0.25)
        self.declare_parameter("max_yaw_rate", 0.5)
        self.declare_parameter("max_linear_accel", 0.5)
        self.declare_parameter("max_yaw_accel", 1.0)
        self.declare_parameter("cmd_timeout", 0.25)
        self.declare_parameter("qos_depth", 10)
        self.declare_parameter("qos_reliability", "RELIABLE")
        self.declare_parameter("qos_history", "KEEP_LAST")

        init = list(self.get_parameter("init_pose").value)
        self.x = float(init[0])
        self.y = float(init[1])
        self.yaw = math.radians(float(init[2]) if len(init) > 2 else 0.0)
        self.vx = self.vy = self.yaw_rate = 0.0            # current body twist

        period = float(self.get_parameter("timer_frequency").value)
        self.global_frame = str(self.get_parameter("global_frame_id").value)
        base = str(self.get_parameter("base_frame_id").value).lstrip("/")
        ns = self.get_namespace().strip("/")
        self.base_frame = base or (f"{ns}/base_link" if ns else "base_link")
        self.max_speed = float(self.get_parameter("max_speed").value)
        self.max_yaw_rate = float(self.get_parameter("max_yaw_rate").value)
        self.max_linear_accel = float(self.get_parameter("max_linear_accel").value)
        self.max_yaw_accel = float(self.get_parameter("max_yaw_accel").value)
        self.cmd_timeout = float(self.get_parameter("cmd_timeout").value)

        qos = qos_profile_from_parameters(self)
        self.pose_pub = self.create_publisher(StatePos, "pose", qos)
        self.odom_pub = self.create_publisher(Odometry, "localization/odom", qos)
        self.create_subscription(Twist, "cmd_vel", self._on_cmd, qos)

        self.cmd = (0.0, 0.0, 0.0)          # last commanded body twist
        self.cmd_time = None
        self.last_time = self.get_clock().now()
        self.create_timer(max(period, 1e-3), self.tick)

        self.get_logger().info(
            f"fake_plant: start ({self.x:.2f}, {self.y:.2f}, "
            f"{math.degrees(self.yaw):.0f} deg), base_frame '{self.base_frame}', "
            f"max_speed {self.max_speed} m/s, accel {self.max_linear_accel} m/s^2.")

    def _on_cmd(self, msg: Twist):
        self.cmd = (msg.linear.x, msg.linear.y, msg.angular.z)
        self.cmd_time = self.get_clock().now().nanoseconds

    def tick(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if dt <= 0.0:
            return

        # Commanded body twist, zeroed once the command goes stale (the accel
        # limit then brakes smoothly instead of snapping to a halt).
        fresh = (self.cmd_time is not None
                 and (now.nanoseconds - self.cmd_time) * 1e-9 <= self.cmd_timeout)
        cvx, cvy, cwz = self.cmd if fresh else (0.0, 0.0, 0.0)

        # Accel-limit the translational change as a vector so heading is not
        # deflected toward whichever axis saturated first.
        dvx, dvy = cvx - self.vx, cvy - self.vy
        change = math.hypot(dvx, dvy)
        max_change = self.max_linear_accel * dt
        if change > max_change > 0.0:
            scale = max_change / change
            dvx *= scale
            dvy *= scale
        self.vx += dvx
        self.vy += dvy
        speed = math.hypot(self.vx, self.vy)
        if speed > self.max_speed > 0.0:
            scale = self.max_speed / speed
            self.vx *= scale
            self.vy *= scale

        self.yaw_rate = _step_toward(self.yaw_rate, cwz, self.max_yaw_accel * dt)
        self.yaw_rate = max(-self.max_yaw_rate, min(self.max_yaw_rate, self.yaw_rate))

        ivx, ivy = rotate_body_to_inertial(self.vx, self.vy, self.yaw)
        self.x += ivx * dt
        self.y += ivy * dt
        self.yaw = wrap_angle(self.yaw + self.yaw_rate * dt)

        self._publish(now.to_msg())

    def _publish(self, stamp):
        pose = StatePos()
        pose.x = self.x
        pose.y = self.y
        pose.theta = _degrees_wrapped(self.yaw)
        self.pose_pub.publish(pose)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.global_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        z, w = quaternion_from_yaw(self.yaw)
        odom.pose.pose.orientation.z = z
        odom.pose.pose.orientation.w = w
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.yaw_rate
        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = FakePlant()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
