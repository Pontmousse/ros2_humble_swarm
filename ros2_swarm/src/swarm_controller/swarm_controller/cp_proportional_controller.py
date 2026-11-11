from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np

# Built-in messages
from geometry_msgs.msg import Twist, Wrench
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool
from std_msgs.msg import Int32

# Custom messages for target position
from swarm_interfaces.msg import StatePos as State
from swarm_interfaces.msg import CoordXY, Markers

# Outlier guards on the target track. Not tuning knobs: the gap is longer than
# the 0.61 s frozen-fix period seen on RM6, so only a real dropout trips it.
TRACK_MAX_GAP = 1.0
TRACK_MAX_JUMP = 0.50

# Floor on the bearing-rate differentiation interval, so a tiny dt between two
# perception frames cannot blow up the computed rate. TRACK_MAX_GAP above doubles
# as the upper cutoff (stale resume -> treat as fresh, rate 0).
BEARING_DT_MIN = 0.02


class ControllerNode(Node):
    """Same approach/heading law and output architecture as
    cp_proportional_controller2.py (velocity-servo wrench, gated by
    capture_mode), but aimed at CONTACT rather than a standoff circle: the
    radial equilibrium sits at contact_distance from the target instead of
    buffer_radius. Adds a DOCK freeze (toggle_dock) and an optional direct
    cmd_vel output for real-hardware bench testing without the wrench sum.
    Contact itself is NOT detected here -- see fake_plant.py / the future
    hardware end-of-course node, which publish contact_detected independently
    of this controller.
    """

    def __init__(self):
        super().__init__('target_orientation_controller')
        self.get_logger().info("Target orientation controller (contact) has been started.")

        self.declare_parameter("init_period", 1.0)
        self.init_duration = self.get_parameter("init_period").value

        self.declare_parameter("timer_frequency", 0.01)
        self.timer_frequency = self.get_parameter("timer_frequency").value

        self.declare_parameter('qos_depth', 10)
        self.declare_parameter('qos_reliability', 'RELIABLE')
        self.declare_parameter('qos_history', 'KEEP_LAST')

        qos_depth = self.get_parameter('qos_depth').value
        qos_reliability = self.get_parameter('qos_reliability').value
        qos_history = self.get_parameter('qos_history').value

        valid_reliabilities = ['RELIABLE', 'BEST_EFFORT']
        valid_histories = ['KEEP_LAST', 'KEEP_ALL']

        if qos_reliability not in valid_reliabilities:
            raise ValueError(f"Invalid qos_reliability: {qos_reliability}")
        if qos_history not in valid_histories:
            raise ValueError(f"Invalid qos_history: {qos_history}")

        self.qos_profile = QoSProfile(
            depth=qos_depth,
            reliability=ReliabilityPolicy.RELIABLE if qos_reliability == 'RELIABLE' else ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST if qos_history == 'KEEP_LAST' else HistoryPolicy.KEEP_ALL
        )

        # Gains & Limits
        self.init_start_time = self.get_clock().now()
        self.max_rotational_speed = 0.2
        self.max_translational_speed = [0.05, 0.15]  # x and y max speeds

        # Standoff point ahead of the robot centre, in cm to match Markers.
        # Used only to pick the closest target, where the units cancel inside
        # argmin.
        self.closest_target_index = None
        self.ref_offset_x = 10.0
        self.ref_offset_y = 0.0

        # Radial equilibrium, METRES, measured from the robot CENTRE. Unlike
        # cp_proportional_controller2.py's buffer_radius (a keep-out circle),
        # this is where we WANT contact: half the RoboMaster's 0.40 m length
        # (spacecraft_visualizer.py robot_length), since the marker sits on
        # the target's surface and the robot's front edge is the contact
        # point. Declared so it can be kept in sync with fake_plant.py's own
        # contact_distance from the launch file, without coupling the two
        # nodes directly.
        self.declare_parameter("contact_distance", 0.20)
        self.contact_distance = self.get_parameter("contact_distance").value

        # Approach / alignment / heading gains, unchanged from
        # cp_proportional_controller2.py -- already validated in metres/
        # radians against the real noise floor and chassis caps; only the
        # equilibrium point above changes, not the law shape.
        self.declare_parameter("approach_gain", 0.9)
        self.declare_parameter("align_gain", 0.05)
        self.declare_parameter("heading_gain", 1.0)
        self.declare_parameter("target_velocity_span", 5.25)
        self.declare_parameter("feedforward_gain", 1.0)

        # Derivative (rate) term on the bearing, damping the tangential "sidling"
        # command so it backs off before the bearing reaches zero (the pure-P
        # align term saturates the lateral cap at large bearing and overshoots
        # across center). Default 0.0 => byte-for-byte current behaviour. NOT
        # coupled to translation_response_scale: that scales the inner velocity
        # servo (velocity_force_gain), this lives in the outer velocity-command
        # loop. align_rate_filter_alpha is a single-pole EMA on the rate
        # (1.0 = passthrough, smaller = heavier smoothing) since differentiating
        # a ~10 Hz noisy bearing amplifies noise.
        self.declare_parameter("align_damping_gain", 0.3)
        self.declare_parameter("align_rate_filter_alpha", 0.5)
        self.align_damping_gain = float(
            self.get_parameter("align_damping_gain").value)
        self.align_rate_filter_alpha = float(
            self.get_parameter("align_rate_filter_alpha").value)

        self.approach_gain = self.get_parameter("approach_gain").value
        self.align_gain = self.get_parameter("align_gain").value
        self.heading_gain = self.get_parameter("heading_gain").value
        self.target_velocity_span = self.get_parameter(
            "target_velocity_span").value
        self.feedforward_gain = self.get_parameter("feedforward_gain").value

        # Capture-wrench output stage (see cp_proportional_controller2.py for
        # the full rationale on the velocity-servo design).
        self.declare_parameter("velocity_force_gain", 6.0)
        self.declare_parameter("yaw_rate_torque_gain", 1.0)
        self.declare_parameter("maximum_force", 0.5)
        self.declare_parameter("maximum_torque", 0.2)
        self.velocity_force_gain = self.get_parameter("velocity_force_gain").value
        self.yaw_rate_torque_gain = self.get_parameter("yaw_rate_torque_gain").value
        self.maximum_force = self.get_parameter("maximum_force").value
        self.maximum_torque = self.get_parameter("maximum_torque").value

        # Shared response-scale knobs from the global /** block of
        # virtual_spacecraft.yaml. The spacecraft node scales this layer's
        # force/torque by the matching factor, dropping each axis' damping ratio
        # by sqrt(scale). Capture is a cascaded velocity servo, so velocity_force_gain
        # is its damping-like gain (translation) and yaw_rate_torque_gain the
        # rotation one; boosting each by 1/sqrt(scale) restores its baseline zeta
        # as the craft is slowed. 1.0 = no-op. Output stays clamped by
        # maximum_force/maximum_torque, so the higher gain only reduces inner-loop
        # lag, it cannot exceed the envelope.
        self.declare_parameter("translation_response_scale", 1.0)
        self.declare_parameter("rotation_response_scale", 1.0)
        t_scale = float(self.get_parameter("translation_response_scale").value)
        r_scale = float(self.get_parameter("rotation_response_scale").value)
        if not (np.isfinite(t_scale) and 0.0 < t_scale <= 1.0):
            raise ValueError("translation_response_scale must be in (0, 1]")
        if not (np.isfinite(r_scale) and 0.0 < r_scale <= 1.0):
            raise ValueError("rotation_response_scale must be in (0, 1]")
        self.velocity_force_gain = self.velocity_force_gain / np.sqrt(t_scale)
        self.yaw_rate_torque_gain = self.yaw_rate_torque_gain / np.sqrt(r_scale)

        # direct_cmd_vel: this node becomes the SOLE writer of cmd_vel and acts
        # as a mode-selected mux so virtual_spacecraft and direct capture can
        # run simultaneously. The nav chain (nav2_pidff -> velocity_smoother) is
        # remapped to publish `cmd_vel_nav` instead of `cmd_vel`; this node
        # republishes that onto `cmd_vel` in every non-capturing mode, and
        # overrides it with its own approach-law Twist while capturing. One
        # writer on cmd_vel, so the two never fight. With direct_cmd_vel False
        # the node publishes capture_wrench and the smoother owns cmd_vel as
        # before.
        self.declare_parameter("direct_cmd_vel", False)
        self.direct_cmd_vel = bool(self.get_parameter("direct_cmd_vel").value)
        # Latest passthrough command from the nav chain (direct mode only).
        self.cmd_vel_nav = Twist()

        # Velocity commands
        self.trq_cmd = None
        self.frc_cmd = np.array([None, None])

        # Our heading, for the map->body rotation of the target-velocity
        # feedforward. StatePos.theta is in DEGREES.
        self.pose_x = None
        self.pose_y = None
        self.pose_theta = None

        # Spacecraft body-frame velocity, the servo sets v_desired against.
        self.craft_vx = None
        self.craft_vy = None
        self.craft_wz = None

        # Map-frame track of the closest target, for the velocity estimate.
        self.target_track = deque()

        # Bearing-rate damping state (see target_callback). Reset to None/0.0 on
        # any track discontinuity so the next frame never differentiates across a
        # gap, target switch, or clock anomaly.
        self._prev_bearing = None
        self._prev_bearing_time = None
        self._prev_closest_index = None
        self._bearing_rate_filt = 0.0

        # Diagnostic state for the [dbg] log: last body-frame target (for the
        # per-frame jump metric), the worst jump seen since the last print, and
        # the time of the last print (self-throttle). Purely observational.
        self._dbg_prev_target = None
        self._dbg_max_jump = 0.0
        self._dbg_last_log = 0.0

        self.declare_parameter("target_hold_timeout", 0.4)   # seconds
        self.target_hold_timeout = float(self.get_parameter("target_hold_timeout").value)
        self.last_target_time = None

        # Auction-assigned attachment point. When fresh+visible, the closest-target
        # pick is restricted to this AP id so the robot docks its OWN point rather
        # than greedily grabbing the nearest. Stale/absent -> today's behaviour.
        self.declare_parameter("assignment_timeout", 0.5)
        self.assignment_timeout = float(self.get_parameter("assignment_timeout").value)
        self.assigned_ap_id = -1
        self.assigned_ap_time = None

        # Subscribers
        self.create_subscription(Markers, 'targets', self.target_callback, self.qos_profile)
        self.create_subscription(Int32, 'assigned_ap', self.assigned_ap_callback, self.qos_profile)
        qos_latest = QoSProfile(depth=1,
                                reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(State, 'pose', self.pose_callback, qos_latest)
        self.create_subscription(Odometry, 'virtual_spacecraft/odom',
                                 self.spacecraft_callback, qos_latest)

        # Exactly one output sink, chosen by direct_cmd_vel. In direct mode this
        # node owns cmd_vel and muxes the nav passthrough (cmd_vel_nav) against
        # its own capture command; otherwise it feeds the summed capture_wrench
        # and the smoother owns cmd_vel directly.
        if self.direct_cmd_vel:
            self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel',
                                                     self.qos_profile)
            self.create_subscription(Twist, 'cmd_vel_nav',
                                     self.cmd_vel_nav_callback, self.qos_profile)
        else:
            self.wrench_pub = self.create_publisher(Wrench, 'capture_wrench',
                                                    self.qos_profile)
        self.cmd_pub_timer = self.create_timer(self.timer_frequency, self.publish_cmd)

        # Capture mode and dock freeze. DOCK wins over CAPTURE: once docked
        # the approach law is skipped and a zero Wrench/Twist is published
        # regardless of capture_mode, so a still-enabled capture_mode (e.g.
        # from a stale/duplicate service call) can never re-energize the
        # output while docked.
        self.capture_mode = False
        self.dock_mode = False
        self.srv = self.create_service(SetBool, 'toggle_controller', self.handle_controller)
        self.dock_srv = self.create_service(SetBool, 'toggle_dock', self.handle_dock)

        # Magnet Client
        self.magnet_client = self.create_client(SetBool, 'toggle_magnet')

    def pose_callback(self, msg: State):
        self.pose_x = msg.x
        self.pose_y = msg.y
        self.pose_theta = msg.theta

    def assigned_ap_callback(self, msg: Int32):
        self.assigned_ap_id = int(msg.data)
        self.assigned_ap_time = self.get_clock().now()

    def spacecraft_callback(self, msg: Odometry):
        self.craft_vx = msg.twist.twist.linear.x
        self.craft_vy = msg.twist.twist.linear.y
        self.craft_wz = msg.twist.twist.angular.z

    def to_body(self, mx, my):
        """Rotate a map-frame vector into the body frame.

        StatePos.theta is DEGREES; feeding it to cos/sin unconverted is a
        known trap in this stack, which is why this is in one place.
        """
        th = np.radians(self.pose_theta)
        cos_t, sin_t = np.cos(th), np.sin(th)
        return (cos_t * mx + sin_t * my, -sin_t * mx + cos_t * my)

    def track_target(self, target_body):
        """Record where the closest target is in the MAP frame.

        Returns True whenever the track is cleared (pose unavailable, or a
        map-frame gap/jump breaking target identity) so the caller can reset the
        bearing-rate derivative across the same discontinuity.
        """
        if self.pose_x is None or self.pose_theta is None:
            self.target_track.clear()
            return True

        th = np.radians(self.pose_theta)
        cos_t, sin_t = np.cos(th), np.sin(th)
        now = self.get_clock().now().nanoseconds * 1e-9
        point = (now,
                 self.pose_x + cos_t * target_body[0] - sin_t * target_body[1],
                 self.pose_y + sin_t * target_body[0] + cos_t * target_body[1])

        track_reset = False
        if self.target_track:
            previous = self.target_track[-1]
            gap = point[0] - previous[0]
            jump = float(np.hypot(point[1] - previous[1],
                                  point[2] - previous[2]))
            if gap > TRACK_MAX_GAP or jump > TRACK_MAX_JUMP:
                self.target_track.clear()
                track_reset = True

        self.target_track.append(point)

        while (len(self.target_track) > 2
               and point[0] - self.target_track[1][0]
               >= self.target_velocity_span):
            self.target_track.popleft()

        return track_reset

    def target_velocity_map(self):
        """Least-squares target velocity in the map frame, m/s, or None."""
        if len(self.target_track) < 2:
            return None
        covered = self.target_track[-1][0] - self.target_track[0][0]
        if covered < self.target_velocity_span:
            return None

        samples = np.array(self.target_track, dtype=float)
        t = samples[:, 0] - samples[:, 0].mean()
        spread = float(t @ t)
        if spread <= 0.0:
            return None
        return np.array([float(t @ (samples[:, 1] - samples[:, 1].mean())),
                         float(t @ (samples[:, 2] - samples[:, 2].mean()))
                         ]) / spread

    def target_callback(self, msg: Markers):
        now = self.get_clock().now()
        if len(msg.markers) == 0:
            if (self.last_target_time is None or
                    (now - self.last_target_time).nanoseconds * 1e-9
                    > self.target_hold_timeout):
                self.frc_cmd = np.array([None, None])
                self.trq_cmd = None
                self.target_track.clear()
                self._prev_bearing = None
                self._prev_closest_index = None
                self._bearing_rate_filt = 0.0
            return
        self.last_target_time = now

        ######################################################
        # Finding the closest target
        ######################################################
        # Restrict candidates to the auction-assigned AP when the assignment is
        # fresh, the id is set, and it is actually visible in this frame;
        # otherwise fall back to closest over all markers (today's behaviour).
        idxs = list(range(len(msg.markers)))
        fresh = (self.assigned_ap_time is not None and
                 (now - self.assigned_ap_time).nanoseconds * 1e-9 < self.assignment_timeout)
        if fresh and self.assigned_ap_id >= 0 and len(msg.ids) == len(msg.markers):
            match = [i for i in idxs if msg.ids[i] == self.assigned_ap_id]
            if match:
                idxs = match

        target_distances = np.array([
            np.linalg.norm(np.array([msg.markers[i].x, msg.markers[i].y]) -
                           np.array([self.ref_offset_x, self.ref_offset_y]))
            for i in idxs
        ])
        self.closest_target_index = idxs[int(np.argmin(target_distances))]

        ######################################################
        # Velocity command - drive to contact, not a standoff
        ######################################################
        target = msg.markers[self.closest_target_index]
        target = np.array([target.x, target.y]) / 100.0
        track_reset = self.track_target(target)
        distance = float(np.linalg.norm(target))

        v_target = None  # set in the else branch; kept defined for the [dbg] log
        if distance < 1e-3:
            self.frc_cmd[0] = 0.0
            self.frc_cmd[1] = 0.0
            # Target at the sensor origin -> bearing undefined; drop the rate
            # history so resume doesn't differentiate across the gap.
            self._prev_bearing = None
            self._bearing_rate_filt = 0.0
        else:
            bearing = float(np.arctan2(target[1], target[0]))
            u_r = target / distance
            u_t = np.array([-u_r[1], u_r[0]])

            # Radial: zero at contact_distance, not buffer_radius -- the only
            # change from cp_proportional_controller2.py's approach law.
            vel = self.approach_gain * (distance - self.contact_distance) * u_r

            # Bearing-rate damping (D on the pure-P sidling term). Rate comes from
            # successive perception bearings, so it is routing-independent (wrench
            # and direct_cmd_vel share this vel). Reset the derivative across any
            # discontinuity so it never differentiates across a gap.
            now_s = now.nanoseconds * 1e-9
            switched = (self._prev_closest_index is not None
                        and self.closest_target_index != self._prev_closest_index)
            if self._prev_bearing is None or track_reset or switched:
                bearing_rate = 0.0
                self._bearing_rate_filt = 0.0
            else:
                dt = now_s - self._prev_bearing_time
                if dt <= 0.0 or dt > TRACK_MAX_GAP:
                    bearing_rate = 0.0
                    self._bearing_rate_filt = 0.0
                else:
                    dt = max(dt, BEARING_DT_MIN)
                    d_bearing = ((bearing - self._prev_bearing + np.pi)
                                 % (2.0 * np.pi) - np.pi)
                    raw_rate = d_bearing / dt
                    self._bearing_rate_filt = (
                        self.align_rate_filter_alpha * raw_rate
                        + (1.0 - self.align_rate_filter_alpha)
                        * self._bearing_rate_filt)
                    bearing_rate = self._bearing_rate_filt

            self._prev_bearing = bearing
            self._prev_bearing_time = now_s
            self._prev_closest_index = self.closest_target_index

            vel = vel + (self.align_gain * bearing
                         - self.align_damping_gain * bearing_rate) * u_t

            v_target = self.target_velocity_map()
            if v_target is not None and self.pose_theta is not None:
                vx, vy = self.to_body(v_target[0], v_target[1])
                vel = vel + self.feedforward_gain * np.array([vx, vy])

            self.frc_cmd[0] = float(vel[0])
            self.frc_cmd[1] = float(vel[1])

        ######################################################
        # Torque command calculation
        ######################################################
        orien = CoordXY()
        orien.x = msg.markers_normals[self.closest_target_index].x
        orien.y = msg.markers_normals[self.closest_target_index].y

        marker_dir = np.array([orien.x, orien.y])
        angle = -float(np.arctan2(marker_dir[0], marker_dir[1]))
        heading_error = (angle + np.pi) % (2.0 * np.pi) - np.pi
        self.trq_cmd = self.heading_gain * heading_error


    ######################################################################################################

    def cmd_vel_nav_callback(self, msg: Twist):
        """Latest nav-chain command (velocity_smoother -> cmd_vel_nav), muxed
        onto cmd_vel in direct mode whenever this node is not capturing."""
        self.cmd_vel_nav = msg

    def publish_cmd(self):
        now = self.get_clock().now()
        elapsed = (now - self.init_start_time).nanoseconds / 1e9
        if elapsed <= 2 * self.init_duration:
            return

        wrench_msg = Wrench()
        twist_msg = Twist()

        active = (not self.dock_mode and self.capture_mode and self.trq_cmd is not None
                  and None not in self.frc_cmd)
        if active:
            vx_des = bound(self.frc_cmd[0], self.max_translational_speed[0])
            vy_des = bound(self.frc_cmd[1], self.max_translational_speed[1])
            wz_des = bound(self.trq_cmd, self.max_rotational_speed)

            if self.direct_cmd_vel:
                twist_msg.linear.x = float(vx_des)
                twist_msg.linear.y = float(vy_des)
                twist_msg.angular.z = float(wz_des)
            else:
                vx = self.craft_vx if self.craft_vx is not None else 0.0
                vy = self.craft_vy if self.craft_vy is not None else 0.0
                wz = self.craft_wz if self.craft_wz is not None else 0.0

                fx = self.velocity_force_gain * (vx_des - vx)
                fy = self.velocity_force_gain * (vy_des - vy)
                fmag = float(np.hypot(fx, fy))
                if fmag > self.maximum_force:
                    scale = self.maximum_force / fmag
                    fx *= scale
                    fy *= scale

                wrench_msg.force.x = float(fx)
                wrench_msg.force.y = float(fy)
                wrench_msg.torque.z = bound(self.yaw_rate_torque_gain * (wz_des - wz),
                                            self.maximum_torque)

        # Publish the active sink. capture_wrench is a summed source (zero is a
        # harmless no-op). In direct mode this node owns cmd_vel and muxes:
        # while capturing it publishes its own approach command, otherwise it
        # republishes the nav chain's latest passthrough (cmd_vel_nav). Either
        # way exactly one Twist per tick, so it never fights the smoother.
        if self.direct_cmd_vel:
            self.cmd_vel_pub.publish(twist_msg if active else self.cmd_vel_nav)
        else:
            self.wrench_pub.publish(wrench_msg)

    ######################################################################################################

    def handle_controller(self, request, response):
        if request.data:
            self.capture_mode = True
            self.get_logger().info('Capture mode -> ON')
            response.success = True
            response.message = 'Controller set to Capture mode'
        else:
            self.capture_mode = False
            self.get_logger().info('Encapsulate mode -> ON')
            response.success = True
            response.message = 'Controller set to Encapsulate mode'
        return response

    def handle_dock(self, request, response):
        self.dock_mode = bool(request.data)
        self.get_logger().info(f'Dock mode -> {"ON" if self.dock_mode else "OFF"}')
        response.success = True
        response.message = f'Dock mode {"enabled" if self.dock_mode else "disabled"}'
        return response

    def call_service(self, client, state: bool, name: str):
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'{name} service not available.')
            return

        request = SetBool.Request()
        request.data = state

        future = client.call_async(request)
        future.add_done_callback(lambda f: self.get_logger().info(
            f'{name} response: {f.result().success if f.result() else "No response"}'))


def bound(input, bound):
    output = max(min(input, bound), -bound)
    return output


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
