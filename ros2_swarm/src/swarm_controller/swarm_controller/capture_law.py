"""Capture law (formerly cp_controller): drive to CONTACT with the auction-assigned attachment point.

Approach/align/heading law with a corridor (face-normal) bearing, bearing-rate damping, a least-squares target
velocity feedforward and a near-target speed taper, output as a velocity-servo capture_wrench gated by
toggle_controller (or straight to cmd_vel with direct_cmd_vel, muxing the nav chain's cmd_vel_nav), a DOCK freeze
(toggle_dock), and the separately capped ENCAPSULATION preparation_wrench along the assigned AP's tangent.
Contact itself is detected elsewhere (geometric_contact.py / contact_switch_node). Plain class driven by guidance.
"""
from collections import deque

import numpy as np
from geometry_msgs.msg import Twist, Wrench
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Int32, String
from swarm_interfaces.msg import CoordXY, Markers
from swarm_interfaces.msg import StatePos as State

# Outlier guards on the target track. Not tuning knobs: the gap is longer than
# the 0.61 s frozen-fix period seen on RM6, so only a real dropout trips it.
TRACK_MAX_GAP = 1.0
TRACK_MAX_JUMP = 0.50

# Floor on the bearing-rate differentiation interval, so a tiny dt between two
# perception frames cannot blow up the computed rate. TRACK_MAX_GAP above doubles
# as the upper cutoff (stale resume -> treat as fresh, rate 0).
BEARING_DT_MIN = 0.02

# A published face normal shorter than this is treated as absent, so the sidling
# law falls back to the raw line-of-sight bearing.
NORMAL_MIN = 1e-6


class CaptureLaw:
    def __init__(self, node, p, wrench_pub, preparation_pub, cmd_vel_pub=None, init_duration=1.0,
                 translation_response_scale=1.0, rotation_response_scale=1.0):
        """p(name, default) reads this law's parameters (capture.*). cmd_vel_pub is set iff direct_cmd_vel."""
        self.get_clock, self.get_logger = node.get_clock, node.get_logger
        self.init_duration, self.init_start_time = init_duration, self.get_clock().now()
        self.max_rotational_speed = 0.2
        self.max_translational_speed = [0.05, 0.15]  # x and y max speeds
        # Standoff point ahead of the robot centre, cm (units cancel inside the closest-target argmin).
        self.closest_target_index = None
        self.ref_offset_x, self.ref_offset_y = 10.0, 0.0
        # Radial equilibrium, metres from the robot CENTRE: where we WANT contact (half the 0.40 m RoboMaster).
        self.contact_distance = p("contact_distance", 0.20)
        self.approach_gain = p("approach_gain", 0.9)
        self.align_gain = p("align_gain", 0.2)
        self.heading_gain = p("heading_gain", 1.0)
        self.target_velocity_span = p("target_velocity_span", 5.25)  # SECONDS of track for the feedforward slope
        self.feedforward_gain = p("feedforward_gain", 1.0)
        # Near-target envelope: inside capture_slow_radius the command tapers to capture_min_speed_frac at contact.
        self.capture_slow_radius = float(p("capture_slow_radius", 0.5))
        self.capture_min_speed_frac = float(p("capture_min_speed_frac", 0.3))
        # D on the sidling term (bearing rate), EMA-filtered (1.0 = passthrough).
        self.align_damping_gain = float(p("align_damping_gain", 0.6))
        self.align_rate_filter_alpha = float(p("align_rate_filter_alpha", 0.5))
        # Velocity-servo output stage, capped by maximum_force / maximum_torque.
        self.velocity_force_gain = p("velocity_force_gain", 6.0)
        self.yaw_rate_torque_gain = p("yaw_rate_torque_gain", 1.0)
        self.maximum_force = p("maximum_force", 0.5)
        self.maximum_torque = p("maximum_torque", 0.2)
        self.preparation_max_force = float(p("preparation_max_force", 0.002))
        if not np.isfinite(self.preparation_max_force) or self.preparation_max_force < 0.0:
            raise ValueError("preparation_max_force must be finite and nonnegative")
        # The spacecraft scales this layer by the response scales; boosting each servo gain by 1/sqrt(scale)
        # restores its damping ratio (output stays clamped, so it cannot exceed the envelope).
        t_scale, r_scale = float(translation_response_scale), float(rotation_response_scale)
        if not (np.isfinite(t_scale) and 0.0 < t_scale <= 1.0):
            raise ValueError("translation_response_scale must be in (0, 1]")
        if not (np.isfinite(r_scale) and 0.0 < r_scale <= 1.0):
            raise ValueError("rotation_response_scale must be in (0, 1]")
        self.velocity_force_gain = self.velocity_force_gain / np.sqrt(t_scale)
        self.yaw_rate_torque_gain = self.yaw_rate_torque_gain / np.sqrt(r_scale)
        # direct_cmd_vel: sole writer of cmd_vel, muxing the nav chain (cmd_vel_nav) outside capture.
        self.direct_cmd_vel = cmd_vel_pub is not None
        self.cmd_vel_pub, self.wrench_pub, self.preparation_pub = cmd_vel_pub, wrench_pub, preparation_pub
        self.cmd_vel_nav = Twist()

        self.trq_cmd = None
        self.frc_cmd = np.array([None, None])
        self.pose_x = self.pose_y = self.pose_theta = None  # StatePos.theta is DEGREES
        self.craft_vx = self.craft_vy = self.craft_wz = None  # virtual spacecraft body-frame velocity
        self.target_track = deque()  # map-frame track of the closest target, for the velocity estimate
        # Bearing-rate state, reset on any track discontinuity.
        self._prev_bearing = None
        self._prev_bearing_time = None
        self._prev_closest_index = None
        self._prev_target_identity = None
        self._bearing_rate_filt = 0.0

        self.target_hold_timeout = float(p("target_hold_timeout", 0.4))   # seconds
        self.last_target_time = None
        # Auction-assigned AP: when fresh and visible, the closest-target pick is restricted to it.
        self.assignment_timeout = float(p("assignment_timeout", 0.5))
        self.assigned_ap_id = -1
        self.assigned_ap_time = None
        self.assignment_locked = False
        self.assignment_locked_time = None
        self.current_mode = ''
        self.preparation_tangent = None
        self.preparation_speed = 0.0
        self.preparation_target_id = -1
        self.preparation_target_time = None
        # DOCK wins over CAPTURE: once docked the output is zero whatever capture_mode says.
        self.capture_mode = False
        self.dock_mode = False

    def pose_callback(self, msg: State):
        self.pose_x = msg.x
        self.pose_y = msg.y
        self.pose_theta = msg.theta

    def assigned_ap_callback(self, msg: Int32):
        if int(msg.data) != self.assigned_ap_id:
            self.preparation_tangent = None
            self.assignment_locked = False
            self.assignment_locked_time = None
            self._prev_bearing = self._prev_target_identity = None
            self._bearing_rate_filt = 0.0
            self.target_track.clear()
        self.assigned_ap_id = int(msg.data)
        self.assigned_ap_time = self.get_clock().now()

    def assignment_locked_callback(self, msg: Bool):
        self.assignment_locked = bool(msg.data)
        self.assignment_locked_time = self.get_clock().now()

    def mode_callback(self, msg: String):
        self.current_mode = msg.data.strip().lower()

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
        self.preparation_tangent = None
        if len(msg.markers) == 0:
            if (self.last_target_time is None or
                    (now - self.last_target_time).nanoseconds * 1e-9
                    > self.target_hold_timeout):
                self.frc_cmd = np.array([None, None])
                self.trq_cmd = None
                self.target_track.clear()
                self._prev_bearing = None
                self._prev_closest_index = None
                self._prev_target_identity = None
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
        identity = ('id', int(msg.ids[self.closest_target_index])) \
            if len(msg.ids) == len(msg.markers) else ('index', self.closest_target_index)
        switched = self._prev_target_identity is not None and identity != self._prev_target_identity
        if switched:
            self.target_track.clear()

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
            u_r = target / distance
            u_t = np.array([-u_r[1], u_r[0]])

            # Corridor (glideslope) bearing: the signed angle between the LOS and
            # the target's face-normal axis, not the raw body-frame LOS. Nulling
            # it slides the craft onto the normal axis so it docks square to the
            # face. The published normal is sign-ambiguous -- the ArUco antipodal
            # flip (virtual_scene.py forces its forward component >= 0), so it may
            # point toward or away from the craft. Disambiguate by the approach:
            # the robot sits inside the face's incidence cone, so the corridor
            # axis is whichever of +/-m aligns with the LOS. Fall back to the raw
            # LOS bearing when no usable normal is present (old behaviour).
            los_bearing = float(np.arctan2(target[1], target[0]))
            n = msg.markers_normals[self.closest_target_index]
            m = np.array([n.y, -n.x])  # published (right, fwd) -> body (fwd, left)
            nmag = float(np.linalg.norm(m))
            if nmag > NORMAL_MIN:
                d = m if float(m @ u_r) >= 0.0 else -m
                bearing = float(
                    (los_bearing - np.arctan2(d[1], d[0])
                     + np.pi) % (2.0 * np.pi) - np.pi)
            else:
                bearing = los_bearing

            # Radial: zero at contact_distance, not buffer_radius -- the only
            # change from the old standoff (buffer_radius) approach law.
            vel = self.approach_gain * (distance - self.contact_distance) * u_r

            # Bearing-rate damping (D on the pure-P sidling term). Rate comes from
            # successive perception bearings, so it is routing-independent (wrench
            # and direct_cmd_vel share this vel). Reset the derivative across any
            # discontinuity so it never differentiates across a gap.
            now_s = now.nanoseconds * 1e-9
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

            # Positive sidling reduces bearing: +D*beta_dot opposes that motion.
            alignment_speed = self.align_gain * bearing + self.align_damping_gain * bearing_rate
            vel = vel + alignment_speed * u_t
            if fresh and identity == ('id', self.assigned_ap_id) and np.isfinite(distance):
                self.preparation_tangent = u_t
                self.preparation_speed = alignment_speed
                self.preparation_target_id = self.assigned_ap_id
                self.preparation_target_time = now

            v_target = self.target_velocity_map()
            if v_target is not None and self.pose_theta is not None:
                vx, vy = self.to_body(v_target[0], v_target[1])
                vel = vel + self.feedforward_gain * np.array([vx, vy])

            if distance < self.capture_slow_radius:
                span = max(self.capture_slow_radius - self.contact_distance, 1e-3)
                reach = max(distance - self.contact_distance, 0.0) / span
                frac = (self.capture_min_speed_frac
                        + (1.0 - self.capture_min_speed_frac) * min(reach, 1.0))
                vel = vel * frac

            self.frc_cmd[0] = float(vel[0])
            self.frc_cmd[1] = float(vel[1])

        self._prev_target_identity = identity

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

    def preparation_wrench(self, now):
        msg = Wrench()
        if (self.current_mode != 'encapsulation' or self.capture_mode or self.dock_mode
                or not self.assignment_locked or self.assigned_ap_id < 0
                or self.preparation_target_id != self.assigned_ap_id
                or self.preparation_tangent is None or self.preparation_max_force == 0.0):
            return msg
        for stamp, timeout in ((self.assigned_ap_time, self.assignment_timeout),
                               (self.assignment_locked_time, self.assignment_timeout),
                               (self.preparation_target_time, self.target_hold_timeout)):
            if stamp is None or not 0.0 <= (now - stamp).nanoseconds * 1e-9 < timeout:
                return msg
        velocity = np.array([self.craft_vx if self.craft_vx is not None else 0.0,
                             self.craft_vy if self.craft_vy is not None else 0.0])
        # Project the servo error too: radial velocity must not produce braking here.
        force = self.velocity_force_gain * (
            self.preparation_speed - float(velocity @ self.preparation_tangent))
        if not np.isfinite(force) or not np.all(np.isfinite(self.preparation_tangent)):
            return msg
        fx, fy = bound(force, self.preparation_max_force) * self.preparation_tangent
        msg.force.x, msg.force.y = float(fx), float(fy)
        return msg

    def publish_cmd(self):
        now = self.get_clock().now()
        elapsed = (now - self.init_start_time).nanoseconds / 1e9
        self.preparation_pub.publish(self.preparation_wrench(now)
                                     if elapsed > 2 * self.init_duration else Wrench())
        if elapsed <= 2 * self.init_duration:
            return

        wrench_msg = Wrench()
        twist_msg = Twist()

        # A command older than target_hold_timeout expires (targets stopped arriving altogether).
        fresh = (self.last_target_time is not None
                 and (now - self.last_target_time).nanoseconds * 1e-9 <= self.target_hold_timeout)
        active = (fresh and not self.dock_mode and self.capture_mode and self.trq_cmd is not None
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


def bound(input, bound):
    output = max(min(input, bound), -bound)
    return output
