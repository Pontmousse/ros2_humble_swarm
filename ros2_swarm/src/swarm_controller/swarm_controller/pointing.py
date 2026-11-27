"""Pointing: heading-hold torque toward the K-nearest landmark centroid (pointing_wrench), with damp-only
fallback, slew limit and the CAPTURE/DOCK hand-off; also shares this robot's landmark-memory centre on
/target_estimate. Plain class driven by the guidance node."""
import math

import numpy as np
from geometry_msgs.msg import Wrench
from nav_msgs.msg import Odometry
from rclpy.time import Time
from std_msgs.msg import String
from swarm_interfaces.msg import Markers, TargetEstimate
from swarm_interfaces.msg import StatePos as State

from .flocking import make_array, nearest_k
from .shared_target import SharedTargets, now_s


class Pointing:
    def __init__(self, node, p, wrench_pub, share_pub=None, init_duration=1.0, rotation_response_scale=1.0,
                 landmark_nearest_k=5, landmarks_freshness_timeout=0.3, share_timeout=2.0):
        """p(name, default) reads this law's parameters (pointing.*)."""
        self.get_clock, self.get_logger = node.get_clock, node.get_logger
        self.init_duration, self.init_start_time = init_duration, self.get_clock().now()
        # Attitude PD -> torque wrench; publisher-side cap so no single layer saturates the spacecraft clamp.
        self.heading_torque_gain = p("heading_torque_gain", 0.6)   # Nm/rad, P on heading error
        self.maximum_torque = p("maximum_torque", 0.2)
        # The spacecraft scales this torque by rotation_response_scale; boosting D by 1/sqrt(scale) holds zeta.
        if not (math.isfinite(rotation_response_scale) and 0.0 < rotation_response_scale <= 1.0):
            raise ValueError("rotation_response_scale must be in (0, 1]")
        self.yaw_damping_gain = p("yaw_damping_gain", 0.15) / math.sqrt(rotation_response_scale)  # Nm/(rad/s)
        self.landmark_nearest_k = int(landmark_nearest_k)  # centroid of the K nearest landmarks; 0 = all
        # Slew limit (Nm/s): a perception freeze used to end in a bang-bang kick that rang the yaw axis up.
        self.torque_slew_rate = float(p("torque_slew_rate", 0.4))
        # Past this landmark age stop CHASING (zero P) but keep -Kd*craft_wz damping live.
        self.landmarks_freshness_timeout = float(landmarks_freshness_timeout)
        self.last_landmarks_time = None

        # Control state
        self.pose = np.array([None, None, None])  # x, y, theta
        self.lc = np.array([None, None])          # landmark centroid
        self.target_lcd = None                    # direction (deg)
        self.craft_wz = None                      # spacecraft yaw rate (rad/s)
        self.current_mode = ''

        # Slew limiter state: the last torque actually published and when.
        self._last_torque = 0.0
        self._last_pub_time = None

        # Shared target estimate: share_pub is None when share_target is off.
        self.share_target, self.share_pub = share_pub is not None, share_pub
        self.shared = SharedTargets(node.get_namespace().strip('/'), float(share_timeout))
        self.centre, self.centre_n = None, 0  # own map-frame landmark-memory centre
        self.wrench_pub = wrench_pub  # zero in CAPTURE/DOCK: sent once

    def pose_callback(self, msg: State):
        self.pose = np.array([msg.x, msg.y, msg.theta])


    def odom_callback(self, msg: Odometry):
        # Yaw rate is frame-invariant, so the body-frame twist is used as-is.
        self.craft_wz = msg.twist.twist.angular.z

    def mode_callback(self, msg: String):
        self.current_mode = msg.data.strip().lower()

    def landmarks_callback(self, msg: Markers):
        # Check if pose is initialized
        if None in self.pose:
            self.get_logger().warn("Pose not initialized yet.")
            return

        # Perception freshness: a stalled detector republishes the same markers
        # at 100 Hz. Recomputing lc/target_lcd from them keeps the centroid
        # pinned under the live pose and the controller chases a ghost forever.
        # Bail BEFORE touching lc/target_lcd -> publish_target_orientation then
        # falls back to damp-only (holds heading, bleeds yaw rate).
        if self.landmarks_stale(msg.header.stamp):
            self.get_logger().warn(
                "Landmark perception stale; holding heading.",
                throttle_duration_sec=5.0)
            return
        self.last_landmarks_time = self.get_clock().now()

        # Fresh frame but no markers: the camera is alive and genuinely sees no
        # target. Drop the target so the controller does NOT keep chasing the
        # last centroid with the live (possibly spinning) heading -- a fresh
        # empty frame re-arming a stale target_lcd was a direct contributor to
        # the yaw ring-up. target_lcd=None -> damp-only below.
        if len(msg.markers) == 0:
            self.target_lcd, self.centre = None, None
            return

        # Rotate all landmarks (cm -> m) into map-frame offsets from the robot
        theta_rad = math.radians(self.pose[2])
        rot_matrix = np.array([
            [math.cos(theta_rad), -math.sin(theta_rad)],
            [math.sin(theta_rad),  math.cos(theta_rad)]
        ])
        rel = np.matmul(rot_matrix, (make_array(msg.markers) / 100.0).T).T
        self.centre, self.centre_n = self.pose[:2] + rel.mean(axis=0), len(rel)  # whole memory: shared estimate
        # Pointing uses the K nearest (same set flocking uses); a rotation keeps ranges
        landmarks_global = self.pose[:2] + nearest_k(rel, self.landmark_nearest_k)

        # Calculate landmark centroid in global frame
        self.lc = np.mean(landmarks_global, axis=0)

        # Compute direction to centroid
        dir_vec = self.lc - self.pose[:2]
        if np.linalg.norm(dir_vec) == 0:
            self.get_logger().warn("Zero direction vector to centroid.")
            return

        dir_vec /= np.linalg.norm(dir_vec)
        self.target_lcd = math.degrees(math.atan2(dir_vec[1], dir_vec[0]))  # in degrees

        # self.get_logger().warn(f"switched orientation target to: {self.target_lcd}\n")

    def landmarks_stale(self, stamp):
        """True if a landmark stamp is older than the freshness timeout (or
        zero/absent, i.e. no detection yet)."""
        if stamp.sec == 0 and stamp.nanosec == 0:
            return True
        age = (self.get_clock().now() - Time.from_msg(stamp)).nanoseconds / 1e9
        return age > self.landmarks_freshness_timeout

    def landmarks_expired(self):
        """Loop-side guard for the detector node dying (topic stops entirely, so
        landmarks_callback never fires to stop the chase)."""
        if self.last_landmarks_time is None:
            return True
        age = (self.get_clock().now() - self.last_landmarks_time).nanoseconds / 1e9
        return age > self.landmarks_freshness_timeout

    def _damping_torque(self):
        """Damping-only torque: -Kd * craft_wz, clamped. Used whenever there is
        no valid heading target (perception lost, no markers, or no pose). It
        holds heading AND actively bleeds any residual/coasting yaw rate, which a
        hard zero cannot do on the undamped plant. craft_wz is from odom, always
        fresh, so this is safe to run while perception is stale."""
        wz = self.craft_wz if self.craft_wz is not None else 0.0
        return bound(-self.yaw_damping_gain * wz, self.maximum_torque)

    def _slew(self, desired, now):
        """Rate-limit the published torque toward `desired` (soft-start). dt is
        clamped so a stalled timer cannot authorise an unbounded step."""
        if self._last_pub_time is None:
            self._last_pub_time = now
            self._last_torque = desired
            return desired
        dt = (now - self._last_pub_time).nanoseconds / 1e9
        self._last_pub_time = now
        dt = min(max(dt, 0.0), 0.05)
        max_step = self.torque_slew_rate * dt
        delta = desired - self._last_torque
        delta = max(-max_step, min(max_step, delta))
        self._last_torque = self._last_torque + delta
        return self._last_torque

    def publish_target_orientation(self):
        # Wait for initialization before triggering the controller
        now = self.get_clock().now()
        if self.current_mode in ('capture', 'dock'):
            # CP owns capture yaw; bypass slew so no old centroid torque lingers.
            self._last_torque, self._last_pub_time = 0.0, now
            self.wrench_pub.publish(Wrench())
            return
        elapsed = (now - self.init_start_time).nanoseconds / 1e9
        if elapsed <= 2 * self.init_duration:
            return

        # Decide the DESIRED torque, then slew-limit it once at the single publish
        # point below. Two regimes:
        #   - valid target (fresh perception, non-empty, pose known) -> full PD
        #   - otherwise (perception lost/stale, no markers, no pose) -> damp-only
        # Damp-only is NOT a hard zero: it keeps -Kd*wz alive so a coasting yaw
        # rate still decays (the fix for the ring-up).
        target = None if self.landmarks_expired() or None in self.pose else self.target_lcd
        if target is None and self.share_target and self.current_mode == 'search' and None not in self.pose \
                and (c := self.shared.centre(now_s(self))) is not None:
            target = math.degrees(math.atan2(c[1] - self.pose[1], c[0] - self.pose[0]))  # blind: face the neighbours' estimate
        if target is None:
            desired = self._damping_torque()
        else:
            # Active pointing: PD torque toward the target heading.
            err = math.radians(wrap_deg(target - self.pose[2]))
            wz = self.craft_wz if self.craft_wz is not None else 0.0
            desired = bound(
                self.heading_torque_gain * err - self.yaw_damping_gain * wz,
                self.maximum_torque)

        wmsg = Wrench()
        wmsg.torque.z = self._slew(desired, now)
        self.wrench_pub.publish(wmsg)

    def publish_estimate(self):
        if self.centre is None or self.landmarks_expired(): return  # only share what we currently see
        m = TargetEstimate(robot=self.shared.me, x=float(self.centre[0]), y=float(self.centre[1]), count=min(self.centre_n, 65535))
        m.header.stamp, m.header.frame_id = self.get_clock().now().to_msg(), 'swarm_map'
        self.share_pub.publish(m)


def bound(input, limit):
    return max(min(input, limit), -limit)


def wrap_deg(angle):
    return (angle + 180.0) % 360.0 - 180.0
