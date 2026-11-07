"""Kinematic localization via two decoupled *linear* Kalman filters.

Drop-in replacement for ``pose_publisher`` that fuses the same raw inputs
(Marvelmind ``mm_pos_unf``, RoboMaster ``odom``, chassis/beacon IMU) plus the
command ``cmd_vel`` into a clean heading, a clean map-frame velocity and a
covariance-smoothed (jump-free) position. It publishes the identical output
contract so every downstream consumer (``position_guidance3`` encapsulation,
``cp_proportional_controller`` chase, avoidance, RViz) is unchanged:

    pose               swarm_interfaces/StatePos   x,y metres; theta DEGREES
    localization/odom  nav_msgs/Odometry           pose in swarm_map,
                                                    twist BODY frame, orient=theta
    TF swarm_map -> odom                            (RViz parity)

Why a kinematic (no mass/inertia) model: the RoboMaster is velocity-controlled
through ``cmd_vel``, so a force->acceleration map is neither available nor
valid. The prediction is therefore a *command-tracking constant-velocity*
model: velocity relaxes toward the commanded velocity with time-constant
``command_tau`` and is corrected by odometry; position integrates velocity and
is anchored by Marvelmind. Because the base is holonomic the map velocity is a
plain 2-vector, so treating theta as "known" (supplied by the heading filter)
keeps the planar filter linear and lets us avoid an EKF entirely:

    heading filter   state [theta, omega]       anchored by absolute odom yaw
    planar filter    state [x, y, vx, vy] (map)  anchored by Marvelmind

Robustness (every measurement can drop):
  * Event-driven updates + continuous prediction: a silent sensor simply stops
    correcting and the covariance grows, so the filter dead-reckons on cmd_vel
    and re-converges with a high gain when the sensor returns.
  * Marvelmind fresh-gating: the ~6 Hz fix is republished at ~100 Hz; the
    position update runs only when (x_m, y_m, timestamp_ms) actually changes,
    so repeats do not act like an artificially shrunk measurement variance.
  * Chi-square innovation gate on the Marvelmind update rejects the recalibration
    jumps that used to snap the whole landmark cloud -- the principled
    replacement for pose_publisher's 3 s drift-correction envelope.
  * Absolute odom-yaw update pins the heading so gyro integration cannot drift
    (the failure mode of the disabled kalman_filter.py).
"""

import math
from collections import deque
from typing import Optional

from geometry_msgs.msg import Twist
from marvelmind_ros2_msgs.msg import HedgePositionAddressed
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from swarm_interfaces.msg import StatePos as State
from tf2_ros import TransformBroadcaster

# Reuse the exact output-contract builders so parity with pose_publisher is
# guaranteed (frame ids, covariance layout, twist handling). Importing these
# module-level helpers does not run or modify the pose_publisher node.
from .pose_publisher import (
    degrees,
    make_localization_odometry,
    make_map_to_odometry_transform,
    quaternion_yaw,
    wrap_angle,
)


# =============================================================================
# Linear Kalman update (shared by both filters)
# =============================================================================

def kf_update(x, P, H, z, R, gate_chi2=None, angle_wrap=False):
    """One linear Kalman correction with optional chi-square outlier gate.

    Returns (x, P, d2). When the normalized innovation squared d2 exceeds
    ``gate_chi2`` the measurement is rejected and (x, P) are returned unchanged.
    ``angle_wrap`` wraps a scalar yaw innovation to [-pi, pi] before the gain so
    a +179 deg / -179 deg pair is treated as the 2 deg step it really is.
    """
    y = z - H @ x
    if angle_wrap:
        y = np.array([wrap_angle(float(v)) for v in y])
    S = H @ P @ H.T + R
    try:
        S_inv = np.linalg.inv(S)
    except np.linalg.LinAlgError:
        return x, P, None
    d2 = float(y @ S_inv @ y)
    if gate_chi2 is not None and d2 > gate_chi2:
        return x, P, d2
    K = P @ H.T @ S_inv
    x_new = x + K @ y
    ident = np.eye(P.shape[0])
    i_kh = ident - K @ H
    # Joseph form keeps P symmetric positive-definite under rounding.
    P_new = i_kh @ P @ i_kh.T + K @ R @ K.T
    P_new = 0.5 * (P_new + P_new.T)
    return x_new, P_new, d2


def circular_mean(angles) -> float:
    """Mean of a set of angles (radians), robust to the +/-pi wrap."""
    if not angles:
        return 0.0
    s = sum(math.sin(a) for a in angles)
    c = sum(math.cos(a) for a in angles)
    return math.atan2(s, c)


# =============================================================================
# Node
# =============================================================================

class PoseKalmanFilterNode(Node):
    def __init__(self):
        super().__init__("pose_kf")

        # --- Lifecycle / framing -------------------------------------------
        self.declare_parameter("init_period", 1.0)
        self.declare_parameter("timer_frequency", 0.02)
        self.declare_parameter("init_orientation", 0.0)       # degrees
        self.declare_parameter("init_pose", [0.0, 0.0])       # x, y metres
        self.declare_parameter("global_frame_id", "swarm_map")
        self.declare_parameter("require_mm_for_init", True)
        self.declare_parameter("mm_address", -1)              # -1 = accept any

        # --- Timeouts (seconds) --------------------------------------------
        self.declare_parameter("gps_timeout", 0.25)
        self.declare_parameter("odometry_timeout", 0.25)
        self.declare_parameter("gyro_timeout", 0.25)
        self.declare_parameter("cmd_timeout", 0.25)

        # --- Measurement noise ---------------------------------------------
        self.declare_parameter("gps_position_variance", 0.01)  # R_gps (m^2)
        self.declare_parameter("r_odom_vel", 0.04)             # (m/s)^2
        self.declare_parameter("r_yaw", 0.01)                  # rad^2
        self.declare_parameter("r_gyro", 0.01)                 # (rad/s)^2

        # --- Process model -------------------------------------------------
        self.declare_parameter("command_tau", 0.25)           # velocity lag (s)
        self.declare_parameter("q_pos", 1.0e-4)
        self.declare_parameter("q_vel", 0.5)
        self.declare_parameter("q_theta", 1.0e-4)
        self.declare_parameter("q_omega", 0.5)
        self.declare_parameter("innovation_gate_chi2", 9.0)   # 2-DOF ~99%
        # If the position gate rejects fixes continuously for this long, the
        # filter has diverged from the beacons (bad seed, teleport, recal) --
        # consistent "outliers" that agree are the truth, so snap to them.
        self.declare_parameter("mm_recovery_timeout", 1.5)    # seconds
        # Divergence recovery is trusted blindly only during INITIAL acquisition
        # (to escape a bad seed). Once the filter has sustained a clean lock for
        # lock_establish_time, a recovery snap must be corroborated by dead-
        # reckoning (odom moved roughly as far as the snap) -- a *sustained*
        # Marvelmind false lock produces a self-consistent rejected cluster that
        # would otherwise be snapped onto while the robot never moved (the
        # blow-up failure mode).
        self.declare_parameter("lock_establish_time", 3.0)    # seconds
        self.declare_parameter("recovery_odom_slack", 0.4)    # metres
        # Robust seed gate (mirrors pose_publisher.robust_mean): Marvelmind emits
        # isolated multi-metre spikes, so anchor on the median cluster, keep only
        # fixes within init_outlier_radius, and refuse to initialize until at
        # least init_min_samples agree -- a slow start beats a garbage anchor.
        self.declare_parameter("init_outlier_radius", 0.35)   # metres
        self.declare_parameter("init_min_samples", 5)
        # Seed must also persist in time, not just in count, so a brief burst of
        # fast distinct fixes cannot anchor the seed on a transient cluster.
        self.declare_parameter("init_min_span", 0.75)         # seconds

        # --- QoS -----------------------------------------------------------
        self.declare_parameter("qos_depth", 10)
        self.declare_parameter("qos_reliability", "RELIABLE")
        self.declare_parameter("qos_history", "KEEP_LAST")

        g = self.get_parameter
        self.init_period = float(g("init_period").value)
        self.timer_frequency = float(g("timer_frequency").value)
        self.init_orientation_rad = math.radians(float(g("init_orientation").value))
        init_pose = list(g("init_pose").value)
        self.init_xy = np.array(
            [float(init_pose[0]), float(init_pose[1])], dtype=float)
        self.global_frame_id = str(g("global_frame_id").value)
        self.require_mm_for_init = bool(g("require_mm_for_init").value)
        self.mm_address = int(g("mm_address").value)

        self.gps_timeout = float(g("gps_timeout").value)
        self.odometry_timeout = float(g("odometry_timeout").value)
        self.gyro_timeout = float(g("gyro_timeout").value)
        self.cmd_timeout = float(g("cmd_timeout").value)

        self.gps_position_variance = float(g("gps_position_variance").value)
        self.r_odom_vel = float(g("r_odom_vel").value)
        self.r_yaw = float(g("r_yaw").value)
        self.r_gyro = float(g("r_gyro").value)

        self.command_tau = max(float(g("command_tau").value), 1.0e-3)
        self.q_pos = float(g("q_pos").value)
        self.q_vel = float(g("q_vel").value)
        self.q_theta = float(g("q_theta").value)
        self.q_omega = float(g("q_omega").value)
        self.innovation_gate_chi2 = float(g("innovation_gate_chi2").value)
        self.mm_recovery_timeout = float(g("mm_recovery_timeout").value)
        self.lock_establish_time = float(g("lock_establish_time").value)
        self.recovery_odom_slack = float(g("recovery_odom_slack").value)
        self.init_outlier_radius = float(g("init_outlier_radius").value)
        self.init_min_samples = int(g("init_min_samples").value)
        self.init_min_span = float(g("init_min_span").value)

        self.namespace = self.get_namespace()

        # --- Filter state ---------------------------------------------------
        self.xh = np.zeros(2)               # [theta, omega]
        self.Ph = np.diag([0.1, 0.1])
        self.xp = np.zeros(4)               # [x, y, vx, vy]
        self.Pp = np.diag([0.09, 0.09, 0.25, 0.25])

        # --- Command input --------------------------------------------------
        self.cmd_vx = self.cmd_vy = self.cmd_wz = 0.0
        self.cmd_time: Optional[int] = None

        # --- Measurement bookkeeping ---------------------------------------
        self.latest_odom: Optional[Odometry] = None
        self.raw_odom_yaw_0: Optional[float] = None
        self.init_yaw_samples = deque(maxlen=200)
        self.seed_xy: Optional[np.ndarray] = None
        self.init_mm_samples = deque(maxlen=60)   # robust position seed (median)
        self.init_mm_times = deque(maxlen=60)     # ns stamps, for the seed-span gate
        self.gate_reject_since: Optional[int] = None  # ns streak start (recovery)
        self.reject_samples = deque(maxlen=60)    # fixes seen during a reject streak
        self.reject_odom_xy0: Optional[np.ndarray] = None  # odom at streak start
        self.has_locked = False       # latched once a clean lock is established
        self.lock_since: Optional[int] = None   # ns of current accepted-run start
        self.last_mm_key = None             # (x_m, y_m, timestamp_ms) fresh-gate
        self.gps_time: Optional[int] = None
        self.odom_time: Optional[int] = None
        self.gyro_time: Optional[int] = None

        # --- Lifecycle ------------------------------------------------------
        self.initialized = False
        self.start_time: Optional[int] = None
        self.last_predict_time: Optional[int] = None

        # --- ROS interfaces -------------------------------------------------
        qos = self._qos_from_parameters()
        localization_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.pose_pub = self.create_publisher(State, "pose", qos)
        self.localization_pub = self.create_publisher(
            Odometry, "localization/odom", localization_qos)

        self.create_subscription(
            HedgePositionAddressed, "mm_pos_unf", self._on_marvelmind, qos)
        self.create_subscription(Odometry, "odom", self._on_odom, qos)
        self.create_subscription(Twist, "cmd_vel", self._on_cmd, qos)
        self.create_subscription(Imu, "imu", self._on_gyro, qos)
        self.create_subscription(Imu, "mm_imu", self._on_gyro, qos)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_timer(max(self.timer_frequency, 1.0e-3), self._tick)

        self.get_logger().info(
            "pose_kf: initializing; waiting %.1fs and a Marvelmind fix%s."
            % (self.init_period,
               "" if self.require_mm_for_init else " (optional)"))

    # -- QoS ---------------------------------------------------------------
    def _qos_from_parameters(self) -> QoSProfile:
        depth = int(self.get_parameter("qos_depth").value)
        reliability = str(self.get_parameter("qos_reliability").value)
        history = str(self.get_parameter("qos_history").value)
        return QoSProfile(
            depth=max(depth, 1),
            reliability=(ReliabilityPolicy.RELIABLE if reliability == "RELIABLE"
                         else ReliabilityPolicy.BEST_EFFORT),
            history=(HistoryPolicy.KEEP_LAST if history == "KEEP_LAST"
                     else HistoryPolicy.KEEP_ALL),
        )

    # -- Helpers -----------------------------------------------------------
    def _now_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    def _cmd_fresh(self) -> bool:
        return (self.cmd_time is not None
                and (self._now_ns() - self.cmd_time) * 1e-9 <= self.cmd_timeout)

    def _odom_xy(self) -> Optional[np.ndarray]:
        """Latest raw odom position (odom frame) -- the dead-reckoning witness
        used to corroborate a recovery snap. None until the first odom arrives."""
        if self.latest_odom is None:
            return None
        p = self.latest_odom.pose.pose.position
        return np.array([float(p.x), float(p.y)])

    @property
    def theta(self) -> float:
        return float(self.xh[0])

    # ======================================================================
    # Subscriptions
    # ======================================================================
    def _on_cmd(self, msg: Twist):
        self.cmd_vx = float(msg.linear.x)
        self.cmd_vy = float(msg.linear.y)
        self.cmd_wz = float(msg.angular.z)
        self.cmd_time = self._now_ns()

    def _on_odom(self, msg: Odometry):
        self.latest_odom = msg
        self.odom_time = self._now_ns()

        raw_yaw = quaternion_yaw(msg.pose.pose.orientation)
        if raw_yaw is None:
            return

        if not self.initialized:
            # Collect a short pre-init average so the yaw anchor is not pinned
            # to a single noisy sample.
            self.init_yaw_samples.append(raw_yaw)
            return

        if self.raw_odom_yaw_0 is None:
            # First odom after init (none collected pre-init): anchor now.
            self.raw_odom_yaw_0 = raw_yaw

        # Update A -- absolute yaw: anti-drift anchor for the heading filter.
        theta_meas = wrap_angle(
            self.init_orientation_rad + wrap_angle(raw_yaw - self.raw_odom_yaw_0))
        self.xh, self.Ph, _ = kf_update(
            self.xh, self.Ph,
            np.array([[1.0, 0.0]]), np.array([theta_meas]),
            np.array([[self.r_yaw]]), angle_wrap=True)
        self.xh[0] = wrap_angle(self.xh[0])

        # Update B -- odom velocity rotated body->map with the current heading.
        c, s = math.cos(self.theta), math.sin(self.theta)
        bvx, bvy = float(msg.twist.twist.linear.x), float(msg.twist.twist.linear.y)
        z = np.array([c * bvx - s * bvy, s * bvx + c * bvy])
        H = np.array([[0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
        self.xp, self.Pp, _ = kf_update(
            self.xp, self.Pp, H, z, self.r_odom_vel * np.eye(2))

    def _on_gyro(self, msg: Imu):
        self.gyro_time = self._now_ns()
        if not self.initialized:
            return
        z = np.array([float(msg.angular_velocity.z)])
        self.xh, self.Ph, _ = kf_update(
            self.xh, self.Ph,
            np.array([[0.0, 1.0]]), z, np.array([[self.r_gyro]]))

    def _on_marvelmind(self, msg: HedgePositionAddressed):
        if self.mm_address >= 0 and int(msg.address) != self.mm_address:
            return

        key = (float(msg.x_m), float(msg.y_m), int(msg.timestamp_ms))
        if key == self.last_mm_key:
            return                           # fresh-gate: ignore 100 Hz repeats
        self.last_mm_key = key

        x_m, y_m = float(msg.x_m), float(msg.y_m)
        # Reject invalid fixes outright: NaN/inf, or the exact (0,0) the hedgehog
        # emits when it has no lock (known failure mode). Never seed from these.
        if not (math.isfinite(x_m) and math.isfinite(y_m)):
            return
        if x_m == 0.0 and y_m == 0.0:
            return

        xy = np.array([x_m, y_m])
        self.seed_xy = xy
        self.init_mm_samples.append(xy)      # median seed buffer (pre-init)
        self.init_mm_times.append(self._now_ns())
        self.gps_time = self._now_ns()

        if not self.initialized:
            return

        # Update A -- Marvelmind position, with chi-square jump rejection.
        H = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]])
        R = self.gps_position_variance * np.eye(2)
        self.xp, self.Pp, d2 = kf_update(
            self.xp, self.Pp, H, xy, R, gate_chi2=self.innovation_gate_chi2)

        if d2 is None or d2 <= self.innovation_gate_chi2:
            self.gate_reject_since = None    # accepted: clear the reject streak
            self.reject_samples.clear()
            self.reject_odom_xy0 = None
            # Lock latch: a continuous run of accepted fixes lasting
            # lock_establish_time means the estimate is genuinely locked, after
            # which recovery snaps demand odom corroboration (see _snap gating).
            now_ns = self._now_ns()
            if self.lock_since is None:
                self.lock_since = now_ns
            elif (not self.has_locked
                  and (now_ns - self.lock_since) * 1e-9 >= self.lock_establish_time):
                self.has_locked = True
                self.get_logger().info(
                    "pose_kf: localization lock established; divergence recovery "
                    "now requires dead-reckoning corroboration.")
            return

        # Rejected as an outlier. A single rejection is a glitch; a *sustained*
        # streak means the estimate itself has diverged (bad seed, teleport, a
        # real recalibration) -- the consistent fixes are the truth, so re-seed.
        now_ns = self._now_ns()
        if self.gate_reject_since is None:
            self.gate_reject_since = now_ns
            self.reject_odom_xy0 = self._odom_xy()   # dead-reckoning reference
            if not self.has_locked:
                # The contested lock is invalid; restart the acquisition timer so
                # a bad seed under dispute never latches has_locked.
                self.lock_since = None
        self.reject_samples.append(xy)
        rejected_for = (now_ns - self.gate_reject_since) * 1e-9
        if rejected_for < self.mm_recovery_timeout:
            self.get_logger().warn(
                "pose_kf: Marvelmind fix rejected as outlier (d2=%.1f > %.1f)."
                % (d2, self.innovation_gate_chi2), throttle_duration_sec=2.0)
            return

        # Streak exceeded the recovery window. Only snap if the rejected fixes
        # themselves form a self-consistent cluster (a genuine shift) -- NOT a
        # single raw fix, which could be one of Marvelmind's multi-metre spikes.
        # Scattered rejects (noise/spikes) yield None, so we keep dead-reckoning.
        cluster = self._robust_mean(self.reject_samples)
        if cluster is None:
            self.get_logger().warn(
                "pose_kf: position gated %.1fs but rejected fixes disagree "
                "(%d samples); holding estimate, still collecting."
                % (rejected_for, len(self.reject_samples)),
                throttle_duration_sec=2.0)
            return

        # A locked estimate only snaps if dead-reckoning corroborates the move.
        # A sustained Marvelmind false lock yields a self-consistent cluster, but
        # the robot never physically travelled there, so odom does not agree --
        # hold and keep dead-reckoning until the beacons return to a plausible
        # fix (which then passes the chi-square gate normally). During initial
        # acquisition (not yet locked) the blind snap is kept, to escape a bad
        # seed whose rescue cluster is also odom-stationary.
        if self.has_locked:
            jump = float(np.hypot(cluster[0] - self.xp[0], cluster[1] - self.xp[1]))
            odom_now = self._odom_xy()
            if odom_now is not None and self.reject_odom_xy0 is not None:
                odom_disp = float(np.hypot(*(odom_now - self.reject_odom_xy0)))
            else:
                odom_disp = 0.0
            if jump - odom_disp > self.recovery_odom_slack:
                self.get_logger().warn(
                    "pose_kf: locked; gated %.1fs and rejected fixes agree, but "
                    "dead-reckoning (odom moved %.3f m) does NOT corroborate a "
                    "%.2f m snap -- holding (likely a Marvelmind false lock)."
                    % (rejected_for, odom_disp, jump),
                    throttle_duration_sec=2.0)
                return    # leave the streak armed; keep dead-reckoning

        self.xp[0], self.xp[1] = float(cluster[0]), float(cluster[1])
        self.xp[2:] = 0.0
        self.Pp[0, 0] = self.Pp[1, 1] = 0.09   # inflate so it re-locks fast
        self.gate_reject_since = None
        self.reject_samples.clear()
        self.reject_odom_xy0 = None
        self.get_logger().warn(
            "pose_kf: position diverged (gated %.1fs); re-seeding to validated "
            "Marvelmind cluster (%.2f, %.2f)." % (rejected_for, cluster[0], cluster[1]))

    # ======================================================================
    # Prediction + publication
    # ======================================================================
    def _predict(self, dt: float):
        alpha = min(dt / self.command_tau, 1.0)
        cmd_fresh = self._cmd_fresh()

        # Heading filter -----------------------------------------------------
        u_omega = self.cmd_wz if cmd_fresh else 0.0
        Fh = np.array([[1.0, dt], [0.0, 1.0 - alpha]])
        self.xh = Fh @ self.xh + np.array([0.0, alpha * u_omega])
        self.xh[0] = wrap_angle(self.xh[0])
        Qh = np.diag([self.q_theta, self.q_omega]) * dt
        self.Ph = Fh @ self.Ph @ Fh.T + Qh
        self.Ph = 0.5 * (self.Ph + self.Ph.T)

        # Planar filter (theta treated as known from the heading filter) ------
        theta = self.theta
        if cmd_fresh:
            c, s = math.cos(theta), math.sin(theta)
            u_vx = c * self.cmd_vx - s * self.cmd_vy
            u_vy = s * self.cmd_vx + c * self.cmd_vy
        else:
            u_vx = u_vy = 0.0
        Fp = np.array([
            [1.0, 0.0, dt, 0.0],
            [0.0, 1.0, 0.0, dt],
            [0.0, 0.0, 1.0 - alpha, 0.0],
            [0.0, 0.0, 0.0, 1.0 - alpha],
        ])
        self.xp = Fp @ self.xp + np.array([0.0, 0.0, alpha * u_vx, alpha * u_vy])
        Qp = np.diag([self.q_pos, self.q_pos, self.q_vel, self.q_vel]) * dt
        self.Pp = Fp @ self.Pp @ Fp.T + Qp
        self.Pp = 0.5 * (self.Pp + self.Pp.T)

    def _robust_mean(self, samples) -> Optional[np.ndarray]:
        """Mean of the Marvelmind fixes within init_outlier_radius of their
        median; None if fewer than init_min_samples agree (mirrors
        pose_publisher.robust_mean, which survives isolated multi-metre spikes).
        Shared by init seeding and divergence recovery so both demand a
        self-consistent cluster -- never a single raw (possibly-spike) fix.
        """
        if len(samples) < self.init_min_samples:
            return None
        s = np.vstack(samples)
        centre = np.median(s, axis=0)
        inliers = s[np.linalg.norm(s - centre, axis=1) <= self.init_outlier_radius]
        if len(inliers) < self.init_min_samples:
            return None
        return np.mean(inliers, axis=0)

    def _try_initialize(self, now_ns: int) -> bool:
        if self.start_time is None:
            self.start_time = now_ns
            return False
        if (now_ns - self.start_time) * 1e-9 < self.init_period:
            return False
        # Robust seed: anchor on the median inlier cluster, refusing to
        # initialize on an isolated Marvelmind spike. Defers (keeps collecting)
        # until init_min_samples agree -- a slow start beats a garbage anchor.
        seed = self._robust_mean(self.init_mm_samples)
        if seed is None:
            if self.require_mm_for_init:
                self.get_logger().warn(
                    "pose_kf: init deferred -- too few agreeing Marvelmind fixes "
                    "(%d collected); still collecting."
                    % len(self.init_mm_samples), throttle_duration_sec=2.0)
                return False
            seed = self.init_xy          # fall back to configured pose
        elif self.require_mm_for_init and self.init_min_span > 0.0:
            # Seed-span gate: the agreeing cluster must also persist in time, so a
            # brief burst of distinct fixes cannot anchor a transient cluster.
            s = np.vstack(self.init_mm_samples)
            centre = np.median(s, axis=0)
            mask = np.linalg.norm(s - centre, axis=1) <= self.init_outlier_radius
            tt = np.array(self.init_mm_times)[mask]
            if len(tt) >= 2 and (tt.max() - tt.min()) * 1e-9 < self.init_min_span:
                self.get_logger().warn(
                    "pose_kf: init deferred -- agreeing Marvelmind cluster spans "
                    "only %.2fs (< %.2fs); still collecting."
                    % ((tt.max() - tt.min()) * 1e-9, self.init_min_span),
                    throttle_duration_sec=2.0)
                return False
        self.xp[:2] = seed
        self.xp[2:] = 0.0
        self.xh[0] = self.init_orientation_rad
        self.xh[1] = 0.0
        if self.init_yaw_samples:
            self.raw_odom_yaw_0 = circular_mean(list(self.init_yaw_samples))
        self.initialized = True
        self.get_logger().info(
            "pose_kf: initialized at (%.2f, %.2f) theta=%.1f deg."
            % (self.xp[0], self.xp[1], math.degrees(self.xh[0])))
        return True

    def _tick(self):
        now_ns = self._now_ns()
        if not self.initialized:
            if not self._try_initialize(now_ns):
                return
            self.last_predict_time = now_ns

        dt = (now_ns - self.last_predict_time) * 1e-9
        self.last_predict_time = now_ns
        dt = min(max(dt, 1.0e-3), 0.2)       # survive scheduler/bag stalls
        self._predict(dt)

        # The output builders need a source odom template (frame ids, twist
        # covariance). Until one arrives, hold publication -- the heading
        # anchor needs odom anyway.
        if self.latest_odom is None:
            self.get_logger().warn(
                "pose_kf: no odom yet; withholding localization output.",
                throttle_duration_sec=2.0)
            return

        self._publish(self.get_clock().now().to_msg())

    def _publish(self, stamp):
        x, y = float(self.xp[0]), float(self.xp[1])
        vx_map, vy_map = float(self.xp[2]), float(self.xp[3])
        theta = self.theta
        omega = float(self.xh[1])

        pose = State()
        pose.x = x
        pose.y = y
        pose.theta = degrees(theta)          # DEGREES, per the contract
        self.pose_pub.publish(pose)

        # localization/odom twist is BODY frame: rotate map velocity by R(-theta)
        # (the consumer re-rotates it by this message's own quaternion).
        c, s = math.cos(theta), math.sin(theta)
        body_vx = c * vx_map + s * vy_map
        body_vy = -s * vx_map + c * vy_map

        localization = make_localization_odometry(
            x, y, theta, body_vx, body_vy, omega,
            self.latest_odom, stamp, self.global_frame_id,
            self.namespace, self.gps_position_variance)
        self.localization_pub.publish(localization)

        try:
            tf = make_map_to_odometry_transform(
                x, y, theta, self.latest_odom, stamp, self.global_frame_id)
            self.tf_broadcaster.sendTransform(tf)
        except ValueError:
            pass                              # odom frame not usable this cycle


def main(args=None):
    rclpy.init(args=args)
    node = PoseKalmanFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
