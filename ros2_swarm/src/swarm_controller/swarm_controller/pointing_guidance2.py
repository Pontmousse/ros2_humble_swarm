import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time

import math, numpy as np

# Custom messages
from swarm_interfaces.msg import StatePos as State
from swarm_interfaces.msg import Markers
from std_msgs.msg import Float64
from geometry_msgs.msg import Wrench
from nav_msgs.msg import Odometry

class GuidanceNode(Node):
    def __init__(self):
        super().__init__('landmark_tracking_controller')
        self.get_logger().info("Landmark tracking controller has been started.")

        # For initial pose initialization using GPS / Odometry readings
        self.declare_parameter("init_period", 1.0)
        self.init_duration = self.get_parameter("init_period").value
        self.init_start_time = self.get_clock().now()

        # For timer
        self.declare_parameter("timer_frequency", 0.01)
        self.timer_frequency = self.get_parameter("timer_frequency").value
        self.get_logger().info(f"Timer frequency: {self.timer_frequency} seconds")

        # Attitude PD -> torque wrench. Budget mirrors the other gated layers so
        # no single layer can saturate the spacecraft clamp on its own.
        self.declare_parameter("heading_torque_gain", 0.6)   # Nm/rad, P on heading error
        self.declare_parameter("yaw_damping_gain", 0.15)     # Nm/(rad/s), D on craft yaw rate
        self.declare_parameter("maximum_torque", 0.2)        # publisher-side cap
        self.heading_torque_gain = self.get_parameter("heading_torque_gain").value
        self.maximum_torque = self.get_parameter("maximum_torque").value

        # Shared response-scale knob from the global /** block of
        # virtual_spacecraft.yaml. The spacecraft node scales this layer's torque
        # by this factor, which drops the yaw damping ratio by sqrt(scale);
        # yaw_damping_gain is boosted by 1/sqrt(scale) to hold zeta constant.
        # No-op at rotation_response_scale 1.0. 1.0 = no change.
        self.declare_parameter("rotation_response_scale", 1.0)
        rotation_response_scale = float(
            self.get_parameter("rotation_response_scale").value)
        if not (math.isfinite(rotation_response_scale)
                and 0.0 < rotation_response_scale <= 1.0):
            raise ValueError("rotation_response_scale must be in (0, 1]")
        self.yaw_damping_gain = (
            self.get_parameter("yaw_damping_gain").value
            / math.sqrt(rotation_response_scale))

        # Slew-rate limit on the published torque (Nm/s). The virtual spacecraft
        # is an undamped double integrator, so the first frame after a multi-
        # second perception freeze used to deliver an instantaneous bang-bang
        # kick (full +/-maximum_torque) into whatever yaw rate had coasted up --
        # successive kicks rang the yaw axis up to ~0.75 rad/s. Ramping the
        # torque (soft-start) lets the error shrink and damping grow before the
        # cap is reached. 0.4 Nm/s reaches the 0.2 Nm cap from zero in ~0.5 s.
        self.declare_parameter("torque_slew_rate", 0.4)       # Nm/s
        self.torque_slew_rate = float(self.get_parameter("torque_slew_rate").value)

        # Perception freshness. landmarks header.stamp freezes when the detector
        # stalls (the topic still republishes at 100 Hz). Past this age the
        # centroid is treated as lost: stop CHASING it (zero the P term), but
        # keep the -Kd*craft_wz damping term live so a coasting yaw rate still
        # bleeds off -- craft_wz comes from odom and is always fresh. Zeroing the
        # whole wrench (as before) removed the only damping and let the undamped
        # plant coast, which is what rang the yaw axis up.
        self.declare_parameter("landmarks_freshness_timeout", 0.3)  # seconds
        self.landmarks_freshness_timeout = float(
            self.get_parameter("landmarks_freshness_timeout").value)
        self.last_landmarks_time = None

        # # # # # # # # # # # # # # # # # # # # # # # # # # # 
        # For QoS
        self.declare_parameter('qos_depth', 10)
        self.declare_parameter('qos_reliability', 'RELIABLE')
        self.declare_parameter('qos_history', 'KEEP_LAST')

        qos_depth = self.get_parameter('qos_depth').value
        qos_reliability = self.get_parameter('qos_reliability').value
        qos_history = self.get_parameter('qos_history').value

        # Validate QoS settings       
        valid_reliabilities = ['RELIABLE', 'BEST_EFFORT']
        valid_histories = ['KEEP_LAST', 'KEEP_ALL']

        if qos_reliability not in valid_reliabilities:
            raise ValueError(f"Invalid qos_reliability: {qos_reliability}")
        if qos_history not in valid_histories:
            raise ValueError(f"Invalid qos_history: {qos_history}")

        # QoS Settings 
        self.qos_profile = QoSProfile(
            depth=qos_depth,
            reliability=ReliabilityPolicy.RELIABLE if qos_reliability == 'RELIABLE' else ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST if qos_history == 'KEEP_LAST' else HistoryPolicy.KEEP_ALL
        )
        # # # # # # # # # # # # # # # # # # # # # # # # # # # 

        # Control state
        self.pose = np.array([None, None, None])  # x, y, theta
        self.lc = np.array([None, None])          # landmark centroid
        self.target_lcd = None                    # direction (deg)
        self.craft_wz = None                      # spacecraft yaw rate (rad/s)

        # Slew limiter state: the last torque actually published and when.
        self._last_torque = 0.0
        self._last_pub_time = None

        # Subscribers
        self.create_subscription(State, 'pose', self.pose_callback, self.qos_profile)
        self.create_subscription(Markers, 'landmarks', self.landmarks_callback, self.qos_profile)
        self.create_subscription(Odometry, 'localization/odom', self.odom_callback, self.qos_profile)

        # Publishers
        self.pub = self.create_publisher(Float64, 'target_orientation', self.qos_profile)
        self.wrench_pub = self.create_publisher(Wrench, 'pointing_wrench', self.qos_profile)
        self.pub_timer = self.create_timer(self.timer_frequency, self.publish_target_orientation)
 

    def pose_callback(self, msg: State):
        self.pose = np.array([msg.x, msg.y, msg.theta])


    def odom_callback(self, msg: Odometry):
        # Yaw rate is frame-invariant, so the body-frame twist is used as-is.
        self.craft_wz = msg.twist.twist.angular.z



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
            self.target_lcd = None
            return

        # Extract landmarks
        landmarks = make_array(msg.markers) / 100.0  # Convert cm to m

        # Transform to global frame from local robot frame
        theta_rad = math.radians(self.pose[2])
        rot_matrix = np.array([
            [math.cos(theta_rad), -math.sin(theta_rad)],
            [math.sin(theta_rad),  math.cos(theta_rad)]
        ])
        # self.get_logger().warn(f"rotation: {rot_matrix}\n")
        # self.get_logger().warn(f"landmarks: {landmarks.T}\n")
        landmarks_global = self.pose[:2] + np.matmul(rot_matrix, landmarks.T).T

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
        elapsed = (now - self.init_start_time).nanoseconds / 1e9
        if elapsed <= 2 * self.init_duration:
            return

        # Decide the DESIRED torque, then slew-limit it once at the single publish
        # point below. Two regimes:
        #   - valid target (fresh perception, non-empty, pose known) -> full PD
        #   - otherwise (perception lost/stale, no markers, no pose) -> damp-only
        # Damp-only is NOT a hard zero: it keeps -Kd*wz alive so a coasting yaw
        # rate still decays (the fix for the ring-up).
        if self.landmarks_expired() or self.target_lcd is None or (None in self.pose):
            desired = self._damping_torque()
        else:
            # Active pointing: publish the advisory setpoint and the PD torque.
            msg = Float64()
            msg.data = self.target_lcd
            self.pub.publish(msg)
            err = math.radians(wrap_deg(self.target_lcd - self.pose[2]))
            wz = self.craft_wz if self.craft_wz is not None else 0.0
            desired = bound(
                self.heading_torque_gain * err - self.yaw_damping_gain * wz,
                self.maximum_torque)

        wmsg = Wrench()
        wmsg.torque.z = self._slew(desired, now)
        self.wrench_pub.publish(wmsg)


def bound(input, limit):
    return max(min(input, limit), -limit)


def wrap_deg(angle):
    return (angle + 180.0) % 360.0 - 180.0


def make_array(landmarks):
    M = len(landmarks)
    arr = np.zeros((M, 2))
    for i in range(M):
        arr[i, 0] = landmarks[i].x
        arr[i, 1] = landmarks[i].y
    return arr


def main(args=None):
    rclpy.init(args=args)
    node = GuidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
