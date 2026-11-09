import math

import numpy as np
import rclpy

from geometry_msgs.msg import Wrench
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_srvs.srv import SetBool

# Custom messages
from swarm_interfaces.msg import CoordXY, Markers, NeighbourStates
from swarm_interfaces.msg import StatePos as State

from .avoidance import neighbour_repulsion


class GuidanceNode(Node):
    def __init__(self):
        super().__init__('target_position_controller')
        self.get_logger().info("Target position controller has been started.")

        # For initial pose initialization using GPS / Odometry readings
        self.declare_parameter("init_period", 1.0)
        self.init_duration = self.get_parameter("init_period").value
        self.init_start_time = self.get_clock().now()

        # For timer
        self.declare_parameter("timer_frequency", 0.01)
        self.timer_frequency = self.get_parameter("timer_frequency").value

        # Shared response-scale knob from the global /** block of
        # virtual_spacecraft.yaml. The spacecraft node scales this layer's
        # integrated wrench by this factor, which drops the damping ratio by
        # sqrt(scale); flk_damping_gain is boosted by 1/sqrt(scale) below to hold
        # zeta constant across the sweep. 1.0 = no-op.
        self.declare_parameter("translation_response_scale", 1.0)
        translation_response_scale = float(
            self.get_parameter("translation_response_scale").value)
        if not (math.isfinite(translation_response_scale)
                and 0.0 < translation_response_scale <= 1.0):
            raise ValueError("translation_response_scale must be in (0, 1]")

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
        self.qos_profile_reliable = QoSProfile(
            depth=qos_depth,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )

        self.qos_profile_best_effort = QoSProfile(
            depth=qos_depth,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )
        
        # # # # # # # # # # # # # # # # # # # # # # # # # # #

        # Control states
        self.pose = np.array([None, None, None])  # x, y, theta
        self.landmarks = [] # list of landmark coordinates in global reference frame
        self.target = np.array([None, None])      # target position
        self.neighbours = [] # list of (x, y, vx, vy) neighbour states, map frame
        # Own map-frame velocity, for the damping term. None until the first
        # odometry arrives, which makes the law degrade to the pure potential
        # rather than to a wrong closing speed.
        self.velocity = None
        self.last_odom_time = None


        # Control gains
        self.Rflk = 0.6
        # Two robots touch at 0.44 m centre-to-centre (2 x the robot_radius 0.22
        # declared in virtual_spacecraft.yaml). Rant was 0.4, i.e. the influence
        # radius sat entirely inside the collision: the repulsion was still
        # exactly 0 N at contact and only reached 1 N once the chassis had
        # overlapped by 4 cm. 0.70 moved the onset 0.26 m clear of contact.
        #
        # 0.26 m is still not enough runway, and the reason is a bound rather
        # than a tuning failure: a brake limited to max_nbh_force can absorb at
        # most sqrt(2 * (2F/m) * runway) however it is shaped, which is
        # 0.456 m/s at Rant = 0.70. The 2026-10-07 flight closed faster than
        # that 11.5% (RM2) / 6.7% (RM6) of the time, peaking at 0.660 m/s, so
        # no gain and no extra term could have held it. Runway is the only
        # factor in that bound we can buy cheaply and it enters under a square
        # root, so 1.05 m -- a 0.61 m runway -- lifts the ceiling to 0.699 m/s
        # and clears the measured peak.
        #
        # The cost is measured on the same bag rather than assumed: separation
        # fell in the newly-covered [0.70, 1.05) band only 9.7% of the time, so
        # the force goes from acting 7.2% of the time to 16.8%, and the median
        # separation (1.88 m) stays far outside it.
        self.Rant = 1.05
        self.flk_gain = 2.5
        # D term for the flocking spring, sized near critical damping
        # (2*zeta*sqrt(k*m), zeta~0.7, m=5.0) to kill the translation ringing
        # seen once flk_gain was raised live. Divided by sqrt(response scale) so
        # that when the spacecraft node scales this layer's wrench by lambda
        # (dropping zeta to sqrt(lambda)*zeta), the boost restores zeta to its
        # 15.0 baseline. 1.0 scale leaves it at 15.0.
        self.flk_damping_gain = 15.0 / math.sqrt(translation_response_scale)
        # Publisher-side cap on the flocking wrench. The spacecraft clamps the
        # SUM of all four sources to its maximum_force (1.0 N), so this is not a
        # safety bound -- it is the share of that envelope the flocking layer may
        # claim before avoidance and box-keeping get theirs. 0.5 N mirrors
        # bounding_box_search.maximum_force: two opposing contributors each
        # capped at the full 1.0 N would cancel to ~0 after the resultant clamp,
        # so neither could ever win (same failure the box force was halved for).
        self.max_flk_force = 1.0
        # Left at 0.78 so this change moves one variable. With the wider radius
        # the same gain saturates at d = 0.65 m instead of 0.55 m, i.e. 0.21 m
        # before contact rather than 0.11 m. That is the right direction: the
        # force-times-runway ceiling is derived by holding max_nbh_force across
        # the whole runway, so saturating earlier is strictly closer to the
        # bound. Restoring the old 0.55 m saturation point would need 0.35 and
        # would cost 0.07 m/s of pure-potential critical speed (0.524 -> 0.455)
        # in exchange for a gentler long-range push.
        self.antflk_gain = 0.38
        # The potential diverges as 1/d^3, so without a cap it reached 18 N at
        # d = 0.15 m. Two things make that unacceptable: the HIL dynamics were
        # only validated to 1.0 N / 0.2 Nm (2026-09-09 bag), and publish_target_position
        # reuses this same vector as a metre-scale waypoint offset at step=1.0,
        # so an 18 N force also threw the position target 18 m away.
        self.max_nbh_force = 1.0
        # Dissipative term, F_damp = damping_gain * max(0, closing speed). The
        # potential above is conservative, so the work it can do between Rant
        # and contact is fixed no matter how fast the pair is closing: it has a
        # single critical speed (0.368 m/s at the old Rant = 0.70, 0.524 m/s
        # now) and the excess above it is untouched. Measured 2026-10-07, RM2
        # and RM6 converged at 0.445 m/s and touched. Damping removes energy
        # instead of storing it, so it scales with the actual approach, and it
        # is at full strength the instant a fast approach crosses Rant -- which
        # is exactly where the potential is weakest (0.087 N at d = 0.676 m
        # under the old radius, so the first 0.1 m of that 0.26 m braking
        # runway was contributing almost nothing).
        #
        # 2.0 is not tuned: two references capped at virtual_spacecraft
        # maximum_speed = 0.25 m/s can close at 0.5 m/s, and 2.0 * 0.5 = 1.0 N
        # spends exactly the validated envelope at that worst case. Raising it
        # buys nothing -- see test_damping_cannot_beat_the_force_times_runway
        # _ceiling: no law bounded by 1.0 N absorbs more than
        # sqrt(2 * (2F/m) * runway), and with Rant = 1.05 damping already
        # reaches that bound (0.699 m/s) exactly. At Rant = 0.70 damping was
        # necessary but not sufficient, capped at 0.456 m/s against a 0.5 m/s
        # worst case; it was the runway, not the gain, that closed the gap.
        self.damping_gain = 2.0
        self.eps = 1e-4



        # velocity commands
        self.vel1 = np.array([0, 0])
        self.vel2 = np.array([0, 0])
        self.step = 1.0

        # ENCAPSULATION gate. vel1 (the landmark flocking potential) is always
        # computed from the landmark stream, but it is only published as a wrench
        # while the supervisor has this layer enabled. Starts False so a node
        # that comes up in SEARCH contributes an explicit zero until landmarks
        # move it to ENCAPSULATION; the supervisor flips it via encapsulation/
        # enable. Left as a plain flag (not a parameter) because the mode, not a
        # launch value, owns it.
        self.encapsulation_enabled = False

        # Forces from neighborhood antiflocking (collision avoidance) for virtual spacecraft
        self.nbh_Fx = 0.0
        self.nbh_Fy = 0.0

        self.last_nbh_time = None
        # Longer than the worst observed empty-table run (0.6 s, measured on
        # RM2/RM6 2026-10-05 at 0.457 m separation), so a UDP gap cannot cancel
        # the repulsion mid-approach. The cost of holding a stale force after a
        # neighbour genuinely leaves is bounded: 1.0 N is a_max 0.2 m/s^2, so
        # 0.75 s of it is 0.056 m of drift.
        self.nbh_hold_timeout = 0.75

        # Perception freshness. landmarks carries header.stamp = the time the
        # detector last produced a frame (frozen on a stalled pipeline, not the
        # 100 Hz republish time). Past this age the camera is treated as lost:
        # the flocking term is zeroed and stale local markers are NOT reprojected
        # by the live pose, mirroring the localization freeze in the spacecraft.
        self.declare_parameter("landmarks_freshness_timeout", 0.3)  # seconds
        self.landmarks_freshness_timeout = float(
            self.get_parameter("landmarks_freshness_timeout").value)
        self.last_landmarks_time = None

        # Subscribers
        self.create_subscription(State, 'pose', self.pose_callback, self.qos_profile_best_effort)
        self.create_subscription(Markers, 'landmarks', self.landmarks_callback, self.qos_profile_best_effort)
        # Own velocity for the damping term. Taken from localization/odom rather
        # than differentiated from `pose`, which would add filter noise straight
        # into a force, and whose .theta is in DEGREES -- a trap that would
        # silently rotate the body-to-map transform below.
        self.create_subscription(Odometry, 'localization/odom', self.odom_callback,
                                 self.qos_profile_best_effort)
        # comms/neighbours (UDP broadcast), not nbh_pos (Marvelmind). The modem's
        # real fix rate with two hedgehogs in the TDMA cycle measures 0.63-0.78
        # Hz, and neighbours_callback only recomputes the avoidance force when a
        # message arrives -- so the force stays frozen for ~1.6 s while the robot
        # keeps moving. The UDP table publishes 50 Hz of valid, dead-reckoned
        # neighbour states over the same link.
        #
        # NeighbourStates, not the legacy position-only Neighbours: the damping
        # term needs the neighbour's velocity to form a relative closing speed.
        # Differencing positions here instead would make a pair travelling
        # together in convoy brake against each other for nothing.
        self.create_subscription(NeighbourStates, 'comms/neighbours',
                                 self.neighbours_callback, self.qos_profile_best_effort)



        # Publishers
        self.wrench_nbh_pub = self.create_publisher(Wrench,"avoidance_wrench", 10)
        # Flocking (landmark APF) wrench, summed by the spacecraft as the
        # ENCAPSULATION layer. Map frame, same frame as vel1. Gated by
        # encapsulation_enabled: zero Wrench when disabled so it contributes
        # nothing rather than dropping out (a published zero keeps the source
        # fresh and the visualizer honest about the layer being present-but-off).
        self.wrench_flk_pub = self.create_publisher(Wrench, "flocking_wrench", 10)
        self.pub = self.create_publisher(CoordXY, 'target_position', self.qos_profile_reliable)
        self.pub_timer = self.create_timer(self.timer_frequency, self.publish_target_position)

        # Mode gate. The supervisor calls this to enter/leave ENCAPSULATION; no
        # other node writes self.encapsulation_enabled. Kept deliberately thin so
        # the longer-term switch condition (manual trigger, consensus) can live
        # entirely in the supervisor without touching this node.
        self.encap_srv = self.create_service(
            SetBool, "encapsulation/enable", self.set_encapsulation)
        

    def pose_callback(self, msg: State):
        self.pose = np.array([msg.x, msg.y, msg.theta])

    def odom_callback(self, msg: Odometry):
        """Store our own velocity in the map frame, for the damping term."""
        self.velocity = np.array(map_frame_velocity(msg))
        self.last_odom_time = self.get_clock().now()

    def landmarks_callback(self, msg: Markers):
        # Check if pose is initialized
        if None in self.pose:
            self.get_logger().warn("Pose not initialized yet (landmarks).",
                                   throttle_duration_sec=5.0)
            self.vel1 = np.array([0, 0])
            return

        # Perception freshness: a stalled detector republishes byte-identical
        # markers at 100 Hz, so the only tell is the frozen header stamp aging
        # out. Reproject the stale local markers by the live pose and the ghost
        # cloud stays rigidly attached to the robot, reading ~0 error forever.
        # Zero the flocking term and bail BEFORE reprojection -> hold.
        if self.landmarks_stale(msg.header.stamp):
            self.get_logger().warn(
                "Landmark perception stale; zeroing flocking and holding.",
                throttle_duration_sec=5.0)
            self.vel1 = np.array([0, 0])
            return
        self.last_landmarks_time = self.get_clock().now()

        # Check if landmarks exist
        if len(msg.markers) == 0:
            # self.get_logger().warn("Received empty landmarks.")
            self.vel1 = np.array([0, 0])
            return

        # Extract landmarks
        landmarks = make_array(msg.markers) / 100.0  # Convert cm to m

        # Transform to global frame from local robot frame
        theta_rad = math.radians(self.pose[2])
        rot_matrix = np.array([
            [math.cos(theta_rad), -math.sin(theta_rad)],
            [math.sin(theta_rad),  math.cos(theta_rad)]
        ])

        self.landmarks = self.pose[:2] + np.matmul(rot_matrix, landmarks.T).T

        # Calculate force direction and amplitude
        self.vel1 = self.flocking_potential(self.landmarks)   


    def set_encapsulation(self, request, response):
        self.encapsulation_enabled = request.data
        response.success = True
        response.message = (
            "encapsulation enabled" if request.data else "encapsulation disabled")
        self.get_logger().info(response.message)
        return response

    def flocking_potential(self, landmarks):
        Fx, Fy = 0.0, 0.0
        x, y = self.pose[0], self.pose[1]
        v = self.own_velocity()

        for i in range(len(landmarks)):
            xp, yp = landmarks[i, 0], landmarks[i, 1]
            dx = xp - x
            dy = yp - y
            distance = np.hypot(dx, dy) + self.eps
            ux, uy = dx / distance, dy / distance

            # Spring-like attractive/repulsive force, damped by closing speed
            closing = 0.0 if v is None else (v[0] * ux + v[1] * uy)
            force_magnitude = self.flk_gain * (distance - self.Rflk) - self.flk_damping_gain * closing

            Fx += force_magnitude * ux
            Fy += force_magnitude * uy

        Fx /= len(landmarks)
        Fy /= len(landmarks)

        return np.array([Fx, Fy])

    
    ######################################################################################################

    def neighbours_callback(self, msg: NeighbourStates):
        # Check if pose is initialized
        if None in self.pose:
            # Untethered this fires at the neighbour rate and, from the timer
            # below, at 20 Hz -- enough log formatting to matter on a Pi that is
            # already starving the guidance timer.
            self.get_logger().warn("Pose not initialized yet (neighbours).",
                                   throttle_duration_sec=5.0)
            # vel2 and nbh_F* are this callback's outputs; vel1 belongs to
            # landmarks_callback, whose identical guard above this one is where
            # the line was copied from. Zeroing vel1 here did the exact
            # inversion of the intent: it destroyed the landmark force and left
            # the avoidance force standing. Latent so far only because
            # self.pose is never reset to None once set, so this branch cannot
            # run after either force has become non-zero -- which is also why
            # it survived review. It stops being latent the moment pose is
            # invalidated on a localization timeout.
            self.vel2 = np.array([0, 0])
            self.nbh_Fx = 0.0
            self.nbh_Fy = 0.0
            return
        
        # NeighbourStates carries the whole table including entries already
        # marked invalid, so presence in the message is not freshness -- `valid`
        # is. Positions are map-frame metres, dead-reckoned to now by the
        # broadcaster; velocities are map-frame and are what the damping term
        # differences against our own.
        neighbours = [
            (n.x, n.y, n.vx, n.vy) for n in msg.neighbours if n.valid
        ]

        # The table empties on a single dropped UDP packet, so zeroing here is
        # not "no neighbour" -- it is "no news". Hold the last force until the
        # gap is long enough to mean the neighbour really is gone.
        if len(neighbours) == 0:
            if self.held_force_expired():
                self.vel2 = np.array([0, 0])
                self.nbh_Fx = 0.0
                self.nbh_Fy = 0.0
            return

        self.neighbours = neighbours

        # Calculate force direction and amplitude
        self.vel2 = self.antiflocking_potential(self.neighbours)
        self.last_nbh_time = self.get_clock().now()
        # self.get_logger().info(f"self.neighbours: {self.neighbours}") # PRINT LOG DEBUG
        # self.get_logger().info(f"Computed vel2 (anti-flocking): {self.vel2}") # PRINT LOG DEBUG

    def held_force_expired(self):
        if self.last_nbh_time is None:
            return True
        age = (self.get_clock().now() - self.last_nbh_time).nanoseconds / 1e9
        return age > self.nbh_hold_timeout

    def landmarks_stale(self, stamp):
        """True if a landmark detection stamp is older than the freshness
        timeout (or zero/absent, i.e. no detection yet)."""
        if stamp.sec == 0 and stamp.nanosec == 0:
            return True
        age = (self.get_clock().now() - Time.from_msg(stamp)).nanoseconds / 1e9
        return age > self.landmarks_freshness_timeout

    def landmarks_expired(self):
        """Loop-side guard: the topic keeps republishing so landmarks_callback
        fires, but if the detector node dies the topic stops entirely. Gate the
        publish loop on arrival time so a dead node still zeroes the flocking
        term rather than holding the last vel1 forever."""
        if self.last_landmarks_time is None:
            return True
        age = (self.get_clock().now() - self.last_landmarks_time).nanoseconds / 1e9
        return age > self.landmarks_freshness_timeout

    def own_velocity(self):
        """Our map-frame velocity, or None if it is too old to damp with.

        Same timeout as the neighbour hold: it is the same question -- how long
        may a force act on news that has not been reconfirmed -- and a second
        independent number would only make the ladder inconsistent. Returning
        None drops the damping term and leaves the potential, which is the
        behaviour before this term existed, rather than damping against a
        velocity we no longer have.
        """
        if self.velocity is None or self.last_odom_time is None:
            return None
        age = (self.get_clock().now() - self.last_odom_time).nanoseconds / 1e9
        if age > self.nbh_hold_timeout:
            return None
        return self.velocity

    def antiflocking_potential(self, neighbours):
        """Repulsion from neighbours: potential plus closing-rate damping."""
        Fx, Fy = neighbour_repulsion(
            self.pose[:2],
            self.own_velocity(),
            neighbours,
            self.Rant,
            self.antflk_gain,
            self.damping_gain,
            self.max_nbh_force,
            epsilon=self.eps,
        )

        self.nbh_Fx = Fx
        self.nbh_Fy = Fy

        return np.array([Fx, Fy])


    ######################################################################################################

    def publish_target_position(self):
        # Wait for initialization before triggering the controller
        now = self.get_clock().now()
        elapsed = (now - self.init_start_time).nanoseconds / 1e9
        if elapsed <= 2 * self.init_duration:
            return
        ############################################################

        if None in self.pose:
            self.get_logger().warn("Pose not initialized yet, skipping publish.",
                                   throttle_duration_sec=5.0)
            return

        # Loop-side freshness guard: covers the detector node dying outright (no
        # more messages -> landmarks_callback never fires to zero vel1). Neighbour
        # avoidance (vel2) is landmark-independent and stays active, so the robot
        # holds against the flock but still avoids collisions.
        if self.landmarks_expired():
            self.vel1 = np.array([0, 0])

        # Calculate next waypoint position target
        self.target = self.pose[:2] + self.step * (self.vel1 + self.vel2)
        # self.get_logger().warn(f"switched position target to: {self.target}\n") 

        msg = CoordXY()
        msg.x = self.target[0]
        msg.y = self.target[1]
        self.pub.publish(msg)

        wmsg = Wrench()
        wmsg.force.x = self.nbh_Fx
        wmsg.force.y = self.nbh_Fy
        self.wrench_nbh_pub.publish(wmsg)

        # Flocking wrench: the same landmark potential vel1, now as a map-frame
        # force, but published only while ENCAPSULATION is active. Disabled ->
        # explicit zero (see the publisher comment). Magnitude clamped to
        # max_flk_force so this layer claims at most its share of the summed
        # envelope; direction is preserved.
        fmsg = Wrench()
        if self.encapsulation_enabled:
            fx, fy = float(self.vel1[0]), float(self.vel1[1])
            mag = math.hypot(fx, fy)
            if mag > self.max_flk_force:
                scale = self.max_flk_force / mag
                fx *= scale
                fy *= scale
            fmsg.force.x = fx
            fmsg.force.y = fy
        self.wrench_flk_pub.publish(fmsg)

def map_frame_velocity(msg):
    """Linear velocity from an Odometry message, rotated into the map frame.

    REP-105: Odometry.pose is in header.frame_id (swarm_map) but Odometry.twist
    is in child_frame_id (<ns>/base_link), i.e. body frame. Neighbour velocities
    arrive already rotated to map frame by state_broadcaster, so ours has to be
    rotated to match or the relative velocity the damping term forms is wrong by
    our own yaw -- which spans the full circle in practice, so the error is not
    a small bias but a sign flip half the time.

    The yaw comes from this message's own quaternion rather than from StatePos,
    whose .theta is in DEGREES; feeding that to cos/sin would silently rotate
    the result by an arbitrary angle.
    """
    t = msg.twist.twist
    q = msg.pose.pose.orientation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    c, s = math.cos(yaw), math.sin(yaw)
    return (t.linear.x * c - t.linear.y * s,
            t.linear.x * s + t.linear.y * c)


def make_array(list_coord):
    # Any sequence of messages with .x / .y => (N, 2) array. Neighbours no
    # longer come through here; they arrive as NeighbourStates and keep their
    # velocities, which this shape would discard.

    M = len(list_coord)
    arr = np.zeros((M, 2))
    for i in range(M):
        arr[i, 0] = list_coord[i].x
        arr[i, 1] = list_coord[i].y
    return arr



def main(args=None):
    rclpy.init(args=args)
    node = GuidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()