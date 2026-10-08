import math

import numpy as np
import rclpy

from geometry_msgs.msg import Wrench
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

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
        self.Rflk = 0.4
        # Two robots touch at 0.44 m centre-to-centre (2 x the robot_radius 0.22
        # declared in virtual_spacecraft.yaml). Rant was 0.4, i.e. the influence
        # radius sat entirely inside the collision: the repulsion was still
        # exactly 0 N at contact and only reached 1 N once the chassis had
        # overlapped by 4 cm. 0.70 puts the onset 0.26 m clear of contact.
        self.Rant = 0.70
        self.flk_gain = 0.5
        # Sized so the repulsion reaches max_nbh_force at d = 0.55 m, 0.11 m
        # before contact -- about one second of warning at the 0.1 m/s design
        # closing speed.
        self.antflk_gain = 0.78
        # The potential diverges as 1/d^3, so without a cap it reached 18 N at
        # d = 0.15 m. Two things make that unacceptable: the HIL dynamics were
        # only validated to 1.0 N / 0.2 Nm (2026-09-09 bag), and publish_target_position
        # reuses this same vector as a metre-scale waypoint offset at step=1.0,
        # so an 18 N force also threw the position target 18 m away.
        self.max_nbh_force = 1.0
        # Dissipative term, F_damp = damping_gain * max(0, closing speed). The
        # potential above is conservative, so the work it can do between Rant
        # and contact is fixed no matter how fast the pair is closing: it has a
        # single critical speed (0.368 m/s here) and the excess above it is
        # untouched. Measured 2026-10-07, RM2 and RM6 converged at 0.445 m/s and
        # touched. Damping removes energy instead of storing it, so it scales
        # with the actual approach, and it is at full strength the instant a
        # fast approach crosses Rant -- which is exactly where the potential is
        # weakest (0.087 N at d = 0.676 m, so the first 0.1 m of the 0.26 m
        # braking runway was contributing almost nothing).
        #
        # 2.0 is not tuned: two references capped at virtual_spacecraft
        # maximum_speed = 0.25 m/s can close at 0.5 m/s, and 2.0 * 0.5 = 1.0 N
        # spends exactly the validated envelope at that worst case. Raising it
        # buys nothing -- see test_damping_cannot_beat_the_force_times_runway
        # _ceiling: no law bounded by 1.0 N over this runway absorbs more than
        # 0.456 m/s, which is still below 0.5. Damping is necessary, not
        # sufficient; closing the rest needs maximum_speed or Rant to move.
        self.damping_gain = 2.0
        self.eps = 1e-4



        # velocity commands
        self.vel1 = np.array([0, 0])
        self.vel2 = np.array([0, 0])
        self.step = 1.0

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
        self.pub = self.create_publisher(CoordXY, 'target_position', self.qos_profile_reliable)
        self.pub_timer = self.create_timer(self.timer_frequency, self.publish_target_position)
        

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


    def flocking_potential(self, landmarks):
        Fx, Fy = 0.0, 0.0
        x, y = self.pose[0], self.pose[1]
        
        for i in range(len(landmarks)):
            xp, yp = landmarks[i, 0], landmarks[i, 1]
            dx = xp - x
            dy = yp - y
            distance = np.hypot(dx, dy) + self.eps

            # Spring-like attractive/repulsive force
            force_magnitude = self.flk_gain * (distance - self.Rflk)  # Attract if > Rflk, repel if < Rflk

            Fx += force_magnitude * (dx / distance)
            Fy += force_magnitude * (dy / distance)

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
            self.vel1 = np.array([0, 0])
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