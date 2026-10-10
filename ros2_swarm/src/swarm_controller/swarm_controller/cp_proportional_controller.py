from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np

# Built-in messages
from geometry_msgs.msg import Twist, Wrench
from std_msgs.msg import Float64
from std_srvs.srv import SetBool

# Custom messages for target position
from swarm_interfaces.msg import StatePos as State
from swarm_interfaces.msg import CoordXY, Markers, Neighbours

# Outlier guards on the target track. Not tuning knobs: the gap is longer than
# the 0.61 s frozen-fix period seen on RM6, so only a real dropout trips it.
#
# The jump is a teleport threshold, which is why it is an absolute distance
# and not a speed -- the size of a bad fix does not depend on how fast we
# sample it. Between consecutive markers the honest worst case is two 52 mm
# fixes disagreeing plus the target's own motion, well under 0.2 m. Beacon 5
# was observed on 2026-10-10 throwing ~2.1 m outliers roughly every 11 s,
# simultaneously visible to both robots, which is what this is sized for.
TRACK_MAX_GAP = 1.0
TRACK_MAX_JUMP = 0.50


class ControllerNode(Node):
    def __init__(self):
        super().__init__('target_orientation_controller')
        self.get_logger().info("Target orientation controller has been started.")

        # For initial pose initialization using GPS / Odometry readings
        self.declare_parameter("init_period", 1.0)
        self.init_duration = self.get_parameter("init_period").value

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
        self.qos_profile = QoSProfile(
            depth=qos_depth,
            reliability=ReliabilityPolicy.RELIABLE if qos_reliability == 'RELIABLE' else ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST if qos_history == 'KEEP_LAST' else HistoryPolicy.KEEP_ALL
        )
        # # # # # # # # # # # # # # # # # # # # # # # # # # # 

        # Gains & Limits
        self.init_start_time = self.get_clock().now()
        self.max_rotational_speed = 0.2
        self.max_translational_speed = [0.05, 0.15] # x and y max speeds

        # Standoff point ahead of the robot centre, in cm to match Markers.
        # Used only to pick the closest target, where the units cancel inside
        # argmin. The approach law is referenced to the robot centre instead,
        # which is what keeps buffer_radius a true circle.
        self.closest_target_index = None
        self.ref_offset_x = 10.0
        self.ref_offset_y = 0.0

        # Keep-out radius around the target, METRES, measured from the robot
        # CENTRE so it is a true circle on the target. Measuring from the
        # standoff point instead makes the radius depend on bearing (60 cm dead
        # ahead but only 40 cm with it behind), which is not a keep-out.
        #
        # Raised 0.40 -> 0.60 on 2026-10-10. The lever arm for every bearing
        # quantity below is this radius, so the 52 mm position noise now
        # subtends 0.087 rad instead of 0.130: the heading saturation ceiling
        # rises 1.5 -> 2.3 and every gain below gains margin rather than
        # losing it. What it costs is inter-robot geometry -- see
        # avoid_override_force.
        self.buffer_radius = 0.60

        # Approach law gains.
        #
        # frc_cmd is a VELOCITY in m/s -- it is published straight into
        # Twist.linear -- so the gain taking a position error in metres to it
        # has units of s^-1; the loop time constant is 1/approach_gain.
        # The previous law put the error in CENTIMETRES into that slot, an
        # implicit 100 s^-1, and bound() then clamped it against
        # max_translational_speed: the linear region was 0.5 mm wide in x and
        # 1.5 mm in y, two orders of magnitude under the 52 mm Marvelmind noise
        # floor, so only the SIGN of the error ever reached the wheels.
        #
        # Three constraints bracket it, and the tightest is the chassis.
        #
        # Sampling ceiling, gain < 3.3. The period that matters is the
        # MEASUREMENT period, not timer_frequency -- frc_cmd only changes when
        # target_callback fires. A zero-order-hold proportional loop is stable
        # for gain * T < 2; Marvelmind gives T = 0.35 s with three beacons
        # sharing the TDMA cycle, and 0.61 s was measured on RM6 while fixes
        # were freezing, so the frozen period binds. Simulation starts
        # oscillating at 4.0, which is where 4.0 * 0.61 crosses 2.
        #
        # Saturation ceiling, gain < 0.96. The linear region is only
        # max_translational_speed / gain wide, and body x caps at 0.05 m/s, so
        # above 0.05 / 0.052 the 52 mm Marvelmind noise floor alone saturates
        # the command and its sign is set by noise. That is how the old law
        # became a relay in the first place -- it had the same cap and an
        # implicit gain of 100 -- so staying under this is the whole point.
        #
        # Tracking floor, gain > 0.58. Proportional-only has no integrator, so
        # a target moving at v is held at buffer_radius + v / gain. RM5 was
        # driven at 0.03 m/s, and 0.58 is where that lag reaches the same
        # 52 mm noise floor, below which it is not the dominant error.
        #
        # 0.9 sits inside all three: 56 mm of linear region against 52 mm of
        # noise, 33 mm of lag, gain * T = 0.55 at the frozen rate. The window
        # is narrow because body x is slow; widening it wants target-velocity
        # feedforward, not more gain. The old gain was 100.
        self.declare_parameter("approach_gain", 0.9)
        # Tangential alignment, (m/s)/rad. At 90 deg off-bore this is
        # 0.079 m/s, which at buffer_radius swings the bearing at 7.5 deg/s --
        # the same order as max_rotational_speed, so the two alignment paths
        # work at similar rates instead of one dominating. Set to 0.0 to steer
        # by rotation alone.
        self.declare_parameter("align_gain", 0.05)
        # Heading gain, (rad/s)/rad, so s^-1 like approach_gain. Bracketed the
        # same three ways, and again the chassis cap binds rather than the
        # sampling budget.
        #
        # Saturation ceiling, gain < 2.3. The linear region is
        # max_rotational_speed / gain, and the bearing comes from Marvelmind
        # through target_bridge, so at buffer_radius the 52 mm position noise
        # subtends 52/600 = 0.087 rad of bearing noise. Above 0.2 / 0.087 that
        # noise alone saturates the command and its sign comes from noise.
        # Still far coarser than the ~2 deg an ArUco bearing would give: the
        # lever arm is the standoff radius, and it shortens as the robot
        # closes, so this ceiling is worst exactly during capture.
        #
        # Sampling ceiling, gain < 3.3. As for approach_gain: 2 / T with
        # T = 0.61 s, the frozen-fix period measured on RM6.
        #
        # Tracking floor, gain > 0.58. A target crossing at 0.03 m/s sweeps
        # the bearing at v / r, and proportional-only holds it at that over
        # the gain. Both the sweep and the noise go as 1 / r, so this floor is
        # the ratio 0.03 / 0.052 and does not move with buffer_radius -- which
        # is why raising the radius buys headroom at the top and costs nothing
        # at the bottom.
        #
        # 1.0 sits inside all three with the most margin where it is scarcest:
        # 11.5 deg of linear region against 5.0 deg of bearing noise, 2.9 deg
        # of lag. The old law was a relay, so it had no gain to quote.
        self.declare_parameter("heading_gain", 1.0)

        # Target-velocity feedforward.
        #
        # Proportional-only has no integrator, so a target moving at v is held
        # at buffer_radius + v / approach_gain. Feeding v forward moves that
        # equilibrium back onto the circle. It does NOT help once the chassis
        # cap binds: the sum is still clamped to max_translational_speed, so a
        # target faster than 0.05 m/s in body x cannot be held by any gain or
        # feedforward. RM2 was opened up at 0.19 m/s on 2026-10-10 that way.
        #
        # The window is a noise budget, not a smoothing preference, and it is
        # in SECONDS rather than samples on purpose. Marvelmind is 52 mm at a
        # 0.35 s period, so a two-point difference carries 0.21 m/s of noise
        # against a 0.03 m/s signal -- seven times the thing being measured.
        # A least-squares slope over M INDEPENDENT fixes has noise
        # sigma / (T * sqrt(M(M^2-1)/12)): M = 15 gives 9 mm/s, under a third
        # of the signal, at the cost of lagging a change in target velocity by
        # about half the span. 15 fixes at 0.35 s is 5.25 s.
        #
        # Counting samples instead would be wrong, because target_callback
        # fires at the MARKERS rate, not the fix rate. target_bridge publishes
        # at 10 Hz by default (--rate), so a 15-SAMPLE window spans 1.5 s and
        # holds only ~4 independent fixes: the repeats are zero-order holds of
        # the same fix and carry the same noise, so they add nothing. That
        # window gives 59 mm/s of noise -- twice the signal -- which this term
        # would then feed straight into a 50 mm/s axis cap and re-create the
        # relay the approach law was just fixed for. A span in seconds is also
        # immune to someone changing --rate.
        #
        # This estimator is built for a target holding a steady velocity,
        # which is how RM5 is driven; it will not track one that manoeuvres.
        self.declare_parameter("target_velocity_span", 5.25)
        # Scales the feedforward. 0.0 disables it and restores the pure
        # proportional law, which is the comparison to run on hardware.
        self.declare_parameter("feedforward_gain", 1.0)

        self.approach_gain = self.get_parameter("approach_gain").value
        self.align_gain = self.get_parameter("align_gain").value
        self.heading_gain = self.get_parameter("heading_gain").value
        self.target_velocity_span = self.get_parameter(
            "target_velocity_span").value
        self.feedforward_gain = self.get_parameter("feedforward_gain").value

        # Collision avoidance during capture.
        #
        # capture_supervisor pauses velocity_smoother on entry, and that is the
        # only node carrying the search-side wrench to the wheels, so while we
        # hold cmd_vel the whole avoidance chain is disconnected -- two robots
        # chasing one target would close with nothing between them. Rather than
        # a second law here, reuse the one position_guidance3 already computes:
        # it keeps publishing avoidance_wrench at 100 Hz throughout capture
        # (publish_target_position has no capture gate), tuned and bag-validated
        # at Rant = 1.05 m, cap 1.0 N, with the closing-rate damping term and
        # the hold-on-UDP-dropout logic. One law covers search and capture.
        #
        # Override rather than sum: the approach law is a velocity command, so
        # summing a repulsion into it creates a spurious equilibrium wherever
        # the two cancel -- the robot would park between target and neighbour
        # under zero command with no reason to leave. Override makes the
        # precedence explicit and keeps the escape unconditional.
        # 0.70 N is a distance in disguise. The potential is monotonic in
        # separation, so thresholding the force thresholds d; with the live
        # gains (gain 0.78, Rant 1.05) 0.70 N is d = 0.71 m.
        #
        # The ceiling on it is geometric, not dynamic. Two robots holding
        # station on the same target both sit on a circle of radius
        # buffer_radius, so they can be at most 2 * buffer_radius = 0.80 m
        # apart. An override reaching past that can never be satisfied: both
        # would be permanently repelling, the target approach would never run,
        # and the pair would oscillate instead of parking on opposite arcs.
        # Raising buffer_radius raises this ceiling with it; raising this past
        # 2 * buffer_radius breaks station keeping. The damping term can still
        # trip the override earlier than 0.71 m on a genuinely fast approach,
        # which is the intent -- the effective radius grows with closing speed.
        #
        # What the radius does NOT do is make the override fire more often. On
        # the circle two robots are 2 * r * sin(dtheta / 2) apart, so 0.71 m
        # corresponds to a wedge of 126 deg at r = 0.40 but only 73 deg at
        # r = 0.60: the pair has to be angularly CLOSER before anything pushes
        # them apart. At the antipodal equilibrium the separation goes
        # 0.80 -> 1.20 m, past Rant = 1.05, so the sub-threshold background
        # repulsion that used to bias them apart (0.36 N at 0.80 m) now reads
        # exactly zero. Inter-robot avoidance is strictly less active after
        # this change, not more; the two robots converging from opposite sides
        # seen on 2026-10-10 is a station-keeping question, not an avoidance
        # one, and wants arc allocation rather than a bigger override.
        self.declare_parameter("avoid_override_force", 0.70)
        self.declare_parameter("avoid_timeout", 0.75)
        self.avoid_override_force = self.get_parameter("avoid_override_force").value
        self.avoid_timeout = self.get_parameter("avoid_timeout").value

        # Velocity commands
        self.trq_cmd = None
        self.frc_cmd = np.array([None,None])

        # Latest avoidance force (map frame, N) and our heading, for the
        # map->body rotation. StatePos.theta is in DEGREES.
        self.avoid = None
        self.avoid_stamp = None
        self.pose_x = None
        self.pose_y = None
        self.pose_theta = None

        # Map-frame track of the closest target, for the velocity estimate.
        # Pruned by age, not by count: see target_velocity_span.
        self.target_track = deque()

        # Subscribers
        self.create_subscription(Markers, 'targets', self.target_callback, self.qos_profile)
        # BEST_EFFORT on both: latest-value-wins streams where a retransmitted
        # stale pose or stale force is worse than a gap, same reason
        # position_guidance3 reads pose BEST_EFFORT. Both writers are RELIABLE,
        # which serves a BEST_EFFORT reader fine.
        qos_latest = QoSProfile(depth=1,
                                reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(State, 'pose', self.pose_callback, qos_latest)
        self.create_subscription(Wrench, 'avoidance_wrench',
                                 self.avoidance_callback, qos_latest)

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', self.qos_profile)
        self.cmd_pub_timer = self.create_timer(self.timer_frequency, self.publish_cmd)

        # Service to control the control mode
        self.capture_mode = False
        self.srv = self.create_service(SetBool, 'toggle_controller', self.handle_controller)

        # Magnet Client
        self.magnet_client = self.create_client(SetBool, 'toggle_magnet')
        # self.call_service(self.magnet_client, True, "Toggle Magnet")
        
        

 
    
    def pose_callback(self, msg: State):
        self.pose_x = msg.x
        self.pose_y = msg.y
        self.pose_theta = msg.theta

    def avoidance_callback(self, msg: Wrench):
        self.avoid = (msg.force.x, msg.force.y)
        self.avoid_stamp = self.get_clock().now()

    def avoidance_body(self):
        """Unit escape DIRECTION in body frame, or None if it must not act.

        None means "no override": either we cannot use the force (no heading,
        no force yet, or it is stale) or the neighbour is far enough that the
        target approach should keep running.
        """
        if self.avoid is None or self.avoid_stamp is None:
            return None
        if self.pose_theta is None:
            return None
        age = (self.get_clock().now() - self.avoid_stamp).nanoseconds * 1e-9
        # Same number as position_guidance3.nbh_hold_timeout. It is the same
        # question -- how long may a force act on news not reconfirmed -- and a
        # second independent value would only make the ladder inconsistent.
        if age > self.avoid_timeout:
            return None

        fx, fy = self.avoid
        magnitude = float(np.hypot(fx, fy))
        if magnitude < self.avoid_override_force:
            return None

        bx, by = self.to_body(fx, fy)
        # Normalised, because frc_cmd is a velocity in m/s and the wrench is in
        # newtons. Returning the raw force would push two different units
        # through one slot and only looked right while bound() saturated
        # everything. The magnitude has already done its job as the trigger at
        # avoid_override_force; what survives is the direction.
        return (bx / magnitude, by / magnitude)

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

        Lifted out of the body frame before differencing so that our own
        motion cancels exactly. Differencing the body-frame position instead
        would measure our velocity and the target's together, and backing ours
        out needs the commanded velocity and yaw rate -- neither of which is
        what the chassis actually did.
        """
        if self.pose_x is None or self.pose_theta is None:
            self.target_track.clear()
            return

        th = np.radians(self.pose_theta)
        cos_t, sin_t = np.cos(th), np.sin(th)
        now = self.get_clock().now().nanoseconds * 1e-9
        point = (now,
                 self.pose_x + cos_t * target_body[0] - sin_t * target_body[1],
                 self.pose_y + sin_t * target_body[0] + cos_t * target_body[1])

        if self.target_track:
            previous = self.target_track[-1]
            gap = point[0] - previous[0]
            jump = float(np.hypot(point[1] - previous[1],
                                  point[2] - previous[2]))
            # A least-squares slope is not robust to outliers, and this stack
            # produces them: a dropped fix leaves a gap the regression would
            # read as a long steady run, and a glitched pose puts one sample
            # metres away. Both restart the track rather than bias it.
            if gap > TRACK_MAX_GAP or jump > TRACK_MAX_JUMP:
                self.target_track.clear()

        self.target_track.append(point)

        # Keep one sample beyond the span so the window stays covered as it
        # slides, rather than alternating between just-short and just-over.
        while (len(self.target_track) > 2
               and point[0] - self.target_track[1][0]
               >= self.target_velocity_span):
            self.target_track.popleft()

    def target_velocity_map(self):
        """Least-squares target velocity in the map frame, m/s, or None.

        None until the track spans target_velocity_span, because that span IS
        the noise budget -- a slope off a short run carries more noise than the
        signal it is meant to feed forward. Note this gates on elapsed TIME,
        not sample count: the markers rate is 10 Hz but the fixes underneath
        change at 0.35 s, so a full-looking buffer can still hold only a
        handful of independent measurements.
        """
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
        if len(msg.markers) == 0:
            # self.get_logger().warn("Received empty targets.")
            self.frc_cmd = np.array([None, None])
            return

        ######################################################
        # Finding the closest target
        ######################################################
        target_distances = np.zeros((len(msg.markers),))
        for i in range(len(msg.markers)):
            target_distances[i] = np.linalg.norm(
                np.array([msg.markers[i].x, msg.markers[i].y]) -
                np.array([self.ref_offset_x, self.ref_offset_y])
            )

        self.closest_target_index = np.argmin(target_distances)
 
        ######################################################
        # Velocity command - hold station on the keep-out circle
        ######################################################
        # Body-frame target position in METRES, from the robot CENTRE.
        target = msg.markers[self.closest_target_index]
        target = np.array([target.x, target.y]) / 100.0
        self.track_target(target)
        distance = float(np.linalg.norm(target))

        if distance < 1e-3:
            # The unit vectors below are undefined here and the bearing is
            # meaningless; a detection this close is physically impossible
            # anyway, so command nothing rather than a direction off noise.
            self.frc_cmd[0] = 0.0
            self.frc_cmd[1] = 0.0
        else:
            bearing = float(np.arctan2(target[1], target[0]))
            u_r = target / distance                      # toward the target
            u_t = np.array([-u_r[1], u_r[0]])            # +bearing direction

            # Radial: the error is (distance - buffer_radius), so the command
            # is ZERO on the keep-out circle and changes sign smoothly across
            # it. The previous version held full scale on BOTH sides and
            # flipped the sign at the boundary -- a relay with no deadband and
            # no fixed point. There was no distance at which the robot was
            # commanded to stop, only a boundary to cross, which is the limit
            # cycle seen on RM6 on 2026-10-10.
            vel = self.approach_gain * (distance - self.buffer_radius) * u_r

            # Tangential: orbiting by delta along u_t changes the bearing by
            # -delta/distance, so a positive gain is negative feedback on
            # bearing. Tangential rather than along body y on purpose -- any
            # alignment term with a radial component makes the hold distance
            # depend on bearing, exactly what buffer_radius above rules out.
            # This duplicates the torque law's job; see align_gain.
            vel = vel + self.align_gain * bearing * u_t

            # Feedforward: at the fixed point the robot has to be MOVING, at
            # the target's velocity, and a proportional law can only produce
            # that from a standing error of v / approach_gain. Supplying it
            # directly puts the equilibrium back on the circle. Skipped
            # entirely while the estimate is None rather than substituting
            # zero, so a short or broken track degrades to the proportional
            # law instead of commanding against a stale velocity.
            v_target = self.target_velocity_map()
            if v_target is not None and self.pose_theta is not None:
                vx, vy = self.to_body(v_target[0], v_target[1])
                vel = vel + self.feedforward_gain * np.array([vx, vy])

            self.frc_cmd[0] = float(vel[0])
            self.frc_cmd[1] = float(vel[1])


        ######################################################
        # Torque command calculation 
        ######################################################
        # Robot (camera) direction in marker/world frame
        orien = CoordXY()
        orien.x = msg.markers_normals[self.closest_target_index].x
        orien.y = msg.markers_normals[self.closest_target_index].y


        marker_dir = np.array([orien.x, orien.y])
        # Argument order and sign flip are the tested convention; only the
        # units below changed. RADIANS, because trq_cmd is published into
        # Twist.angular and bounded against max_rotational_speed in rad/s.
        # It used to be DEGREES against the same bound, so the linear region
        # was 0.2 deg wide and every heading error above that saturated: the
        # same relay as the old approach law, with a 57.3x implicit gain.
        # Both robots then held a 1.2-1.4 s, +-13 deg bearing limit cycle on
        # 2026-10-10 -- period 4*T about the 0.32 s Marvelmind update, which
        # is a relay closed around the heading integrator.
        angle = -float(np.arctan2(marker_dir[0], marker_dir[1]))
        heading_error = (angle + np.pi) % (2.0 * np.pi) - np.pi
        self.trq_cmd = self.heading_gain * heading_error
        # self.get_logger().warn(f"Closest target: {target}, angle: {self.trq_cmd:.2f} degrees\n") # Debugging output


    ######################################################################################################

    def publish_cmd(self):
        # Wait for initialization before triggering the controller
        now = self.get_clock().now()
        elapsed = (now - self.init_start_time).nanoseconds / 1e9
        if elapsed <= 2 * self.init_duration:
            return
        #################################################################
        # Debugg
        # self.get_logger().warn(f"Force command: {str(self.frc_cmd)}")
        # self.get_logger().warn(f"Torque command: {str(self.trq_cmd)}")

        if not self.capture_mode:
            # self.get_logger().info('Encapsulate mode is ON, publishing commands.')
            return
        
        # Checked before the target guards below, so a neighbour closing on us
        # is still pushed away when no target is in view -- during capture
        # nothing else writes cmd_vel, so returning early here would leave the
        # pair to converge uncommanded.
        avoid = self.avoidance_body()

        if avoid is None and (self.trq_cmd is None or None in self.frc_cmd):
            return

        # Keep facing the target while backing off: it costs nothing and leaves
        # us oriented to re-approach once the neighbour clears.
        trq = (bound(self.trq_cmd, self.max_rotational_speed)
               if self.trq_cmd is not None else 0.0)

        frc = np.zeros((2,))
        if avoid is not None:
            # Escape at the axis caps. They are asymmetric (0.05 vs 0.15), so
            # this distorts the direction toward body y. It cannot reverse it:
            # each axis keeps its sign, so the commanded velocity always has a
            # positive projection on the true escape direction and separation
            # still grows.
            for i in range(2):
                frc[i] = avoid[i] * self.max_translational_speed[i]
        else:
            # Still bounded: the approach law tapers to zero on the keep-out
            # circle, but a target first seen far away commands more than the
            # chassis can do.
            for i in range(2):
                frc[i] = bound(self.frc_cmd[i],
                               self.max_translational_speed[i])

        msg = Twist()
        msg.linear.x = frc[0]
        msg.linear.y = frc[1]
        msg.angular.z = trq
        self.cmd_pub.publish(msg)


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
