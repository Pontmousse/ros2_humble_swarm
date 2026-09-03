import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import numpy as np

# Standard messages
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from std_srvs.srv import SetBool

# Custom messages
from swarm_interfaces.msg import StatePos as State
from swarm_interfaces.msg import CoordXY


class ControllerNode(Node):

    def __init__(self):
        super().__init__('target_orientation_controller')

        self.get_logger().info(
            "Target orientation controller has been started."
        )

        # ============================================================
        # Parameters
        # ============================================================

        # Initial pose initialization period
        self.declare_parameter("init_period", 1.0)
        self.init_duration = self.get_parameter("init_period").value

        # Timer period
        self.declare_parameter("timer_frequency", 0.01)
        self.timer_frequency = self.get_parameter(
            "timer_frequency"
        ).value

        self.get_logger().info(
            f"Timer frequency: {self.timer_frequency} seconds"
        )

        # ============================================================
        # QoS
        # ============================================================

        self.declare_parameter('qos_depth', 10)
        self.declare_parameter('qos_reliability', 'RELIABLE')
        self.declare_parameter('qos_history', 'KEEP_LAST')

        qos_depth = self.get_parameter('qos_depth').value
        qos_reliability = self.get_parameter(
            'qos_reliability'
        ).value
        qos_history = self.get_parameter(
            'qos_history'
        ).value

        # Validate QoS settings
        valid_reliabilities = [
            'RELIABLE',
            'BEST_EFFORT'
        ]

        valid_histories = [
            'KEEP_LAST',
            'KEEP_ALL'
        ]

        if qos_reliability not in valid_reliabilities:
            raise ValueError(
                f"Invalid qos_reliability: {qos_reliability}"
            )

        if qos_history not in valid_histories:
            raise ValueError(
                f"Invalid qos_history: {qos_history}"
            )

        self.qos_profile = QoSProfile(
            depth=qos_depth,

            reliability=(
                ReliabilityPolicy.RELIABLE
                if qos_reliability == 'RELIABLE'
                else ReliabilityPolicy.BEST_EFFORT
            ),

            history=(
                HistoryPolicy.KEEP_LAST
                if qos_history == 'KEEP_LAST'
                else HistoryPolicy.KEEP_ALL
            )
        )

        # ============================================================
        # Robot state and targets
        # ============================================================

        # Robot pose:
        # [x, y, theta]
        self.pose = np.array(
            [np.nan, np.nan, np.nan],
            dtype=float
        )

        # Target:
        # [x_target, y_target, theta_target]
        #
        # np.nan means that particular target is not available.
        self.target = np.array(
            [np.nan, np.nan, np.nan],
            dtype=float
        )

        # ============================================================
        # Gains and limits
        # ============================================================

        self.init_start_time = self.get_clock().now()

        self.max_rotational_speed = 2.0
        self.max_translational_speed = 1.0

        # ============================================================
        # Controller mode
        # ============================================================

        self.capture_mode = False

        self.srv = self.create_service(
            SetBool,
            'toggle_controller',
            self.handle_controller
        )

        # ============================================================
        # Velocity commands
        # ============================================================

        # Always keep commands numeric.
        #
        # If there is no target for one control component,
        # that component simply remains zero.
        self.trq_cmd = 0.0
        self.frc_cmd = np.zeros(2, dtype=float)

        # ============================================================
        # Subscribers
        # ============================================================

        self.create_subscription(
            State,
            'pose',
            self.pose_callback,
            self.qos_profile
        )

        self.create_subscription(
            CoordXY,
            'target_position',
            self.target_position_callback,
            self.qos_profile
        )

        self.create_subscription(
            Float64,
            'target_orientation',
            self.target_orientation_callback,
            self.qos_profile
        )

        # ============================================================
        # Publisher
        # ============================================================

        self.cmd_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            self.qos_profile
        )

        # ============================================================
        # Timer
        # ============================================================

        self.cmd_pub_timer = self.create_timer(
            self.timer_frequency,
            self.publish_cmd
        )


    # ================================================================
    # Target position callback
    # ================================================================

    def target_position_callback(self, msg: CoordXY):

        self.target[0] = msg.x
        self.target[1] = msg.y


    # ================================================================
    # Target orientation callback
    # ================================================================

    def target_orientation_callback(self, msg: Float64):

        # Normalize target angle to [-180, 180)
        self.target[2] = (
            (msg.data + 180.0) % 360.0
        ) - 180.0


    # ================================================================
    # Pose callback / controller
    # ================================================================

    def pose_callback(self, msg: State):

        # Update robot pose
        self.pose = np.array(
            [
                msg.x,
                msg.y,
                msg.theta
            ],
            dtype=float
        )

        # self.get_logger().info(
        #     "pose: {}".format(self.pose)
        # )

        # ============================================================
        # Translation controller
        # ============================================================

        # Default:
        # no translation command
        force = np.zeros(2, dtype=float)

        # Translation is enabled ONLY if both x and y targets exist.
        position_target_available = (
            np.isfinite(self.target[0])
            and np.isfinite(self.target[1])
        )

        if position_target_available:

            deadband_frc = 0.2
            gain_frc = 1.5

            # Global position error
            error = (
                self.target[:2]- self.pose[:2]
            )

            # Proportional controller
            force = gain_frc * error

            # Deadband
            if np.linalg.norm(force) < deadband_frc:
                force = np.zeros(
                    2,
                    dtype=float
                )

            # self.get_logger().info(
            #     "force: {}".format(np.linalg.norm(force)) + " <===> " + "deadband: {}".format(np.linalg.norm(deadband_frc))
            # )

        # ============================================================
        # Rotation controller
        # ============================================================

        # Default:
        # no rotation command
        torque = 0.0

        # Rotation is enabled if orientation target exists.
        orientation_target_available = np.isfinite(
            self.target[2]
        )

        if orientation_target_available:

            deadband_trq = 1.0
            gain_trq = 0.05

            # Shortest angular error in degrees:
            # [-180, 180)
            angle_error = (
                self.target[2]
                - self.pose[2]
                + 180.0
            ) % 360.0 - 180.0

            # Proportional controller
            torque = gain_trq * angle_error

            # Deadband
            if abs(torque) < deadband_trq:
                torque = 0.0

        # ============================================================
        # Convert global translation command to robot-local frame
        # ============================================================

        controller_input = np.array(
            [
                force[0],
                force[1],
                torque
            ],
            dtype=float
        )

        controller_output = glob_to_loc(
            controller_input,
            self.pose[2]
        )

        self.frc_cmd = controller_output[:2]
        self.trq_cmd = controller_output[2]


    # ================================================================
    # Publish velocity command
    # ================================================================

    def publish_cmd(self):

        # ------------------------------------------------------------
        # Wait for initialization
        # ------------------------------------------------------------

        now = self.get_clock().now()

        elapsed = (
            now - self.init_start_time
        ).nanoseconds / 1e9

        if elapsed <= 2 * self.init_duration:
            return

        # ------------------------------------------------------------
        # Capture mode
        # ------------------------------------------------------------

        # If capture mode is ON,
        # this controller does not publish commands.
        if self.capture_mode:
            return

        # ------------------------------------------------------------
        # Apply velocity limits
        # ------------------------------------------------------------

        trq = bound(
            self.trq_cmd,
            self.max_rotational_speed
        )

        frc = np.zeros(
            2,
            dtype=float
        )

        for i in range(2):

            frc[i] = bound(
                self.frc_cmd[i],
                self.max_translational_speed
            )

        # ------------------------------------------------------------
        # Build Twist message
        # ------------------------------------------------------------

        msg = Twist()

        msg.linear.x = float(frc[0])
        msg.linear.y = float(frc[1])

        msg.angular.z = float(trq)

        self.cmd_pub.publish(msg)


    # ================================================================
    # Toggle controller service
    # ================================================================

    def handle_controller(
        self,
        request,
        response
    ):

        if request.data:

            self.capture_mode = True

            self.get_logger().info(
                'Capture mode -> ON'
            )

            response.success = True
            response.message = (
                'Controller set to Capture mode'
            )

        else:

            self.capture_mode = False

            self.get_logger().info(
                'Encapsulate mode -> ON'
            )

            response.success = True
            response.message = (
                'Controller set to Encapsulate mode'
            )

        return response


# ====================================================================
# Bound value
# ====================================================================

def bound(value, limit):

    return max(
        min(value, limit),
        -limit
    )


# ====================================================================
# Global -> local coordinate transformation
# ====================================================================

def glob_to_loc(input_cmd, theta):

    """
    input_cmd:
        np.array([Fx, Fy, torque])

    theta:
        robot orientation in degrees
    """

    theta = np.radians(theta)

    rotation_matrix = np.array(
        [
            [
                np.cos(theta),
                -np.sin(theta)
            ],
            [
                np.sin(theta),
                np.cos(theta)
            ]
        ]
    )

    # Global -> local
    coord_trans = np.linalg.inv(
        rotation_matrix
    )

    result = (
        coord_trans
        @ input_cmd[0:2]
    )

    output = np.array(
        [
            result[0],
            result[1],
            input_cmd[2]
        ],
        dtype=float
    )

    return output


# ====================================================================
# Main
# ====================================================================

def main(args=None):

    rclpy.init(args=args)

    node = ControllerNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()