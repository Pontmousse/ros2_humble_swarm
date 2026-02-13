import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import numpy as np

# Custom messages
from swarm_interfaces.msg import StatePos as State 
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

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
        
        # Control entities in global reference frame
        self.pose = np.array([None,None,None]) # robot pose
        self.target = None

        # Gains & Limits
        self.init_start_time = self.get_clock().now()
        self.max_rotational_speed = 10.0

        # Velocity commands
        self.trq_cmd = None

        # Subscribers
        self.create_subscription(State, 'pose', self.pose_callback, self.qos_profile)
        self.create_subscription(Float64, 'target_orientation', self.target_orientation_callback, self.qos_profile)

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', self.qos_profile)
        self.cmd_pub_timer = self.create_timer(self.timer_frequency, self.publish_cmd)


    def target_orientation_callback(self, msg:Float64):
        self.target = (msg.data + 180) % 360 - 180

    def pose_callback(self, msg: State):
        if self.target is None:
            return
        
        self.pose = np.array([msg.x, msg.y, msg.theta])

        # calculating pointing torque command
        deadband_trq = 0.1
        gain = 0.05
        angle_error = (self.target - self.pose[2] + 180) % 360 - 180
        torque = gain * angle_error
        if np.linalg.norm(torque) < deadband_trq:
            torque = 0.0

        self.trq_cmd = torque


    def publish_cmd(self):
        # Wait for initialization before triggering the controller
        now = self.get_clock().now()
        elapsed = (now - self.init_start_time).nanoseconds / 1e9
        if elapsed <= 2 * self.init_duration:
            return
        
        if self.trq_cmd is None:
            return
        
        trq = bound_input(self.trq_cmd, self.max_rotational_speed)

        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.angular.z = trq
        self.cmd_pub.publish(msg)


def bound_input(value, bound_trq):
    sign = 1 if value >= 0 else -1
    bounded_ang_vel = min(abs(value), bound_trq)
    return bounded_ang_vel * sign


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
