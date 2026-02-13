import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np

# Custom messages
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from std_srvs.srv import SetBool

# Custom messages for target position
from swarm_interfaces.msg import StatePos as State 
from swarm_interfaces.msg import CoordXY

class ControllerNode(Node):
    def __init__(self):
        super().__init__('target_orientation_controller')
        self.get_logger().info("Target orientation controller has been started.")

        # For initial pose initialization using GPS / Odometry readings
        self.declare_parameter("init_period", 1.0)
        self.init_duration = self.get_parameter("init_period").value

        # For timer
        self.declare_parameter("timer_frequency", 0.01)
        self.timer_frequency = self.get_parameter("timer_frequency").value
        self.get_logger().info(f"Timer frequency: {self.timer_frequency} seconds")

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
        self.target = np.array([None,None,None])

        # Gains & Limits
        self.init_start_time = self.get_clock().now()
        self.max_rotational_speed = 0.5
        self.max_translational_speed = 0.2

        # Service to control the control mode
        self.capture_mode = False
        self.srv = self.create_service(SetBool, 'toggle_controller', self.handle_controller)

        # Velocity commands
        self.trq_cmd = None
        self.frc_cmd = np.array([None,None])

        # Subscribers
        self.create_subscription(State, 'pose', self.pose_callback, self.qos_profile)
        self.create_subscription(CoordXY, 'target_position', self.target_position_callback, self.qos_profile)
        self.create_subscription(Float64, 'target_orientation', self.target_orientation_callback, self.qos_profile)
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', self.qos_profile)
        self.cmd_pub_timer = self.create_timer(self.timer_frequency, self.publish_cmd)


    def target_position_callback(self, msg:CoordXY):
        self.target[0] = msg.x
        self.target[1] = msg.y

    def target_orientation_callback(self, msg:Float64):
        self.target[2] = (msg.data + 180) % 360 - 180

    def pose_callback(self, msg: State):
        # self.get_logger().info(f"nself.target: {self.target}") # PRINT LOG DEBUG
        if None in self.target:
            return
        self.pose = np.array([msg.x, msg.y, msg.theta])


        # calculating position force command
        deadband_frc = 0.5
        gain = 1.5
        error = self.target[:2] - self.pose[:2]
        force = gain * error

        # self.get_logger().info(f"np.linalg.norm(force): {np.linalg.norm(force)}") # PRINT LOG DEBUG

        if np.linalg.norm(force) < deadband_frc:
            force = np.zeros((2,))
        

        # calculating pointing torque command
        deadband_trq = 0.5
        gain = 0.05
        angle_error = (self.target[2] - self.pose[2] + 180) % 360 - 180
        torque = gain * angle_error
        if np.linalg.norm(torque) < deadband_trq:
            torque = 0.0


        # convert input to local frame
        input = np.append(force, torque).astype(float)
        output = glob_to_loc(input, self.pose[2])
        self.frc_cmd = output[:2]
        self.trq_cmd = output[2]
        

    def publish_cmd(self):
        # Wait for initialization before triggering the controller
        now = self.get_clock().now()
        elapsed = (now - self.init_start_time).nanoseconds / 1e9
        if elapsed <= 2 * self.init_duration:
            return
        #################################################################
        # If capture mode is on, do not publish commands
        if self.capture_mode:
            # self.get_logger().info('Capture mode is ON, not publishing commands.')
            return

        if self.trq_cmd == None:
            return
        
        if None in self.frc_cmd:
            return

        trq = bound(self.trq_cmd, self.max_rotational_speed)
        frc = np.zeros((2,))
        for i in range(2):
            frc[i] = bound(self.frc_cmd[i], self.max_translational_speed)

        msg = Twist()
        msg.linear.x = frc[0]
        msg.linear.y = frc[1]
        msg.angular.z = trq
        self.cmd_pub.publish(msg)

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


def bound(input, bound):
    output = max(min(input, bound), -bound)
    return output

def glob_to_loc(input, theta):
    '''
    input = np.array of size (1,3)
    '''
    theta = np.radians(theta)
    coord_trans = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
    coord_trans = np.linalg.inv(coord_trans)
    result = coord_trans @ input[0:2]

    output = np.append(result,input[2]).astype(float)
    return output

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
