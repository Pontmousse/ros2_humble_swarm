import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np

# Built-in messages
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from std_srvs.srv import SetBool

# Custom messages for target position
from swarm_interfaces.msg import StatePos as State 
from swarm_interfaces.msg import CoordXY, Markers, Neighbours

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

        # Geometric Visual Camera to Aruco position
        self.closest_target_index = None
        self.ref_offset_x = 10.0
        self.ref_offset_y = 0.0

        # Velocity commands
        self.trq_cmd = None
        self.frc_cmd = np.array([None,None])

        # Subscribers
        self.create_subscription(Markers, 'targets', self.target_callback, self.qos_profile)
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', self.qos_profile)
        self.cmd_pub_timer = self.create_timer(self.timer_frequency, self.publish_cmd)

        # Service to control the control mode
        self.capture_mode = False
        self.srv = self.create_service(SetBool, 'toggle_controller', self.handle_controller)

        # Magnet Client
        self.services_called = False
        self.wheel_client = self.create_client(SetBool, 'engage_wheels')

        self.magnet_client = self.create_client(SetBool, 'toggle_magnet')
        self.call_service(self.magnet_client, True, "Toggle Magnet")
        
        

 
    
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
        # Calculating force command - approaching the target
        ######################################################
        # Extracting closest target position
        target = msg.markers[self.closest_target_index]
        target = np.array([target.x, target.y])

        dx = target[0] - self.ref_offset_x
        dy = target[1] - self.ref_offset_y
        self.frc_cmd[0] = dx
        self.frc_cmd[1] = dy

        ######################################################
        # Calculating force command - aligning with the target
        ######################################################

        # Add sliding y-component based on angle to target
        angle = np.arctan2(dy, dx)
        alignment_gain = 200  # Small angle-based nudge
        self.frc_cmd[1] += alignment_gain * angle  # simple y correction


        ######################################################
        # Calculating force command - avoiding nearby agents
        ######################################################



        ######################################################
        # Torque command calculation 
        ######################################################
        # Robot (camera) direction in marker/world frame
        orien = CoordXY()
        orien.x = msg.markers_normals[self.closest_target_index].x
        orien.y = msg.markers_normals[self.closest_target_index].y


        marker_dir = np.array([orien.x, orien.y])
        angle = np.arctan2(marker_dir[0], marker_dir[1])  # Angle in radians
        angle = -1*np.degrees(angle)  # Convert to degrees and flip sign (tested)

        
        
        # Calculating torque command        
        self.trq_cmd = (angle + 180) % 360 - 180
        # self.get_logger().warn(f"Closest target: {target}, angle: {self.trq_cmd:.2f} degrees\n") # Debugging output
        
        ########################################
        # Check distance and trigger magnet
        distance = np.hypot(dx, dy)
        if distance < 1 and not self.services_called:
            # self.call_service(self.magnet_client, True, "Toggle Magnet")
            self.call_service(self.wheel_client, False, "Disengage Wheels")
            self.services_called = True

 
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

        if self.services_called:
            return
        
        if not self.capture_mode:
            # self.get_logger().info('Encapsulate mode is ON, publishing commands.')
            return
        
        if self.trq_cmd == None:
            return
        
        if None in self.frc_cmd:
            return

        trq = bound(self.trq_cmd, self.max_rotational_speed)
        frc = np.zeros((2,))
        for i in range(2):
            frc[i] = bound(self.frc_cmd[i], self.max_translational_speed[i])

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
