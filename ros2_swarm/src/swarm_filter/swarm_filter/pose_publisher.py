import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np

from marvelmind_ros2_msgs.msg import HedgePositionAddressed
from nav_msgs.msg import Odometry

from transforms3d.euler import quat2euler
from swarm_interfaces.msg import StatePos as State # Custom message

def degrees(angle):
    # The angle in the Kalman filter state is in radians, this is just when publishing, we publish in degrees
    # takes angle in radians (radian per second)
    # convert it to degree between -180 and 180 degrees

    angle = angle / np.pi * 180 # convert to degree
    angle = (angle + 180) % 360 - 180 # wrap angle

    return angle

###################################################################################################################33

class PosePublisherNode(Node):
    def __init__(self):
        super().__init__('pose_publisher')
        self.get_logger().info("Pose publisher has been started.")

        # Initial pose
        self.x = np.array([0.0, 0.0, 0.0])

        self.declare_parameter("init_orientation", 0.0) # in degrees
        self.x[2] = self.get_parameter("init_orientation").value / 180 * np.pi

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

        
        # For initial pose initialization using GPS / Odometry readings
        self.declare_parameter("init_period", 1.0)
        self.gps_initializing = True
        self.odom_initializing = True
        self.init_duration = self.get_parameter("init_period").value
        self.init_start_time = self.get_clock().now()
        self.pos_buffer = []
        self.angle_buffer = []
        self.init_pos = np.zeros(2)
        self.init_angle = 0.0
        self.get_logger().info("Initializing initial position/orientation... Please keep the devices stationary.")

        # Subscriptions (GPS, Odometry, IMU + MM_IMU)
        self.create_subscription(HedgePositionAddressed, 'mm_pos', self.indoor_gps_callback, self.qos_profile)
        self.create_subscription(Odometry, 'odom', self.odom_callback, self.qos_profile)

        # Publisher for estimated state
        self.declare_parameter("timer_frequency", 0.01)
        self.timer_frequency = self.get_parameter("timer_frequency").value
        self.pose_pub = self.create_publisher(State, 'pose', self.qos_profile)
        self.pub_timer = self.create_timer(self.timer_frequency, self.publish_pose)

    ########################################################################

    def indoor_gps_callback(self, msg: HedgePositionAddressed):
        now = self.get_clock().now()
        elapsed = (now - self.init_start_time).nanoseconds / 1e9

        # === Initialization Phase ===
        if self.gps_initializing:
            pos = np.array([
                msg.x_m,
                msg.y_m
            ])
            self.pos_buffer.append(pos)

            if elapsed >= self.init_duration:
                self.init_pos = np.mean(self.pos_buffer, axis=0)
                self.x[:2] = self.init_pos
                self.gps_initializing = False
                self.get_logger().info("Indoor GPS initialization complete.")
                self.get_logger().info(f"Initial position: {self.init_pos} meters")
            return
        
        ### ============================================================= ###
        self.x[:2] = np.array([msg.x_m, msg.y_m])

    ########################################################################

    def odom_callback(self, msg: Odometry):
        now = self.get_clock().now()
        elapsed = (now - self.init_start_time).nanoseconds / 1e9

        # === Initialization Phase ===
        if self.odom_initializing:
            q = msg.pose.pose.orientation
            quat = [q.w, q.x, q.y, q.z]
            rpy = quat2euler(quat, axes='sxyz')
            yaw = rpy[2]
            self.angle_buffer.append(yaw)

            if elapsed >= self.init_duration:
                self.init_angle = np.mean(self.angle_buffer, axis=0) - self.x[2]
                self.odom_initializing = False
                self.get_logger().info("Yaw angle initialization complete.")
                self.get_logger().info(f"Initial angle wrap: {self.init_angle} rad")
            return
        
        ### ========================================================== ###

        q = msg.pose.pose.orientation
        quat = [q.w, q.x, q.y, q.z]
        rpy = quat2euler(quat, axes='sxyz')
        yaw = rpy[2]
        self.x[2] = yaw - self.init_angle # incremental

    ########################################################################

    def publish_pose(self):
        msg = State()

        msg.x = self.x[0]
        msg.y = self.x[1]
        theta = self.x[2] 
        msg.theta = degrees(theta)
        self.pose_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PosePublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
